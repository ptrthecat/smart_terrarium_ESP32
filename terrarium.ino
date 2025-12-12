#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>

#define LED_PIN 2
#define LDR_PIN 34

const int LDR_MAX = 4095;

const char* WIFI_SSID = "IEEE 802.11XD";
const char* WIFI_PASS = "nfxt4057";

const char* DIODE_ENDPOINT = "http://13.60.41.57:3001/api/db/diodes";
const char* EP_LIGHT       = "http://13.60.41.57:3001/api/db/light-intensity";
const char* EP_TEMP        = "http://13.60.41.57:3001/api/db/temperatures";
const char* EP_HUM         = "http://13.60.41.57:3001/api/db/humidities";
const char* EP_WATER       = "http://13.60.41.57:3001/api/db/water-levels";

// co ile wysyłać metryki (tu: ~5 minut)
const unsigned long SEND_EVERY_MS = 60UL * 1000UL;  // 300 000 ms
unsigned long last_send_ms = 0;

bool last_led_state = false;

// ====== POST helpery ======
bool post_value(const char* url, float value) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http; WiFiClient client;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  String body = String("{\"value\":") + String(value, 2) + "}";
  int code = http.POST(body);
  Serial.printf("POST %s -> %d | %s\n", url, code, body.c_str());
  http.end();
  return code >= 200 && code < 300;
}

bool post_diode_status(bool status) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http; WiFiClient client;
  http.begin(client, DIODE_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  String body = String("{\"status\":") + (status ? "true" : "false") + "}";
  int code = http.POST(body);
  Serial.printf("POST %s -> %d | %s\n", DIODE_ENDPOINT, code, body.c_str());
  http.end();
  return code >= 200 && code < 300;
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Łącze z Wi-Fi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Wi-Fi OK, IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi FAIL — działam offline");
  }
}

// ====== BME280 ======
Adafruit_BME280 bme;
bool bme_ok = false;

// ====== Water Level: drugi I2C (SDA=D23, SCL=D19) ======
TwoWire water_i2c = TwoWire(1);
const uint8_t WL_ADDR_LOW  = 0x77;   // dolne 8 sekcji
const uint8_t WL_ADDR_HIGH = 0x78;   // górne 12 sekcji
const int     WL_THRESHOLD = 100;    // próg detekcji

int read_water_level_percent() {  // 0..100% lub -1
  uint8_t low[8]  = {0};
  uint8_t high[12]= {0};

  water_i2c.requestFrom((int)WL_ADDR_LOW, 8);
  for (int i = 0; i < 8; i++) { if (!water_i2c.available()) return -1; low[i] = water_i2c.read(); }

  water_i2c.requestFrom((int)WL_ADDR_HIGH, 12);
  for (int i = 0; i < 12; i++) { if (!water_i2c.available()) return -1; high[i] = water_i2c.read(); }

  int sections = 0;
  for (int i = 0; i < 20; i++) {
    uint8_t v = (i < 8) ? low[i] : high[i - 8];
    bool wet = (v > WL_THRESHOLD);
    if (wet) sections++; else break;
  }
  return sections * 5;
}

// ====== S T E P P E R  (28BYJ-48 + ULN2003) ======
// Piny 
const int IN1 = 13;  // D13  
const int IN2 = 12;  // D12  
const int IN3 = 14;  // D14
const int IN4 = 27;  // D27

const int stepsPerRevolution = 2048;                       // ok. 2048 kroków/obrót
const int numFields = 9;                                   // 9 pól na tarczy
const int stepsPerField = stepsPerRevolution / numFields;  // ~227 kroków/pole

const int stepDelay = 3;          // ms między krokami
const int emptyHoldDelay = 1000;  // ms postój na PUSTYM (tu: 1 s)

int stepIndex = 0;     // 0..3
int currentField = 0;  // start na polu 0

// 0 = PUSTE, 1 = PEŁNE
int fieldsPattern[numFields] = {
  1, 0, 0,
  1, 0, 0,
  1, 0, 0
};

void stepMotor(int stepIdx) {
  switch (stepIdx) {
    case 0: digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  break;
    case 1: digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  break;
    case 2: digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  break;
    case 3: digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); break;
  }
}

void stepFields(int fieldsToMove) {
  long totalSteps = (long)fieldsToMove * stepsPerField;
  for (long s = 0; s < totalSteps; s++) {
    stepMotor(stepIndex);
    stepIndex = (stepIndex + 1) & 3;  // modulo 4
    delay(stepDelay);
  }
}

void releaseMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Jedna „iteracja” logiki silnika — wywoływana w Twoim loop()
void runStepperOnce() {
  // jeżeli stoimy na PEŁNYM (1), przeskocz do najbliższego PUSTEGO
  if (fieldsPattern[currentField] == 1) {
    int nextField    = (currentField + 1) % numFields;
    int fieldsToMove = 1;
    if (fieldsPattern[nextField] == 1) {        // jeśli kolejne też PEŁNE
      fieldsToMove = 2;
      nextField = (currentField + 2) % numFields;
    }
    stepFields(fieldsToMove);
    currentField = nextField;
  }

  // teraz stoimy na PUSTYM
  releaseMotor();
  delay(emptyHoldDelay);

  // wyznacz następne PUSTE (z pominięciem PEŁNYCH)
  int nextField    = (currentField + 1) % numFields;
  int fieldsToMove = 1;
  if (fieldsPattern[nextField] == 1) {
    fieldsToMove = 2;
    nextField = (currentField + 2) % numFields;
  }
  stepFields(fieldsToMove);
  currentField = nextField;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // piny silnika
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  releaseMotor();
  currentField = 0;

  connectWifi();

  last_send_ms = millis() - SEND_EVERY_MS;  // wyślij natychmiast, potem co ~5 min

  // I2C główne (BME280 na D21/D22)
  Wire.begin(21, 22);
  if (bme.begin(0x77)) {
    bme_ok = true;
    Serial.println("BME280 OK @0x77");
  } else {
    bme_ok = false;
    Serial.println("BME280 NIE WYKRYTY");
  }

  // I2C dla czujnika poziomu wody (D23/D19)
  water_i2c.begin(23, 19);
  delay(50);
}

void loop() {
  // --- LDR ---
  int   light_intensity = analogRead(LDR_PIN);               // 0..4095
  float relative_light_intensity = (light_intensity * 100.0) / LDR_MAX;
  Serial.print("Jasność: "); Serial.print(relative_light_intensity); Serial.println("%");

  // --- LED + POST diody przy zmianie ---
  bool led_state = (light_intensity > 1000);
  digitalWrite(LED_PIN, led_state ? HIGH : LOW);
  if (led_state != last_led_state) {
    if (WiFi.status() != WL_CONNECTED) connectWifi();
    post_diode_status(led_state);
    last_led_state = led_state;
  }

  // --- BME280 ---
  float temperature_C = NAN, relative_humidity = NAN;
  if (bme_ok) {
    temperature_C     = bme.readTemperature();
    relative_humidity = bme.readHumidity();
    Serial.print("Temperatura: "); Serial.print(temperature_C); Serial.println("°C");
    Serial.print("Wilgotność:  "); Serial.print(relative_humidity); Serial.println("%");
  } else {
    Serial.println("Brak danych BME280");
  }

  // --- Poziom wody ---
  int water_level_percent = read_water_level_percent();
  if (water_level_percent >= 0) {
    Serial.print("Poziom wody: "); Serial.print(water_level_percent); Serial.println("%");
  } else {
    Serial.println("Poziom wody: błąd odczytu");
  }

  // === odliczanie do kolejnego POST metryk ===
  unsigned long now = millis();
  unsigned long elapsed = now - last_send_ms;
  unsigned long remaining_ms = (elapsed >= SEND_EVERY_MS) ? 0UL : (SEND_EVERY_MS - elapsed);
  unsigned long remaining_s  = (remaining_ms + 999UL) / 1000UL;

  Serial.print("POST countdown: ");
  Serial.print(remaining_s);
  Serial.println(" s");

  // --- wysyłka metryk ---
  if (elapsed >= SEND_EVERY_MS) {
    if (WiFi.status() != WL_CONNECTED) connectWifi();
    post_value(EP_LIGHT, relative_light_intensity);
    if (bme_ok && !isnan(temperature_C))     post_value(EP_TEMP, temperature_C);
    if (bme_ok && !isnan(relative_humidity)) post_value(EP_HUM,  relative_humidity);
    if (water_level_percent >= 0)            post_value(EP_WATER, water_level_percent);
    last_send_ms = now;
  }

  Serial.println("-----------------------");

  runStepperOnce();  
}







