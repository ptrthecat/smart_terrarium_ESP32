#include <Wire.h>
#include <Adafruit_BME280.h>

#define LED_PIN 2    
#define LDR_PIN 4    // czujnik światła podłączony do D4 

// maksymalna wartość przy 3.3V
const int LDR_MAX = 4095;

Adafruit_BME280 bme; 

bool bme_ok = false;

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  Wire.begin(21, 22);

  if (bme.begin(0x77)) {
    bme_ok = true;
    Serial.println("BME280 OK @0x77");
  } else {
    bme_ok = false;
    Serial.println("BME280 NIE WYKRYTY");
  }
}

void loop() {

  int light_intensity = analogRead(LDR_PIN); // 0 - 4095
  float relative_light_intensity = (light_intensity * 100.0) / LDR_MAX; // 0..100 %
  Serial.print("Jasność: ");
  Serial.print(relative_light_intensity);
  Serial.println("%");

  if (light_intensity > 1000) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  if (bme_ok) {
    float temperature_C = bme.readTemperature();    
    float relative_humidity = bme.readHumidity();        

    Serial.print("Temperatura: ");
    Serial.print(temperature_C);
    Serial.println("°C");

    Serial.print("Wilgotność:  ");
    Serial.print(relative_humidity);
    Serial.println("%");
  } else {
    Serial.println("Brak danych BME280");
  }

  Serial.println("-----------------------");
  delay(500);
}
