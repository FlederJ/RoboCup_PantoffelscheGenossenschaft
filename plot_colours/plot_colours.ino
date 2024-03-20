#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 colour1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);
Adafruit_TCS34725 colour2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);


uint16_t r, r2, g, g2, b, b2, h, h2;

void setup() {
  Serial.begin(115200);
  Serial.println("Hello World!");

  Wire.begin();
  Wire.setClock(1000000);

  if (!colour1.begin(TCS34725_ADDRESS, &Wire) || !colour2.begin(TCS34725_ADDRESS, &Wire))
  {
    Serial.println("RGB Farbsensor Verdrahtung prüfen!");
    while (1);
  }
}

void loop() {
  delay(10);

  // Sensor 1: lesen starten
  colour1.setInterrupt(false);
  delay(60);
  colour1.getRawData(&r, &g, &b, &h);
  colour1.setInterrupt(true);

  // Sensor 2: lesen starten
  colour2.setInterrupt(false);
  delay(60);
  colour2.getRawData(&r2, &g2, &b2, &h2);
  colour2.setInterrupt(true);

  // Send sensor data over serial
  Serial.print(r);
  Serial.print(",");
  Serial.print(g);
  Serial.print(",");
  Serial.print(b);
  Serial.print(",");
  Serial.print(h);
  Serial.print(",");

  Serial.print(r2);
  Serial.print(",");
  Serial.print(g2);
  Serial.print(",");
  Serial.print(b2);
  Serial.print(",");
  Serial.println(h2);
}
