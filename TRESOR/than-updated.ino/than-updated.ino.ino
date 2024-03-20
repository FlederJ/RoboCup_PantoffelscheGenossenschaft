#include <Arduino.h>
#include "ZumoMotors.h"
#include "ZumoReflectanceSensorArray.h"

#include <Wire.h>
//#include <SoftwareWire.h>
#include "Adafruit_TCS34725.h"

//SoftwareWire Wire1(A0, A1);  // Create a software I2C bus on pins A0 (SDA) and A1 (SCL)

Adafruit_TCS34725 colour1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 colour2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);



ZumoMotors motors;
ZumoReflectanceSensorArray reflektionsSensoren;

uint16_t r;
uint16_t r2;
uint16_t g;
uint16_t g2;
uint16_t b;
uint16_t b2;
uint16_t h;
uint16_t h2;
bool wl;
bool wr;
bool bl;
bool br;
bool gl;
bool gr;

int rotationSpeedCalculation(int RSArray[6], int Length, float SensorValue /* Value of the individual sensor*/) {
  int SensorArrayValue = 0;
  float SensorValueDifference = 0.25;  //Gewichtung der Sensoren(Mitte am wenigsten, aussen am meisten
  float RightSensorOffset = Length / 2;
  for (int i = 0; i < Length; i++) {
    if (i < Length / 2 /*falls rechts*/) {
      SensorArrayValue = SensorArrayValue + SensorValue;
      SensorValue = SensorValue - SensorValueDifference;
    } else /*falls links*/ {
      SensorArrayValue = SensorArrayValue - SensorValue;
      SensorValue = SensorValue + SensorValueDifference;
    }
  }
  Serial.print("RotationSpeedCalc: ");
  return SensorArrayValue;
}


int *returnLeflectanceValue(int RSArray[6])  // 0 = weiß, 1 = schwarz
{
  int RSReturnArray[6];
  //Serial.print("\n");//sizeof(RSArray[0]));
  Serial.print("RSReturnArray: ");
  for (int i = 0; i < 6; i++) {

    if (RSArray[i] > 300) {
      int DarkModifier = RSArray[i] - 600;
      if (DarkModifier < 0) {
        DarkModifier = 0;
      }
      RSReturnArray[i] = 1000 + DarkModifier;
      Serial.print(RSReturnArray[i]);
      Serial.print(" ");
    } else if (RSArray[i] <= 300) {
      RSReturnArray[i] = 0;
      Serial.print(RSReturnArray[i]);
      Serial.print(" ");
    }
  }
  Serial.print("\n");
}

//;

// only with test Arduinos, not the self-build one
void turnL() {
  motors.setSpeeds(-1000, 1000);
  delay(1000);
  motors.setSpeeds(0, -0);
}

void turnR() {
  motors.setSpeeds(1000, -1000);
  delay(1000);
  motors.setSpeeds(0, -0);
}

void turn() {
  motors.setSpeeds(1000, -1000);
  delay(2000);
  motors.setSpeeds(0, -0);
}

int soize(bool Array[]) {
  return sizeof(Array) / sizeof(Array[0]);
}

void printArray(bool Array[]) {
  for (int i = 0; i < 5; i++) {
    Serial.print("Array Part ");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(Array[i]);
    Serial.print("\n");
  }
}

void *combineBoolArray(bool Array1[5], bool Array2[5], int size, bool Array[5]) {
  for (int i = 0; i < size; i++) {
    Array[i] = Array1[i] || Array2[i];
    //Serial.print(Array1[i]);Serial.print(Array2[i]);Serial.print(Array1[i] || Array2[i]);
  }
}

void *colourCheck(uint16_t RGBHArray[4], bool left, bool resultArray[6]) {
  int r = RGBHArray[0];
  int g = RGBHArray[1];
  int b = RGBHArray[2];
  int h = RGBHArray[3];
  // bl wl gl br wr gr
  int maxC = max(max(r, g), b);
  if (left && h >= 2000 && h < 7000) {
    if (g == maxC) {
      //Serial.print("Green left");
      resultArray[2] = true;
    }
  } else if (!left && h >= 2300 && h <= 5500) {
    if (g == maxC) {
      //Serial.print("Green right");
      resultArray[5] = true;
    }
  } else if (left && h >= 7000) {
    //Serial.print("White left");
    resultArray[1] = true;
  } else if (!left && h >= 10000) {
    // Serial.print("White right");
    resultArray[4] = true;
  } else if (left && h < 7000) {
    // Serial.print("Black left");
    resultArray[0] = true;
  } else if (!left && h < 10000) {
    // Serial.print("Black right");
    resultArray[3] = true;
  }
  // Serial.print(" ");
}

void driving(bool bl, bool wl, bool gl, bool br, bool wr, bool gr) { /* Die b und w sind noch von colour, aber später mit reflectanceArray machen und daraus die Stärke des "Schwankens" berechnen*/
  bool b = br || bl;
  bool w = wr || wl;  // evtl nen Timer einbauen falls zu lange nur weiß war, wackeln und dann wieder zurück zu linie finden
  bool g = gr || gl;
  if (wl && wr) {
    Serial.print("drive forward");
    Serial.print("\n");
    motors.setSpeeds(200, 200);
  } else if (bl && wr) {
    Serial.print("to left");
    Serial.print("\n");
    motors.setRightSpeed(300);
  } else if (br && wl) {
    Serial.print("to right");
    Serial.print("\n");
    motors.setLeftSpeed(300);
  } else if (br && bl) {
    motors.setSpeeds(-100, -130);  // wackeleffekt
  } else if (gr && DoGreen(1)) {
    Serial.print("turn right");
    Serial.print("\n");
    turnR();
  } else if (gl && DoGreen(0)) {
    Serial.print("turn left");
    Serial.print("\n");
    turnL();
  } else if (gr && gl && DoGreen(2)) {
    Serial.print("turn");
    Serial.print("\n");
    turn();
  }
}

bool DoGreen(int LRB) {
  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t h;
  int count;
  if (LRB == 0) {
    motors.setSpeeds(75, 75);
    bool seeGreen = false;

    while (!seeGreen) {
      colour2.getRawData(&r, &g, &b, &h);
      int maxC = max(max(r, g), b);
      if (h < 7000 && !(maxC == g)) {
        return true;
      }
      Serial.print(count);
      Serial.print("\t");
      if (count >= 350) {
        return false;
      }
      count++;
    }
  } else if (LRB == 1) {
    motors.setSpeeds(75, 75);
    bool seeGreen = false;
    while (!seeGreen) {
      colour1.getRawData(&r, &g, &b, &h);
      int maxC = max(max(r, g), b);
      if (h < 10000 && !(maxC == g)) {
        return true;
      }
      Serial.print(count);
      Serial.print("\t");
      if (count >= 350) {
        return false;
      }
      count++;
    }
  } else if (LRB == 2) {
    return true;
  }
}

/*    Notizen:
   Weiß ist unter 300
   R steht für Reflectance
   S steht für Sensor
   C steht für Color
   die Color-Werte verhalten sich fucking weird
   der Color-Sensor nur zur Überprüfung falls man unsicher ist
   Blibliothek ist kaputt, rechter Sensor funktioniert auf dem Roboter nur bei anderer Bibliothek
   unterschied bei funnktionierenden Color Sensoren auch nur reflectance (Reflectance = R + G + B)
   Verhältnis von Grün nicht sehr anders als bei Weiß, Weiß ist sogar manchmal "grüner"
   Frage: nutzen wir nur die Reflectance Dinger? Wie sieht grün auf dem RSArray aus? Was wäre einfacher, alles auf dem RSA oder getrennt Schwarz/Weiß auf RSA und Grün auf dem Color Sensor?
   Tom Fragen nach der besseren Bibliothek-Version
*/

/*
   wenn schwarz:
   Wert auf 1000 setzen (1000 entspricht einem Multiplikator von 1
   alles, was höher ist als Schwellenwert für schwarz wird zu 1000 addiert
   wenn weiß:
   wert auf 0 setzen und dann alles, was niedriger ist als schwellenwert von 0 subtrahieren (ohne oder mit modifikator)
*/

void setup() {
  Serial.begin(9600);

  Wire.begin();  // Initialize the first I2C bus

  // Initialize the second I2C bus (Wire1)
  Wire1.begin();  // Use the appropriate pins for SDA (A0) and SCL (A1)
  delay(10000);
  Serial.println("Hello World");
  // Initialize the first color sensor with Wire (the default bus)
  if (!colour1.begin()) {
    delay(10000);
    Serial.println("RGB Farbsensor 1 Verdrahtung prüfen! Programm Ende.");
    while (1);
  }
  Serial.println("Sensor 1 gefunden...");
  // Initialize the second color sensor with Wire1 (the second bus)
  if (!colour2.begin()) {
    delay(10000);
    Serial.println("RGB Farbsensor 2 Verdrahtung prüfen! Programm Ende.");
    while (1);
  }
  Serial.println("Sensor 2 gefunden...");

  // ES FUNKTIONIERT KP WIEEEEEEEEEEEEEEEEEEEEEEEE nur mit Due Programming Port 


  motors.flipLeftMotor(false);
  motors.flipRightMotor(true);
  reflektionsSensoren.init();
  //WICHTIG! falls tof-sensor: tof.setAddress(0x30); // Adresse des Sensors ändern, sodass keine Interference entsteht
  
  delay(1000);
}

void resetValues() {
  bool br = false;
  bool bl = false;
  bool wr = false;
  bool wl = false;
  bool gr = false;
  bool gl = false;
}

void loop() {
  //resetValues();
  /*int RValues[6] = {100, 100, 100, 100, 100, 100};
  reflektionsSensoren.read(RValues);
  int LLSensor = RValues[0];
  int LMSensor = RValues[1];
  int LRSensor = RValues[2];
  int RLSensor = RValues[3];
  int RMSensor = RValues[4];
  int RRSensor = RValues[5];
  Serial.print("Links: "); Serial.print(LLSensor) ; Serial.print("\t");
  Serial.print(LMSensor) ; Serial.print("\t");
  Serial.print(LRSensor) ; Serial.print("\t");
  Serial.print("Rechts: ") ; Serial.print( RLSensor) ; Serial.print( "\t");
  Serial.print(RMSensor) ; Serial.print( "\t");
  Serial.print(RRSensor) ; Serial.print( "\n");*/

  /*returnLeflectanceValue(RValues);
  Serial.print(rotationSpeedCalculation(returnLeflectanceValue(RValues), 6, 1.5));
  Serial.print("\n");*/
  delay(10);

  // Sensor 1: lesen starten
  colour1.setInterrupt(false);
  // nach 50 ms haben wir Werte; lieber 60ms warten, um sicher zu sein
  delay(60);
  // ein Array für die Werte der Farben und SW des Farbsensors
  bool resultArray[6] = { false, false, false, false, false, false };
  // Werte in unseren Variablen speichern:
  colour1.getRawData(&r, &g, &b, &h);
  // Sensor 1: lesen beenden
  colour1.setInterrupt(true);
  uint16_t RGBH1Array[4] = { r, g, b, h };
  // Werte ausgeben
  /*Serial.print(" rechts: ");
  Serial.print(" R: "); Serial.print(r);
  Serial.print(" G: "); Serial.print(g);
  Serial.print(" B: "); Serial.print(b);
  Serial.print(" H: "); Serial.print(h); //weiß ist ca. 19150 ; schwarz ist ca. 1650 ; grün 2500-5300
  colourCheck(RGBH1Array,false,resultArray);
  Serial.print("\t");*/
  // Sensor 2: lesen starten
  colour2.setInterrupt(false);
  // nach 50 ms haben wir Werte; lieber 60ms warten, um sicher zu sein  2000, 4000, 2000
  delay(60);
  // Werte in unseren Variablen speichern:
  colour2.getRawData(&r2, &g2, &b2, &h2);
  // Sensor 2: lesen beenden
  colour2.setInterrupt(true);
  uint16_t RGBH2Array[4] = { r2, g2, b2, h2 };
  // Werte ausgeben
  /*Serial.print(",\tlinks: ");
  Serial.print(" R: "); Serial.print(r2);
  Serial.print(" G: "); Serial.print(g2);
  Serial.print(" B: "); Serial.print(b2);
  Serial.print(" H: "); Serial.print(h2); //weiß ist ca. 14000 ; schwarz ist ca 1500 ; grün 2500-3950
  colourCheck(RGBH2Array,true,resultArray);
  Serial.print("\t");
  Serial.print("\n"); */
  // colourCheck ist: bl wl gl br wr gr
  colourCheck(RGBH1Array, false, resultArray);
  colourCheck(RGBH2Array, true, resultArray);
  //bool colourArray[5];
  //combineBoolArray(colourCheck((RGBH1Array),true,),colourCheck(RGBH2Array,false),3,colourArray); maybe not needed anymore
  //Serial.print(colourArray[0]);Serial.print(colourArray[2]);Serial.print(colourArray[5]);
  for (int i = 0; i < 6; i++) {
    if (resultArray[i]) {
      Serial.print('1');
      Serial.print(' ');
    } else if (!resultArray[i]) {
      Serial.print('0');
      Serial.print(' ');
    }
  }
  Serial.print("\n");
  // driving(resultArray[0],resultArray[1],resultArray[2],resultArray[3],resultArray[4],resultArray[5]);
  motors.setSpeeds(50, 50);
}
