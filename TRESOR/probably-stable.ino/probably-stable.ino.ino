#include <Arduino.h>
#include <Wire.h>
#include <RescueBoardMotors.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <QTRSensors.h>

Adafruit_TCS34725 colour1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);
Adafruit_TCS34725 colour2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);

// tof deklarieren
VL53L0X abstandsSensor = VL53L0X();
const uint8_t NEW_TOF_ADDRESS = 0x30;

#define PIN_SCHIEBESCHALTER D12 // Schieberegler für allgemeine Steuerung

RescueBoardMotors motors = RescueBoardMotors();
QTRSensors reflektionsSensoren = QTRSensors(); 

const uint8_t SENSOR_LEISTE_ANZAHL_SENSOREN = 6;
const uint8_t SENSOR_LEISTE_PINS[] = {D6, D0, D1, D7, D8 ,D9};
uint16_t RValues[SENSOR_LEISTE_ANZAHL_SENSOREN] = {};

uint16_t distance;
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



int rotationSpeedCalculation(int RSArray[SENSOR_LEISTE_ANZAHL_SENSOREN]) {
  /* Och Ludwig das ergibt keinen Sinn
  int SensorArrayValue = 0;
  float SensorValueDifference = 0.25;  //Gewichtung der Sensoren(Mitte am wenigsten, aussen am meisten
  float RightSensorOffset = Length / 2;
  for (int i = 0; i < Length; i++) {
    if (i < Length / 2 /falls rechts/) {
      SensorArrayValue = SensorArrayValue + SensorValue;
      SensorValue = SensorValue - SensorValueDifference;
    } else /falls links/ {
      SensorArrayValue = SensorArrayValue - SensorValue;
      SensorValue = SensorValue + SensorValueDifference;
    }
    Serial.print("\t" + String(SensorValue));
  }
  Serial.print("RotationSpeedCalc: ");
  return SensorArrayValue;
  */ 
  // Output: von -1 bis 1 (links ist -, rechts ist +, 0 ist geradeaus)
  // Array analysiert: Wenn schwarz mitte -> keine Änderung, schwarz außen -> mehr Änderung, schwarz GANZ außen -> viel änderung
  int averageRotationSpeed = 0;
  for (int i = 0; i < SENSOR_LEISTE_ANZAHL_SENSOREN; i++) {
    int rotationSpeed = 0;
    int location = 0;
    if (i < 3) location = i - 2;
    else location = i - 3;
    float temp_value = ((float)((int)abs(i - 2.5)))/4;

    rotationSpeed = (int)(temp_value * RSArray[i]) * location;

    Serial.println(String(i) + "-> " + location + " " + temp_value + " " + String(RSArray[i]) + "-> " + rotationSpeed);

    averageRotationSpeed += rotationSpeed;
  }
  return averageRotationSpeed;
}

void returnReflectanceValue(uint16_t RSArray[SENSOR_LEISTE_ANZAHL_SENSOREN], int RSReturnArray[SENSOR_LEISTE_ANZAHL_SENSOREN])  // 0 = weiß, 1000 = schwarz
{
  Serial.print("RSReturnArray: ");
  for (int i = 0; i < 6; i++) {
    float temp_value = (abs(i - 2.5))/2; 
    int temporary_array_part = (int)RSArray[i] - temp_value * 600;
    Serial.print(String(temporary_array_part) + ", ");
    if (temporary_array_part > 1500) {
      int DarkModifier = temporary_array_part - 2500;
      RSReturnArray[i] = 1000 + DarkModifier;
      Serial.print(RSReturnArray[i]);
      Serial.print(" ");
    } else if (temporary_array_part <= 1500) {
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

int bool_array_size(bool array[]) {
  return sizeof(array) / sizeof(array[0]);
}

void printArray(bool array[]) {
  for (int i = 0; i < 5; i++) {
    Serial.print("Array Part ");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(array[i]);
    Serial.print("\n");
  }
}

void *combineBoolArray(bool array1[5], bool array2[5], int size, bool result_array[5]) {
  for (int i = 0; i < size; i++) {
    result_array[i] = array1[i] || array2[i];
    //Serial.print(Array1[i]);Serial.print(Array2[i]);Serial.print(Array1[i] || Array2[i]);
  }
  return 0; // why, idk it gave errors
}

void *colourCheck(uint16_t RGBHArray[4], bool left, bool resultArray[SENSOR_LEISTE_ANZAHL_SENSOREN]) {
  int r = RGBHArray[0];
  int g = RGBHArray[1];
  int b = RGBHArray[2];
  int h = RGBHArray[3];
  // bl wl gl br wr gr
  int maxC = max(max(r, g), b);
  /*
  ONLY USING TO CHECK FOR GREEN:
  
  new code:
  // TODO
  
  */
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
  return 0; // idk still errors if not
}

/*
void driving() { /Die b und w sind noch von colour, aber später mit reflectanceArray machen und daraus die Stärke des "Schwankens" berechnen
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
*/

void setMotorSpeeds(int rotation) {
  const int baseSpeed = 50;  // Base speed for both motors
  const float rotationScale = 0.5;  // Scaling factor for rotation to motor speed conversion

  // Calculate motor speeds
  float leftSpeed = baseSpeed + (rotation * rotationScale);
  float rightSpeed = baseSpeed - (rotation * rotationScale);
  Serial.print("\nl:" + String(leftSpeed) + " r:" + String(rightSpeed) + "\n");
  // Set motor speeds
  motors.setSpeeds(leftSpeed, rightSpeed);
}


void drive(int rotation) {
  int cleaned_rotation = (int)(rotation/20);
  Serial.println("cleaned rotation: " + String(cleaned_rotation));
  if (cleaned_rotation < 0) {
    motors.setLeftSpeed(20 + cleaned_rotation);
  }
  else {
    motors.setRightSpeed(20 + cleaned_rotation);
  }
}


bool DoGreen(int LRB) { // TODO calibrate and stuff
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
  } 
  else if (LRB == 1) {
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

  return false;
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
  Serial.begin(115200);
  delay(3000);
  Serial.println("Hello World!");

  pinMode(PIN_SCHIEBESCHALTER, INPUT_PULLDOWN); // LOW, wenn nicht gedrückt

  // I2C Bus 1x für alle Bus-Teilnehmer initialisieren (sonst crasht das Betriebssystem)

  Wire.begin();           // Bus I2C0
  Wire.setClock(1000000); // 1MHz Kommunikationsgeschwindigkeit
  if (!colour1.begin(TCS34725_ADDRESS, &Wire))
  {
    delay(10000); // damit wir Zeit haben den Serial Monitor zu öffnen nach dem Upload
    Serial.println("RGB Farbsensor 1 Verdrahtung prüfen!");
  } else {
    Serial.println("Sensor 1 läuft");
  }

  // I2C geht jetzt, hier zweiten Bus deklarieren

  Wire1.begin(); // Bus I2C1
  Wire1.setClock(1000000); // 1MHz Kommunikationsgeschwindigkeit

  if (!colour2.begin(TCS34725_ADDRESS, &Wire1))
  {
    delay(10000); // damit wir Zeit haben den Serial Monitor zu öffnen nach dem Upload
    Serial.println("RGB Farbsensor 2 Verdrahtung prüfen!");
  } else {
    Serial.println("Sensor 2 läuft");
  }

  /* I2C-Devices:
  0x29: Farbsensor 1 ( & 2 auf Wire1)
  0x30: Abstandssensor
  0x60: anderer Sensor, auf Board festgelötet
  0x6A: 6-Achsen-Bewegungssensor auf Board festgelötet
  */

  motors.initialize();
  motors.flipLeftMotor(false);
  motors.flipRightMotor(true);


  abstandsSensor.setBus(&Wire);
  abstandsSensor.setAddress(NEW_TOF_ADDRESS);
  if (!abstandsSensor.init()) {
      delay(10000); // damit wir Zeit haben den Serial Monitor zu öffnen nach dem Upload
      Serial.println("ToF Verdrahtung prüfen! Roboter aus- und einschalten! Programm Ende.");
      while (1);                                                                                                                                                                                                                                                                                                                                                                                                                                                        
  }
  // Einstellung: Fehler, wenn der Sensor länger als 1000ms lang nicht reagiert
  abstandsSensor.setTimeout(1000);
  // Reichweiter vergrößern (macht den Sensor ungenauer)
  abstandsSensor.setSignalRateLimit(0.1);
  abstandsSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  abstandsSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  // lasse Sensor die ganze Zeit an
  abstandsSensor.startContinuous();

  reflektionsSensoren.setTypeRC();
  reflektionsSensoren.setSensorPins(SENSOR_LEISTE_PINS, SENSOR_LEISTE_ANZAHL_SENSOREN);


  /*if (!reflektionsSensoren.init()) { // klappt nicht weil init kein bool sondern void macht
        delay(10000); // damit wir Zeit haben den Serial Monitor zu öffnen nach dem Upload
        Serial.println("Reflektionssensoren Verdrahtung prüfen!"); 
  }*/





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
  reflektionsSensoren.read(RValues); // averages ohne Akku:
  
  int LLSensor = RValues[0]; // average white value: ~ 1700 +- 50
  int LMSensor = RValues[1]; // white: ~ 1409 +- 50 (eher weniger)
  int LRSensor = RValues[2]; // white: ~ 1121 +- 50
  int RLSensor = RValues[3]; // white: ~ 1164 +- 50
  int RMSensor = RValues[4]; // white: ~ 1423 +- 50
  int RRSensor = RValues[5]; // white: ~ 1884 +- 50
  Serial.print("RValues\tLinks: " + String(LLSensor) + " ");
  Serial.print(String(LMSensor) + " ");
  Serial.print(String(LRSensor) + " ");
  Serial.print("Rechts: " + String(RLSensor) + " ");
  Serial.print(String(RMSensor) + " ");
  Serial.print(String(RRSensor) + "\t");
  
  int RS_Result_Array[SENSOR_LEISTE_ANZAHL_SENSOREN] = {};
  returnReflectanceValue(RValues, RS_Result_Array);
  int rotation = rotationSpeedCalculation(RS_Result_Array);
  Serial.print("rotation-speed: " + String(rotation) + "\t");
  if (digitalRead(PIN_SCHIEBESCHALTER)) {
    setMotorSpeeds(rotation);
  } else {
    motors.setSpeeds(0,0);
  }
  
  delay(10);
  // Sensor 1: lesen starten
  colour1.setInterrupt(false);
  // nach 50 ms haben wir Werte; lieber 60ms warten, um sicher zu sein
  delay(60);
  // ein Array für die Werte der Farben und SW des Farbsensors
  bool resultArray[SENSOR_LEISTE_ANZAHL_SENSOREN] = { false, false, false, false, false, false };
  // Werte in unseren Variablen speichern:
  colour1.getRawData(&r, &g, &b, &h);
  // Sensor 1: lesen beenden
  colour1.setInterrupt(true);
  uint16_t RGBH1Array[4] = { r, g, b, h };
  // Werte ausgeben
  Serial.print("RGB rechts: ");
  Serial.print(" R: " + String(r));
  Serial.print(" G: " + String(g));
  Serial.print(" B: " + String(b));
  Serial.print(" H: " + String(h)); //weiß ist ca. 19150 ; schwarz ist ca. 1650 ; grün 2500-5300
  colourCheck(RGBH1Array,false,resultArray);  
  Serial.print("  ");
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
  
  Serial.print(", links: ");
  Serial.print(" R: " + String(r2));
  Serial.print(" G: " + String(g2));
  Serial.print(" B: " + String(b2));
  Serial.print(" H: " + String(h2)); //weiß ist ca. 14000 ; schwarz ist ca 1500 ; grün 2500-3950
  colourCheck(RGBH2Array,true,resultArray);
  Serial.print("\t");

  
  // colourCheck ist: bl wl gl br wr gr
  colourCheck(RGBH1Array, false, resultArray);
  colourCheck(RGBH2Array, true, resultArray);
  //bool colourArray[5];
  //combineBoolArray(colourCheck((RGBH1Array),true,),colourCheck(RGBH2Array,false),3,colourArray); maybe not needed anymore
  //Serial.print(colourArray[0]);Serial.print(colourArray[2]);Serial.print(colourArray[5]);
  /*
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
  */
  delay(60);
  distance = abstandsSensor.readRangeContinuousMillimeters();
  Serial.print("Distance: " + String(distance) + "\t");

  Serial.print("\n"); 

  


  // driving(resultArray[0],resultArray[1],resultArray[2],resultArray[3],resultArray[4],resultArray[5]); OLD
  
}
