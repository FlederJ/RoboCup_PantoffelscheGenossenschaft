#include <Arduino.h>
#include <Wire.h>
#include <RescueBoardMotors.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <QTRSensors.h>
#include <WiFiNINA.h>

Adafruit_TCS34725 colour1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);
Adafruit_TCS34725 colour2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);

// tof deklarieren
VL53L0X abstandsSensor = VL53L0X();
const uint8_t NEW_TOF_ADDRESS = 0x30;

#define PIN_SCHIEBESCHALTER D12 // Schieberegler für allgemeine Steuerung
#define PIN_KALIBRIEREN A6

RescueBoardMotors motors = RescueBoardMotors();
QTRSensors reflektionsSensoren = QTRSensors(); 

const uint8_t num_reflectance_sensors = 6;
const uint8_t SENSOR_LEISTE_PINS[] = {D6, D0, D1, D7, D8 ,D9};
uint16_t RValues[num_reflectance_sensors] = {};

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

int white_threshold = 800;

int rotationSpeedCalculation(int RSArray[num_reflectance_sensors]) {
  
  // Output: von -1 bis 1 (links ist -, rechts ist +, 0 ist geradeaus)
  // Array analysiert: Wenn schwarz mitte -> keine Änderung, schwarz außen -> mehr Änderung, schwarz GANZ außen -> viel änderung
  int averageRotationSpeed = 0;
  for (int i = 0; i < num_reflectance_sensors; i++) {
    int rotationSpeed = 0;
    int location = 0;
    if (i < 3) location = i - 2;
    else location = i - 3;
    float temp_value = ((float)((int)abs(i - 2.5)))/4; // -0.5 to 0.5

    rotationSpeed = (int)abs((temp_value * RSArray[i])) * ( i < 3 ? -1 : 1);

    // Serial.println(String(i) + "-> " + location + " " + temp_value + " " + String(RSArray[i]) + "-> " + rotationSpeed);

    averageRotationSpeed += rotationSpeed;
  }
  return averageRotationSpeed;
  // "Fehler", den ich entdeckt habe:
  /*
  wenn z.B. ganze linke Seite schwarz ist(was eigentlich nicht der Fall sein sollte außer bei Abbiegen oder ungünstiger Linie) biegt er scharf links ab, aber das muss 
  auf den Fall von "GRÜN" abgestimmt werden
  TODO
  */
  // maayyyyybe: nach kalibrieren sehr viel genauer drehen, also wenn bisschen links (z.B. 1300 vs 1200 an Mitte) auch wirklich nach links drehen
  // dann auch langsamer fahren
}

void returnReflectanceValue(uint16_t RSArray[num_reflectance_sensors], int RSReturnArray[num_reflectance_sensors])  // 0 = weiß, 1000 = schwarz
{
  Serial.print("RSReturnArray: ");
  for (int i = 0; i < 6; i++) {
    // float temp_value = (abs(i - 2.5))/2;  /* this was necessary due to too much height and no Akku attached. No longer needed) */
    int temporary_array_part = (int)RSArray[i]; // - temp_value * 600; /*same thing here*/
    Serial.print(String(temporary_array_part) + ", ");
    if (temporary_array_part > white_threshold /*TODO calibrate*/) {
      int DarkModifier = temporary_array_part - 2500;
      RSReturnArray[i] = 1000 + DarkModifier;
      Serial.print(RSReturnArray[i]);
      Serial.print(" ");
    } else if (temporary_array_part <= white_threshold) {
      RSReturnArray[i] = 0;
      Serial.print(RSReturnArray[i]);
      Serial.print(" ");
    }
  }
  Serial.print("\n");
  
}

void calibrate(uint16_t calibrated_values[num_reflectance_sensors * 2]) {
  uint16_t calibration_array[num_reflectance_sensors];
  uint16_t total_array[num_reflectance_sensors];
  int total_iter = 20;
  Serial.print("\nWHITE:\n");
  for (int i = 0; i < total_iter; i++) {
    reflektionsSensoren.read(calibration_array);
    for (int j = 0; j < num_reflectance_sensors; j++) {
      total_array[j] += calibration_array[j];
    }
    delay(500);
    //motors.setSpeeds(50 * (i % 5 == 0 ? 0 : (i % 2 == 0 ? 1 : -1)), 50 * (i % 5 == 0 ? 0 : (i % 2 == 0 ? 1 : -1)));
  }
  Serial.print("Calibrated: ");
  for (int i = 0; i < num_reflectance_sensors; i++) {
    calibrated_values[i] = total_array[i] / total_iter;
    Serial.print(String(calibrated_values[i]) + " ");
  }
  Serial.print("\n");
  
  // collect data for 5 seconds
  // take average for each slot
  // repeat for black


}

//;

/*
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
*/

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

bool isGreen(uint16_t RGBHArray[4], bool left) {
  int r = RGBHArray[0];
  int g = RGBHArray[1];
  int b = RGBHArray[2];
  int h = RGBHArray[3];
  // 
  int maxColor = max(max(r, g), b);
  int difference_r = g - r;
  int difference_b = g - b;
  int minBrightness = 5000;
  int maxBrightness = 19000; // just some big value which probably is never going to be white
  int differencethreshold = 400; 
  /*
  ONLY USING TO CHECK FOR GREEN:
  new code:
  here:
  Sensor 1: 275 335 271 913
  Sensor 2: 195 281 227 668
  OLD

  */
  Serial.print(String(r)+" ");Serial.print(String(g) + " ");Serial.print(String(b) + " ");
  Serial.print("Max Color: " + String(maxColor)+ " Brightness: ");Serial.print(String(h) + " diff r: ");Serial.print(String(difference_r) + " diff b: ");Serial.print(String(difference_b) + " ");
  
  if (maxColor == g && h <= maxBrightness && h >= minBrightness && difference_r >= differencethreshold && difference_b >= differencethreshold) {
    return true;
  }
  return false;
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
  //Serial.print("\nl:" + String(leftSpeed) + " r:" + String(rightSpeed) + "\n");
  // Set motor speeds
  motors.setSpeeds(leftSpeed, rightSpeed);
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
      int maxColor = max(max(r, g), b);
      if (h < 7000 && !(maxColor == g)) {
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
      int maxColor = max(max(r, g), b);
      if (h < 10000 && !(maxColor == g)) {
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
  pinMode(PIN_KALIBRIEREN, INPUT);   
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
  reflektionsSensoren.setSensorPins(SENSOR_LEISTE_PINS, num_reflectance_sensors);


  /*if (!reflektionsSensoren.init()) { // klappt nicht weil init kein bool sondern void macht
        delay(10000); // damit wir Zeit haben den Serial Monitor zu öffnen nach dem Upload
        Serial.println("Reflektionssensoren Verdrahtung prüfen!"); 
  }*/





  delay(1000);
}


void loop() {
  if (digitalRead(PIN_KALIBRIEREN)) {
    Serial.print("Calibrate\n");
    uint16_t calibrated_values[num_reflectance_sensors * 2];
    calibrate(calibrated_values);
    Serial.print("calibrated: ");
    for (int i = 0; i < num_reflectance_sensors; i++) {
      Serial.print(String(calibrated_values[i]) + " ");
    }
     Serial.print("\n");
  }


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
  
  int RS_Result_Array[num_reflectance_sensors] = {};
  returnReflectanceValue(RValues, RS_Result_Array);
  int rotation = rotationSpeedCalculation(RS_Result_Array);
  Serial.print("rotation-speed: " + String(rotation) + "\t");
  if (digitalRead(PIN_SCHIEBESCHALTER)) {
    setMotorSpeeds(rotation);
  } else {
    motors.setSpeeds(0,0); // TODO look if speeds are 0 and if not do 0
  }
  
  delay(10);
  // Sensor 1: lesen starten
  colour1.setInterrupt(false);
  // nach 50 ms haben wir Werte; lieber 60ms warten, um sicher zu sein
  delay(60);

  // ein Array für die Werte der Farben und SW des Farbsensors
  // bool resultArray[num_reflectance_sensors] = { false, false, false, false, false, false };

  bool isGreenLeft = false;
  bool isGreenRight = false;
  Serial.println("\n");
  // Werte in unseren Variablen speichern:
  colour1.getRawData(&r, &g, &b, &h);
  // Sensor 1: lesen beenden
  colour1.setInterrupt(true);
  uint16_t RGBH1Array[4] = { r, g, b, h };
  /*
  // Werte ausgeben
  Serial.print("RGB rechts: ");
  Serial.print(" R: " + String(r));
  Serial.print(" G: " + String(g));
  Serial.print(" B: " + String(b));
  Serial.print(" H: " + String(h)); //weiß ist ca. 19150 ; schwarz ist ca. 1650 ; grün 2500-5300
  */
  isGreenRight = isGreen(RGBH1Array,false);  
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
  /*
  Serial.print(", links: ");
  Serial.print(" R: " + String(r2));
  Serial.print(" G: " + String(g2));
  Serial.print(" B: " + String(b2));
  Serial.print(" H: " + String(h2)); //weiß ist ca. 14000 ; schwarz ist ca 1500 ; grün 2500-3950
  */
  isGreenLeft = isGreen(RGBH2Array,true);  
  Serial.print("\t");

  Serial.println("\nLeft: " + String(((isGreenLeft) ? "True" : "False")) + "  Right: " + String(((isGreenRight) ? "True" : "False")));


  delay(60);
  distance = abstandsSensor.readRangeContinuousMillimeters();
  Serial.print("Distance: " + String(distance) + "\t");

  Serial.print("\n"); 

  
}
