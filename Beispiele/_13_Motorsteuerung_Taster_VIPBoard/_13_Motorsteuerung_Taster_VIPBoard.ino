/** importiert Arduino automatisch, muss man also hier nicht unbedingt auch noch mal importieren: */
#include <Arduino.h>

/**
 * !!! Immer darauf achten, dass unten in der Statusleiste...
 *     ... das richtige Arduino Board eingestellt ist
 *     ... das richtige "Sketch File" ausgewählt ist (das ändert sich nämlich nicht automatisch)
 *     ... die richtige C/C++ Konfiguration eingestellt ist (sonst gibt es noch mehr "rote swiggels")
 * 
 * :: Motoren des Rescue Board v2 ansteuern ::
 * 
 * Hardware-Aufbau:
 * Taster:                  Arduino Nano RP2040 Connect:
 *  Pin1                <-- GND (LOW, wenn gedrückt)
 *  Pin2                <-- D17
 * Motorsteuerung links:    Motor oder Arduino Nano RP2040 Connect:
 *  VIN                 <-- VIN (Auf dem Steckbrett 5V USB-Spannung, auf dem echten Roboter 7,4V Akkuspannung.)
 *  VCC                 <-- 3,3V
 *  VMM                 --> nicht benutzen (Motorspannung VIN verpolunggeschützter Ausgang zur Versorgung von anderen Sachen)
 *  GND                 <-- GND
 *  GND                 <-- GND
 *  MODE                <-- 3,3V via 10k
 *  AENABLE             <-- D4 (PWM: Geschwindigkeit Motor 1)
 *  APHASE              <-- D5 (Richtung Motor 1)
 *  AOUT1               --> Motor 1
 *  AOUT2               --> Motor 1
 *  BENABLE             <-- D11 (PWM: Geschwindigkeit Motor 2)
 *  BPHASE              <-- D10 (Richtung Motor 2)
 *  BOUT1               --> Motor 2
 *  BOUT2               --> Motor 2
 * Motorsteuerung rechts:   Motor oder Arduino Nano RP2040 Connect:
 *  VIN                 <-- VIN (Auf dem Steckbrett 5V USB-Spannung, auf dem echten Roboter 7,4V Akkuspannung.)
 *  VCC                 <-- 3,3V
 *  VMM                 --> nicht benutzen (Motorspannung VIN verpolunggeschützter Ausgang zur Versorgung von anderen Sachen)
 *  GND                 <-- GND
 *  GND                 <-- GND
 *  MODE                <-- 3,3V via 10k
 *  AENABLE             <-- D6 (PWM: Geschwindigkeit Motor 3)
 *  APHASE              <-- D7 (Richtung Motor 3)
 *  AOUT1               --> Motor 3
 *  AOUT2               --> Motor 3
 *  BENABLE             <-- D9 (PWM: Geschwindigkeit Motor 4)
 *  BPHASE              <-- D8 (Richtung Motor 4)
 *  BOUT1               --> Motor 4
 *  BOUT2               --> Motor 4
*/

#include <VIPBoardMotors.h>
// Dieses Objekt repräsentiert 4 Motor-Kanäle (1 Motor pro Kanal):
VIPBoardMotors motors = VIPBoardMotors();

const uint8_t PIN_TASTER = D17;

void setup() {
  // Taster initialisieren:
  pinMode(PIN_TASTER, INPUT_PULLUP); // HIGH, wenn nicht gedrückt

  motors.initialize();
  // falls man global die Motor-Drehrichtung ändern möchte:
  motors.flipLeftFrontMotor(false); // nur notwendig, wenn man true reinschreibt
  motors.flipLeftRearMotor(false); // nur notwendig, wenn man true reinschreibt
  motors.flipRightFrontMotor(false); // nur notwendig, wenn man true reinschreibt
  motors.flipRightRearMotor(false); // nur notwendig, wenn man true reinschreibt

  // !!! ACHTUNG !!!
  //  Es ist besser ein Programm mit großem Stromverbrauch (wie die Motoren)
  //  nicht automatisch loslaufen zu lassen, um das Flashen (Programm hochladen) nicht zu stören
  //  und den USB zu schützen.
  //  1. Wollen wir nicht, dass der Roboter einfach losfährt, wenn wir in Ruhe flashen wollen.
  //  2. Wollen die Motoren viel Strom. Wenn nur der USB und nicht zusätzlich der Akku angeschlossen
  //     sind, kann der USB durch den hohen Stromverbrauch gestört oder sogar permanent geschädigt
  //     werden.
  //  Aus diesem Grund läuft dieses Programm erst los, wenn ein Taster gedrück wird:
  //  START: Taster hinten (beim USB) drücken.
  //  STOP:  Weißen kleinen RESET-Taster auf dem Arduino drücken.
  //         Damit wird das Programm neu gestarted und wir bleiben wieder hier hängen.
  while (digitalRead(PIN_TASTER)); // Endlosschleife, solange Pin HIGH (LOW=gedrückt)
}

// endlos wiederholen:
void loop() {                  // links          / rechts
  motors.setSpeeds(-255, 255); // 100% rückwärts / 100% vorwärts
  delay(1000); // für 1 Sekunde
  motors.setLeftSpeed(0);      //   0%           / 100% vorwärts
  delay(1000); // für 1 Sekunde
  motors.setRightSpeed(0);     //   0%           /   0%
  delay(1000); // für 1 Sekunde
  motors.setSpeeds(255, -255); // 100% vorwärts  / 100% rückwärts
  delay(1000); // für 1 Sekunde
}
