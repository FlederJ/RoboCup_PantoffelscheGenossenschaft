/** importiert Arduino automatisch, muss man also hier nicht unbedingt auch noch mal importieren: */
#include <Arduino.h>
#include <Servo.h>

/**
 * !!! Immer darauf achten, dass unten in der Statusleiste...
 *     ... das richtige Arduino Board eingestellt ist
 *     ... das richtige "Sketch File" ausgewählt ist (das ändert sich nämlich nicht automatisch)
 *     ... die richtige C/C++ Konfiguration eingestellt ist (sonst gibt es noch mehr "rote swiggels")
 * 
 * :: Externen Servo ansteuern ::
 * 
 * Hardware-Aufbau:
 * Servo:                       Arduino Due / Arduino Nano RP2040 Connect:
 *   Control (meist orange) <-- D11 (irgendein PWM-fähiger Pin)
 *   VCC     (meist rot)    <-- VIN und NUR an USB: 5V -> OK
 *                              !!! NICHT an den Akku anschließen, da die meisten Servos bei mehr als 6V kaputt gehen !!!
 *   GND     (meist braun)  <-- GND
*/

// Objekt, dass den Servo repräsentiert:
Servo ein_servo = Servo();

// Alle verfügbaren Servo-Pins auf dem Rescue Board v2:
const uint8_t SERVO_PIN_LINKS_GANZ_VORNE = D2;
const uint8_t SERVO_PIN_LINKS_VORNE = D3;
const uint8_t SERVO_PIN_RECHTS_GANZ_VORNE = D10;
const uint8_t SERVO_PIN_RECHTS_VORNE = D11;

void setup() {
    // Servo dem Pin zuweisen, mit dem er gesteuert werden soll:
    ein_servo.attach(SERVO_PIN_RECHTS_VORNE);
}

void loop() {
    // Typischerweise haben wir es mit 180°-Servos zu tun.
    //  Der Arm kann also nur zwischen 0° und 180° hin- und herschwenken.
    //  Bei Segel-Winden-Servos (können mehr als 360°) könnte das anders sein...
    ein_servo.write(0); // nach ganz links fahren
    delay(2000);   
    ein_servo.write(90); // in die Mitte fahren
    delay(2000);   
    ein_servo.write(179); // nach ganz rechts fahren
    delay(2000);   
}
