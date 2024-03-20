/**
 * https://www.philohome.com/wedo2reverse/connect.htm
 * 
 * blau:    6   M1      Motor PWM (VBAT)
 * grün:    5   M2      Motor PWM (VBAT)    VIN (wenn ID1 über 2k2 nach GND verbunden, aber dann kann man über ID1/2 nicht mehr kommunizieren)
 * gelb:    4   GND     GND                 GND
 * orange:  3   VCC     3,3V                VIN via StepUp (Boost-Converter) + Kondensatoren -> 5V
 * rot:     2   ID1     hub(TX)->device(RX) D0  (2k2 -> GND, um ID2 anzuschalten, aber dann kann man über ID1/2 nicht mehr kommunizieren)
 * braun:   1   ID2     hub(RX)<-device(TX) D1
 * 
 * Die Library ist gepatcht für Arduino Nano RP2040 Connect und verwendet Serial1 (D0/D1).
*/

#include <Arduino.h>
#include <Wire.h>
#include <ColorSensor.h>
#include <Adafruit_BNO055.h>
#include <SRF08.h>

// SPIKE Prime Farbsensor Emulator:
ColorSensor myDevice = ColorSensor();
// Kompass:
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
// Ultraschallsensoren:
SRF08 srf08_1 = SRF08(0x70);
SRF08 srf08_2 = SRF08(0x71);
SRF08 srf08_3 = SRF08(0x72);
SRF08 srf08_4 = SRF08(0x73);

// aus diesen Variablen liest die Library die Werte, die an den SPIKE gesendet werden:
uint16_t us12_us34_heading[3] = {0xFFFF, 0xFFFF, 0xFFFF}; // Ultraschallsensor 3 und 4 in cm / 2, Kompass "Heading" in °
// Verbindungs-Status
bool connection_status = false;
bool compass_detected = false;

const bool debug = false;

void setup() {
    // aus diesen Variablen liest die Library die Werte, die an den SPIKE gesendet werden:
    myDevice.setSensorRGB_I(us12_us34_heading);

    // Die Onboard LED wird anzeigen, ob wir kommunizieren und welche Sensoren zur Verfügung stehen:
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
 
    if (debug) {
        Serial.begin(115200);
        while (!Serial);
    }

    Wire.begin();

    if (!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        if (debug) {
            Serial.print("BNO055 Verdrahtung prüfen!");
        }
        digitalWrite(LED_BUILTIN, LOW);
        compass_detected = false;
    } else {
        digitalWrite(LED_BUILTIN, HIGH);
        compass_detected = true;
    }
    bno.setExtCrystalUse(true);

    srf08_1.init(&Wire);
    srf08_2.init(&Wire);
    srf08_3.init(&Wire);
    srf08_4.init(&Wire);
}

const uint8_t NO_VALUE_AVALABLE8 = 0xFF;
const uint16_t NO_VALUE_AVALABLE16 = 0xFFFF;

uint8_t toValidRange(int16_t distance) {
    if (distance < 0) {
        return NO_VALUE_AVALABLE8;
    } else {
        return (uint8_t) constrain(distance, 0, NO_VALUE_AVALABLE8 - 1);
    }
}

void loop() {
    sensors_event_t kompass_event;
    bno.getEvent(&kompass_event);
    if (compass_detected) {
        us12_us34_heading[2] = (uint16_t) kompass_event.orientation.x; // 0..359
    } else {
        us12_us34_heading[2] = NO_VALUE_AVALABLE16;
    }

    if (srf08_1.readRange()) {
        us12_us34_heading[0] = toValidRange(srf08_1.getDistance()) << 8;
        srf08_1.startRangeReading();
    }
    if (srf08_2.readRange()) {
        us12_us34_heading[0] |= (toValidRange(srf08_2.getDistance())) & 0x00FF;
        srf08_2.startRangeReading();
    }
    if (srf08_3.readRange()) {
        us12_us34_heading[1] = toValidRange(srf08_3.getDistance()) << 8;
        srf08_3.startRangeReading();
    }
    if (srf08_4.readRange()) {
        us12_us34_heading[1] |= (toValidRange(srf08_4.getDistance())) & 0x00FF;
        srf08_4.startRangeReading();
    }

    // mit dem SPIKE kommunizieren
    myDevice.process();
    /**
     * On Connected:
     *   On Init success:
     *     SPIKE -> Arduino: SET_COMBINATION_MODE
     *     SPIKE <- Arduino: no ACK, because the packet does not seem to have any payload attached
     *                               (hardcoding combination mode in the library anyways, because the SPIKE expects this)
     *     SPIKE -> Arduino: GET_VALUE(COLOR)
     *     SPIKE <- Arduino: color value
     *     repeat every 0.2s:
     *       SPIKE -> Arduino: NACK
     *       SPIKE <- Arduino: combination values: (sensorColor, reflectedLight, sensorRGB[0], sensorRGB[1], sensorRGB[2])
     *                                            Unfortunately, HSV, ambient color and sensorRGB[3] (general intensity)
     *                                            are not expected and never queried by the SPIKE.
    */
    if (myDevice.isConnected() && !connection_status) {
        connection_status = true;
        digitalWrite(LED_BUILTIN, HIGH);
    } else if (!myDevice.isConnected() && connection_status) {
        connection_status = false;
        digitalWrite(LED_BUILTIN, LOW);
    }
}
