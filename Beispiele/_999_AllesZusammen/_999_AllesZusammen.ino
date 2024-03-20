/** importiert Arduino automatisch, muss man also hier nicht unbedingt auch noch mal importieren: */
#include <Arduino.h>

/**
 * Beispiel:
 * - auf dem Board "TM Rescue v2 2023"
 * - am Arduino Nano RP2040 Connect (!!! geht nicht mit Arduino Due !!!)
 * - Sensoren verteilt auf 2 I2C Busse
 * - geht nur mit Akku (USB reicht der Strom nicht aus)
 * 
 * I2C Adressen:
 * LSM6DSOX 0x6A
 * ATECC608 0x60
 * AS7262   0x49
 * VL53L0X  0x29 (umprogrammiert auf 0x30)
 * VL53L5CX 0x29 (umprogrammiert auf 0x35)
 * TCS34725 0x29 (darf erst nach der Umprogrammierung der anderen Sensoren initialisiert werden)
*/

#include <Wire.h>

#include "onboard_6achsen_imu.h"
#include "spektrometer_farbsensor_links.h"
#include "spektrometer_farbsensor_rechts.h"
#include "rgb_farbsensor_links.h"
#include "tof_1kanal_rechts.h"
#include "tof_64kanal_links.h"
#include "tof_64kanal_rechts.h"
#include "helligkeitssensoren.h"

void setup() {
    // Serial Monitor
    Serial.begin(115200);
    // damit wir Zeit haben den Serial Monitor zu öffnen, bevor Initialisierungs-Fehler passieren:
    delay(5000);

    // I2C Bus links vorbereiten
    Wire1.begin();
    Wire1.setClock(1000000);

    // I2C Bus rechts vorbereiten
    Wire.begin();
    Wire.setClock(1000000);

    pinMode(LED_BUILTIN, OUTPUT);

    initialisiereHelligkeitssensoren(); // von oben geschaut von links nach rechts: D6, D0, D1, D7, D8, D9
    initialisiereImu(); // mit Adresse 0x6A

    // I2C Bus links (Wire1):
    initialisiereSpekrometer_links(); // mit Adresse 0x49
    // Reihenfolge ist hier wichtig, da am Anfang alle die Adresse 0x29 haben:
    initialisiereTof64Kanal_links(); // mit NEUE_ADDRESSE (0x35)
    initialisiereRgbSensor_links(); // mit Adresse 0x29

    // I2C Bus rechts (Wire):
    initialisiereSpekrometer_rechts(); // mit Adresse 0x49
    // Reihenfolge ist hier wichtig, da am Anfang alle die Adresse 0x29 haben:
    initialisiereTof1Kanal_rechts(); // mit NEUE_ADDRESSE2 (0x30)
    initialisiereTof64Kanal_rechts(); // mit NEUE_ADDRESSE (0x35)

    Serial.println("Initialisierung abgeschlossen");
}

void loop() {
    leseHelligkeitssensoren();
    leseImu();
    leseSpektrometer_links();
    leseSpektrometer_rechts();
    leseRgbSensor_links();
    leseTof1Kanal_rechts();
    leseTof64Kanal_links();
    leseTof64Kanal_rechts();

    Serial.println(
        "Beschleunigung x=" + String(acc.x) + " y=" + String(acc.y) + " z=" + String(acc.z)
        + " Rotationsgeschwindigkeit x=" + String(gyro.x) + " y=" + String(gyro.y) + " z=" + String(gyro.z));
    Serial.println(
        "links: violett=" + String(violett_links) + " blau=" + String(blau_links) + " grün=" + String(gruen_links)
        + " gelb=" + String(gelb_links) + " orange=" + String(orange_links) + " rot=" + String(rot_links));
    Serial.println(
        "rechts: violett=" + String(violett_rechts) + " blau=" + String(blau_rechts) + " grün=" + String(gruen_rechts)
        + " gelb=" + String(gelb_rechts) + " orange=" + String(orange_rechts) + " rot=" + String(rot_rechts));
    Serial.println("links: R:" + String(rgb_rot_links) + " G:" + String(rgb_gruen_links) + " B:" + String(rgb_blau_links) + " C:" + String(rgb_helligkeit_links));
    Serial.println("rechts: Abstand: " + String(abstand_rechts));
    Serial.print("Reflektionsleiste: ");
    for (int i = 0; i < SENSOR_LEISTE_ANZAHL_SENSOREN; i++) {
        Serial.print(String(helligkeiten[i]) + '\t');
    }
    Serial.println();
    if (helligkeiten[0] > 1000) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }
    Serial.println("ToF64 links:");
    for (int y = 0; y <= 8 * (8 - 1); y += 8) {
            for (int x = 8 - 1; x >= 0; x--) {
                Serial.print("\t");
                Serial.print(messDaten_links.distance_mm[x + y]);
            }
            Serial.println();
    }
    Serial.println();
    Serial.println("ToF64 rechts:");
    for (int y = 0; y <= 8 * (8 - 1); y += 8) {
            for (int x = 8 - 1; x >= 0; x--) {
                Serial.print("\t");
                Serial.print(messDaten_rechts.distance_mm[x + y]);
            }
            Serial.println();
    }
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
}
