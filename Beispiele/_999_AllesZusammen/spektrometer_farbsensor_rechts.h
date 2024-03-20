#pragma once

#include <Arduino.h>
#include "common.h"

// Bibliothek ("library") importieren
#include <AS726X.h>

// Sensor-Objekt erzeugen
AS726X farbsensor_rechts = AS726X();

// Einstellungen

// Einstellungen anwenden
void initialisiereSpekrometer_rechts() {
    if (!farbsensor_rechts.begin(Wire)) {
        Serial.println("Spektrometer Farbsensor rechts Verdrahtung prüfen! Programm Ende.");
        while (1);
    }
    if (farbsensor_rechts.getVersion() != SENSORTYPE_AS7262) {
        Serial.println("Rechts: Falscher Sensor! Muss AS7262 sein!");
        while (1);
    }
    farbsensor_rechts.setMeasurementMode(CONTINUOUS_READ_ALL);
    farbsensor_rechts.setIntegrationTime(1);
    // Die Lampe verbraucht viel Strom! Auf 12.5mA begrenzen, da sonst 3V3 einbricht und kein Sensor mehr funktioniert.
    farbsensor_rechts.setBulbCurrent(0);
    farbsensor_rechts.enableBulb(); // Lampe AN
}

// Hier werden die Sensorwerte abgespeichert. Man kann sie im Hauptprogramm verwenden.
float violett_rechts, blau_rechts, gruen_rechts, gelb_rechts, orange_rechts, rot_rechts;

// Sensorwerte aktualisieren (also den Sensor auslesen)
void leseSpektrometer_rechts() {
    if (farbsensor_rechts.dataAvailable()) {
        // Bei Verbindungsverlust werden alle Werte automatisch 0 (VERBINDUNG_VERLOREN):
        violett_rechts = farbsensor_rechts.getViolet();
        blau_rechts = farbsensor_rechts.getBlue();
        gruen_rechts = farbsensor_rechts.getGreen();
        gelb_rechts = farbsensor_rechts.getYellow();
        orange_rechts = farbsensor_rechts.getOrange();
        rot_rechts = farbsensor_rechts.getRed();

        if (violett_rechts == VERBINDUNG_VERLOREN && blau_rechts == VERBINDUNG_VERLOREN &&
            gruen_rechts == VERBINDUNG_VERLOREN && gelb_rechts == VERBINDUNG_VERLOREN &&
            orange_rechts == VERBINDUNG_VERLOREN && rot_rechts == VERBINDUNG_VERLOREN) {
            // Fehler:
            Serial.println("Spektrometer rechts Verdrahtung prüfen!");
        }
    }
}
