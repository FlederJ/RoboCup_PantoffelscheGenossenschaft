#pragma once

#include <Arduino.h>
#include "common.h"

// Bibliothek ("library") importieren
#include <AS726X.h>

// Sensor-Objekt erzeugen
AS726X farbsensor_links = AS726X();

// Einstellungen

// Einstellungen anwenden
void initialisiereSpekrometer_links() {
    if (!farbsensor_links.begin(Wire1)) {
        Serial.println("Spektrometer Farbsensor links Verdrahtung prüfen! Programm Ende.");
        while (1);
    }
    if (farbsensor_links.getVersion() != SENSORTYPE_AS7262) {
        Serial.println("Links: Falscher Sensor! Muss AS7262 sein!");
        while (1);
    }
    farbsensor_links.setMeasurementMode(CONTINUOUS_READ_ALL);
    farbsensor_links.setIntegrationTime(1);
    // Die Lampe verbraucht viel Strom! Auf 12.5mA begrenzen, da sonst 3V3 einbricht und kein Sensor mehr funktioniert.
    farbsensor_links.setBulbCurrent(0);
    farbsensor_links.enableBulb(); // Lampe AN
}

// Hier werden die Sensorwerte abgespeichert. Man kann sie im Hauptprogramm verwenden.
float violett_links, blau_links, gruen_links, gelb_links, orange_links, rot_links;

// Sensorwerte aktualisieren (also den Sensor auslesen)
void leseSpektrometer_links() {
    if (farbsensor_links.dataAvailable()) {
        // Bei Verbindungsverlust werden alle Werte automatisch 0 (VERBINDUNG_VERLOREN):
        violett_links = farbsensor_links.getViolet();
        blau_links = farbsensor_links.getBlue();
        gruen_links = farbsensor_links.getGreen();
        gelb_links = farbsensor_links.getYellow();
        orange_links = farbsensor_links.getOrange();
        rot_links = farbsensor_links.getRed();

        if (violett_links == VERBINDUNG_VERLOREN && blau_links == VERBINDUNG_VERLOREN &&
            gruen_links == VERBINDUNG_VERLOREN && gelb_links == VERBINDUNG_VERLOREN &&
            orange_links == VERBINDUNG_VERLOREN && rot_links == VERBINDUNG_VERLOREN) {
            // Fehler:
            Serial.println("Spektrometer links Verdrahtung prüfen!");
        }
    }
}
