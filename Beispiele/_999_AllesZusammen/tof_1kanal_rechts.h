#pragma once

#include <Arduino.h>
#include "common.h"

// Bibliothek ("library") importieren
#include <VL53L0X.h>

// Sensor-Objekt erzeugen
VL53L0X abstandsSensor_rechts = VL53L0X();

/** optional: Stoppuhr, um zu Verbindungsverluste zu erkennen */
#include <Chrono.h> 
Chrono abstandStatischStoppuhr_rechts = Chrono(Chrono::MILLIS, false); // noch nicht gestartet

// Einstellungen anwenden
void initialisiereTof1Kanal_rechts() {
    abstandsSensor_rechts.setBus(&Wire);
    abstandsSensor_rechts.setAddress(NEUE_ADDRESSE_TOF_1KANAL);
    if (!abstandsSensor_rechts.init()) {
        Serial.println("ToF rechts Verdrahtung prüfen! Roboter aus- und einschalten! Programm Ende.");
        while (1);
    }
    abstandsSensor_rechts.setTimeout(500);
    abstandsSensor_rechts.setSignalRateLimit(0.1);
    abstandsSensor_rechts.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    abstandsSensor_rechts.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    // lasse Sensor die ganze Zeit an
    abstandsSensor_rechts.startContinuous();

    abstandStatischStoppuhr_rechts.start();
}

// Hier werden die Sensorwerte abgespeichert. Man kann sie im Hauptprogramm verwenden.
uint16_t abstand_rechts = 0;

// Sensorwerte aktualisieren (also den Sensor auslesen)
uint16_t vorheriger_abstand_rechts = VERBINDUNG_VERLOREN;
void leseTof1Kanal_rechts() {
    if (!abstandsSensor_rechts.timeoutOccurred()) {
        abstand_rechts = abstandsSensor_rechts.readRangeContinuousMillimeters();
        // statt 65535 kann es auch passieren, dass sich der Wert einfach nicht mehr ändert
        if (abstand_rechts != 65535 && !abstandStatischStoppuhr_rechts.hasPassed(1000)) {
            // alles OK
            if (vorheriger_abstand_rechts != abstand_rechts) {
                // merken: der Wert hat sich verändert
                vorheriger_abstand_rechts = abstand_rechts;
                abstandStatischStoppuhr_rechts.restart();
            }
            return; // rausgehen aus der Funktion, damit wir nicht zum Fehler kommen
        }
    }

    // Fehler:
    abstand_rechts = VERBINDUNG_VERLOREN;
    abstandStatischStoppuhr_rechts.restart(); // um bei Wackelkontakt Wiederverbindung zu erlauben
    Serial.println("ToF rechts Verdrahtung prüfen! Roboter aus- und einschalten!");
}
