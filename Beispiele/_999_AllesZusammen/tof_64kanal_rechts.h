#pragma once

#include <Arduino.h>
#include "common.h"

// Bibliothek ("library") importieren
#include <SparkFun_VL53L5CX_Library.h>

// Sensor-Objekt erzeugen
SparkFun_VL53L5CX lidar_rechts = SparkFun_VL53L5CX();

Einstellungen einstellungen_rechts = ACHT_MAL_ACHT;

/** optional: Stoppuhr, um zu Verbindungsverluste zu erkennen */
#include <Chrono.h> 
Chrono keineNeuenDatenStoppuhr_rechts = Chrono(Chrono::MILLIS, false); // noch nicht gestartet

// Einstellungen anwenden
void initialisiereTof64Kanal_rechts() {
    Serial.println("Initialisierung des 64-Kanal ToF kann bis zu 10 Sekunden dauern...");
    if (!lidar_rechts.begin(NEUE_ADDRESSE_TOF_64KANAL, Wire)) {
        Serial.println("ToF64 rechts Verdrahtung prüfen! Roboter aus- und einschalten! Programm Ende.");
        while (1);
    }
    if (!lidar_rechts.setResolution(einstellungen_rechts.aufloesung) ||
        !lidar_rechts.setRangingFrequency(einstellungen_rechts.maxMessfrequenz)) {  // siehe oben
            Serial.println("ToF64 rechts Auflösung oder Messfrequenz konnte nicht geändert werden! Programm Ende.");
            while (1);
    }

    lidar_rechts.startRanging();
    keineNeuenDatenStoppuhr_rechts.start();
}

// Hier werden die Sensorwerte abgespeichert. Man kann sie im Hauptprogramm verwenden.
VL53L5CX_ResultsData messDaten_rechts;

// Sensorwerte aktualisieren (also den Sensor auslesen)
void leseTof64Kanal_rechts() {
    if (lidar_rechts.isDataReady()) {
        // diese Zeile speichert bereits die Daten ab:
        if (lidar_rechts.getRangingData(&messDaten_rechts)) {
            // alles OK
            keineNeuenDatenStoppuhr_rechts.restart();
            return;  // rausgehen aus der Funktion, damit wir nicht zum Fehler kommen
        } // else: Fehler
    } // else: wenn es keine neuen Daten gibt, müssen wir sie auch nicht lesen

    // Fehler:
    if (keineNeuenDatenStoppuhr_rechts.hasPassed(1000)) {
        Serial.println("ToF64 rechts Verdrahtung prüfen! Roboter aus- und einschalten!");
    }
}
