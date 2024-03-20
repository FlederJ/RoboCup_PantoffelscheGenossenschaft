#pragma once

#include <Arduino.h>
#include "common.h"

// Bibliothek ("library") importieren
#include <SparkFun_VL53L5CX_Library.h>

// Sensor-Objekt erzeugen
SparkFun_VL53L5CX lidar_links = SparkFun_VL53L5CX();

Einstellungen einstellungen_links = ACHT_MAL_ACHT;

/** optional: Stoppuhr, um zu Verbindungsverluste zu erkennen */
#include <Chrono.h> 
Chrono keineNeuenDatenStoppuhr_links = Chrono(Chrono::MILLIS, false); // noch nicht gestartet

// Einstellungen anwenden
void initialisiereTof64Kanal_links() {
    Serial.println("Initialisierung des 64-Kanal ToF kann bis zu 10 Sekunden dauern...");
    if (!lidar_links.begin(NEUE_ADDRESSE_TOF_64KANAL, Wire1)) {
        Serial.println("ToF64 links Verdrahtung prüfen! Roboter aus- und einschalten! Programm Ende.");
        while (1);
    }
    if (!lidar_links.setResolution(einstellungen_links.aufloesung) ||
        !lidar_links.setRangingFrequency(einstellungen_links.maxMessfrequenz)) {  // siehe oben
            Serial.println("ToF64 links Auflösung oder Messfrequenz konnte nicht geändert werden! Programm Ende.");
            while (1);
    }

    lidar_links.startRanging();
    keineNeuenDatenStoppuhr_links.start();
}

// Hier werden die Sensorwerte abgespeichert. Man kann sie im Hauptprogramm verwenden.
VL53L5CX_ResultsData messDaten_links;

// Sensorwerte aktualisieren (also den Sensor auslesen)
void leseTof64Kanal_links() {
    if (lidar_links.isDataReady()) {
        // diese Zeile speichert bereits die Daten ab:
        if (lidar_links.getRangingData(&messDaten_links)) {
            // alles OK
            keineNeuenDatenStoppuhr_links.restart();
            return;  // rausgehen aus der Funktion, damit wir nicht zum Fehler kommen
        } // else: Fehler
    } // else: wenn es keine neuen Daten gibt, müssen wir sie auch nicht lesen

    // Fehler:
    if (keineNeuenDatenStoppuhr_links.hasPassed(1000)) {
        Serial.println("ToF64 links Verdrahtung prüfen! Roboter aus- und einschalten!");
    }
}
