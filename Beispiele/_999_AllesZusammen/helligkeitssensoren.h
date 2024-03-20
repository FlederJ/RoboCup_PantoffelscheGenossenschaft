#pragma once

#include <Arduino.h>

// Bibliothek ("library") importieren
#include <QTRSensors.h>

// Sensor-Objekt erzeugen
QTRSensors sensorLeiste = QTRSensors();

// Einstellungen
const uint8_t SENSOR_LEISTE_ANZAHL_SENSOREN = 6;
const uint8_t SENSOR_LEISTE_PINS[] = {D6, D0, D1, D7, D8, D9};

// Einstellungen anwenden
void initialisiereHelligkeitssensoren() {
    sensorLeiste.setTypeRC();
    sensorLeiste.setSensorPins(SENSOR_LEISTE_PINS, SENSOR_LEISTE_ANZAHL_SENSOREN);
}

// Hier werden die Sensorwerte abgespeichert. Man kann sie im Hauptprogramm verwenden.
uint16_t helligkeiten[SENSOR_LEISTE_ANZAHL_SENSOREN];

// Sensorwerte aktualisieren (also den Sensor auslesen)
void leseHelligkeitssensoren() {
    sensorLeiste.read(helligkeiten);
}
