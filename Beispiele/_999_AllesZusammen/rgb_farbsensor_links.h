#pragma once

#include <Arduino.h>
#include "common.h"

// Bibliothek ("library") importieren
#include <Adafruit_TCS34725.h>

// Einstellungen
// Sensor-Objekt erzeugen
Adafruit_TCS34725 rgbSensor_links = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);

/** optional: Stoppuhr, um zu Verbindungsverluste zu erkennen */
#include <Chrono.h> 
Chrono helligkeitStatischStoppuhr_links = Chrono(Chrono::MILLIS, false); // noch nicht gestartet

// Einstellungen anwenden
void initialisiereRgbSensor_links() {
    if (!rgbSensor_links.begin(TCS34725_ADDRESS, &Wire1)) {
        Serial.println("RGB Farbsensor links Verdrahtung prüfen! Programm Ende.");
        while (1);
    }
    helligkeitStatischStoppuhr_links.start();
}

// Hier werden die Sensorwerte abgespeichert. Man kann sie im Hauptprogramm verwenden.
uint16_t rgb_rot_links, rgb_gruen_links, rgb_blau_links, rgb_helligkeit_links;

// Sensorwerte aktualisieren (also den Sensor auslesen)
uint16_t vorheriges_rot_links, vorheriges_gruen_links, vorheriges_blau_links, vorherige_helligkeit_links = VERBINDUNG_VERLOREN;
void leseRgbSensor_links() {
    rgbSensor_links.getRawData(&rgb_rot_links, &rgb_gruen_links, &rgb_blau_links, &rgb_helligkeit_links);
    /** Dieser Mechanismus hier ist gefährlich, wenn es passieren kann, dass die Sensoren lange Zeit das selbe sehen:
     *  In meinen Versuchen habe ich oben den Gain von 4x auf 16x gestellt, um mehr Rauschen zu bekommen.
     *  Mit Timeout 5s sehe ich keine False-Negatives mehr: */
    if (!helligkeitStatischStoppuhr_links.hasPassed(5000)) {
        // alles OK
        if (vorheriges_rot_links != rgb_rot_links || vorheriges_gruen_links != rgb_gruen_links ||
            vorheriges_blau_links != rgb_blau_links || vorherige_helligkeit_links != rgb_helligkeit_links) {
            // merken: der Wert hat sich verändert
            vorheriges_rot_links = rgb_rot_links;
            vorheriges_gruen_links = rgb_gruen_links;
            vorheriges_blau_links = rgb_blau_links;
            vorherige_helligkeit_links = rgb_helligkeit_links;
            helligkeitStatischStoppuhr_links.restart();
        }
        return; // rausgehen aus der Funktion, damit wir nicht zum Fehler kommen
    }

    // Fehler:
    rgb_rot_links = VERBINDUNG_VERLOREN;
    rgb_gruen_links = VERBINDUNG_VERLOREN;
    rgb_blau_links = VERBINDUNG_VERLOREN;
    rgb_helligkeit_links = VERBINDUNG_VERLOREN;
    helligkeitStatischStoppuhr_links.restart(); // um bei Wackelkontakt Wiederverbindung zu erlauben
    Serial.println("RGB Sensor links Verdrahtung prüfen!");
}
