/** importiert Arduino automatisch, muss man also hier nicht unbedingt auch noch mal importieren: */
#include <Arduino.h>

#include <iostream>
#include <fstream>
using namespace std;

/**
 * !!! Immer darauf achten, dass unten in der Statusleiste...
 *     ... das richtige Arduino Board eingestellt ist
 *     ... das richtige "Sketch File" ausgewählt ist (das ändert sich nämlich nicht automatisch)
 *     ... die richtige C/C++ Konfiguration eingestellt ist (sonst gibt es noch mehr "rote swiggels")
 * 
 * :: Externen 6-Kanal Reflektionssensor auslesen (misst die Intensität von reflektiertem Infrarot-Licht) ::
 * :: Serial Plotter ausprobieren ::
 * :: Hauptprogramm-Schleife in Zustände unterteilen ::
 * :: Programm in Funktionen unterteilen ::
 * 
 * Hardware-Aufbau:
 * Zumo Line Sensor:    Arduino Due / Arduino Nano RP2040 Connect:
 *              A0  <-> D1/TX (mitting links)
 *              A2  <-> D0/RX (links)
 *              5   <-> D6 (ganz links)
 *              5V  <-- 3V3 (!! sonst machen wir die Digitalpins des Arduino kaputt !!)
 *              GND <-- GND
 *              11  <-> D7 (mittig rechts)
 *              A3  <-> D8 (rechts)
 *              4   <-> D9 (ganz rechts)
 *              VIN <-- VIN (Auf dem Steckbrett 5V USB-Spannung (dann leuchten auch die sichtbaren LEDs nicht),
 *                           auf dem echten Roboter 7,4V Akkuspannung (dann leuchten die LEDs).
 *                           Es muss also beim Wecksel der Versorgungsspannung erneut kalibriert werden.)
 * 
 * Der QTR-6RC ist ein externer Sensor, der an jeden Digital Pin angeschlossen werden kann (Siehe SENSOR_LEISTE_PINS).
 * !!! Beim Arduino Nano RP2040 Connect darf der Sensor nicht an Pins A6 oder A7 angeschlossen werden,
 *     da diese nur Eingänge sind und die Bibliothek den Pin sowohl als Ein- als auch als Ausgang verwendet.
*/

#include <QTRSensors.h>
QTRSensors sensorLeiste = QTRSensors();
const uint8_t SENSOR_LEISTE_ANZAHL_SENSOREN = 6;
const uint8_t SENSOR_LEISTE_PINS[] = {D6, D0, D1, D7, D8, D9};

enum Modus {
    /* Werte im Serial Monitor anzeigen. */
    WERTE_LOGGEN,
    /* Geht nur in der offiziellen Arduino IDE: Komma-Separierte Liste für den Serial Plotter. */
    IM_SERIAL_PLOTTER_ZEIGEN,
    // selbst gemachtes in file schreiben
    IN_FILE_SCHREIBEN,
};

/** hier einstellen, was das Programm mit den Sensorwerten anfangen soll: */
Modus modus = IN_FILE_SCHREIBEN;

void setup() {
    Serial.begin(115200);

    sensorLeiste.setTypeRC();
    sensorLeiste.setSensorPins(SENSOR_LEISTE_PINS, SENSOR_LEISTE_ANZAHL_SENSOREN);


    Serial.println("Initialisierung abgeschlossen");
}

// hier speichern wir die 6 Sensorwerte ab:
uint16_t helligkeiten[SENSOR_LEISTE_ANZAHL_SENSOREN];

void loop() {
    readLineBrightness();

    switch (modus) {
        case WERTE_LOGGEN:
            werteLoggen();
            break;

        case IM_SERIAL_PLOTTER_ZEIGEN:
            fuerPlotterLoggen();
            delay(100); // damit man im Plotter auch was erkennt und nicht alles so schnell vorbei fliegt
            break;

        case IN_FILE_SCHREIBEN:
            ofstream MyFile("logging_bw.txt");
            for (int i = 0; i < SENSOR_LEISTE_ANZAHL_SENSOREN; i++) {
                MyFile << String(helligkeiten[i]) + '\t'; // alles in eine Zeile
            }
            MyFile << "\n";
            delay(10); // für sicherheit der dateiübertragung
            MyFile.close();
            Serial.println("geschrieben");
            break;

    }
}

void readLineBrightness() {
  // unkalibierte Werte (siehe Library-Beispiele, wie man kalibriert)
  sensorLeiste.read(helligkeiten);
}

void werteLoggen() {
  for (int i = 0; i < SENSOR_LEISTE_ANZAHL_SENSOREN; i++) {
    Serial.print(String(helligkeiten[i]) + '\t'); // alles in eine Zeile
  }
  Serial.println(); // neue Zeile beginnen
}


void fuerPlotterLoggen() {
  for (int i = 0; i < SENSOR_LEISTE_ANZAHL_SENSOREN; i++) {
    Serial.print(String(helligkeiten[i]) + ','); // alles in eine Zeile
  }
  Serial.println(); // neue Zeile beginnen
}
