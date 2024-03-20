/** importiert Arduino automatisch, muss man also hier nicht unbedingt auch noch mal importieren: */
#include <Arduino.h>

/**
 * !!! Immer darauf achten, dass unten in der Statusleiste...
 *     ... das richtige Arduino Board eingestellt ist
 *     ... das richtige "Sketch File" ausgewählt ist (das ändert sich nämlich nicht automatisch)
 *     ... die richtige C/C++ Konfiguration eingestellt ist (sonst gibt es noch mehr "rote swiggels")
 * 
 * :: Zahlen und Text an das Handy schicken ::
 * :: Programm interaktiv durchschreiten (zum Debugen). ::
 * 
 * Hardware-Aufbau:
 * Arduino Nano RP2040 Connect <-> Android Handy mit "RemoteXY" App
 * (!! Arduino Due geht nicht !!)
*/
#define REMOTEXY_BLUETOOTH_NAME "RemoteXY"
#define REMOTEXY_ACCESS_PASSWORD "123"
#include <RemoteXY_Arduino_BLE.h>

/** Diesen online Editor benutzen, um die Benutzeroberfläche zu bauen.
 *  (5 GUI Elemente kostenlos, sonst muss man Premium kaufen.)
 * https://remotexy.com/en/editor/
 * Da kommt dann die Definition "RemoteXY_CONF" und der "RemoteXY" struct bei raus,
 * die hier reinkopiert werden müssen. "RemoteXY_CONF" kodiert, wie die GUI aussehen
 * soll. Der Arduino teilt der App also mit, wie sie aussehen soll. "RemoteXY" 
 * ist zum Datenaustausch: Das Handy kann schreiben und lesen, der Arduino auch. Die
 * Daten werden via BLE synchronisiert. Hier in diesem Bereich also nichts umbenennen.
*/
// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 27 bytes
  { 255,1,0,0,0,20,0,16,31,1,2,0,20,12,22,11,2,26,31,31,
  79,78,0,79,70,70,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t switch_1; // =1 if switch ON and =0 if OFF 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

void setup() {
    Serial.begin(115200);
    delay(5000); // 5 Sekunden blockieren, damit wir Zeit haben den seriellen Monitor zu öffnen
    warteAufEnter();

    initialisiereRemoteXY(RemoteXY_CONF, &RemoteXY, REMOTEXY_ACCESS_PASSWORD);
}

enum Zustand {
    ZUSTAND_A, // 1 Sekunde delay
    ZUSTAND_B  // warte auf Kommando vom Handy
};

// Start-Zustand
Zustand zustand = ZUSTAND_A;

float a_value = 0.0;

void loop() {
    kommuniziereRemoteXY();

    // RemoteXY.onlineGraph_1 = a_value++;

    // switch (zustand) {
    //     case ZUSTAND_A:
    //         if (RemoteXY.switch_1) {
    //             zustand = ZUSTAND_B;
    //             // Serial.println("Hauptprogrammschleife läuft... (Zustand=" + String(zustand) + ")");
    //         }
    //         break;
    //     case ZUSTAND_B:
    //         if (!RemoteXY.switch_1) {
    //             zustand = ZUSTAND_A;
    //             // Serial.println("Hauptprogrammschleife läuft... (Zustand=" + String(zustand) + ")");
    //         }
    //         break;
    // }
}

/**
 * Es muss mindestens ein Zeichen eingegeben werden,
 * oder man stellt den Serial Monitor auf "Terminal Mode" und gibt ENTER ein.
*/
void warteAufEnter() {
    Serial.println("Weiter mit <ENTER>...");
    while (!Serial.available()) {
        // noch kein Zeichen (z.B. ENTER) empfangen
    }
    while (Serial.available()) {
        // Alle Zeichen lesen und damit den Zeichen-Puffer leeren.
        Serial.read();
    }
}
