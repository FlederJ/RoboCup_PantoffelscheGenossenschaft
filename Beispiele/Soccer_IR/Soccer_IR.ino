#include <Arduino.h>
#include <Wire.h>
#include <SRF08.h>

const uint8_t BODENSENSOR_ROT_PIN = 38;   // D38/A14 red LED   / IR-red-sensitive Photodiode
const uint8_t BODENSENSOR_GRUEN_PIN = 39; // D39/A15 green LED / yellow-green-sensitive Photodiode
const uint8_t TASTER_PIN = 33;            // D33 gedrückt=LOW, nicht gedrückt=HIGH
const uint8_t LED_PIN = 35;               // D35 HIGH=AN, LOW=AUS

// Ultraschallsensor:
SRF08 srf08_1 = SRF08(0x75);

void setup() {
    Serial.begin(115200);
    while (!Serial);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(TASTER_PIN, INPUT_PULLUP);

    pinMode(BODENSENSOR_ROT_PIN, INPUT);
    pinMode(BODENSENSOR_GRUEN_PIN, INPUT);

    Wire2.begin(); // Pins 25/SDA1, 24/SCL2
    srf08_1.init(&Wire2);
}

void loop() {
    int bodensensor_rot = analogRead(BODENSENSOR_ROT_PIN);
    int bodensensor_gruen = analogRead(BODENSENSOR_GRUEN_PIN);
    uint8_t taster_nicht_gedrueckt = digitalRead(TASTER_PIN);

    int16_t ultraschall_abstand = -1;
    if (srf08_1.readRange()) {
        ultraschall_abstand = srf08_1.getDistance();
        srf08_1.startRangeReading();
    }

    Serial.println(
        "Boden RD: " + String(bodensensor_rot) + 
        "\tBoden GN: " + String(bodensensor_gruen) +
        "\tTaster: " + String(taster_nicht_gedrueckt) +
        "\tUS[cm]: " + String(ultraschall_abstand)
    );

    if (taster_nicht_gedrueckt) {
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(LED_PIN, LOW);
    } else {
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
    }

    delay(100);
}
