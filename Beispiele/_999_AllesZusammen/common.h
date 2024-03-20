#pragma once

const uint16_t VERBINDUNG_VERLOREN = 0;

const uint8_t NEUE_ADDRESSE_TOF_1KANAL = 0x30;
const uint8_t NEUE_ADDRESSE_TOF_64KANAL = 0x35;

#include <SparkFun_VL53L5CX_Library.h>
typedef struct Einstellungen {
    uint8_t aufloesung;
    uint8_t bildSeitenlaenge;
    uint8_t maxMessfrequenz;
} Einstellungen_t;
// Bei meinem Sensor habe ich hier ein paar "kaputte Pixel":
const Einstellungen ACHT_MAL_ACHT = { VL53L5CX_RESOLUTION_8X8, 8, 15 }; // 8x8: max 15Hz
// Bei meinem Sensor sieht hier alles gut aus:
const Einstellungen VIER_MAL_VIER = { VL53L5CX_RESOLUTION_4X4, 4, 60 }; // 4x4: max 60Hz
