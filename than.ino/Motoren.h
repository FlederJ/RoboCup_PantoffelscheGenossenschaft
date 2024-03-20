/**
 * Motor-Bibliothek für den ZumoBot
 * 
 * Tom Markuske
 */

#ifndef __MOTOREN__
#define __MOTOREN__

// Arduino-Bibliotheken importieren
#include "Arduino.h"

/*
 * Pin-Definitionen
 */
#define PIN_MOTOR_LINKS_RICHTUNG1          23
#define PIN_MOTOR_LINKS_RICHTUNG2          22
#define PIN_MOTOR_LINKS_GESCHWINDIGKEIT    3 // PWM: 0 (Stopp) bis 255 (so schnell wie möglich)
#define PIN_MOTOR_RECHTS_RICHTUNG1         25
#define PIN_MOTOR_RECHTS_RICHTUNG2         24
#define PIN_MOTOR_RECHTS_GESCHWINDIGKEIT   2 // PWM: 0 (Stopp) bis 255 (so schnell wie möglich)
#define PIN_StandBY                        53

/**
 * Funktionsprototypen
 */
void initialisiere_motoren(void);
void stoppe_motoren(void);
void hebelsteuerung(int leistung_links, int leistung_rechts);
void hebelsteuerung(int wartezeit, int leistung_links, int leistung_rechts, bool danach_bremsen);
void motor_ansteuern(int richtungs_pin1, int richtungs_pin2, int geschwindigkeits_pin, int geschwindigkeit);

#endif
