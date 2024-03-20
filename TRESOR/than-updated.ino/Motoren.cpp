/**
 * Motor-Bibliothek für den ZumoBot
 * 
 * Tom Markuske
 */

// Definitionen für die "Motoren-Bibliothek" importieren
#include "Motoren.h"

#include "ZumoMotors.h"
ZumoMotors motoren;

/**
 * Initialisiert die Motor-Pins und hält den Motor an
 */
void initialisiere_motoren(void) {
  Serial.print("Initialisiere Motoren... ");
  pinMode(PIN_MOTOR_LINKS_RICHTUNG1, OUTPUT);
  pinMode(PIN_MOTOR_LINKS_RICHTUNG2, OUTPUT);
  pinMode(PIN_MOTOR_LINKS_GESCHWINDIGKEIT, OUTPUT);
  pinMode(PIN_MOTOR_RECHTS_RICHTUNG1, OUTPUT);
  pinMode(PIN_MOTOR_RECHTS_RICHTUNG2, OUTPUT);
  pinMode(PIN_MOTOR_RECHTS_GESCHWINDIGKEIT, OUTPUT);
  pinMode(PIN_StandBY, OUTPUT);
  digitalWrite(PIN_StandBY, HIGH);
  motoren.flipLeftMotor(true);
  motoren.flipRightMotor(true);
  stoppe_motoren();
  Serial.println("FERTIG ;-)");
}

/**
 * Hält beide Motoren an
 */
void stoppe_motoren(void) {
  digitalWrite(PIN_MOTOR_LINKS_GESCHWINDIGKEIT, LOW);
  digitalWrite(PIN_MOTOR_RECHTS_GESCHWINDIGKEIT, LOW);
}

/**
 * Steuert den linken und rechten Motor gleichzeitig an.
 * Funktion blockiert nicht (wie "AN" bei den EV3-Blöcken).
 * 
 * leistung_links: Geschwindigkeit linker Motor -255 (rückwärts) bis 255 (vorwärts)
 * leistung_rechts: Geschwindigkeit rechter Motor -255 (rückwärts) bis 255 (vorwärts)
 * 
 */
void hebelsteuerung(int leistung_links, int leistung_rechts) {
  hebelsteuerung(0, leistung_links, leistung_rechts, false);
}

int zumo_leistung_rechts(int leistung) {
  return constrain(leistung * 0.645, -255, 255);
}
int zumo_leistung_links(int leistung) {
  return constrain(leistung * 0.645, -255, 255);
}

/**
 * Steuert den linken und rechten Motor gleichzeitig an.
 * Funktion blockiert wartezeit Millisekunden (wie "für n Sekunden" bei den EV3-Blöcken).
 * 
 * leistung_links: Geschwindigkeit linker Motor -255 (rückwärts) bis 255 (vorwärts)
 * leistung_rechts: Geschwindigkeit rechter Motor -255 (rückwärts) bis 255 (vorwärts)
 * wartezeit: wie lang die Funktion blockieren soll in Millisekunden
 * danach_bremsen: true: Motoren werden nach der Wartezeit wieder abgeschaltet
 */
void hebelsteuerung(int wartezeit, int leistung_links, int leistung_rechts, bool danach_bremsen) {
  motor_ansteuern(PIN_MOTOR_LINKS_RICHTUNG1, PIN_MOTOR_LINKS_RICHTUNG2, PIN_MOTOR_LINKS_GESCHWINDIGKEIT, (zumo_leistung_links(leistung_links)));
  motor_ansteuern(PIN_MOTOR_RECHTS_RICHTUNG1, PIN_MOTOR_RECHTS_RICHTUNG2, PIN_MOTOR_RECHTS_GESCHWINDIGKEIT, (zumo_leistung_rechts(leistung_rechts)));
  motoren.setSpeeds(leistung_links, leistung_rechts);
  delay(wartezeit);
  if (danach_bremsen) {
    motor_ansteuern(PIN_MOTOR_LINKS_RICHTUNG1, PIN_MOTOR_LINKS_RICHTUNG2, PIN_MOTOR_LINKS_GESCHWINDIGKEIT, 0);
    motor_ansteuern(PIN_MOTOR_RECHTS_RICHTUNG1, PIN_MOTOR_RECHTS_RICHTUNG2, PIN_MOTOR_RECHTS_GESCHWINDIGKEIT, 0);
    motoren.setSpeeds(0, 0);
  }  
}

/**
 * richtungs_pin1: Richtungspin 1 für den Motor, der angesteuert werden soll
 * richtungs_pin2: Richtungspin 2 für den Motor, der angesteuert werden soll
 * geschwindigkeits_pin: Geschwingkeitspin für den Motor, der angesteuert werden soll
 * geschwindigkeit: -255 (rückwärts) bis 255 (vorwärts)
 */
void motor_ansteuern(int richtungs_pin1, int richtungs_pin2, int geschwindigkeits_pin, int geschwindigkeit) {
  if (geschwindigkeit < -255 || geschwindigkeit > 255) {
    // fehler
    stoppe_motoren();
    return;
  }
  if (geschwindigkeit < 0) {
    digitalWrite(richtungs_pin1, LOW);
    digitalWrite(richtungs_pin2, HIGH);
  } else {
    digitalWrite(richtungs_pin1, HIGH);
    digitalWrite(richtungs_pin2, LOW);
  }
  analogWrite(geschwindigkeits_pin, abs(geschwindigkeit));
}
