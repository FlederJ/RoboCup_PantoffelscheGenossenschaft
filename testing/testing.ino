#include <Wire.h>
#include "SWI2C.h" // this should make an additional I2C bus possible
/* see here: https://github.com/Andy4495/SWI2C/blob/main/examples/SWI2C_example/SWI2C_example.ino for more info */



// Define the pins for the new I2C bus
#define SDA_PIN 2
#define SCL_PIN 3

const int sec_sens_pos = 10; // position of our second sensor 

// declare the second bus

SWI2C second_sensor = SWI2C(SDA_PIN, SCL_PIN, sec_sens_pos);

void setup() {
    Wire.begin(); // Initialize the original Wire library for the default bus
    second_sensor.begin(); // Initialize the additional I2C bus
}

void loop() {

}
