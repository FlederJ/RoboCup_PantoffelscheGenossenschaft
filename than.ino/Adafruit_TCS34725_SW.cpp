/**************************************************************************/
/*!
    @file     Adafruit_TCS34725_SW.cpp
    @author   KTOWN (Adafruit Industries)
			  Tom Markuske
    @license  BSD (see license.txt)

    Driver for the TCS34725 digital color sensors.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include <stdlib.h>
#include <math.h>

#include "Adafruit_TCS34725_SW.h"

/**
 * Das Mapping von Arduino Pin zu Atmega Pin ist hier dokumentiert:
 * https://www.arduino.cc/en/Hacking/PinMapping2560
 */

// Sensor SDA -> Arduino Pin 18 -> Prozessor-Pin PD3
#define SDA_PORT PORTD
#define SDA_PIN 3

// Sensor SCL -> Arduino Pin 19 -> Prozessor-Pin PD2
#define SCL_PORT PORTD
#define SCL_PIN 2

#include "SoftWire.h"
SoftWire softWire = SoftWire();

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_TCS34725_SW::write8 (uint8_t reg, uint32_t value)
{
  softWire.beginTransmission(TCS34725_ADDRESS);
  #if ARDUINO >= 100
  softWire.write(TCS34725_COMMAND_BIT | reg);
  softWire.write(value & 0xFF);
  #else
  softWire.send(TCS34725_COMMAND_BIT | reg);
  softWire.send(value & 0xFF);
  #endif
  softWire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t Adafruit_TCS34725_SW::read8(uint8_t reg)
{
  softWire.beginTransmission(TCS34725_ADDRESS);
  #if ARDUINO >= 100
  softWire.write(TCS34725_COMMAND_BIT | reg);
  #else
  softWire.send(TCS34725_COMMAND_BIT | reg);
  #endif
  softWire.endTransmission();

  softWire.requestFrom(TCS34725_ADDRESS, 1);
  #if ARDUINO >= 100
  return softWire.read();
  #else
  return softWire.receive();
  #endif
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t Adafruit_TCS34725_SW::read16(uint8_t reg)
{
  uint16_t x; uint16_t t;

  softWire.beginTransmission(TCS34725_ADDRESS);
  #if ARDUINO >= 100
  softWire.write(TCS34725_COMMAND_BIT | reg);
  #else
  softWire.send(TCS34725_COMMAND_BIT | reg);
  #endif
  softWire.endTransmission();

  softWire.requestFrom(TCS34725_ADDRESS, 2);
  #if ARDUINO >= 100
  t = softWire.read();
  x = softWire.read();
  #else
  t = softWire.receive();
  x = softWire.receive();
  #endif
  x <<= 8;
  x |= t;
  return x;
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void Adafruit_TCS34725_SW::enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  delay(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);  
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void Adafruit_TCS34725_SW::disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
Adafruit_TCS34725_SW::Adafruit_TCS34725_SW(tcs34725IntegrationTime_t it, tcs34725Gain_t gain)
{
  _tcs34725Initialised = false;
  _tcs34725IntegrationTime = it;
  _tcs34725Gain = gain;
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
boolean Adafruit_TCS34725_SW::begin(void)
{
  softWire.begin();
  
  /* Make sure we're actually connected */
  uint8_t x = read8(TCS34725_ID);
  if ((x != 0x44) && (x != 0x10))
  {
    return false;
  }
  _tcs34725Initialised = true;

  /* Set default integration time and gain */
  setIntegrationTime(_tcs34725IntegrationTime);
  setGain(_tcs34725Gain);

  /* Note: by default, the device is in power down mode on bootup */
  enable();

  return true;
}
  
/**************************************************************************/
/*!
    Sets the integration time for the TC34725
*/
/**************************************************************************/
void Adafruit_TCS34725_SW::setIntegrationTime(tcs34725IntegrationTime_t it)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_ATIME, it);

  /* Update value placeholders */
  _tcs34725IntegrationTime = it;
}

/**************************************************************************/
/*!
    Adjusts the gain on the TCS34725 (adjusts the sensitivity to light)
*/
/**************************************************************************/
void Adafruit_TCS34725_SW::setGain(tcs34725Gain_t gain)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_CONTROL, gain);

  /* Update value placeholders */
  _tcs34725Gain = gain;
}

/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
void Adafruit_TCS34725_SW::getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (!_tcs34725Initialised) begin();

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);
  
  /* Set a delay for the integration time */
  switch (_tcs34725IntegrationTime)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      delay(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      delay(24);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      delay(50);
      break;
    case TCS34725_INTEGRATIONTIME_101MS:
      delay(101);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      delay(154);
      break;
    case TCS34725_INTEGRATIONTIME_700MS:
      delay(700);
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t Adafruit_TCS34725_SW::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * pow(n, 3)) + (3525.0F * pow(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to lux
*/
/**************************************************************************/
uint16_t Adafruit_TCS34725_SW::calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}


void Adafruit_TCS34725_SW::setInterrupt(boolean i) {
  uint8_t r = read8(TCS34725_ENABLE);
  if (i) {
    r |= TCS34725_ENABLE_AIEN;
  } else {
    r &= ~TCS34725_ENABLE_AIEN;
  }
  write8(TCS34725_ENABLE, r);
}

void Adafruit_TCS34725_SW::clearInterrupt(void) {
  softWire.beginTransmission(TCS34725_ADDRESS);
  #if ARDUINO >= 100
  softWire.write(TCS34725_COMMAND_BIT | 0x66);
  #else
  softWire.send(TCS34725_COMMAND_BIT | 0x66);
  #endif
  softWire.endTransmission();
}


void Adafruit_TCS34725_SW::setIntLimits(uint16_t low, uint16_t high) {
   write8(0x04, low & 0xFF);
   write8(0x05, low >> 8);
   write8(0x06, high & 0xFF);
   write8(0x07, high >> 8);
}
