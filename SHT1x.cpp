/**
 * SHT1x Library
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 * Based on previous work by:
 *    Maurice Ribble: <www.glacialwanderer.com/hobbyrobotics/?p=5>
 *    Wayne ?: <ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html>
 *
 * Manages communication with SHT1x series (SHT10, SHT11, SHT15)
 * temperature / humidity sensors from Sensirion (www.sensirion.com).
 */
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "SHT1x.h"

/* ================  Public methods ================ */
/**
 * Reads the current temperature
 */
float SHT1x::readTemperature(const TempUnit unit)
{
  int _val;                // Raw value returned from sensor
  float _temperature;      // Temperature derived from raw value

  // Conversion coefficients from SHT15 datasheet
  constexpr  float D1 = -40.0;  // for 14 Bit @ 5V
  float D2; // for 14 Bit DEGC
  if ( unit == TempUnit::C) {
    D2 = 0.01;
  } else if( unit == TempUnit::F) {
    D2 = 0.018;
  }

  // Fetch raw value
  _val = readTemperatureRaw();

  // Convert raw value to degrees Celsius
  _temperature = (_val * D2) + D1;

  return (_temperature);
}

/**
 * Reads current temperature-corrected relative humidity
 */
float SHT1x::readHumidity()
{
  int _val;                    // Raw humidity value returned from sensor
  float _linearHumidity;       // Humidity with linear correction applied
  float _correctedHumidity;    // Temperature-corrected humidity
  float _temperature;          // Raw temperature value

  // Conversion coefficients from SHT15 datasheet
  constexpr  float C1 = -4.0;       // for 12 Bit
  constexpr  float C2 =  0.0405;    // for 12 Bit
  constexpr  float C3 = -0.0000028; // for 12 Bit
  constexpr  float T1 =  0.01;      // for 14 Bit @ 5V
  constexpr  float T2 =  0.00008;   // for 14 Bit @ 5V

  // Command to send to the SHT1x to request humidity
  constexpr uint8_t _gHumidCmd = 0b00000101;

  // Fetch the value from the sensor
  sendCommandSHT(_gHumidCmd);
  waitForResultSHT();
  _val = getData16SHT();
  skipCrcSHT();

  // Apply linear conversion to raw value
  _linearHumidity = C1 + C2 * _val + C3 * _val * _val;

  // Get current temperature for humidity correction
  _temperature = readTemperature(TempUnit::C);

  // Correct humidity value for current temperature
  _correctedHumidity = (_temperature - 25.0 ) * (T1 + T2 * _val) + _linearHumidity;

  return (_correctedHumidity);
}


/* ================  Private methods ================ */

/**
 * Reads the current raw temperature value
 */
float SHT1x::readTemperatureRaw()
{
  int _val;
  // Command to send to the SHT1x to request Temperature
  constexpr uint8_t  _gTempCmd  = 0b00000011;

  sendCommandSHT(_gTempCmd);
  waitForResultSHT();
  _val = getData16SHT();
  uint8_t _crc = getCRC();
  uint8_t crc = crc8(_gTempCmd, 8);
  crc = crc8(_val, 16, crc);
  crc = reverseByte(crc);
  return (_val);
}

/**
 */
int SHT1x::shiftIn(int _numBits)
{
  int ret = 0;
  int i;

  for (i=0; i<_numBits; ++i)
  {
     digitalWrite(_clockPin, HIGH);
     delay(10);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
     ret = ret*2 + digitalRead(_dataPin);
     digitalWrite(_clockPin, LOW);
  }

  return(ret);
}

/**
 */
void SHT1x::sendCommandSHT(uint8_t  _command)
{
  int ack;

  // Transmission Start
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, LOW);

  // The command (3 msb are address and must be 000, and last 5 bits are command)
  shiftOut(_dataPin, _clockPin, MSBFIRST, _command);

  // Verify we get the correct ack
  digitalWrite(_clockPin, HIGH);
  pinMode(_dataPin, INPUT);
  ack = digitalRead(_dataPin);
  if (ack != LOW) {
    Serial.println("Ack Error 0");
  }
  digitalWrite(_clockPin, LOW);
  ack = digitalRead(_dataPin);
  if (ack != HIGH) {
    Serial.println("Ack Error 1");
  }
}

/**
 */
void SHT1x::waitForResultSHT()
{
  int ack;
  pinMode(_dataPin, INPUT);

  for(int i= 0; i < 100; ++i)
  {
    delay(10);
    ack = digitalRead(_dataPin);
    if (ack == LOW) {
      break;
    }
  }

  if (ack == HIGH) {
    Serial.println("Ack Error 2"); // Can't do serial stuff here, need another way of reporting errors
  }
}

/**
 */
int SHT1x::getData16SHT()
{
  int val;

  // Get the most significant bits
  pinMode(_dataPin, INPUT);
  pinMode(_clockPin, OUTPUT);
  val = shiftIn(8);
  val *= 256;

  // Send the required ack
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);

  // Get the least significant bits
  pinMode(_dataPin, INPUT);
  val |= shiftIn(8);

  return val;
}

/**
 */
void SHT1x::skipCrcSHT()
{
  // Skip acknowledge to end trans (no CRC)
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);

  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);
}

int SHT1x::getCRC()
{
  
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);

  pinMode(_dataPin, INPUT);
  int val = shiftIn(8);

  // Skip acknowledge to end trans (no CRC)
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);

  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);
  return val;
}

uint8_t SHT1x::crc8(int data, int size, uint8_t init)
{
  uint8_t crc = init;
  for ( int i = size-1 ; i >= 0 ; -- i) {
    if(((data >> i) & 0x1) != ((crc >> 7) & 0x1)){
      crc = crc << 1;
      crc ^= 0x30;
      crc |= 0x1;
    } else {
      crc = crc << 1;
    }
    crc &= 0xFF;
  }
  return crc;
}

uint8_t SHT1x::reverseByte(uint8_t data)
{
  uint8_t r_data = 0;
  for( int i = 0 ; i < 8 ; ++ i ) {
    bitWrite(r_data, i, bitRead(data, 7-i));
  }
  return r_data;
}