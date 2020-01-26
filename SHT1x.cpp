/**
 * SHT1x Library
 *
 * Copyright 2020 Chih-Yu Hsiang <y252328@gmail.com>
 * Based on previous work by:
 *    Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
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
float SHT1x::readTemperature(const TempUnit unit, const bool checkSum)
{
  int _val;                // Raw value returned from sensor
  float _temperature;      // Temperature derived from raw value

  // Conversion coefficients from SHT15 datasheet
  constexpr  float D1 = -40.0;  // for 14 Bit @ 5V
  float D2;
  if ( unit == TempUnit::C) {
    D2 = 0.01;
  } else if( unit == TempUnit::F) {
    D2 = 0.018;
  }

  // Fetch raw value
  constexpr uint8_t  _gTempCmd  = 0b00000011;
  transStart();
  writeByte(_gTempCmd);
  waitForResult();
  _val = readByte(true);
  _val = _val << 8;
  _val |= readByte(checkSum);
  if (checkSum) {
    uint8_t _crc = readByte(false);
    uint8_t crc = crc8(_gTempCmd, 8, crc_init);
    crc = crc8(_val, 16, crc);
    crc = reverseByte(crc);
    if (_crc != crc) {
      Serial.println(F("SHT1x checksum error"));
    }
  }
  // Convert raw value to degrees Celsius
  _temperature = (_val * D2) + D1;

  return (_temperature);
}

/**
 * Reads current temperature-corrected relative humidity
 */
float SHT1x::readHumidity(const bool checkSum)
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
  transStart();
  writeByte(_gHumidCmd);
  waitForResult();
  _val = readByte(true);
  _val = _val << 8;
  _val |= readByte(checkSum);
  if (checkSum) {
    uint8_t _crc = readByte(false);
    uint8_t crc = crc8(_gHumidCmd, 8, crc_init);
    crc = crc8(_val, 16, crc);
    crc = reverseByte(crc);
    if (_crc != crc) {
      Serial.println(F("SHT1x checksum error"));
    }
  }

  // Apply linear conversion to raw value
  _linearHumidity = C1 + C2 * _val + C3 * _val * _val;

  // Get current temperature for humidity correction
  _temperature = readTemperature(TempUnit::C);

  // Correct humidity value for current temperature
  _correctedHumidity = (_temperature - 25.0 ) * (T1 + T2 * _val) + _linearHumidity;

  return (_correctedHumidity);
}

/**
* communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
*       _____________________________________________________         ________
* DATA:                                                      |_______|
*          _    _    _    _    _    _    _    _    _        ___     ___
* SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
*/
void SHT1x::connectionReset() {
  pinMode(_dataPin, OUTPUT);

  // initial state
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, LOW);

  for( uint8_t i = 0 ; i < 9 ; ++ i) {
    digitalWrite(_clockPin, HIGH);
    digitalWrite(_clockPin, LOW);
  }
  transStart();
}

/*
* soft reset sht
* must read status register after write for updating crc_init
*/
void SHT1x::softReset() {
  constexpr uint8_t _gResetCmd = 0b00011110;
  connectionReset();
  transStart();
  writeByte(_gResetCmd);
  delay(10);
  readStatusReg();
}

/*
* read status register and update crc_init
*/
uint8_t SHT1x::readStatusReg(const bool checkSum=true) {
  constexpr uint8_t cmd = 0b00000111;
  transStart();
  writeByte(cmd);
  uint8_t _val = readByte(checkSum);
  if (checkSum) {
    uint8_t _crc = readByte(false);
    crc_init = reverseByte(_val) & 0xF0;
    uint8_t crc = crc8(cmd, 8, crc_init);
    crc = crc8(_val, 8, crc);
    crc = reverseByte(crc);
    if (_crc != crc) {
      Serial.println(F("SHT1x checksum error"));
    }
  }
  return _val;
}


/*
* write status register
* must read status register after write for updating crc_init
*/
void SHT1x::writeStatusReg(const uint8_t value) {
  constexpr uint8_t cmd = 0b00000110;
  transStart();
  writeByte(cmd);
  writeByte(value);
  readStatusReg(true);
}

/* ================  Private methods ================ */
/**
 * read data from sensor
 */
int SHT1x::shiftIn(const int _numBits)
{
  int ret = 0;
  int i;
  pinMode(_dataPin, INPUT);
  for (i=0; i<_numBits; ++i)
  {
     digitalWrite(_clockPin, HIGH);
     delay(10);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
     ret = ret*2 + digitalRead(_dataPin);
     digitalWrite(_clockPin, LOW);
  }

  return(ret);
}

/*
* start trans. 
* ----------------------------------------------------------------------------------
* generates a transmission start 
*       _____         ________
* DATA:      |_______|
*           ___     ___
* SCK : ___|   |___|   |______
*/
void SHT1x::transStart() {
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, LOW);
}


/**
 *  write one byte data to sht
 */
void SHT1x::writeByte(const uint8_t data)
{
  int ack;

  pinMode(_dataPin, OUTPUT);

  // The command (3 msb are address and must be 000, and last 5 bits are command)
  shiftOut(_dataPin, _clockPin, MSBFIRST, data);

  // Verify we get the correct ack
  pinMode(_dataPin, INPUT);
  digitalWrite(_clockPin, HIGH);
  ack = digitalRead(_dataPin);
  if (ack != LOW) {
    Serial.println(F("SHT1x send command error"));
  }
  digitalWrite(_clockPin, LOW);
}

/**
 *  wait for measurement
 */
void SHT1x::waitForResult()
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
    Serial.println(F("SHT1x wait result timeout")); // Can't do serial stuff here, need another way of reporting errors
  }
}

/**
 * read one byte from sht
*/
int SHT1x::readByte(const bool ack)
{
  int val = shiftIn(8);
  
  // Send the required ack
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, !ack);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);
  return val;
}

/*
* calculate CRC-8
*/
uint8_t SHT1x::crc8(const unsigned int data, const int size, const uint8_t init)
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

/*
* reverse a byte
*/
uint8_t SHT1x::reverseByte(const uint8_t data)
{
  uint8_t r_data = 0;
  for( int i = 0 ; i < 8 ; ++ i ) {
    bitWrite(r_data, i, bitRead(data, 7-i));
  }
  return r_data;
}