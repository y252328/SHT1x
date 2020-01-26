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
#ifndef SHT1x_h
#define SHT1x_h

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

enum class TempUnit : uint8_t{
  C,
  F
};

class SHT1x
{
  public:
    SHT1x(int dataPin, int clockPin):_dataPin(dataPin), _clockPin(clockPin){
      pinMode(_clockPin, OUTPUT);
    }
    void init() {
      connectionReset();
      readStatusReg(true);
    }
    float readHumidity(const bool checkSum=true);
    float readTemperature(const TempUnit unit,const bool checkSum=true);
    void connectionReset();
    void softReset();
    uint8_t readStatusReg(const bool checkSum=true);
    void writeStatusReg(const uint8_t value);
  private:
    int _dataPin;
    int _clockPin;
    uint8_t crc_init;
    int shiftIn(const int _numBits);
    void transStart();
    void writeByte(const uint8_t data);
    int readByte(const bool ack);
    void waitForResult();
    uint8_t crc8(const unsigned int data, const int size, const uint8_t init=0);
    uint8_t reverseByte(const uint8_t data);
};

#endif
