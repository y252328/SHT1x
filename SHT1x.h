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
    SHT1x(int dataPin, int clockPin):_dataPin(dataPin), _clockPin(clockPin){}
    float readHumidity(bool checkSum=false);
    float readTemperature(const TempUnit unit, bool checkSum=false);
  private:
    int _dataPin;
    int _clockPin;
    int shiftIn(int _numBits);
    void sendCommandSHT(uint8_t _command);
    void waitForResultSHT();
    int getData16SHT();
    void skipCrcSHT();
    int getCRC();
    uint8_t crc8(int data, int size, uint8_t init=0);
    uint8_t reverseByte(uint8_t data);
};

#endif
