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
    SHT1x(int dataPin, int clockPin):_dataPin(dataPin), _clockPin(clockPin){}
    float readHumidity(const bool checkSum=true);
    float readTemperature(const TempUnit unit,const  bool checkSum=true);
  private:
    int _dataPin;
    int _clockPin;
    int shiftIn(const int _numBits);
    void sendCommand(const uint8_t _command);
    void waitForResult();
    int getData16();
    void endTrans();
    int getCRC();
    uint8_t crc8(const int data, const int size, const uint8_t init=0);
    uint8_t reverseByte(const uint8_t data);
};

#endif
