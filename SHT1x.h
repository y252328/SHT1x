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

class SHT1x
{
  public:
    SHT1x(int dataPin, int clockPin):_dataPin(dataPin), _clockPin(clockPin){}
    float readHumidity();
    float readTemperatureC();
    float readTemperatureF();
  private:
    int _dataPin;
    int _clockPin;
    float readTemperatureRaw();
    int shiftIn(int _numBits);
    void sendCommandSHT(int _command);
    void waitForResultSHT();
    int getData16SHT();
    void skipCrcSHT();
    int getCRC();
    int crc8(int data, int size, int init=0);
    int reverseByte(int data);
};

#endif
