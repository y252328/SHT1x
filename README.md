# SHT1x Temperature / Humidity Sensor Library for Arduino
Copyright 2020 Chih-Yu Hsiang <y252328@gmail.com> \
Copyright 2009 Jonathan Oxer jon@oxer.com.au / http://www.practicalarduino.com  \
Copyright 2008 Maurice Ribble ribblem@yahoo.com / http://www.glacialwanderer.com

This repository is based on [practicalarduino/SHT1x](https://github.com/practicalarduino/SHT1x) and fully support all commands of sht1x. 

Provides a simple interface to the SHT1x series (SHT10, SHT11, SHT15)
and SHT7x series (SHT71, SHT75) temperature / humidity sensors from
Sensirion, http://www.sensirion.com. These sensors use a "2-wire"
communications buss that is similar to I2C and can co-exist on the same
physical wire as I2C devices.

## Installation
Download the directory "SHT1x" and move it into the "libraries"
directory inside your sketchbook directory, then restart the Arduino
IDE. You will then see it listed under File->Examples->SHT1x.

## Usage
The library is instantiated as an object with methods. 
Include it in your sketch and then create an object, specifying the pins to use for communication with the sensor and initial it:
``` c++
#include <SHT1x.h>
#define dataPin 10
#define clockPin 11
SHT1x sht1x(dataPin, clockPin);
sht1x.init();
```

You can then call methods on that object within your program. In this
example we created an object called "sht1x", but it could have been
called whatever you like. A simple example program is included with
the library and can be accessed from the File->Examples->SHT1x menu.

### Read Temperature

The `readTemperature()` method returns a float within the valid range of the sensor of -40 to +123.8C (-40 to +254.9F).
A value of -40 is returned in the event of a communication error with
the sensor.

Example:
```c++
// read temperature and covert to degree Celsius
float tempC = sht1x.readTemperature(TempUnit::C); 

// read temperature and covert to degree Fahrenheit
float tempF = sht1x.readTemperature(TempUnit::F);
```

### Read Humidity

The `readHumidity()` method returns a float within the valid range of the sensor of 0 to 100%.
A negative value is returned in the event of a communication error with
the sensor.

Example:
```c++
float humidity = sht1x.readHumidity();
```

### Connection Reset
If communication with the device is lost the `connectionReset()` method
 can reset the serial interface.
 (interface only)

Example:
```c++
sht1x.connectionReset(); 
```

### Soft Reset
The `softReset()` method can
resets the interface, clears the
status register to default values.

Example:
```c++
sht1x.softReset(); 
```

### Read Status Register
The `readStatusReg()` method will read status register from sht1x
and return it.

Example:
```c++
auto reg_value = sht1x.readStatusReg(); 
```

### Write Status Register
The `writeStatusReg()` method will write value to sht1x status register.

Example:
```c++
sht1x.readStatusReg(value); 
```

### CRC
This library support CRC-8. In default, the CRC is used, but it can be disabled via the argument.

Example:
```c++
// read temperature without CRC
float tempC = sht1x.readTemperature(TempUnit::C, false); 

// read humidity without CRC
float humidity = sht1x.readHumidity(false);

// read status register without CRC
float humidity = sht1x.readStatusReg(false);
```