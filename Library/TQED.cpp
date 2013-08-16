/*
  TQED.cpp - Quadrature decoder library for Arduino using I2C
  Revision 1.1
  Copyright (c) 2012 Adriaan Swanepoel.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "TQED.h"

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Wire.h>
#include <inttypes.h>

union quadruplebyte
{
  long value;
  unsigned char bytes[4];
};

TQED::TQED(uint8_t address)
{
    deviceaddress = address;
    Wire.begin(); 
}

long TQED::getCount()
{
  Wire.beginTransmission(deviceaddress);
  Wire.write(COUNTERREG);
  Wire.endTransmission();

  Wire.requestFrom(deviceaddress, (uint8_t)4);
  int retry = 0;
  while((Wire.available() < 4) && (retry < RETRYCOUNT))
    retry++;
  
  if (Wire.available() >= 4)  
  {
	union quadruplebyte count; 	
	//read 4 bytes and turn them into long	
	count.bytes[0] = Wire.read();
	count.bytes[1] = Wire.read();
	count.bytes[2] = Wire.read();
	count.bytes[3] = Wire.read();
	
  
    	return count.value; 
  } else return 0;
}

bool TQED::centerCount()
{
  Wire.beginTransmission(deviceaddress);
  Wire.write(CENTERREG);
  Wire.endTransmission();
  return true;
}

bool TQED::resetCount()
{
  Wire.beginTransmission(deviceaddress);
  Wire.write(RESETREG);
  Wire.endTransmission();
  return true;
}

bool TQED::setAddress(uint8_t newaddress)
{
  Wire.beginTransmission(deviceaddress);
  Wire.write(SETADDRESSREG);
  Wire.write(newaddress);
  Wire.endTransmission();
  deviceaddress = newaddress;
  delay(10);
  return true;
}
