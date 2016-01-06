/*
  I2C.cpp - I2C library
  Copyright (c) 2011-2012 Wayne Truchsess.  All right reserved.
  Rev 5.0 - January 24th, 2012
          - Removed the use of interrupts completely from the library
            so TWI state changes are now polled. 
          - Added calls to lockup() function in most functions 
            to combat arbitration problems 
          - Fixed scan() procedure which left timeouts enabled 
            and set to 80msec after exiting procedure
          - Changed scan() address range back to 0 - 0x7F
          - Removed all Wire legacy functions from library
          - A big thanks to Richard Baldwin for all the testing
            and feedback with debugging bus lockups!
  Rev 4.0 - January 14th, 2012
          - Updated to make compatible with 8MHz clock frequency
  Rev 3.0 - January 9th, 2012
          - Modified library to be compatible with Arduino 1.0
          - Changed argument type from boolean to uint8_t in pullUp(), 
            setSpeed() and receiveByte() functions for 1.0 compatability
          - Modified return values for timeout feature to report
            back where in the transmission the timeout occured.
          - added function scan() to perform a bus scan to find devices
            attached to the I2C bus.  Similar to work done by Todbot
            and Nick Gammon
  Rev 2.0 - September 19th, 2011
          - Added support for timeout function to prevent 
            and recover from bus lockup (thanks to PaulS
            and CrossRoads on the Arduino forum)
          - Changed return type for stop() from void to
            uint8_t to handle timeOut function 
  Rev 1.0 - August 8th, 2011
  
  This is a modified version of the Arduino Wire/TWI 
  library.  Functions were rewritten to provide more functionality
  and also the use of Repeated Start.  Some I2C devices will not
  function correctly without the use of a Repeated Start.  The 
  initial version of this library only supports the Master.


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


#include <inttypes.h>
#include "I2CIO.h"
#include "Serial.h"



uint8_t I2C::bytesAvailable = 0;
uint8_t I2C::bufferIndex = 0;
uint8_t I2C::totalBytes = 0;
uint16_t I2C::timeOutDelay = 0;

I2C::I2C()
{
}


////////////// Public Methods ////////////////////////////////////////



void I2C::begin( I2C_HandleTypeDef * hhi2c)
{
//
// when using STM32 processors, all all hardware initialization
// are suposedly made by code generated using the CubeMX
// 
//
  Handle=hhi2c;
}

void I2C::end()
{
  
}

void I2C::timeOut(uint16_t _timeOut)
{
  timeOutDelay = _timeOut;
}

void I2C::setSpeed(uint8_t _fast)
{
//
// with STM32 mcu speed is set at initialization time
//
}
  
void I2C::pullup(uint8_t activate)
{
//
// with STM32 mcu pull up are set at initialization time
//
}

void I2C::scan()
{
  uint16_t tempTime = timeOutDelay;
  uint8_t totalDevicesFound = 0;
  Serial.println(F("Scanning for devices...please wait"));
  Serial.println();
  for(uint8_t s = 0; s <= 0x7F; s++)
  {
    returnStatus = 0;
    returnStatus = HAL_I2C_IsDeviceReady(Handle, s, 2, 80); // 2 retries, 80ms timeout
    if(returnStatus)
    {
      if(returnStatus == 1)
      {
        Serial.println(F("There is a problem with the bus, could not complete scan"));
        timeOutDelay = tempTime;
        return;
      }
    }
    else
    {
      Serial.print(F("Found device at address - "));
      Serial.print(F(" 0x"));
      Serial.println(s,HEX);
      totalDevicesFound++;
    }
    stop();
  }
  if(!totalDevicesFound){Serial.println(F("No devices found"));}
  timeOutDelay = tempTime;
}


uint8_t I2C::available()
{
  return(bytesAvailable);
}

uint8_t I2C::receive()
{
  bufferIndex = totalBytes - bytesAvailable;
  if(!bytesAvailable)
  {
    bufferIndex = 0;
    return(0);
  }
  bytesAvailable--;
  return(data[bufferIndex]);
}

  
/*return values for new functions that use the timeOut feature 
  will now return at what point in the transmission the timeout
  occurred. Looking at a full communication sequence between a 
  master and slave (transmit data and then readback data) there
  a total of 7 points in the sequence where a timeout can occur.
  These are listed below and correspond to the returned value:
  1 - Waiting for successful completion of a Start bit
  2 - Waiting for ACK/NACK while addressing slave in transmit mode (MT)
  3 - Waiting for ACK/NACK while sending data to the slave
  4 - Waiting for successful completion of a Repeated Start
  5 - Waiting for ACK/NACK while addressing slave in receiver mode (MR)
  6 - Waiting for ACK/NACK while receiving data from the slave
  7 - Waiting for successful completion of the Stop bit

  All possible return values:
  0           Function executed with no errors
  1 - 7       Timeout occurred, see above list
  8 - 0xFF    See datasheet for exact meaning */ 


/////////////////////////////////////////////////////

uint8_t I2C::write(uint8_t address, uint8_t registerAddress)
{
  returnStatus = 0;
  returnStatus = HAL_I2C_Master_Transmit(Handle,address<<1,NULL,0,timeOutDelay);
  return returnStatus;
}

uint8_t I2C::write(int address, int registerAddress)
{
  return(write((uint8_t) address, (uint8_t) registerAddress));
}

uint8_t I2C::write(uint8_t address, uint8_t registerAddress, uint8_t data)
{
	returnStatus = 0;
	uint8_t d[2];
		
	/* Format array to send */
	d[0] = registerAddress;
	d[1] = data;
	
	/* Try to transmit via I2C */
	returnStatus = HAL_I2C_Master_Transmit(Handle, (uint16_t)address<<1, (uint8_t *)d, 2, timeOutDelay);
	return(returnStatus);
}

uint8_t I2C::write(int address, int registerAddress, int data)
{
  return(write((uint8_t) address, (uint8_t) registerAddress, (uint8_t) data));
}

uint8_t I2C::write(uint8_t address, uint8_t registerAddress, char *data)
{
  uint8_t bufferLength = strlen(data);
  returnStatus = 0;
  returnStatus = write(address, registerAddress, (uint8_t*)data, bufferLength);
  return(returnStatus);
}

uint8_t I2C::write(uint8_t address, uint8_t registerAddress, uint8_t *dataBuffer, uint8_t numberBytes)
{
	returnStatus = HAL_I2C_Mem_Write(Handle, address<<1, registerAddress, registerAddress > 0xFF ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, dataBuffer, numberBytes, 1000);
  return returnStatus;
}

uint8_t I2C::read(int address, int numberBytes)
{
  return read((uint8_t) address, (uint8_t) numberBytes);
}

uint8_t I2C::read(uint8_t address, uint8_t numberBytes) // one byte, no register
{
  bytesAvailable = 0;
  bufferIndex = 0;
	/* Receive single byte without specifying  */
  returnStatus= HAL_I2C_Master_Receive(Handle, (uint16_t)address<<1, data, numberBytes, timeOutDelay);
  if (returnStatus != HAL_OK){
		return 0;
  }
  return numberBytes; // TODO: should we return something else ?
}

uint8_t I2C::read(int address, int registerAddress, int numberBytes)
{
  return(read((uint8_t) address, (uint8_t) registerAddress, (uint8_t) numberBytes));
}

uint8_t I2C::read(uint8_t address, uint8_t registerAddress, uint8_t numberBytes)
{
  bytesAvailable = 0;
  bufferIndex = 0;
  returnStatus=0;
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)address<<1, &registerAddress, 1, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return 1;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(Handle, address<<1, data, numberBytes, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return 0;
	}
	
	/* Return OK */
	return 0;
}

uint8_t I2C::read(uint8_t address, uint8_t numberBytes, uint8_t *dataBuffer)
{
  bytesAvailable = 0;
  bufferIndex = 0;
  returnStatus = 0;
	/* Receive multi bytes without specifying  */
	if (HAL_I2C_Master_Receive(Handle, (uint16_t)address<<1, dataBuffer, numberBytes, timeOutDelay) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return 1;
	}
	
	/* Return OK */
	return 0;
}

uint8_t I2C::read(uint8_t address, uint8_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer)
{
  bytesAvailable = 0;
  bufferIndex = 0;
  returnStatus = 0;
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)address<<1, &registerAddress, 1, timeOutDelay) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return 1;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(Handle, (uint16_t)address << 1, dataBuffer, numberBytes, timeOutDelay) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return 1;
	}
	
	/* Return OK */
	return 0;
}



I2C I2c = I2C();

