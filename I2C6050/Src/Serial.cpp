/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

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
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Serial.h"


#define SERIAL_BUFFER_SIZE 64


struct ring_buffer
{
	unsigned char buffer[SERIAL_BUFFER_SIZE];
	volatile unsigned int head;
	volatile unsigned int tail;
};


ring_buffer rx_buffer = { { 0 }, 0, 0 };
ring_buffer tx_buffer = { { 0 }, 0, 0 };


HardwareSerial Serial(&huart1,&rx_buffer,&tx_buffer);

HardwareSerial::HardwareSerial(UART_HandleTypeDef *hhuart, ring_buffer *rx_buffer, ring_buffer *tx_buffer)
{
	Handle = hhuart;
	_rx_buffer = rx_buffer;
	_tx_buffer = tx_buffer;
}

void HardwareSerial::begin(unsigned long baud)
{
	uint16_t baud_setting; // already set in STM32 initializer
}
void HardwareSerial::end()
{
  // wait for transmission of outgoing data
	// clear any received data
	_rx_buffer->head = _rx_buffer->tail;
}

int HardwareSerial::available(void)
{
	return (unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % SERIAL_BUFFER_SIZE;
}


int HardwareSerial::peek(void)
{
	if (_rx_buffer->head == _rx_buffer->tail) {
		return -1;
	}
	else {
		return _rx_buffer->buffer[_rx_buffer->tail];
	}
}

int HardwareSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
	if (_rx_buffer->head == _rx_buffer->tail) {
		return -1;
	}
	else {
		unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
		_rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % SERIAL_BUFFER_SIZE;
		return c;
	}
}

void HardwareSerial::flush()
{
	transmitting = false;
}

size_t HardwareSerial::write(uint8_t c)
{
	int status;
	status = HAL_UART_Transmit(Handle, &c, 1, 1000);
	if (status != HAL_OK)
	{
		return 0;
	}
	return 1;
}

HardwareSerial::operator bool() {
	return true;
}
