/******************************************************************************
 * AVRRA: The AVR Robotics API
 * serial.h - full duplex serial driver for USART/USART0
 * 
 * Copyright (c) 2008, Michael E. Ferguson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * - Neither the name of AVRRA nor the names of its contributors 
 *   may be used to endorse or promote products derived from this software 
 *   without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef AVRRA_SERIAL_HW
#define AVRRA_SERIAL_HW

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "utils.h"

#ifndef RX_BUFFER_SIZE
	#define RX_BUFFER_SIZE	64
#endif

static unsigned char rx_buffer[RX_BUFFER_SIZE];

static volatile int rx_buffer_head = 0;
static volatile int rx_buffer_tail = 0;

#ifdef __AVR_ATmega168__
    #define USART0_RX_vect   USART_RX_vect
#endif

/********************************serial Functions*****************************/

/** initializes serial transmit and receive at baud, 8-N-1 */
void serialInit(long baud){
#ifdef UDR0
	UBRR0H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;	
	UBRR0L = ((F_CPU / 16 + baud / 2) / baud - 1);
	
	// enable rx and tx
	SetBit(UCSR0B, RXEN0);
	SetBit(UCSR0B, TXEN0);
	
	// enable interrupt on complete reception of a byte
	SetBit(UCSR0B, RXCIE0);
	// defaults to 8-bit, no parity, 1 stop bit
#else
	UBRRH = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
	UBRRL = ((F_CPU / 16 + baud / 2) / baud - 1);
	
	// enable rx and tx
	SetBit(UCSRB, RXEN);
	SetBit(UCSRB, TXEN);
	
	// enable interrupt on complete reception of a byte
	SetBit(UCSRB, RXCIE);
#endif
}

/** Sends a character out the serial port. */
void serialWrite(byte data){
#ifdef UDR0
	while (bit_is_clear(UCSR0A, UDRE0))
		;
	UDR0 = data;
#else
	while (bit_is_clear(UCSRA, UDRE))
		;
	UDR = data;
#endif
}

/* = number of bytes available in buffer */
int serialAvailable(){
	return (RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail) % RX_BUFFER_SIZE;
}

/** = a byte from the buffer */
unsigned char serialRead(){
	// if the head isn't ahead of the tail, we don't have any characters
	if (rx_buffer_head == rx_buffer_tail) {
		return -1;
	} else {
		unsigned char c = rx_buffer[rx_buffer_tail];     
		rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_SIZE;
		return c;
	}
}

void serialFlush(){
	rx_buffer_head = rx_buffer_tail;
}

#ifdef UDR0
ISR(USART0_RX_vect){
	unsigned char c = UDR0;	
#else
ISR(USART_RXC_vect){
	unsigned char c = UDR;
#endif
	int i = (rx_buffer_head + 1) % RX_BUFFER_SIZE;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if (i != rx_buffer_tail) {
		rx_buffer[rx_buffer_head] = c;
		rx_buffer_head = i;
	}
}

/** transmits text via serial connection */
void Print(const char *psz){
	const char *pch;
	for (pch = psz; *pch != 0; ++pch)
		serialWrite(*pch);
}

/** Prints a number to the LCD (0-999) */
void PrintNumber(int value){
	if(value < 0){
		serialWrite('-');
		value = -value;
	}
	if(value > 999){
		value = 999;
	}
	unsigned char data[3] = "   "; 
	data[0] = value/100 + 48;
	data[1] = (value/10)%10 + 48;
	data[2] = value%10 + 48;
	serialWrite(data[0]);
	serialWrite(data[1]);
	serialWrite(data[2]);
}

/** Prints a number to the LCD (0-99999) */
void PrintInteger(int value){
	if(value < 0){
		serialWrite('-');
		value = -value;
	}
	unsigned char data[5] = "     "; 
	data[0] = value/10000 + 48;
	data[1] = (value/1000)%10 + 48;
	data[2] = ((value/100)%100)%10 + 48;
	data[3] = (((value/10)%1000)%100)%10 + 48;
	data[4] = value%10 + 48;
	serialWrite(data[0]);
	serialWrite(data[1]);
	serialWrite(data[2]);
	serialWrite(data[3]);
	serialWrite(data[4]);
}

#endif

/**********************************End of serial.h*****************************
 * REVISIONS:
 *  3-20-09 - serialRead() now returns unsigned char -MEF
 */
