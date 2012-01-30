/******************************************************************************
 * AVRRA: The AVR Robotics API
 * smooth.h - full duplex serial driver for USART1 (ATMEGA324P only)
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

#ifndef AVRRA_SERIAL1_HW
#define AVRRA_SERIAL1_HW

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "utils.h"

#ifndef RX_BUFFER_SIZE1
	#define RX_BUFFER_SIZE1	64
#endif

static unsigned char rx_buffer1[RX_BUFFER_SIZE1];

// making these volatile keeps the compiler from optimizing loops of available()
static volatile int rx_buffer_head1 = 0;
static volatile int rx_buffer_tail1 = 0;

/********************************serial Functions*****************************/

/** initializes serial transmit and receive at baud, 8-N-1 */
void serial1Init(long baud){
	//UBRR1H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
	//UBRR1L = ((F_CPU / 16 + baud / 2) / baud - 1);
	
	UBRR1H = 0;
	UBRR1L = 7;
	
	// enable rx and tx
	SetBit(UCSR1B, RXEN1);
	SetBit(UCSR1B, TXEN1);
	
	// enable interrupt on complete reception of a byte
	SetBit(UCSR1B, RXCIE1);
	// defaults to 8-bit, no parity, 1 stop bit
}

/** Sends a character out the serial port. */
void serial1Write(byte data){
	while (bit_is_clear(UCSR1A, UDRE1))
		;
	UDR1 = data;
}

/** = number of bytes available in buffer */
int serial1Available(){
	return (RX_BUFFER_SIZE1 + rx_buffer_head1 - rx_buffer_tail1) % RX_BUFFER_SIZE1;
}

/** = a byte from the buffer */
unsigned char serial1Read(){
	// if the head isn't ahead of the tail, we don't have any characters
	if (rx_buffer_head1 == rx_buffer_tail1) {
		return -1;
	} else {
		unsigned char c = rx_buffer1[rx_buffer_tail1];
		rx_buffer_tail1 = (rx_buffer_tail1 + 1) % RX_BUFFER_SIZE1;
		return c;
	}
}

void serial1Flush(){
	rx_buffer_head1 = rx_buffer_tail1;
}

ISR(USART1_RX_vect){
	unsigned char c = UDR1;

	int i = (rx_buffer_head1 + 1) % RX_BUFFER_SIZE1;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if (i != rx_buffer_tail1) {
		rx_buffer1[rx_buffer_head1] = c;
		rx_buffer_head1 = i;
	}
	
	
}

/** transmits text via serial connection */
void serial1Print(const char *psz){
	const char *pch;
	for (pch = psz; *pch != 0; ++pch)
		serial1Write(*pch);
}

#endif

/**********************************End of serial1.h****************************
 * REVISIONS */
