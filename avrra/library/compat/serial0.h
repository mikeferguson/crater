//***************************************************************************//
//                     Copyright 2008 Michael E. Ferguson                    //
//***************************************************************************//
// serial0.h - full duplex serial driver for USART0							 //
// This is a component of the AVRRA ROBOTIC DEVELOPMENT SYSTEM 		         //
//***************************************************************************//
// CURRENTLY ONLY FOR 324P

#ifndef AVRRA_SERIAL0_HW
#define AVRRA_SERIAL0_HW

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

//*******************************serial Functions****************************//
/** initializes serial transmit and receive at baud, 8-N-1 */
void serialInit(long baud)
{
	//UBRR0H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
	//UBRR0L = ((F_CPU / 16 + baud / 2) / baud - 1);
	
	UBRR0H = 0;
	UBRR0L = 7;
	
	// enable rx and tx
	SetBit(UCSR0B, RXEN0);
	SetBit(UCSR0B, TXEN0);
	
	// enable interrupt on complete reception of a byte
	SetBit(UCSR0B, RXCIE0);
	// defaults to 8-bit, no parity, 1 stop bit
}

/* Sends a character out the serial port. */
void serialWrite(byte data){
	while (bit_is_clear(UCSR0A, UDRE0))
		;
	UDR0 = data;
}

/* = number of bytes available in buffer */
int serialAvailable()
{
	return (RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail) % RX_BUFFER_SIZE;
}

/* = a byte from the buffer */
signed char serialRead()
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (rx_buffer_head == rx_buffer_tail) {
		return -1;
	} else {
		unsigned char c = rx_buffer[rx_buffer_tail];
		rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_SIZE;
		return c;
	}
}

void serialFlush()
{
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of rx_buffer_head but before writing
	// the value to rx_buffer_tail; the previous value of rx_buffer_head
	// may be written to rx_buffer_tail, making it appear as if the buffer
	// were full, not empty.
	rx_buffer_head = rx_buffer_tail;
}

ISR(USART0_RX_vect){
	unsigned char c = UDR0;

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

/* transmits text via serial connection */
void Print(const char *psz)
{
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
	//serialSetData('\n');
}

#endif

//*********************************End of serial0.h**************************//
// REVISIONS
// 3/21/08 - Created from serial.h
// 4/15/08 - Print number now takes an int
// 5/1/08  - Print number now handles negative numbers
// 5/3/08  - serial0Init now serialInit
