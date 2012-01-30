//***************************************************************************//
//                     Copyright 2008 Michael E. Ferguson                    //
//***************************************************************************//
// serial.h - full duplex serial driver 									 //
// This is a component of the AVRRA ROBOTIC DEVELOPMENT SYSTEM 		         //
//***************************************************************************//

#ifndef AVRRA_SERIAL_HW
#define AVRRA_SERIAL_HW

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "utils.h"

#define RX_BUFFER_SIZE	64

unsigned char rx_buffer[RX_BUFFER_SIZE];

int rx_buffer_head = 0;
int rx_buffer_tail = 0;

//*******************************serial Functions****************************//
/** initializes serial transmit and receive at baud, 8-N-1 */
void serialInit(long baud)
{
#if defined(__AVR_ATmega168__)
	UBRR0H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
	UBRR0L = ((F_CPU / 16 + baud / 2) / baud - 1);
	
	// enable rx and tx
	SetBit(UCSR0B, RXEN0);
	SetBit(UCSR0B, TXEN0);
	
	// enable interrupt on complete reception of a byte
	SetBit(UCSR0B, RXCIE0);
#else	
	UBRRH = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
	UBRRL = ((F_CPU / 16 + baud / 2) / baud - 1);
	
	// enable rx and tx
	SetBit(UCSRB, RXEN);
	SetBit(UCSRB, TXEN);
	
	// enable interrupt on complete reception of a byte
	SetBit(UCSRB, RXCIE);
#endif
	// defaults to 8-bit, no parity, 1 stop bit
}

/* Sends a character out the serial port. */
void serialWrite(byte data){
#if defined(__AVR_ATmega168__)
	while (bit_is_clear(UCSR0A, UDRE0))
		;
	UDR0 = data;
#else
	while (bit_is_clear(UCSRA, UDRE))
		;
	UDR = data;
#endif
	// added for use of LCD
	//delayms(15);
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

#if defined(__AVR_ATmega168__)
ISR(USART_RX_vect)
#else
ISR(USART_RXC_vect)
#endif
{
#if defined(__AVR_ATmega168__)
	unsigned char c = UDR0;
#else
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

/* transmits text via serial connection */
void Print(const char *psz)
{
	const char *pch;

	for (pch = psz; *pch != 0; ++pch)
		serialWrite(*pch);
}

/** Prints an eight bit (0-255) number to the LCD */
void PrintNumber(unsigned char value){
	unsigned char data[3] = "   "; 
	data[0] = value/100 + 48;
	data[1] = (value/10)%10 + 48;
	data[2] = value%10 + 48;
	serialWrite(data[0]);
	serialWrite(data[1]);
	serialWrite(data[2]);
	//serialSetData('\n');
}

/* LEGACY FUNCTIONS */ 
void serialSetData(byte data){
	serialWrite(data);
}
byte serialGetData(){
	return serialRead();
}

void ACK(){
#ifndef NO_ACK
	Print("?ACK");
	serialWrite('\n');
#endif
}

void NCK(){
#ifndef NO_ACK
	Print("?NCK");
	serialWrite('\n');
#endif
}

void sysMsg(const char *msg){
	serialWrite('?');
	Print(msg);
	serialWrite('\n');
}

void sysReading(int value){
	serialWrite('?');
	PrintNumber(value);
	serialWrite('\n');
}

#endif

//*********************************End of serial.h***************************//
// REVISIONS
// 3/15/08 - Added ACK, NCK - MEF
