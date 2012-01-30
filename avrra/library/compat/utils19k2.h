//***************************************************************************//
//                     Copyright 2008 Michael E. Ferguson                    //
//***************************************************************************//
// utils.h - serveral low-level functions, also includes device driver for 	 //
//		using serial port for debug (at 19.2K)		 						 //
// This is a component of the AVRRA ROBOTIC DEVELOPMENT SYSTEM 		         //
//***************************************************************************//

#ifndef AVRRA_UTILS
#define AVRRA_UTILS

#include <avr/io.h>
#include <stdio.h>


//*********************************some Macros*******************************//
#define SetBit(sfr, bit) ((sfr) |= (1 << (bit)))
#define ClearBit(sfr, bit) ((sfr) &= ~(1 << (bit)))

/* waits (pauses) for ms milliseconds (assumes internal clock at 8MHz) */
void delayms(unsigned int ms)
{
	int i;

	while (ms-- > 0)
	{
		/* 8192 (8k) clock cycles for 1ms; each time through loop
		   is 5 cycles (for loop control + nop) */
		for (i = 0; i < 1638; ++i)
			asm("nop");
	}
}


//********************************math Functions*****************************//
/** = absolute value */
int abs(int i){
	if(i < 0)
		return -i;
	return i;
}


//*******************************serial Functions****************************//
/** initializes serial transmit and receive at 19.2k baud, 8-N-1 */
void serialInit()
{
	SetBit(UCSRB, TXEN);	// enable transmit
	SetBit(UCSRB, RXEN);	// enable receive

	/* UBRR = 25 gives us 19.2k baud */
	UBRRH = 0;
	UBRRL = 25;		// change to 12 for 38.4k baud

	/* set UCSRC (not UBRRH), asynch, no parity, 1 stop bit, 8 data bits */
	UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);
}

/* = one character from the recieve buffer. (0 if none) */
char serialGetData(){
	if (bit_is_clear(UCSRA, RXC))
		return 0;
	return UDR;
}

/* Sends a character out the serial port. */
void serialSetData(char data){
	while (bit_is_clear(UCSRA, UDRE))
		;
	UDR = data;
	// added for use of LCD
	delayms(15);
}

/* transmits text via serial connection */
void Print(const char *psz)
{
	const char *pch;

	for (pch = psz; *pch != 0; ++pch)
		serialSetData(*pch);
}

/** Prints an eight bit (0-255) number to the LCD */
void PrintNumber(unsigned char value){
	unsigned char data[3] = "   "; 
	data[0] = value/100 + 48;
	data[1] = (value/10)%10 + 48;
	data[2] = value%10 + 48;
	serialSetData(data[0]);
	serialSetData(data[1]);
	serialSetData(data[2]);
	//serialSetData('\n');
}

/** Prints an eight bit (0-255) number to the LCD */
void PrintNumber4(int value){
	unsigned char data[4] = "    "; 
	if(value < 0){
		serialSetData('-');
		value = -value;
	}else{
        serialSetData(' ');
    }   
    data[0] = value/1000 + 48;
	data[1] = ((value/100)%10) + 48;
	data[2] = ((value/10)%10) + 48;
	data[3] = (value%10) + 48;
	serialSetData(data[0]);
	serialSetData(data[1]);
	serialSetData(data[2]);
	serialSetData(data[3]);
	//serialSetData('\n');
}

#endif

//*******************************End of utils19k2.h**************************//
// REVISIONS
