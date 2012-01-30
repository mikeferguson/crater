//***************************************************************************//
//                     Copyright 2008 Michael E. Ferguson                    //
//***************************************************************************//
// utils.h - serveral low-level functions, 			 						 //
// This is a component of the AVRRA ROBOTIC DEVELOPMENT SYSTEM 		         //
//***************************************************************************//

#ifndef AVRRA_UTILS
#define AVRRA_UTILS

#include <avr/io.h>
#include <stdio.h>

#define byte 	unsigned char

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

#endif

//*********************************End of utils.h****************************//
// REVISIONS
