/******************************************************************************
 * AVRRA: The AVR Robotics API
 * utils.h - serveral low-level functions.
 * 
 * Copyright (c) 2004-2009, Michael E. Ferguson
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

#ifndef AVRRA_UTILS
#define AVRRA_UTILS

#include <avr/io.h>
#include <stdio.h>

#define byte 	unsigned char

#define SetBit(sfr, bit) ((sfr) |= (1 << (bit)))
#define ClearBit(sfr, bit) ((sfr) &= ~(1 << (bit)))

/** units conversions */
#define INCHES(cm)      ((cm * 4)/10)   
#define URCP(cm)        ((cm * 15)/7)

#define INCH_TO_CM(in)  ((in * 10)/4)

/** waits (pauses) for ms milliseconds */
void delayms(unsigned int ms)
{
	// this is only to be used for less than 17ms!!!!
	long cycles = F_CPU/4884L;
	int i;

	//cli();
	while (ms-- > 0)
	{
		/* FCPU/1000 clock cycles for 1ms; each time through loop
		   is 5 cycles (for loop control + nop) */
		for (i = 0; i < cycles; ++i)
			asm("nop");
	}
	//sei();
}

#endif

/**********************************End of utils.h******************************
 * REVISIONS
 *   3-01-09 - Added unit conversion functions */
