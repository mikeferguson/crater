/******************************************************************************
 * AVRRA: The AVR Robotics API
 * analog.h - device driver for a low-level 8/10-bit interface to the analog 
 *  ports on an AVR 
 *  NOTE: ports must be configured as floating inputs in order to use the ADC
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

#ifndef AVRRA_ANALOG
#define AVRRA_ANALOG

#include <avr/io.h>
#include "utils.h"

/********************************analog Functions*****************************/

/** Initializes onboard ADC to give 10-bit results 0-5VDC  */
void analogInit(void){
	ADMUX = 0x00;	// Aref(5V)
	ADCSRA = 0x86;	// enable, no auto trigger, no interrupt, clk/64
}

/** = sample from channel(0-7), with zeroed range (0V=0, 5V=1023) */
int analogGetData10(char channel){
	ADMUX &= ~0x1F;		// clear channel selection (low 5 bits)
	ADMUX |= channel%8;	// select specified channel
	
	SetBit(ADCSRA, ADSC);	// ADC start conversion
	
	while (bit_is_clear(ADCSRA, ADIF))
		;

	return ADC;
}

/** = sample from channel(0-7), with zeroed range (0V=0, 5V=255) */
unsigned char analogGetData8(byte channel){
	return (unsigned char) (analogGetData10(channel)/4);
}

#endif
/********************************End of analog.h*******************************
 * REVISIONS:
 *  2-13-09 - merged with analog10.h, defaults to 10-bit precision
 */
