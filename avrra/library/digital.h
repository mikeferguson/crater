/******************************************************************************
 * AVRRA: The AVR Robotics API
 * digital.h- device driver for a low-level interface to the digital ports
 *  on an AVR. THIS FILE IS SLATED FOR A REWRITE!
 * 
 * Copyright (c) 2004-2008, Michael E. Ferguson
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

#ifndef AVRRA_DIGITAL
#define AVRRA_DIGITAL

#include <avr/io.h>

/******************************digital Definitions****************************/
#define AVRRA_INPUT		0
#define AVRRA_OUTPUT	0xFF
#define AVRRA_LOW		0
#define AVRRA_HIGH		0xFF
#define AVRRA_PULLUP    0xFF

/*******************************digital Functions*****************************/

/** Initializes digial driver system. */
void digitalInit(void){
	/* No initilization needed. Included only for compliance with
		the driver model */
}

/** = the value of the requested line (0x00|0xFF). */
unsigned char digitalGetData(char channel){
	unsigned char data= 0;	
	switch(channel/8){
#ifdef PINA
		case 0:
			data= PINA&( 1<<(channel%8) );
			break;
#endif
		case 1:
			data= PINB&( 1<<(channel%8) );
			break;
		case 2:
			data= PINC&( 1<<(channel%8) );
			break;
		case 3:
			data= PIND&( 1<<(channel%8) );
			break;
	}
	if(data>0) 
		data= 0xFF;
	return data;
}

/** Sets the value of the requested line (1|0). */
void digitalSetData(char channel, unsigned char data){
	if(data > 0){
		switch(channel/8){
#ifdef PORTA
			case 0:
				PORTA= PORTA|( 1<<(channel%8) );
				break;
#endif
			case 1:
				PORTB= PORTB|( 1<<(channel%8) );
				break;
			case 2:
				PORTC= PORTC|( 1<<(channel%8) );
				break;
			case 3:
				PORTD= PORTD|( 1<<(channel%8) );
				break;
		}
	}else{
		switch(channel/8){
#ifdef PORTA
			case 0:
				PORTA= PORTA&( 0xFF - (1<<(channel%8)) );
				break;
#endif
			case 1:
				PORTB= PORTB&( 0xFF - (1<<(channel%8)) );
				break;
			case 2:
				PORTC= PORTC&( 0xFF - (1<<(channel%8)) );
				break;
			case 3:
				PORTD= PORTD&( 0xFF - (1<<(channel%8)) );
				break;
		}
	}
}

/** still under development */
void digitalSetDirection(char channel, unsigned char dir){
	if(dir>0){
		switch(channel/8){
#ifdef DDRA
			case 0:
				DDRA= DDRA|( 1<<(channel%8) );
				break;
#endif
			case 1:
				DDRB= DDRB|( 1<<(channel%8) );
				break;
			case 2:
				DDRC= DDRC|( 1<<(channel%8) );
				break;
			case 3:
				DDRD= DDRD|( 1<<(channel%8) );
				break;
		}
	}else{
		switch(channel/8){
#ifdef DDRA
			case 0:
				DDRA= DDRA&( 0xFF - (1<<(channel%8)) );
				break;
#endif			
			case 1:
				DDRB= DDRB&( 0xFF - (1<<(channel%8)) );
				break;
			case 2:
				DDRC= DDRC&( 0xFF - (1<<(channel%8)) );
				break;
			case 3:
				DDRD= DDRD&( 0xFF - (1<<(channel%8)) );
				break;
		}
	}
}

#endif

/*******************************End of digital.h*******************************
 * REVISIONS
 *  3-5-09 MEF - defined AVRRA_PULLUP for clarity.
 */
