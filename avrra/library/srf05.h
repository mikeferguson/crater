/******************************************************************************
 * AVRRA: The AVR Robotics API
 * srf05.h- device driver for Devantech SRF-05 in mode 2 (single wire)
 *  THIS VERSION IS FOR F_CPU = 14745600 (AVRRA MINI/LITE BOARD)
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

// not freq. dependent, disables interrupts, terrible code in all...

#ifndef AVRRA_SENSOR_SRF05
#define AVRRA_SENSOR_SRF05

#include "digital.h"

/*********************************sonar Functions*****************************/

/** Does nothing, provided only for compliance with driver model */
void srf05Init(char channel){
	
}

// ACTUALLY IN INCHES RIGHT NOW - MAKE DECISION LATER ON WHAT SUPPORT
/** = sample from channel, in URCP (64p per ft) */
unsigned char srf05GetData(char channel){
	int sample;
	// Set channel to output
	digitalSetDirection(channel, AVRRA_OUTPUT);
	digitalSetData(channel, AVRRA_HIGH);
	// 15us Ping
	for (sample = 0; sample < 30; ++sample)	
		asm("nop");
	digitalSetData(channel, AVRRA_LOW);
	// Set channel to input
	digitalSetDirection(channel, AVRRA_INPUT);
	// wait for channel to go high
	cli();
	while(digitalGetData(channel)==AVRRA_LOW){};
	// when channel goes high, start counting
	sample = 0;
	while(digitalGetData(channel)==AVRRA_HIGH){
		asm("nop");
		asm("nop");
		sample++;
	}
	// convert numbers (uS/148 = inch)
	// do some debug here to find conversion
	
	// @8MHz - 32 loops = 3in  = 444uS	--> 14uS/cy
	//		   110loops = 12in = 1776us --> 16uS/cy
	//		   161loops = 18in = 2664us --> 17uS/cy
	//		   275loops = 30in = 4440us --> 16uS/cy
	//		   Calling it 16uS/cycle
	// convert numbers (uS/148 = inch)
	// 		   #cycles/9 =inches
	
	// WITH F_CPU = 14745600
	//  
	
	sei();
	sample = sample/17;
	if(sample>100)
		sample = 100;
		
	return (unsigned char) sample;
}

#endif

/*********************************End of srf05.h********************************
 * REVISIONS: */
