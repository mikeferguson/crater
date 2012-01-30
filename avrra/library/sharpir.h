/*******************************************************************************
 * AVRRA: The AVR Robotics API
 * sharpir.h - device driver for using Sharp analog IR rangers
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
 ******************************************************************************/

#ifndef AVRRA_SENSOR_SHARP_IR
#define AVRRA_SENSOR_SHARP_IR

#include "analog.h"
#include "digital.h"

/*******************************gp2d120 Functions******************************/

/** Initializes GP2D120 driver system. */
void gp2shortInit(char channel){
	digitalSetDirection(channel,AVRRA_INPUT);
	digitalSetData(channel,AVRRA_LOW);
	analogInit();
}

/** = the distance to the nearest object on the requested channel in CM.*/
int gp2shortGetData(char channel){
	int sample;
	// Get data
	sample = analogGetData8(channel);
	// if the ADC reading is too low, then we are really far away from anything
	if(sample < 15)
		return 254;	// max range
	// Magic numbers to get cm
	sample= 704/(sample+3);
	// This appears to work
	return sample - 1;
}


/*******************************gp2d12 Functions*******************************/

/** Initializes GP2D12 driver system. */
void gp2d12Init(char channel){
	digitalSetDirection(channel,AVRRA_INPUT);
	digitalSetData(channel,AVRRA_LOW);
	analogInit();
}

/** = the distance to the nearest object on the requested channel in cm.*/
int gp2d12GetData(char channel){
	int sample;
	// Get data
	sample = analogGetData8(channel);
	// if the ADC reading is too low, then we are really far away from anything
	if(sample < 10)
		return 254;	// max range
	// Magic numbers to get cm
	sample= 1309/(sample-3);
	// This appears to work
	return sample - 1;
}


/*****************************gp2y0a02yk Functions*****************************/

/** Initializes GP2Y0A02YK driver system. */
void gp2longInit(char channel){
	digitalSetDirection(channel,AVRRA_INPUT);
	digitalSetData(channel,AVRRA_LOW);
	analogInit();
}

/** = the distance to the nearest object on the requested channel in CM.*/
int gp2longGetData(char channel){
	int sample;
	// Get data
	sample = analogGetData8(channel);
	// if the ADC reading is too low, then we are really far away from anything
	if(sample < 20)
		return 320;	// max range
	// Magic numbers to get cm
	sample= 2870/(sample-3);
	// This appears to work
	return sample - 1;
}

#endif

/*******************************End of sharpir.h********************************
 * REVISIONS
 * 11-08-08 - All getData functions now return type int (due to possible 
 *            overflow with gp2longGetData().
 *  3-01-09 - All getData functions now return CM readings, and 254 at max range
 */
 
