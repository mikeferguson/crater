/******************************************************************************
 * AVRRA: The AVR Robotics API
 * maxbotix.h- device driver Maxbotix EZ line of sensors
 * 
 * Copyright (c) 2009, Michael E. Ferguson
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

#ifndef AVRRA_SENSOR_MAXBOTIX
#define AVRRA_SENSOR_MAXBOTIX

#include "digital.h"
#include "analog.h"

/*******************************maxbotix Functions****************************/

/** Initialize, if in free-ranging mode, call with control == analog */
void maxbotixInit(char control, char analog){
    // Set control to output
	digitalSetDirection(control,AVRRA_OUTPUT);
	digitalSetData(control,AVRRA_LOW);  
    // set analog to input
	digitalSetDirection(analog,AVRRA_INPUT);
	digitalSetData(analog,AVRRA_LOW);
	analogInit();
}

/** Needs to be called before getData, unless in free-ranging mode */
void maxbotixPing(char control){
    digitalSetData(control, AVRRA_HIGH);
    delayms(25);
    digitalSetData(control, AVRRA_LOW);
}

/** = sample from channel, in cm */
int maxbotixGetData(char analog){
    return (analogGetData10(analog)*5)/4;
}

#endif

/*******************************End of maxbotix.h******************************
 * REVISIONS: */


