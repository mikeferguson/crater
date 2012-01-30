/******************************************************************************
 * AVRRA: The AVR Robotics API
 * servo.h - device driver for 2 hobby servos on Timer1 (PD4/PD5) (PB1/PB2 on 
 *  m168) THIS VERSION IS FOR F_CPU = 14745600 (AVRRA MINI/LITE BOARD)
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

#ifndef AVRRA_SERVO
#define AVRRA_SERVO

#include "digital.h"

#define SERVO_CENTER	346
#define SERVO_LEFT		230
#define SERVO_RIGHT		461

#define SERVO_INVERT	1
#define SERVO_NO_INVERT	0

#ifdef __AVR_ATmega168__
#define SERVO_A         0x09        // servo output A is on PB1
#define SERVO_B         0x0A        // servo output B is on PB2
#else
#define SERVO_A			0x1D		// servo output A is on PD5
#define SERVO_B			0x1C		// servo output B is on PD4
#endif

int ServoPos[2];					// positions for servos
int ServoInv[2];					// invert output?
int ServoAdj[2];					// center adjust for each
int ServoNoWait=1;

/*********************************servo Functions*****************************/

int servoDiff(int pos1, int pos2){
	if((pos1 + 90) > (pos2 + 90)){
		return (pos1+90) - (pos2+90);
	}else{
		return (pos2+90) - (pos1+90);
	}
}
	
/** Sets position of servo */
void servoSetPosition(int channel, int position){
	int pos = position*5;
	pos = pos/2;
	if(channel == SERVO_B){
		// OC1B
		if(ServoInv[1] > 0){
			pos = - pos;
		}
		OCR1B = SERVO_CENTER + ServoAdj[1] + pos;
		// delay here, 3ms per degree	
        if(ServoNoWait)
        #ifdef TGT_HAS_CLOCK
		    wait(6 * servoDiff(ServoPos[1], position));
        #else   
            delayms(6 * servoDiff(ServoPos[1], position));
        #endif
		ServoPos[1] = position;
	}else if(channel == SERVO_A){
		// OC1A
		if(ServoInv[0] > 0){
			pos = - pos;
		}
		OCR1A = SERVO_CENTER + ServoAdj[0] + pos;
		// delay here, 3ms per degree
		#ifdef TGT_HAS_CLOCK
		    wait(6 * servoDiff(ServoPos[0], position));
        #else   
            delayms(6 * servoDiff(ServoPos[0], position));
        #endif
		ServoPos[0] = position;
	}
}

/** Starts the servo driver for channel*/
void servoInit(int channel, int centerAdj, int invert){
	// turn on channel
	digitalSetDirection(channel, AVRRA_OUTPUT);
	
	// start timer 1
	ICR1 = 5760;		// upper threshold for counting,gives 40Hz update rate
	TCCR1B = 0x1B;		// use clk/64
	TCNT1 = 0;
	
	// set position
	//servoSetPosition(channel, position);
	
	// start outputs
	if(channel == SERVO_B){
		// enable PD4/OC1B
		ServoInv[1] = invert;
		ServoAdj[1] = centerAdj;
		OCR1B = SERVO_CENTER + ServoAdj[1];
		TCCR1A |= 0x22;
	}else{
		// enable PD5/OC1A
		ServoInv[0] = invert;
		ServoAdj[0] = centerAdj;
		OCR1A = SERVO_CENTER + ServoAdj[0];
		TCCR1A |= 0x82;		
	}	
}

/** Sets the center for servo *
  center is now set during initilization
void setCenter(int channel, int data){
	if(channel == SERVO_B){
		// OC1B
		state[TLTC] = data;
	}else if(channel == SERVO_A){
		// OC1A
		state[PANC] = data;
	}
}*/

/** = position of servo */
int servoGetPosition(int channel){
	if(channel == SERVO_B){
		return ServoPos[1];
	}else if(channel == SERVO_A){
		return ServoPos[0];
	}else{
		return 0;
	}
}

#endif

/*********************************End of servo.h********************************
 * REVISIONS:
 *  1-28-09 - No longer requires clock library - MEF
 *  1-28-09 - Added ServoNoWait - MEF */
