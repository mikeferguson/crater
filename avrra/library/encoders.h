/******************************************************************************
 * AVRRA: The AVR Robotics API
 * encoders.h - device driver for quadrature encoders, 2 channels
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

/* USER: you should define two macros to convert CPI and CPD, as well as pins
 *   for L_ENC_A, L_ENC_B, R_ENC_A, R_ENC_B:
 *   
 *   #define COUNTS_INCH(i)      (246 * (i)) *   #define COUNTS_DEGREE(a)    ((27 * (a))/2)
 * 
 *   #define L_ENC_A             0x1A   // a pin number
 * 
 * TODO: redo the encoder pin definitions.
 */

#ifndef AVRRA_ENCODERS
#define AVRRA_ENCODERS

#include <avr/interrupt.h>
#include "digital.h"

// these were static!!!
volatile int rCount;
volatile int lCount;

/* This enables the closed loop control */
void encodersInit(void){
	// all are inputs
	digitalSetDirection(L_ENC_B, AVRRA_INPUT);
	digitalSetDirection(R_ENC_B, AVRRA_INPUT);
	digitalSetDirection(L_ENC_A, AVRRA_INPUT);
	digitalSetDirection(R_ENC_A, AVRRA_INPUT);
    digitalSetData(L_ENC_B, AVRRA_LOW);
    digitalSetData(R_ENC_B, AVRRA_LOW);
    
    // enable rising edge interrupt on INT0 (L_ENC_A)
    //      & rising edge interrupt on INT1 (R_ENC_A)
    MCUCR |= 0x0F;
    GICR |= 0xC0;
    //GICR |= 0x40;
    // NOTE: we still need to globally enable interrupts
}

/* Counter get/set macros */
#define getLcount 		lCount
#define getRcount 		rCount
#define clearCounters 	lCount = rCount = 0

/* right side closed loop counter */
ISR(INT0_vect){
	//if(digitalGetData(R_ENC_B) > 0){
    // digitalGetData is too slow - hard code it...
    if(PINC&0x20){
        // going backward?
        rCount--;
    }else{
        rCount++;
    }
}

/* left side closed loop counter */
ISR(INT1_vect){
    //if(digitalGetData(L_ENC_B) > 0){
    if(PINC&0x04){
        // going forward?
        lCount++;
    }else{
        lCount--;
    }
}

/** causes robot to move x inches (positive = forward) */
void moveX(int x){
	int goalR= rCount + (x * COUNTS_PER_INCH) - OVERSHOOT_COMP;
	//int goalL= lCount + (x * COUNTS_PER_INCH) - OVERSHOOT_COMP;
    sei();
	if(x > 0){
		motorLeft(FORWARD,REG_SPEED);
		motorRight(FORWARD,REG_SPEED);
		//while((rCount < goalR) && (lCount < goalL)){
		while(rCount < goalR){
            // slow as we approach (start at 2 inches out
			if(rCount > (goalR -600)){
				int p_term = rCount - (goalR -600);
				motorLeft(FORWARD,REG_SPEED - (p_term/3));
				motorRight(FORWARD,REG_SPEED - (p_term/3));
			}
		}
	}else{
		motorLeft(BACKWARD,REG_SPEED);
		motorRight(BACKWARD,REG_SPEED);
        while(rCount < goalR);
	}
	motorStop();
    cli();
}

/** turns angle degrees (positive = right) */
void turnX(int angle){
	int goal;//, goalC = 0; // goal counter
	// we are stopped on entry, so this is safe...
	clearCounters;
    sei();
    goal = angle * COUNTS_PER_DEGREE;
	if(angle < 0){
        goal = -goal;
		motorLeft(BACKWARD,110);
		motorRight(FORWARD,110);
	}else{
		motorLeft(FORWARD,110);
		motorRight(BACKWARD,110);
    }
    while(rCount < (goal-ANGLE_OVERSHOOT));
	motorStop();
    cli();
}
#endif
