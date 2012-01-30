/******************************************************************************
 * AVRRA: The AVR Robotics API
 * lite.h - device file for AVRRA Lite Board V1.0
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
 
// Only one device file!
#ifndef AVRRA_BOARD
#define AVRRA_BOARD

// mega324P on LITE board
#define F_CPU 14745600L

#include <avr/interrupt.h>
#include "../utils.h"
#include "../digital.h"

// board specific port definitions
#define PIN_RPWM		0x0B		// PB[3] - OC0A - RIGHT PWM
#define PIN_LPWM		0x0C		// PB[4] - OC0B - LEFT PWM
									// PB[5] - MOSI 
									// PB[6] - MISO 
                            		// PB[7] - SCK
								
									// PC[0] - RESERVED FOR I2C
									// PC[1] - RESERVED FOR I2C
#define PIN_LENC_B		0x12		// PC[2] - LEFT ENCODER B
#define PIN_LDIR		0x13		// PC[3] - LEFT DIRECTION
#define PIN_RDIR		0x14		// PC[4] - RIGHT DIRECTION
#define PIN_RENC_B		0x15		// PC[5] - RIGHT ENCODER B
#define PIN_PROG_LED	0x16		// PC[6] - PROGRAMMING LED
#define PIN_RENC_A		0x17		// PC[7] - RIGHT ENCODER A (INT)
		
									// PD[0/1] - SERIAL 
                                    // PD[2/3] - BIOLOID
                              		// PD[4] - OC1B - 
                                	// PD[5] - OC1A - 
                                	// PD[6] - 0C2B - 
#define PIN_LENC_A		0x1F		// PD[7] - LEFT ENCODER A (INT)

#define LEFT 				-1
#define RIGHT   			1

/**************************Standard Motor Definitions*************************/

#define FORWARD 			1
#define BACKWARD			-1
#define FULL_SPEED			255
#define HALF_SPEED			127
//#define REG_SPEED			160			// Tweak this as neccessary
#define STOP				0			

int LeftSpeed;
int RightSpeed;
int SLeftSpeed;
int SRightSpeed;
/* we compare to OCR1A/B for R/L motor speeds */
#define lPWM		OCR0B
#define rPWM		OCR0A
/* set direction (input to H-bridge) and wave output mode */
#define LFwd()		( ClearBit(PORTC, PC3),   SetBit(TCCR0A, COM0B1), ClearBit(TCCR0A, COM0B0) )
#define LRev()		(   SetBit(PORTC, PC3),   SetBit(TCCR0A, COM0B1),   SetBit(TCCR0A, COM0B0) )
#define LStop()		( ClearBit(PORTC, PC3), ClearBit(TCCR0A, COM0B1), ClearBit(TCCR0A, COM0B0) )
#define RFwd()		( ClearBit(PORTC, PC4),   SetBit(TCCR0A, COM0A1), ClearBit(TCCR0A, COM0A0) )
#define RRev()		(   SetBit(PORTC, PC4),   SetBit(TCCR0A, COM0A1),   SetBit(TCCR0A, COM0A0) )
#define RStop()		( ClearBit(PORTC, PC4), ClearBit(TCCR0A, COM0A1), ClearBit(TCCR0A, COM0A0) )

/* sets up microprocessor for PWM control of motors */
void motorInit(void){
	/* set up ports */
	digitalSetDirection(PIN_LDIR,AVRRA_OUTPUT);	
	digitalSetDirection(PIN_RDIR,AVRRA_OUTPUT);
	digitalSetDirection(PIN_LPWM,AVRRA_OUTPUT);
	digitalSetDirection(PIN_RPWM,AVRRA_OUTPUT);
	TCNT0 = 0;

	/* start with motors disconnected from Timer/Counter output */
	TCCR0A = 0x03;	// 00 00 00 00
	TCCR0B = 0x03;	// (WAS OX0B); 000 01 011 (512 Hz) /64 C

	/* OCR1A/B are the values that the timer is compared to; a match will
	   cause the output to change; small values mean the motor runs for a
	   short period (slower); larger values are longer times (faster)*/
	lPWM = rPWM = 0;	// (value is irrelevant since outputs are disconnected)
}

/** pwm values can range from -255 (full-speed reverse)
    to 255 (full-speed forward), with 0 indicating a stop */
void motorLeft(int pwm){
	if (pwm == 0){
		LStop();
	}else{
		if (pwm >= 0){
			LFwd();
		}else{
			LRev();
			pwm = -pwm;
		}
		if (pwm > 255)
			pwm = 255;
		lPWM = pwm;		// set width for PWM
	}
	LeftSpeed = pwm;
}

/** pwm values can range from -255 (full-speed reverse)
    to 255 (full-speed forward), with 0 indicating a stop */
void motorRight(int pwm){
	if (pwm == 0){
		RStop();
	}else{
		if (pwm >= 0){
			RFwd();
		}else{
			RRev();
			pwm = -pwm;
		}
		if (pwm > 255)
			pwm = 255;
		rPWM = pwm;		// set width for PWM
	}
	RightSpeed = pwm;
}

/** Stops both motors */
void motorStop(void){
	SLeftSpeed = LeftSpeed;
	SRightSpeed = RightSpeed;
	LStop();
	RStop();
}

/** Restarts both motors */
void motorResume(void){
	motorRight(SRightSpeed);
	motorLeft(SLeftSpeed);
}

/**************************CLOSED-LOOP FUNCTIONS**************************/
static volatile int rCount= 0;
static volatile int rLastB= 0;
static volatile int lCount= 0;
static volatile int lLastB= 0;

/** This enables the closed loop control, still need to call sei() */
void closedInit(){
	// all are inputs
	digitalSetDirection(PIN_LENC_B, AVRRA_INPUT);
	digitalSetDirection(PIN_RENC_B, AVRRA_INPUT);
	digitalSetDirection(PIN_LENC_A, AVRRA_INPUT);
    digitalSetDirection(PIN_RENC_A, AVRRA_INPUT);	
    digitalSetData(PIN_LENC_B, AVRRA_LOW);
    digitalSetData(PIN_RENC_B, AVRRA_LOW);

	//rLastB= digitalGetData(PIN_RENC_B);
	//lLastB= digitalGetData(PIN_LENC_B);

	PCICR |= (1 << PCIE3) | (1 << PCIE2) ;  // enable PC interrupt 2 & 3
	//PCMSK2 = 0x80; 		// enable interrupt on PC7 (RENC_A)
	PCMSK3 = 0x80; 		// enable interrupt on PD7 (LENC_A)
}

/** Counter get/set macros */
#define getLcount 		lCount
#define getRcount 		rCount
#define clearCounters 	lCount = rCount = 0

/** NOTE ALL OF THIS IS HACKED UP, COUNTING IS ALWAYS POSTIVE */

/** right side closed loop counter */
ISR(PCINT2_vect){
	rCount++;
    /*int RightA = digitalGetData(PIN_RENC_A);
	int RightB = digitalGetData(PIN_RENC_B);
	// right has moved
	if(RightA == RightB){
		// CW = positive rotation
		if(rLastB == RightB){
			// 1 hit
			rCount++;
		}else{
			// 2 hits
			rCount++;
			rCount++;
		}
	}else{
		// CCW
		if(rLastB == RightB){
			// 1 hit
			rCount--;
		}else{
			// 2 hits
			rCount--;
			rCount--;
		}	
	}
	rLastB = RightB;	*/
}

/** left side closed loop counter */
ISR(PCINT3_vect){
    lCount++;
	/*int LeftA = digitalGetData(PIN_LENC_A);
	int LeftB = digitalGetData(PIN_LENC_B);
	// left has moved
	if(LeftA == LeftB){
		// CW
		if(lLastB == LeftB){
			// 1 hit
			lCount--;
		}else{
			// 2 hits
			lCount--;
			lCount--;
		}
	}else{
		// CCW = positive rotation
		if(lLastB == LeftB){
			// 1 hit
			lCount++;
		}else{
			// 2 hits
			lCount++;
			lCount++;
		}	
	}
	lLastB = LeftB;*/
}

/** causes robot to move x centimeters (positive = forward) */
void moveX(int x){
    int goal = COUNTS_CM(x);
    clearCounters;
	if(x > 0){
        motorLeft(REG_SPEED);
		motorRight(REG_SPEED);
	}else{
        goal = -goal;
        motorLeft(-REG_SPEED);
		motorRight(-REG_SPEED);
    }
    while(lCount < goal); 	
    motorStop();	
}

/** turns angle degrees (positive = right) */
void turnX(int angle){
    int goal= COUNTS_DEGREE(angle);  
    clearCounters;      
    if(angle < 0){
        goal = -goal;
		motorLeft(-130);
		motorRight(130);
    }else{
		motorLeft(130);
		motorRight(-130);
    }
    while(lCount < goal); 
    motorStop();
    /*goal = -1 * COUNTS_DEGREE(goal);
	clearCounters;
    if(angle < 0){
		motorLeft(-130);
		motorRight(130);
        while(rCount < goal); 
	}else{
		motorLeft(130);
		motorRight(-130);
        while(rCount > goal); 
    }*/
}

#endif

