/******************************************************************************
 * AVRRA: The AVR Robotics API
 * a4.h- device file for XR-A4 robot, a simple ATMEGA168 powered robot.
 *   See documentation on robot at: http://www.blunderingbotics.com/xra4.php
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

// Only one device file
#ifndef AVRRA_DEV
#define AVRRA_DEV

#define F_CPU 14745600

#include "../utils.h"
#include "../serial.h"
#include "../sharpir.h"
#include "../smooth.h"

// Robot specific port definitions
#define PIN_RDIR        0x08        // PB[0] - Right Motor Direction
#define PIN_LBMP        0x09		// PB[1] - Left Bumper    
#define PIN_RBMP        0x0A		// PB[2] - Right Bumper
								    // PB[3] - MOSI   
								    // PB[4] - MISO   
						    		// PB[5] - SCK 
                                    // PB[6] - XTAL
                                    // PB[7] - XTAL
								
#define PIN_HEAD_IR	    0x10		// PC[0] - IR Ranger
#define PIN_RIGHT_IR    0x11        // PC[1] - IR Ranger
                                    // PC[2] - 
                                    // PC[3] - 
                                    // PC[4] - 
                                    // PC[5] - 
		
								    // PD[0] - Serial In
								    // PD[1] - Serial out
#define PIN_ENC_A       0x1A        // PD[2] - INT0 Enc
#define PIN_ENC_B       0x1B        // PD[3] - INT1 Enc
#define PIN_PROG_LED    0x1C		// PD[4] - Programming LED
#define PIN_LPWM        0x1D        // PD[5] - OC0B - Motor LPWM
#define PIN_RPWM        0x1E        // PD[6] - OC0A - Motor RPWM
#define PIN_LDIR        0x1F		// PD[7] - Left Motor Direction

#define SetupLed() SetBit(DDRD, DDD4)	// set direction for D4 as output
#define LedOff()   SetBit(PORTD, PD4)	// set D4 to high to turn off LED
#define LedOn()    ClearBit(PORTD, PD4)	// set D4 to low to turn on LED

#ifdef XR_SERIES
  #include "../../xrseries/xrprint.h"
  #include "statea4.h"
#endif

#define LEFT				-1
#define RIGHT				1

/**************************Standard Motor Definitions*************************/

#define FORWARD 			1
#define BACKWARD			-1
#define FULL_SPEED			255
#define HALF_SPEED			127
#define REG_SPEED			140			// Tweak this as neccessary
#define STOP				0			

int LeftSpeed;
int RightSpeed;
int SLeftSpeed;
int SRightSpeed;

/** we compare to OCR1A/B for R/L motor speeds */
#define lPWM		OCR0B
#define rPWM		OCR0A

/** set direction (input to H-bridge) and wave output mode */
#define LFwd()		( ClearBit(PORTD, PD7),   SetBit(TCCR0A, COM0B1), ClearBit(TCCR0A, COM0B0) )
#define LRev()		(   SetBit(PORTD, PD7),   SetBit(TCCR0A, COM0B1),   SetBit(TCCR0A, COM0B0) )
#define LStop()		( ClearBit(PORTD, PD7), ClearBit(TCCR0A, COM0B1), ClearBit(TCCR0A, COM0B0) )
#define RFwd()		( ClearBit(PORTB, PB0),   SetBit(TCCR0A, COM0A1), ClearBit(TCCR0A, COM0A0) )
#define RRev()		(   SetBit(PORTB, PB0),   SetBit(TCCR0A, COM0A1),   SetBit(TCCR0A, COM0A0) )
#define RStop()		( ClearBit(PORTB, PB0), ClearBit(TCCR0A, COM0A1), ClearBit(TCCR0A, COM0A0) )

/** Initializes motors */
void motorInit(void){
	/* set up ports */
	digitalSetDirection(PIN_LDIR,AVRRA_OUTPUT);	
	digitalSetDirection(PIN_RDIR,AVRRA_OUTPUT);
	digitalSetDirection(PIN_LPWM,AVRRA_OUTPUT);
	digitalSetDirection(PIN_RPWM,AVRRA_OUTPUT);
	TCNT0 = 0;

	/* start with motors disconnected from Timer/Counter output */
	TCCR0A = 0x03;	// 00 00 00 11
	TCCR0B = 0x03;	// 000 00 011 (900 Hz) /64 C

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

/** Set both motors */
void motors(int lpwm, int rpwm){
	motorLeft(lpwm);
	motorRight(rpwm);
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

/******************************Encoder Functions******************************/
static volatile int eCount= 0;
static volatile int eLastB= 0;

/** This enables the closed loop control */
void closedInit(){
	// all are inputs
	digitalSetDirection(PIN_ENC_A, AVRRA_INPUT);
	digitalSetDirection(PIN_ENC_B, AVRRA_INPUT);
	
	eLastB= digitalGetData(PIN_ENC_B);
	
    // enable interrupts on INT0
    EICRA = 0x01;
    EIMSK = 0x01;
}

/** Counter get/set macros */
#define getEcount 		eCount
#define clearCounters 	eCount = 0

/** closed loop counter */
ISR(INT0_vect){
	int encA = digitalGetData(PIN_ENC_A);
	int encB = digitalGetData(PIN_ENC_B);
	// enc has moved
	if(encA == encB){
		// CW = positive rotation
		if(eLastB == encB){
			// 1 hit
			eCount++;
		}else{
			// 2 hits
			eCount++;
			eCount++;
		}
	}else{
		// CCW
		if(eLastB == encB){
			// 1 hit
			eCount--;
		}else{
			// 2 hits
			eCount--;
			eCount--;
		}	
	}
	eLastB = encB;	
}

/** causes robot to move x inches (positive = forward) *
void moveX(int x){
	int goal= eCount + x;   // make this correct!!!
	if(x > 0){
		motorLeft(FORWARD,REG_SPEED);
		motorRight(FORWARD,REG_SPEED);
		while((rCount < goalR) || (lCount < goalL));
	}else{
		motorLeft(BACKWARD,REG_SPEED);
		motorRight(BACKWARD,REG_SPEED);
		while((rCount > goalR) || (lCount < goalL));
	}
	motorStop();	
}*/

/** turns angle degrees (positive = left) *
void turnX(int angle){
	int goal, goalC = 0; // goal counter
	// we are stopped on entry, so this is safe...
	clearCounters;
    //rCount = 0;
	//lCount = 0;
	if(angle > 0){
		// turn left
		goal = angle - 10;
		// minus 10 is slow down time (we need to PID this later)
		motorLeft(BACKWARD*REG_SPEED);
		motorRight(REG_SPEED);
	}else{
		goal = (-angle) - 10;
		motorLeft(REG_SPEED);
		motorRight(BACKWARD*REG_SPEED);
	}
	// counting is done by interrupts...
	while(goalC != 3){
		if((lCount > goal) || (lCount < -goal)){
			//motorLeft(0);
			goalC|= 0x01;
		}
		if((rCount > goal) || (rCount < -goal)){
			//motorRight(0);
			goalC|= 0x02;
		}
	}
	motorStop();
}*/

/*******************************Init Functions********************************/

/** Initializes the robot systems */
void devInit(void){
	// Debugging LED setup
	SetupLed();
	LedOff();
	// Set up port for start switch
	//digitalSetData(START,AVRRA_LOW);
	//digitalSetDirection(START,AVRRA_INPUT);
	// Set up IR
	gp2longInit(PIN_HEAD_IR);
    gp2d12Init(PIN_RIGHT_IR);
}

#endif

/*******************************End of dev/a4.h********************************
 * REVISIONS
 * 2/3/08 - motorX now takes unsigned char speed
 * 4/22/08 - updated to conform to new model
 * 11/08/08 - motorLeft,Right are now single parameter functions */
