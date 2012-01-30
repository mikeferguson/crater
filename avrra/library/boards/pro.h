/******************************************************************************
 * AVRRA: The AVR Robotics API
 * pro.h - device file for AVRRA Pro Board V1.1
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

/* TODO: add setTvelocity(x,angle)? 
 *       single vs. dual axis head?             
 *       add moveHead(pan, tilt)
 */
 
// Only one device file!
#ifndef AVRRA_DEV
#define AVRRA_DEV

// mega324P on PRO board
#define F_CPU 14745600L

#include <avr/interrupt.h>
#include "../utils.h"
#include "../serial.h"
#include "../sharpir.h"
#include "../srf05.h"
#include "../servo.h"
#include "../avrcam.h"
#include "../smooth.h"

// Robot specific port definitions
#define PIN_HEAD_IR		0x00		// PA[0] - HEAD IR
#define PIN_LEFT_IR		0x01		// PA[1] - LEFT IR
#define PIN_RIGHT_IR	0x02		// PA[2] - RIGHT IR
									// PA[3] 
									// PA[4] 
									// PA[5] 
									// PA[6]  
#define PIN_GO		    0x07		// PA[7] 
								
#define PIN_SONAR		0x08		// PB[0] - HEAD SONAR
#define PIN_LBMP		0x09		// PB[1] - LEFT BUMPER
#define PIN_RBMP		0x0A		// PB[2] - RIGHT BUMPER
#define PIN_RPWM		0x0B		// PB[3] - OC0A - RIGHT PWM
#define PIN_LPWM		0x0C		// PB[4] - OC0B - LEFT PWM
									// PB[5] - MOSI 
									// PB[6] - MISO 
#define PIN_BUZZER		0x0F		// PB[7] - SCK (BUZZER VIA JP1)
								
									// PC[0] - RESERVED FOR I2C
									// PC[1] - RESERVED FOR I2C
#define PIN_LENC_B		0x12		// PC[2] - LEFT ENCODER B
#define PIN_LDIR		0x13		// PC[3] - LEFT DIRECTION
#define PIN_RDIR		0x14		// PC[4] - RIGHT DIRECTION
#define PIN_RENC_B		0x15		// PC[5] - RIGHT ENCODER B
#define PIN_PROG_LED	0x16		// PC[6] - PROGRAMMING LED
#define PIN_RENC_A		0x17		// PC[7] - RIGHT ENCODER A (INT)
		
									// PD[0] - SERIAL IN
									// PD[1] - SERIAL OUT
									// PD[2] - AVRCAM IN
									// PD[3] - AVRCAM OUT
#define PIN_PAN_SERVO	0x1C		// PD[4] - OC1B - PAN SERVO
#define PIN_TILT_SERVO	0x1D		// PD[5] - OC1A - TILT SERVO
#define PIN_ALIVE		0x1E		// PD[6] - 0C2B - EYES/ALIVE LEDS
#define PIN_LENC_A		0x1F		// PD[7] - LEFT ENCODER A (INT)

#ifdef XR_SERIES
  #include "../../xrseries/xrprint.h"
  #include "../clock.h"
  #include "statepro.h"
#endif

#define LEFT 				-1
#define RIGHT   			1

/**************************Standard Motor Definitions*************************/

#define FORWARD 			1
#define BACKWARD			-1
#define FULL_SPEED			255
#define HALF_SPEED			127
#define REG_SPEED			160			// Tweak this as neccessary
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

/** This enables the closed loop control */
void closedInit(){
	// all are inputs
	digitalSetDirection(PIN_LENC_B, AVRRA_INPUT);
	digitalSetDirection(PIN_RENC_B, AVRRA_INPUT);
	digitalSetDirection(PIN_LENC_A, AVRRA_INPUT);
	digitalSetDirection(PIN_RENC_A, AVRRA_INPUT);
	
	rLastB= digitalGetData(PIN_RENC_B);
	lLastB= digitalGetData(PIN_LENC_B);
	
	PCICR = 0x0C;		// enable PC interrupt 2 & 3
	PCMSK2 = 0x80; 		// enable interrupt on PC7 (RENC_A)
	PCMSK3 = 0x80; 		// enable interrupt on PD7 (LENC_A)
}

/** Counter get/set macros */
#define getLcount 		lCount
#define getRcount 		rCount
#define clearCounters 	lCount = rCount = 0

/** right side closed loop counter */
ISR(PCINT2_vect){
	int RightA = digitalGetData(PIN_RENC_A);
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
	rLastB = RightB;	
}

/** left side closed loop counter */
ISR(PCINT3_vect){
	int LeftA = digitalGetData(PIN_LENC_A);
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
	lLastB = LeftB;
}

/** causes robot to move x inches (positive = forward) */
void moveX(int x){
	int goalR= rCount + (x * 16) - 10;
	int goalL= lCount + (x * 16) - 10;
	if(x > 0){
		motorLeft(REG_SPEED);
		motorRight(REG_SPEED);
		while((rCount < goalR) || (lCount < goalL)){
			// slow as we approach (start at 2 inches out
			/*if(rCount > (goal -32)){
				int p_term = rCount - (goal -32);
				motorLeft(REG_SPEED - (3 * p_term));
				motorRight(REG_SPEED - (3 * p_term));
			}*/
		}
	}else{
		motorLeft(BACKWARD*REG_SPEED);
		motorRight(BACKWARD*REG_SPEED);
		while((rCount > goalR) || (lCount > goalL)){
			// slow as we approach (start at 2 inches out
			/*if(rCount < (goal +32)){
				int p_term = (goal + 32) - rCount;
				motorLeft(BACKWARD*REG_SPEED - (3 * p_term));
				motorRight(BACKWARD*REG_SPEED - (3 * p_term));
			}*/
		}
	}
	motorStop();	
}

// Should we measure both, and only stop when both are above level??? (no false trips)

/** turns angle degrees (positive = right) */
void turnX(int angle){
	int goal, goalC = 0; // goal counter
	// we are stopped on entry, so this is safe...
	clearCounters;
	if(angle < 0){
		// turn left
        goal = (-angle) - 10;
		// minus 10 is slow down time (we need to PID this later)
		motorLeft(BACKWARD*REG_SPEED);
		motorRight(FORWARD*REG_SPEED);
	}else{
		goal = angle - 10;
		motorLeft(FORWARD*REG_SPEED);
		motorRight(BACKWARD*REG_SPEED);
	}
	// counting is done by interrupts...
	while(goalC != 3){
		if((lCount > goal) || (lCount < -goal)){
			//motorLeft(FORWARD,0);
			goalC|= 0x01;
		}
		if((rCount > goal) || (rCount < -goal)){
			//motorRight(FORWARD,0);
			goalC|= 0x02;
		}
	}
	motorStop();
}

/******************************INIT FUNCTIONS*****************************/

/** Initializes robot systems */
void devInit(void){
	// Setup Ports
	gp2d12Init(PIN_HEAD_IR);
	gp2d12Init(PIN_LEFT_IR);
	gp2d12Init(PIN_RIGHT_IR);
	digitalSetDirection(PIN_GO,AVRRA_INPUT);
	srf05Init(PIN_SONAR);
	//digitalSetDirection(PIN_LBMP, AVRRA_INPUT);
	//digitalSetDirection(PIN_RBMP, AVRRA_INPUT);
	digitalSetDirection(PIN_BUZZER, AVRRA_OUTPUT);
	closedInit();
	// eventually we will be reading presets from EEPROM...
	servoInit(PIN_TILT_SERVO, 167, SERVO_INVERT);
	servoInit(PIN_PAN_SERVO, 5, SERVO_NO_INVERT);	
	// sei() will be done by system later...
}

#endif

