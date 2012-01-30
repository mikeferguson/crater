/******************************************************************************
 * CRATER: a low-cost fire fighting robot.
 *  this is the special device file, defines hardware used. 
 * 
 * Copyright (c) 2008-2009, Michael E. Ferguson
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
 * - Neither the name of the copyright holders nor the names of
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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
#ifndef AVRRA_DEV
#define AVRRA_DEV

#define F_CPU 8000000

#include <avr/interrupt.h>
#include "avrra/library/digital.h"
#include "avrra/library/utils.h"
#include "avrra/library/sharpir.h"
#include "avrra/library/smooth.h"

// Robot specific port definitions
#define PIN_LCD_E       0x08        // PB[0] - Right Motor Direction
#define PIN_PAN_SERVO   0x09		// PB[1] - Pan Servo   
#define PIN_FAN         0x0A		// PB[2] - Fan output
#define PIN_LCD_RS      0x0B        // PB[3] - MOSI
#define PIN_LCD_D4      0x0C	    // PB[4] - MISO   
#define PIN_LCD_D5      0x0D		// PB[5] - SCK 
#define PIN_LCD_D6      0x0E        // PB[6] - 
#define PIN_LCD_D7      0x0F        // PB[7] - 
								
#define PIN_HEAD_IR	    0x10		// PC[0] - IR Ranger
#define PIN_PHOTO       0x11        // PC[1] - Photodiode L
#define PIN_IR          0x12        // PC[2] - 
#define PIN_START       0x13        // PC[3] - 
#define PIN_RENC_B      0x14        // PC[4] - Right Encoder B
#define PIN_LENC_B      0x15        // PC[5] - Left Encoder B
		
                                    // PD[0] - Serial In
								    // PD[1] - Serial out
#define PIN_RENC_A      0x1A        // PD[2] - INT0 Right Encoder A
#define PIN_LENC_A      0x1B        // PD[3] - INT1 Left Encoder A
#define PIN_RDIR        0x1C		// PD[4] - RIGHT DIRECTION
#define PIN_LPWM        0x1D        // PD[5] - OC0B - Motor LPWM
#define PIN_RPWM        0x1E        // PD[6] - OC0A - Motor RPWM
#define PIN_LDIR        0x1F		// PD[7] - Left Motor Direction

#include "lcd.h"

#define LEFT				-1
#define RIGHT				1

// 141:1 Gear Motors, 141CPR Encoders, 2.75" Wheels = 6.5C/CM
#define COUNTS_CM(i)        ((13 * (i))/2)
#define COUNTS_DEGREE(a)    ((15 * (a))/18)   // was 8/9
// Adjust this...
#define ANGLE_OVERSHOOT     0
#define OVERSHOOT_COMP      0

#define     GET_HEAD_IR     smooth(&gp2d12GetData,PIN_HEAD_IR)

/**************************Standard Motor Definitions*************************/

#define FORWARD 			1
#define BACKWARD			-1
#define FULL_SPEED			255
#define HALF_SPEED			127
#define REG_SPEED			200			// Tweak this as neccessary
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
#define RFwd()		( ClearBit(PORTD, PD4),   SetBit(TCCR0A, COM0A1), ClearBit(TCCR0A, COM0A0) )
#define RRev()		(   SetBit(PORTD, PD4),   SetBit(TCCR0A, COM0A1),   SetBit(TCCR0A, COM0A0) )
#define RStop()		( ClearBit(PORTD, PD4), ClearBit(TCCR0A, COM0A1), ClearBit(TCCR0A, COM0A0) )

/** Initializes motors 
        some motor code adapted from original ARC-board code by Larry Barello */
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
inline void motors(int lpwm, int rpwm){
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

// these were static!!!
volatile int rCount;
volatile int lCount;

/* This enables the closed loop control */
void encodersInit(void){
	// all are inputs
	digitalSetDirection(PIN_LENC_B, AVRRA_INPUT);
	digitalSetDirection(PIN_RENC_B, AVRRA_INPUT);
	digitalSetDirection(PIN_LENC_A, AVRRA_INPUT);
	digitalSetDirection(PIN_RENC_A, AVRRA_INPUT);
    digitalSetData(PIN_LENC_B, AVRRA_LOW);
    digitalSetData(PIN_RENC_B, AVRRA_LOW);

    // enable rising edge interrupt on INT0 (R_ENC_A)
    //      & rising edge interrupt on INT1 (L_ENC_A)
    EICRA |= 0x0F;      // rising edge
    EIMSK |= 0x01;      // enable mask (0x03 to enable left also)
    // NOTE: we still need to globally enable interrupts
}

/* Counter get/set macros */
#define getLcount 		lCount
#define getRcount 		rCount
#define clearCounters 	lCount = 0; rCount = 0

/* right side closed loop counter */
ISR(INT0_vect){
    rCount++;
}

/* left side closed loop counter */
ISR(INT1_vect){
    lCount++;
}

/** causes robot to move x CM (positive = forward) */
void moveX(int x){
    int goalR = COUNTS_CM(x);
    clearCounters;
	if(x > 0){
		motorLeft(REG_SPEED);
		motorRight(REG_SPEED);
	}else{
		motorLeft(-REG_SPEED);
		motorRight(-REG_SPEED);
	}
    while(rCount < goalR);
	motorStop();
}

/** turns angle degrees (positive = right) */
void turnX(int angle){
	int goal= -1 * COUNTS_DEGREE(angle);  
	clearCounters;    
    if(angle < 0){
		motorLeft(-160);
		motorRight(160);
	}else{
        goal = -goal;
		motorLeft(160);
		motorRight(-160);
    }
    while(rCount < goal);
	motorStop();
}

/*********************************servo Functions*****************************/
int ServoPos;	// position for pan servo (PB1/OC1A)
#define SERVO_CENTER	160

int servoDiff(int pos1, int pos2){
	if((pos1 + 90) > (pos2 + 90)){
		return (pos1+90) - (pos2+90);
	}else{
		return (pos2+90) - (pos1+90);
	}
}
	
/** Sets position of servo (position in degrees) */
void servoSetPosition(int position){
	int pos = (position*5)/4;
    OCR1A = SERVO_CENTER + pos;
    // delay here, 3ms per degree
    delayms(3 * servoDiff(ServoPos, position));
	ServoPos = position;
}

/** Starts the servo driver for channel*/
void servoInit(){
	// turn on channel
	digitalSetDirection(PIN_PAN_SERVO, AVRRA_OUTPUT);
	
	// start timer 1
	ICR1 = 3124;		// upper threshold for counting,gives 40Hz update rate
	TCCR1B = 0x1B;		// use clk/64    00011011
	TCNT1 = 0;

	// enable PB1/OC1A
	OCR1A = SERVO_CENTER;
	TCCR1A |= 0x82;		
}

/** = position of servo */
int servoGetPosition(){
	return ServoPos;
}

#endif

/*******************************End of crater.h********************************
 * REVISIONS
 * 12/29/08 - Created from a4.h
 * 1/2/09 - added servo code, C/CM, CPD are now functional macros
 * 4/1/09 - moved encoder code into this file
 */

