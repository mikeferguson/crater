/******************************************************************************
 * AVRRA: The AVR Robotics API
 * a4.h- device file for AVRRA Mini board, revisions A & B
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
#include "../digital.h"

// Robot specific port definitions
#define PIN_L_ENC_B     0x08        // PB[0] - Left Encoder B
								    // PB[3] - MOSI   
								    // PB[4] - MISO   
						    		// PB[5] - SCK 
                                    // PB[6] - XTAL
                                    // PB[7] - XTAL

#define PIN_R_ENC_B     0x10        // PC[0] - Right Encoder B		

								    // PD[0] - Serial In
								    // PD[1] - Serial out
#define PIN_L_ENC_A     0x1A        // PD[2] - Left Encoder A (INT0)
#define PIN_R_ENC_A     0x1B        // PD[3] - Right Encoder A (INT1)
#define PIN_RDIR        0x1C		// PD[4] - Right Motor Direction
#define PIN_LPWM        0x1D        // PD[5] - OC0B - Motor LPWM
#define PIN_RPWM        0x1E        // PD[6] - OC0A - Motor RPWM
#define PIN_LDIR        0x1F		// PD[7] - Left Motor Direction

/*#ifdef XR_SERIES
  #include "../../xrseries/xrprint.h"
  #include "statea4.h"
#endif*/

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
#define RFwd()		( ClearBit(PORTB, PD4),   SetBit(TCCR0A, COM0A1), ClearBit(TCCR0A, COM0A0) )
#define RRev()		(   SetBit(PORTB, PD4),   SetBit(TCCR0A, COM0A1),   SetBit(TCCR0A, COM0A0) )
#define RStop()		( ClearBit(PORTB, PD4), ClearBit(TCCR0A, COM0A1), ClearBit(TCCR0A, COM0A0) )

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

#endif

/******************************End of dev/mini.h*******************************
 * REVISIONS: */
