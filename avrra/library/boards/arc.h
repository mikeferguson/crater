//***************************************************************************//
//                   Copyright 2004-2008 Michael E. Ferguson                 //
//***************************************************************************//
// arc.h - device driver for using H-Bridge,LED on ARC1.1 PCB  				 //
// This is a component of the AVRRA ROBOTIC DEVELOPMENT SYSTEM   		     //
//***************************************************************************//
// Base code is from Seattle Robotics Society Workshop Code Utils.h

// TODO: THIS CODE DOES NOT FOLLOW THE NEWER MOTOR SYSTEM

#ifndef AVRRA_ARC
#define AVRRA_ARC

#define F_CPU 8000000

#include "..\utils19k2.h"

//******************************LED on ARC V1.1******************************//
#define SetupLed() SetBit(DDRB, DDB4)	// set direction for B4 as output
#define LedOff()   SetBit(PORTB, PB4)	// set B4 to high to turn off LED
#define LedOn()    ClearBit(PORTB, PB4)	// set B4 to low to turn on LED

//**************************Legacy Motor Definitions*************************//

#define FORWARD 			1
#define BACKWARD			0
#define FULL_SPEED			255
#define HALF_SPEED			127
#define REG_SPEED			200			// Tweak this as neccessary
#define STOP				0			

char LeftSpeed;
char RightSpeed;
char LeftDir;
char RightDir;
char SLeftSpeed;
char SRightSpeed;
char SLeftDir;
char SRightDir;

/*
*	motor control via H-bridge IC and PWM using Timer/Counter 1
*/

/*	A motor will turn when its inputs differ. The direction depends on which
	input is positive and which is ground. (If both are the same, the motor
	will stop moving!)

	When we go forward, we send a low (0) signal to the H-bridge for direction.
	We also send a PWM (pulse width modulation) signal to the H-bridge. When
	that signal is high, the motor turns; when it is low, the motor stops.

	So, we send a pulse that's high for the width specified, then drop it
	low for the remainder of the pulse frame, returning it to high at the
	end of that frame, when the next pulse starts.
	 ___   ___   ___
	|   |_|   |_|   |_		wave for 75% full speed forward

	When we go in reverse, we send a high (1) signal to the H-bridge for
	direction. In this case, the motor turns when the PWM signal is low,
	and stops when it's high.

	In this case, we sent a pulse that's low for the width specified, then
	raise it for the remainder of the frame.
	     _     _     _
	 ___| |___| |___| |		wave for 75% full speed in reverse


	The code below sets up Timer/Counter 1 as follows:
	- Waveform Generation Mode: Fast PWM (0101) -- counting from 0-255
			(then wrapping)
	- Compare Output Mode: this determines how the signals on OC1A/PD5
				(right motor) and OC1B/PD4 (left motor) change when the
				counter matches the value in OCR1A or OCR1B (our "PWM"
				values indicating the desired speeds of the motors)
				and at the maximum count value (255, end of frame).
			clear on match, set at TOP (10) [for forward, see above], or
			set on match, clear at TOP (11) [for reverse, see above]
	- Clock Select: clkIO/64 (011)
			8 Mhz / 64 (prescaler) = 128 kHz for counter
			128 kHz / 256 (clock pulses/frame) = 512 Hz for PWM [frame ~2 ms]

	See the Atmel ATmega16 data sheet for details on these settings and how
	they correspond to bits in TCCR1A/B (Timer/Counter 1 Control Registers).
*/

/* define all the stuff doing I/O here; there are two reasons for this:
   1) it makes the code easier to read/understand
   2) it makes it easy to make changes later, e.g. using a different port */

/* PC3 controls left motor direction (forward/reverse) */
#define SetupLDir()	SetBit(DDRC, DDC3)
/* PC4 controls right motor direction (forward/reverse) */
#define SetupRDir()	SetBit(DDRC, DDC4)
/* PWM output on PD4/OC1B for left motor, PD5/OC1A for right motor; these
   pins are connected to H-bridge; we just need to send signals */
#define SetupLPWM()	SetBit(DDRD, DDD4)
#define SetupRPWM()	SetBit(DDRD, DDD5)
/* we compare to OCR1A/B for R/L motor speeds */
#define lPWM		OCR1B
#define rPWM		OCR1A
/* set direction (input to H-bridge) and wave output mode */
#define LFwd()		( ClearBit(PORTC, PC3),   SetBit(TCCR1A, COM1B1), ClearBit(TCCR1A, COM1B0) )
#define LRev()		(   SetBit(PORTC, PC3),   SetBit(TCCR1A, COM1B1),   SetBit(TCCR1A, COM1B0) )
#define LStop()		( ClearBit(PORTC, PC3), ClearBit(TCCR1A, COM1B1), ClearBit(TCCR1A, COM1B0) )
#define RFwd()		( ClearBit(PORTC, PC4),   SetBit(TCCR1A, COM1A1), ClearBit(TCCR1A, COM1A0) )
#define RRev()		(   SetBit(PORTC, PC4),   SetBit(TCCR1A, COM1A1),   SetBit(TCCR1A, COM1A0) )
#define RStop()		( ClearBit(PORTC, PC4), ClearBit(TCCR1A, COM1A1), ClearBit(TCCR1A, COM1A0) )

/* sets up microprocessor for PWM control of motors */
void motorInit(void)
{
	/* set up ports */
	SetupLDir();
	SetupRDir();
	SetupLPWM();
	SetupRPWM();

	TCNT1 = 0;

	/* see comment above for info on PWM initialization */
	/* start with motors disconnected from Timer/Counter output */
	TCCR1A = 0x01;	// 00 00 00 01
	TCCR1B = 0x0B;	// 000 01 011 (512 Hz) /64 C

	/* OCR1A/B are the values that the timer is compared to; a match will
	   cause the output to change; small values mean the motor runs for a
	   short period (slower); larger values are longer times (faster)*/
	lPWM = rPWM = 0;	// (value is irrelevant since outputs are disconnected)
}

/* pwm values can range from -255 (full-speed reverse)
   to 255 (full-speed forward), with 0 indicating a stop */
void SetLeftMotorPWM(int pwm)
{
	if (pwm == 0)
	{
		LStop();
	}
	else
	{
		if (pwm >= 0)
		{
			LFwd();
		}
		else
		{
			LRev();
			pwm = -pwm;
		}
		if (pwm > 255)
			pwm = 255;
		lPWM = pwm; 
	}
}

/* pwm values can range from -255 (full-speed reverse)
   to 255 (full-speed forward), with 0 indicating a stop */
void SetRightMotorPWM(int pwm)
{
	if (pwm == 0)
	{
		RStop();
	}
	else
	{
		if (pwm >= 0)
		{
			RFwd();
		}
		else
		{
			RRev();
			pwm = -pwm;
		}
		if (pwm > 255)
			pwm = 255;
		rPWM = pwm; 
	}
}


//************************Legacy Motor Functions*****************************//

/** Sets left motor characteristics */
void motorLeft(char direction, unsigned char speed){
	int ispeed= -speed;
	if(direction == FORWARD){
		ispeed = speed;
	}
	//ispeed = ispeed/127;
	SetLeftMotorPWM(ispeed);
	LeftSpeed = speed;
	LeftDir = direction;
}

/** Sets right motor characteristics */
void motorRight(char direction, unsigned char speed){
	int ispeed= -speed;
	if(direction == FORWARD){
		ispeed = speed;
	}
	//ispeed = ispeed/127;
	SetRightMotorPWM(ispeed);
	RightSpeed = speed;
	RightDir = direction;
}

/** Stops both motors */
void motorStop(void){
	SLeftSpeed = LeftSpeed;
	SLeftDir = LeftDir;
	SRightSpeed = RightSpeed;
	SRightDir = RightDir;
	LStop();
	RStop();
	//Delay1s();
}

/** Restarts both motors */
void motorResume(void){
	motorRight(SRightDir,SRightSpeed);
	motorLeft(SLeftDir,SLeftSpeed);
}

#include "../encoders.h"

#endif

//********************************End of arc.h*******************************//
// REVISIONS
// 2/3/08 - motorX now takes unsigned char speed
