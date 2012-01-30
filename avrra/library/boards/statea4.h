/******************************************************************************
 * AVRRA: The AVR Robotics API
 * a4.h- state file for XR-A4 robot, a simple ATMEGA168 powered robot.
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
 
// Only one device file!
#ifndef AVRRA_DEV_STATE
#define AVRRA_DEV_STATE

// we store variables in state
int state[12];	// was byte
#define 	RSTATE		state[0]
#define 	PANC		1	
#define		TLTC		2	
#define  	PAN			3
#define 	TILT		4	
#define 	IR_LE		5
#define     IR_RI		6
#define		IR_HD		7
#define     SNR_HD		8
//#define   IR_LF		9
//#define 	IR_RF		10

// these are defined for user friendly use of sensors
#define		HEAD_IR		state[IR_HD]
#define 	LEFT_IR		state[IR_LE]
#define 	RIGHT_IR	state[IR_RI]
// sonar is too slow to update, so we only do it when we want the value. 
// WARNING: be sure to keep at least 40ms between calls to sonar!
#define 	SONAR		state[SNR_HD] //sonarGetData(PIN_SONAR)
#define 	GET_LEFT_IR		smooth(&gp2d12GetData,PIN_RIGHT_IR)
#define 	GET_RIGHT_IR	smooth(&gp2d12GetData,PIN_RIGHT_IR)
#define     GET_HEAD_IR     smooth(&gp2longGetData,PIN_HEAD_IR)

// 0 = halt, 1 = go.
#define 	HALT 		0
#define 	GO			1

/** This function will setup the states. */
void stateInit(){
	// set our defaults or read in from EEProm
	
}

/** This function will write state variables to the EEPROM. */
void saveState(){
	//write data to eeprom
	sysMsg("Write unsupported.");
}

void update(){
	state[IR_HD] = smooth(&gp2longGetData,PIN_HEAD_IR);
    state[IR_LE] = smooth(&gp2shortGetData,PIN_RIGHT_IR);    
    state[IR_RI] = state[IR_LE];
}

#endif
