/******************************************************************************
 * AVRRA: The AVR Robotics API
 * statepro.h - state file for AVRRA Pro board
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

// Only one device state file!
#ifndef AVRRA_DEV_STATE
#define AVRRA_DEV_STATE

// we store variables in state
int state[12];	// was byte
#define 	RSTATE			state[0]
#define 	PANC			1	
#define		TLTC			2	
#define  	PAN				3
#define 	TILT			4	
#define 	IR_LE			5
#define     IR_RI			6
#define		IR_HD			7
#define     SNR_HD			8
#define 	OBJ_X			9
#define     OBJ_Y			10

// these are defined for user friendly use of sensors
#define		HEAD_IR			state[IR_HD]
#define 	LEFT_IR			state[IR_LE]
#define 	RIGHT_IR		state[IR_RI]
// sonar is too slow to update, so we only do it when we want the value. 
// WARNING: be sure to keep at least 40ms between calls to sonar!
#define 	SONAR			srf05GetData(PIN_SONAR)
#define 	GET_LEFT_IR		smooth(&gp2d12GetData,PIN_LEFT_IR)
#define 	GET_RIGHT_IR	smooth(&gp2d12GetData,PIN_RIGHT_IR)
#define     GET_HEAD_IR     smooth(&gp2d12GetData,PIN_HEAD_IR)

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
	state[IR_LE] = GET_LEFT_IR;
	state[IR_RI] = GET_RIGHT_IR;	
	state[IR_HD] = smooth(&gp2d12GetData,PIN_HEAD_IR);
	//sonar is too slow to update - do it when asked for...
	//state[SNR_HD] = sonarGetData(PIN_SONAR);

	state[PAN] = servoGetPosition(PIN_PAN_SERVO);
	state[TILT] = servoGetPosition(PIN_TILT_SERVO);

  #ifdef TGT_HAS_CAMERA
	//camera update??
	if(camStatus == CAM_TRACKING){
		// wait for update to finish
		//if(numObjRec != numObjects){
		if(newVisData > 0){	
			// find biggest object?
			state[OBJ_X] = (objects[0].xmax - objects[0].xmin)/2 + objects[0].xmin;
			state[OBJ_Y] = (objects[0].ymax - objects[0].ymin)/2 + objects[0].ymin;
			newVisData = 0;
		}else{
			state[OBJ_X] = 0;
			state[OBJ_Y] = 0;
		}
	}
  #endif
}

#endif
