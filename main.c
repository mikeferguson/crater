/******************************************************************************
 * CRATER - a low-cost fire fighting robot.
 * Senior division winner of the 2009 Trinity Fire Fighting Contest
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

#define SERVO_LEFT_END      -75
#define SERVO_RIGHT_END     105
#define SERVO_ZERO          0
#define SERVO_LEFT          -75
#define SERVO_RIGHT         90

// Device file
#include "crater.h"
// planner and such
#include "behavior.h"

int main()
{		
    int wait;
    // Set up port for start switch
	lcdInit();
    // Set up motors and encoders  
    motorInit();  
    encodersInit();
    servoInit();        
	// Set up IRs
	gp2d12Init(PIN_HEAD_IR);
    digitalSetDirection(PIN_IR,AVRRA_INPUT);    
	digitalSetData(PIN_IR,AVRRA_LOW);
    // Set up OP599A - analog already initialized
    digitalSetDirection(PIN_PHOTO,AVRRA_INPUT);    
	digitalSetData(PIN_PHOTO,AVRRA_LOW);
    // Set up fan & test
    digitalSetData(PIN_FAN, AVRRA_LOW); 
    digitalSetDirection(PIN_FAN, AVRRA_OUTPUT);
    digitalSetData(PIN_FAN, AVRRA_HIGH);
    delayms(25);
    digitalSetData(PIN_FAN, AVRRA_LOW);

    lcdPrint("Wait... ");
    wait = digitalGetData(PIN_START);
    while(wait){
        // standard delay .. 
		delayms(10);
		wait = digitalGetData(PIN_START);
	}

	// Start our map
	mapInit();
    lcdClear();	
	PrintHeading(lheading,nStart);
    sei();

    // Run Behaviors
	while(1){
        // update odometer
        while(rCount > COUNTS_CM(1) ){
            // odometer is in CM
            odometer = odometer - 1;
            rCount = rCount - COUNTS_CM(1);
        }
        // now run behaviors
		if(arbitrate()>0){
            // let motors wind down
            delayms(500);
            clearCounters;
			plan();
            clearCounters;
		}
	}
}
