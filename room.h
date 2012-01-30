/******************************************************************************
 * CRATER: a low-cost fire fighting robot.
 *  this is the room logic file
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

#define PHOTO_THRESH        128
#define PHOTO_NEAR          15

/** pan for the fire, returns >0 if we find it */
int panForFire(){
    servoSetPosition(0);
    turnX(-60);
    clearCounters;
    motors(REG_SPEED/2,-REG_SPEED/2);    
    while(analogGetData8(PIN_PHOTO) > PHOTO_THRESH){
        if(rCount > COUNTS_DEGREE(260)){
            motorStop();  
            return 0;
        }
    };
    motorStop();
    return 1;
}   

/** the original fight fire. */
void fightFire(){
	int i,j;
    lcdPrint("Fire!!!!");
	// pointed at the fire, move at it
    clearCounters;
    motors(REG_SPEED,REG_SPEED);
	// now don't run into wall...
    while(digitalGetData(PIN_IR) == AVRRA_HIGH);
    motorStop();
	// close enough now, put it out
	digitalSetData(PIN_FAN,AVRRA_HIGH);
	// pan back and forth
    servoSetPosition(-60);
    for(i=0;i<3;i++){
        for(j=-60;j<60;j++){
            servoSetPosition(j);
            delayms(5);
        }
    }	
    digitalSetData(PIN_FAN,AVRRA_LOW);
    // check to make sure we are ok	
    moveX(-10);
    if(panForFire() > 0) fightFire();   
    if(panForFire() > 0) fightFire();     
  	lcdPrint("DONE....");
	// return home
    while(1);
}

/** checks for fire, this will not return if there is a fire */
void isFire(){
	// check for fire
    if(panForFire() > 0){
        delayms(1000);
        fightFire();
    }
    // no fire - return to arbitrate
	lcdPrint("No Fire ");	
	return;
}
