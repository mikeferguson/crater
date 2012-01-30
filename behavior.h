/*******************************************************************************
 *
 *                     			xrb5 - Behavior
 *                                                                           
 *******************************************************************************
 *                   Copyright 2008-2009 Michael E. Ferguson                    
 ******************************************************************************/

// Behavior Status Register - default to 0x00
//  bit 0:      hi = check for left wall
//  bit 1:      hi = check for forward wall
//  bit 2:      hi = check for right wall
//  bit 6:      hi = ENTERING_ROOM
//  bit 7:      hi = EXITING_ROOM
char BREG;
int odometer;                   // the metric odometer
	
#define CHECK_LEFT          0x01
#define CHECK_FORWARD       0x02
#define CHECK_RIGHT         0x04
#define ENTERING_ROOM       0x40
#define EXITING_ROOM        0x80

#include "plan.h"
#include "room.h"

/******************************************************************************
 * These primitive behaviors each run one iteration and then return          
 */

/** Bang-bang iterative follow, should be called at least a minimum of 10-15Hz */
int followRight(unsigned char dist){
    // position head
    servoSetPosition(45);  
    if(GET_HEAD_IR < dist){ 
		if(GET_HEAD_IR < ((dist * 3) / 5)){
			// motor way to the left
			motorLeft(REG_SPEED-40);
			motorRight(REG_SPEED+40);
		}else{
			// motor to the left...	
			motorLeft(REG_SPEED-20);
			motorRight(REG_SPEED+20);
		}
	}else{
		// motor to the right...
		motorLeft(REG_SPEED+20);
		motorRight(REG_SPEED-20);
	}
	return 0;
}
int followLeft(unsigned char dist){
    // position head
    servoSetPosition(-45);     
    if(GET_HEAD_IR < dist){ 
		if(GET_HEAD_IR < ((dist * 3) / 5)){
			// motor way to the right
			motorRight(REG_SPEED-40);
			motorLeft(REG_SPEED+40);
		}else{
			// motor to the right...	
			motorRight(REG_SPEED-20);
			motorLeft(REG_SPEED+20);
		}
	}else{
		// motor to the left...
		motorRight(REG_SPEED+20);
		motorLeft(REG_SPEED-20);
	}
	return 0;
}
/** = one run of Arbitration loop, 0 if cruise, >0 otherwise */
int arbitrate(){

    // if entering room, we don't care about neighborhoods
    if((BREG&ENTERING_ROOM)&&(odometer < 0)){
        motorStop();        
        // already well inside the room...
		lcdPrint("InRoom  ");
		// this call will not return if a fire is found
        isFire();
        // no fire, call planner to exit room
		return 2;
    }

    // we have PASSED the NEIGHBORHOOD! - TESTING CODE ONLY!!!!
    if(odometer < (-2 * NBHD_SIZE)){
        motorStop();
        lcdPrint("FAIL... ");
    }

    // if we are in neighborhood, check our surroundings...
    if(odometer < 0){
        if(BREG&CHECK_RIGHT){            
            servoSetPosition(SERVO_RIGHT);
            if(GET_HEAD_IR > 46){            
                motorStop();                
                // call metric planner - to plot course of action
                
                // call topological planner
                return 1;
            }
        }
        if(BREG&CHECK_LEFT){
            servoSetPosition(SERVO_LEFT);
            if(GET_HEAD_IR > 46){
                motorStop();                
                // call metric planner - to plot course of action
                
                // call topological planner
                return 1;
            }
        }
        if(BREG&CHECK_FORWARD){
            servoSetPosition(SERVO_ZERO);
            if(GET_HEAD_IR < 30){       
                motorStop();  
                delayms(100);
                // move to wall, avoiding side wall
                //moveX(((GET_HEAD_IR-32)*3)/16);
                moveX(GET_HEAD_IR - 16);                
                // call topological planner
                return 1;            
            }        
        }
    }
    // ... roll forward
    if(BREG&ENTERING_ROOM){
        // follow left wall into room!
        if(servoGetPosition() != -45)
            servoSetPosition(-45);
        if(GET_HEAD_IR < 52){
            followLeft(30);
        }else{
            // motor straightish
		    motorLeft(REG_SPEED);
		    motorRight(REG_SPEED);
        }
    }else{
        // normal right side wall follow
        if(servoGetPosition() != 45)
            servoSetPosition(45);	   
        // if wall exists, follow right wall
        if(GET_HEAD_IR < 52){
            followRight(30);    // was 20?
        }else{
		    // motor straightish
		    motorLeft(REG_SPEED+10);
		    motorRight(REG_SPEED);
	    }
    }	
    return 0;
}
