/******************************************************************************
 * CRATER: a low-cost fire fighting robot.
 *  mapping and planning module
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
void PrintHeading(signed char heading, signed char node){
	switch(heading){
		case 0: lcdPrint("N"); break;
		case 1: lcdPrint("W"); break;
		case 2: lcdPrint("S"); break;
		case 3: lcdPrint("E"); break;
        default: lcdPrint(" ");	
    }
	lcdPrint(",");
	switch(node){
		case 0: lcdPrint("nStart"); return; break;
		case 1: lcdPrint("n1 "); break;
		case 2:	lcdPrint("n2 "); break;
		case 3:	lcdPrint("n3 "); break;
		case 4: lcdPrint("RM1"); break;
		case 5: lcdPrint("n4 "); break;
		case 6: lcdPrint("n5 "); break;
		case 7: lcdPrint("RM2"); break;
		case 8: lcdPrint("n6 "); break;
		case 9:	lcdPrint("RM3"); break;
		case 10: lcdPrint("n7 "); break;
		case 11: lcdPrint("RM4"); break;
        default: lcdPrint("   ");
	}
	lcdPrint("   ");
}

/*******************************************************************************
 * lotsa constants for all things mapalicious
 */

// array indexes for directions
#define NORTH		0
#define WEST		1
#define SOUTH		2
#define EAST		3

// node description flags
#define RM_VIS		-2
#define VISITED		-1
#define CLEAR		0
#define IS_HALL		1
#define IS_ROOM		2

// node mapping to array index, nasty looking, but hides details
#define tNodes 		13			// total number of nodes
#define nStart		0
#define n1			1
#define n2			2
#define n3			3
#define RM1			4
#define n4			5
#define n5			6
#define RM2			7
#define n6			8
#define RM3			9
#define n7			10
#define RM4			11
#define mNULL		-1			// null index in map array 

// limited metrics
#define NBHD_SIZE   15          // radius of node nieghborhoods, CM
#define ROOM_DIST   40          // how far into a room do we go, CM 

/*******************************************************************************
 * node definition
 */
typedef struct{
	signed char dirs[4];		// direction index links
	unsigned char paths[4];		// distance along path (cm)
	signed char flags;			// flags for this node
	signed char dist;			// distance to goal (only used in return)
} mapNode_t; 

// map and state variables
mapNode_t map[tNodes];

signed char lheading;			// our heading when we left node (N,W,S,E)
signed char lnode;				// this is that last node we left


/*******************************************************************************
 * helper functions
 */
 
/** = new heading, after shifting current heading left */
signed char shiftHeadingL(signed char heading){
	if(heading<EAST){
		heading = heading + 1;
	}else{
		heading = NORTH;
	}
	return heading;
}

/** = new heading, after shifting current heading right */
signed char shiftHeadingR(signed char heading){
	if(heading>NORTH){
		heading = heading - 1;
	}else{
		heading = EAST;
	}
	return heading;
}

#include "trinity.h"

/*******************************************************************************
 * This is the map setup - we create the a priori map. 
 */
void mapInit(){
	int x;
	// general setup
	for(x= 0; x<tNodes; x++){
		map[x].dirs[NORTH] = mNULL;
		map[x].dirs[WEST] = mNULL;
		map[x].dirs[SOUTH] = mNULL;
		map[x].dirs[EAST] = mNULL;
		map[x].paths[NORTH] = 0;
		map[x].paths[WEST] = 0;
		map[x].paths[SOUTH] = 0;
		map[x].paths[EAST] = 0;
		map[x].flags = IS_HALL;
		map[x].dist = 0;
	}
	
    // specific setup
	mapSpecInit();
}

/*******************************************************************************
 * The real action. Robot is parked when we hit this. Triggered by a loss of 
 *  a wall or appearance of a forward wall. Takes lastnode, last heading, 
 *	current heading and computes new node location. Updates map info as 
 *  needed (i.e. add a visited flag). Then plans course. Moves robot to new
 *  heading, sets BREG, and returns control (... to low level behaviors). 
 *  Returns new node heading.
 */
int plan(){
	signed char nnode=0;		// next node
	signed char nheading=0;		// next heading
	signed char tnode;			// temp node
	signed char theading;		// temp heading
	signed char priority;		// used in search pattern
	
	// this should be our new current node
	signed char curnode = map[lnode].dirs[lheading];
	
	// update flags to show visited
	if(map[curnode].flags > 0){
        map[curnode].flags= -map[curnode].flags;
	}	

    // if in room, we need to exit
    if(map[curnode].flags == RM_VIS){		        
        // no fire, already facing exit       
        // setup parameters for room exit
	    odometer = ROOM_DIST - 10;
		BREG = CHECK_FORWARD|EXITING_ROOM;   
        // no planning required
        lnode = curnode;
        lheading= shiftHeadingR(lheading);
        lheading= shiftHeadingR(lheading);
        // skip the rest of the stuff below, return to arbitrate
        //  we will follow wall out of room, until we hit a forward wall
        return curnode;
    }
	
	// we give priority to going right, then continuing in the same direction.
	//  but loop requires we shiftRightx2 then left to get node directly ahead
	theading= shiftHeadingR(shiftHeadingR(lheading));
	priority = -3;
	do{
		// find our next node and heading
		theading= shiftHeadingL(theading);
		tnode= map[curnode].dirs[theading];
		// check if it is a node
		if((tnode != mNULL)){
			// is it of higher priority?
			if((map[tnode].flags > priority)){
				// yep, update
				nnode= tnode;
				nheading= theading;
				priority = map[nnode].flags;
			}
		}
		// run 4 times (one full circle)
	}while((theading!=shiftHeadingR(shiftHeadingR(lheading))));

	// logic to turn from lheading to nheading! 
	theading= nheading-lheading;
    switch(theading){
        case 1:
        case -3:
            turnX(-90);     // LEFT
            break;
        case 2:
        case-2:
            turnX(180);
            break;
        case -1:
        case 3:
            if(map[nnode].flags == IS_ROOM)
                turnX(90);		// RIGHT
            else
                turnX(94);
            break;
        default:    
            break;
    }
	PrintHeading(nheading,nnode);

    if((nnode == n5) && (nheading=SOUTH)){
        servoSetPosition(-20);
        // WIGGLE!
        lcdPrint("WIGGLE! ");
        delayms(200);
        // turn until free
        motorLeft(140); 
        motorRight(-140);
        while(GET_HEAD_IR < 70);
        motorStop();       
        servoSetPosition(0);
        moveX(12);
        turnX(-10);       
    } 

    // update globals
	lheading= nheading;
	lnode= curnode;
    BREG = 0; 
	if(map[nnode].flags == IS_ROOM){
        // entering room
        BREG = ENTERING_ROOM;
        if((nnode == RM3) || (nnode == RM4))
            odometer = 60;
        else
            odometer = ROOM_DIST;
    }else if(map[nnode].dirs[nheading] == mNULL){
        // if we can, forward wall detection is least problematic!
        BREG = CHECK_FORWARD;
        odometer = map[lnode].paths[lheading] - NBHD_SIZE;
    }else{
        // oh well, these might work!
        if(map[nnode].dirs[shiftHeadingR(nheading)] != mNULL)
            BREG |= CHECK_RIGHT;
        if(map[nnode].dirs[shiftHeadingL(nheading)] != mNULL)
            BREG |= CHECK_LEFT;
        odometer = map[lnode].paths[lheading] - NBHD_SIZE/2;
    }

	// return control (to arbitrate)	
	return (int) curnode;
}
