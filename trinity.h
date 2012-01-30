/******************************************************************************
 * CRATER: a low-cost fire fighting robot.
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

/** This function creates the trinity 2009 map */
void mapSpecInit(){
    // Trinity specific setup
	map[nStart].dirs[NORTH] = n1;
	map[nStart].paths[NORTH] = 120;         // in CM
	map[nStart].dirs[EAST] = n4;
    map[nStart].paths[EAST] = 112;           	
	map[nStart].dist = 0;					// is goal
	
	map[n1].dirs[SOUTH] = nStart;
	map[n1].paths[SOUTH] = 120; 			
	map[n1].dirs[EAST] = n2;
	map[n1].paths[EAST] = 112; 
	map[n1].dist = 1;
	
	map[n2].dirs[WEST] = n1;
	map[n2].paths[WEST] = 112;
	map[n2].dirs[SOUTH] = n3;
	map[n2].paths[SOUTH] = 80; //74; 
	map[n2].dist = 2;
	
	map[n3].dirs[NORTH] = n2;
	map[n3].paths[NORTH] = 74;
	map[n3].dirs[WEST] = RM1;
	map[n3].dirs[SOUTH] = n4;
	map[n3].paths[SOUTH] = 56; //46;
	map[n3].dist = 2;
		
	map[RM1].dirs[EAST] = n3;
	map[RM1].flags = IS_ROOM;
	map[RM1].dist = 100;
		
	map[n4].dirs[NORTH] = n3;
	map[n4].paths[NORTH] = 46;
    map[n4].dirs[WEST] = nStart;
    map[n4].paths[WEST] = 112;
	map[n4].dirs[SOUTH] = n5;
	map[n4].paths[SOUTH] = 72;
	map[n4].dirs[EAST] = n6;
	map[n4].paths[EAST] = 56;
	map[n4].dist = 1;
	
	map[n5].dirs[NORTH] = n4;
	map[n5].paths[NORTH] = 76;
	map[n5].dirs[WEST] = RM2;
	map[n5].dist = 2;
	
	map[RM2].dirs[EAST] = n5;
	map[RM2].flags = IS_ROOM;
	map[RM2].dist = 100;
	
	map[n6].dirs[NORTH] = RM3;
	map[n6].dirs[WEST] = n4;
	map[n6].paths[WEST] = 46;			
	map[n6].dirs[EAST] = n7;
	map[n6].paths[EAST] = 46;	
	map[n6].dist = 2;
		
	map[RM3].dirs[SOUTH] = n6;
	map[RM3].flags = IS_ROOM;
	map[RM3].dist = 100;
	
	map[n7].dirs[WEST] = n6;
	map[n7].paths[WEST] = 46;
	map[n7].dirs[SOUTH] = RM4;
	map[n7].dist = 3;
	
	map[RM4].dirs[NORTH] = n7;
	map[RM4].flags = IS_ROOM;
	map[RM4].dist = 100;
	
	// Now initialize for first node
	map[nStart].flags = VISITED;
	lnode = nStart;
    //servoSetPosition(SERVO_RIGHT);
    //if(GET_HEAD_IR > 30){
        lheading = NORTH;        
        BREG = CHECK_FORWARD;
    //}else{
	//    lheading = EAST;
    //    BREG = CHECK_FORWARD | CHECK_RIGHT;
    //    map[n2].flags = VISITED;
    //}	
    odometer = map[lnode].paths[lheading] - NBHD_SIZE;
}
