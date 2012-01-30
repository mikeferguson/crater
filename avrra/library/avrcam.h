/******************************************************************************
 * AVRRA: The AVR Robotics API
 * avrcam.h - driver for AVRCam from JRobot.net. Uses serial1, Mega324P only
 *      NOTE: serial1.h should not be used when using this file...
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

// we now externally define this, and use it to include the file
#ifdef TGT_HAS_CAMERA

// Only one camera per robot (currently)
#ifndef XR_VISION
#define XR_VISION

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "utils.h"
#include "serial.h"

// this allows use of the AVRcamVIEW software to see tracking
//#define AVR_CAM_INTERFACE

// Vision driver modes of operation
#define VMODE_NONE		0
#define VMODE_PING		1	
#define VMODE_A			2	// A recieved, looking for C
#define VMODE_C			3		
#define VMODE_K			4
#define VMODE_N			5
#define VMODE_RAW		6	// pass a raw image
#define VMODE_ET		7	// tracking enabled
#define VMODE_TFRAME	8	// recieving a tracking frame.
#define VMODE_TCOLOR	9	
#define VMODE_TXMIN		10
#define VMODE_TXMAX		11
#define VMODE_TYMIN		12
#define VMODE_TYMAX		13
static int visionMode;

typedef struct{
    int color;
	int xmin;
	int xmax;
	int ymin;
	int ymax;
}t_object;

// tracked data
t_object objects[8];
int numObjects;
int newVisData;

// tracking reception data
int numObjRec;				// #of objects to recieve

// camera status
int camStatus;
#define CAM_ERROR		0
#define CAM_OK			1
#define CAM_TRACKING 	2

static int rawPackets;

/** Starts the vision system and serial port 1 */
void visionInit(){
	// initialise visual system
	numObjects = 0;
	visionMode = VMODE_NONE;
	newVisData = 0;
	// start serial port 1
	UBRR1H = 0;
	UBRR1L = 7;
	// enable rx and tx
	SetBit(UCSR1B, RXEN1);
	SetBit(UCSR1B, TXEN1);
	// enable interrupt on complete reception of a byte
	SetBit(UCSR1B, RXCIE1);
}

/** Sends a character out the serial port. */
void serial1Write(byte data){
	while (bit_is_clear(UCSR1A, UDRE1))
		;
	UDR1 = data;
}

/** This will decide if camera is functioning. */
int pingCam(){
	int timeouts=0;
	visionMode = VMODE_PING;
	camStatus = CAM_ERROR;
	serial1Write('P');
	serial1Write('G');
	serial1Write('\r');
	// Wait max of half second
	while(timeouts<20){
		delayms(25);
		if(camStatus > CAM_ERROR){
			return 1;
		}
		timeouts++;
	}
	return 0;
}	

/** Pass a raw camera image back to the AVRCamView program */
void passRawCam(){
	if(camStatus==CAM_OK){	
		visionMode = VMODE_RAW;
		rawPackets = 0;
		serial1Write('D');
		serial1Write('F');
		serial1Write('\r');
		// quick hack to make it work...
		//Print("ACK\r");
	}else{
		Print("NCK\r");
	}
}

void passSegCam(){
	// this function will send seg cam info back to pc
}

/** Enable tracking, interrupt driven response */
void enableTracking(){
	visionMode = VMODE_ET;
	camStatus=CAM_TRACKING;
	serial1Write('E');
	serial1Write('T');
	serial1Write('\r');
	numObjects = 0;
	newVisData = 0;
}

/** Disable tracking */
void disableTracking(){
	visionMode = VMODE_NONE;
	camStatus=CAM_OK;
	serial1Write('D');
	serial1Write('T');
	serial1Write('\r');
  #ifdef AVR_CAM_INTERFACE
	Print("ACK\r	numObjects = 0;");
  #endif

}

/** Set color map for AVRCAM */
void setColorMap(){
    // 16 bytes for red, green and blue
    int bytes = 48;
	serial1Write('S');
	serial1Write('M');
    while(bytes > 0){
        while(serialAvailable() == 0)
            ;
		serial1Write(serialRead());
        bytes--;
    }    
    // Send ACK
    Print("ACK\r");
}

/** interrupt driven camera handler... */
ISR(USART1_RX_vect){
	unsigned char c = UDR1;
	
	switch(visionMode){
		case VMODE_RAW:
			// pass raw image upwards
			serialWrite(c);
			if(c == 0x0F){
				rawPackets++;
			}
			if(rawPackets > 72){
				visionMode = VMODE_NONE;
			}
			break;
		case VMODE_PING:
			// check to see if Ack or Nck.
			if(c == 'A'){
				visionMode = VMODE_A;
				camStatus = CAM_OK;
			}else{
				visionMode = VMODE_N;
			}
			serialWrite(c);
			break;
		case VMODE_A:
		case VMODE_N:
			// C recieved
			serialWrite(c);
			visionMode = VMODE_C;
			break;
		case VMODE_C:
			// K recieved
			serialWrite(c);
			visionMode = VMODE_K;
			break;
		case VMODE_K:
			// \r recieved
			serialWrite(c);
			visionMode = VMODE_NONE;
			break;
		case VMODE_ET:
			// beginning of ET
			if(c == 0x0A){
				visionMode = VMODE_TFRAME;
			}
		#ifdef AVR_CAM_INTERFACE
			serialWrite(c);
		#endif			
			break;
		case VMODE_TFRAME:
			// this is our number of tracked objects
			numObjRec = c;
			if(numObjRec > 8){
				numObjRec = 0;
				visionMode = VMODE_ET;
			}else{
				numObjects = 0;
				visionMode = VMODE_TCOLOR;
			}
		#ifdef AVR_CAM_INTERFACE
			serialWrite(c);
		#endif			
			break;
		case VMODE_TCOLOR:
			objects[numObjects].color = c;
			visionMode = VMODE_TXMIN;
		#ifdef AVR_CAM_INTERFACE
			serialWrite(c);
		#endif			
			break;
		case VMODE_TXMIN:
			objects[numObjects].xmin = c;
			visionMode = VMODE_TYMAX;
		#ifdef AVR_CAM_INTERFACE
			serialWrite(c);
		#endif			
			break;
		case VMODE_TYMAX:
			objects[numObjects].ymax = c;
			visionMode = VMODE_TXMAX;
		#ifdef AVR_CAM_INTERFACE
			serialWrite(c);
		#endif
			break;
		case VMODE_TXMAX:
			objects[numObjects].xmax = c;
			visionMode = VMODE_TYMIN;
		#ifdef AVR_CAM_INTERFACE
			serialWrite(c);
		#endif
			break;
		case VMODE_TYMIN:
			objects[numObjects].ymin = c;
			if(numObjects < numObjRec){
				numObjects++;
				visionMode = VMODE_TCOLOR;
			}else{
				//sysMsg("gtf");
				visionMode = VMODE_ET;
				newVisData = 1;
			}		
		#ifdef AVR_CAM_INTERFACE
			serialWrite(c);
		#endif			
			break;
		default:
			// do nothing with extra input
			break;
	}
}

#endif

#endif
