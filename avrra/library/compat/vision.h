/*
 * XR Series Camera Interface
 * Copyright 2008 Michael E Ferguson
 *
 * This file handles camera interactions
 * 
 * TODO: dump frame, camera interaction
 */

#ifndef XR_VISION

#define XR_VISION
#define TGT_HAS_CAMERA
#define AVRCAM_SPEED 115200

#include "serial1.h"

static int camOK;

typedef struct{
    int color;
	int x;
	int y;
}t_object;

t_object objects[8];
int numObjects;

/* This will decide if camera is functioning. */
int pingCam(){
	int timeout= 0;
	serial1Flush();
	serial1Write('P');
	serial1Write('G');
	serial1Write('\r');
	// wait for a return, half second max...
	while((serial1Available() == 0)&&(timeout < 20)){
		timeout++;
		delayms(25);
	}
	byte data = serial1Read();
	if(data == 'A'){
		camOK= 1;
		Print("ACK\r");
		return 1;
	}else{
		camOK= 0;
		Print("NCK\r");
		return 0;
	}
	delayms(50);
	serial1Flush();
}

void visionInit(){
	// start serial port 1
	serial1Init(AVRCAM_SPEED);
	pingCam();
	numObjects = 0;
}

void passRawCam(){
	if(camOK>0){// this function will send raw cam info back to pc
		serial1Flush();
		serial1Write('D');
		serial1Write('F');
		serial1Write('\r');
		Print("ACK\r");
		unsigned int packets = 0;
		byte data;
		serial1Read(); // A
		serial1Read(); // C
		serial1Read(); // K
		serial1Read(); // \r
		// 72 packets @ 180 bytes + 4 bytes for ACK\r
		while(packets < 73){
			data = serial1Read();
			serialWrite(data);
			if(data == 0x0F){
				packets++;
			}
		}
	}else{
		Print("NCK\r");
	}
}

void passSegCam(){
	// this function will send seg cam info back to pc
}

/* This will update the object array. */
void updateCam(){
	if(camOK > 0){
		serial1Flush();
		int obj, wait = 0;
		serial1Print("ET\r");
		delayms(25);
		numObjects= 0;
		// wait for a return, 150ms max...
		while((serial1Available() == 0) && (wait < 15)){
			delayms(10);
			wait++;
		}
		if(serial1Available() == 0){
			sysMsg("no objects");
		}else{
			// read in the object information
			for(obj = serialRead(); numObjects <= obj; numObjects++){
				int x1, x2, y1, y2;
				objects[numObjects].color = serial1Read();
				x1 = serial1Read();
				//sysReading(x1);
				y1 = serial1Read();
				x2 = serial1Read();
				//sysReading(x2);
				y2 = serial1Read();
				objects[numObjects].x = (x1 + x2)/2;
				objects[numObjects].y = (y1 + y2)/2;
			}
		}
		serial1Print("DT\r");
		delayms(25);
		serial1Flush();
	}else{
		numObjects= 0;
	}
}

// color map upload

#endif
