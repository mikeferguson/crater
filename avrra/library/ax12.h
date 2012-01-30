/******************************************************************************
 * AVRRA: The AVR Robotics API
 * ax12.h - device driver for AX-12 servo bus, using Serial1 (mega324P)
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

#ifndef AVRRA_AX12
#define AVRRA_AX12

#include <avr/interrupt.h>
#include "utils.h"

/** EEPROM AREA **/
#define AX_MODEL_NUMBER_L 			0
#define AX_MODEL_NUMBER_H 			1
#define AX_VERSION 					2
#define AX_ID 						3
#define AX_BAUD_RATE 				4
#define AX_RETURN_DELAY_TIME 		5
#define AX_CW_ANGLE_LIMIT_L 		6
#define AX_CW_ANGLE_LIMIT_H 		7
#define AX_CCW_ANGLE_LIMIT_L 		8
#define AX_CCW_ANGLE_LIMIT_H 		9
#define AX_SYSTEM_DATA2 			10
#define AX_LIMIT_TEMPERATURE 		11
#define AX_DOWN_LIMIT_VOLTAGE 		12
#define AX_UP_LIMIT_VOLTAGE 		13
#define AX_MAX_TORQUE_L 			14
#define AX_MAX_TORQUE_H 			15
#define AX_RETURN_LEVEL 			16
#define AX_ALARM_LED 				17
#define AX_ALARM_SHUTDOWN 			18
#define AX_OPERATING_MODE 			19
#define AX_DOWN_CALIBRATION_L 		20
#define AX_DOWN_CALIBRATION_H 		21
#define AX_UP_CALIBRATION_L 		22
#define AX_UP_CALIBRATION_H 		23
/** RAM AREA **/
#define AX_TORQUE_ENABLE 			24
#define AX_LED 						25
#define AX_CW_COMPLIANCE_MARGIN 	26
#define AX_CCW_COMPLIANCE_MARGIN 	27
#define AX_CW_COMPLIANCE_SLOPE 		28
#define AX_CCW_COMPLIANCE_SLOPE 	29
#define AX_GOAL_POSITION_L 			30
#define AX_GOAL_POSITION_H 			31
#define AX_GOAL_SPEED_L 			32
#define AX_GOAL_SPEED_H 			33
#define AX_TORQUE_LIMIT_L 			34
#define AX_TORQUE_LIMIT_H 			35
#define AX_PRESENT_POSITION_L 		36
#define AX_PRESENT_POSITION_H 		37
#define AX_PRESENT_SPEED_L 			38
#define AX_PRESENT_SPEED_H 			39
#define AX_PRESENT_LOAD_L 			40
#define AX_PRESENT_LOAD_H 			41
#define AX_PRESENT_VOLTAGE 			42
#define AX_PRESENT_TEMPERATURE		43
#define AX_REGISTERED_INSTRUCTION 	44
#define AX_PAUSE_TIME 				45
#define AX_MOVING 					46
#define AX_LOCK 					47
#define AX_PUNCH_L 					48
#define AX_PUNCH_H 					49
/** Status Return Levels **/
#define AX_RETURN_NONE 				0
#define AX_RETURN_READ 				1
#define AX_RETURN_ALL 				2
/** Instruction Set */
#define AX_PING						1
#define AX_READ_DATA 				2
#define AX_WRITE_DATA 				3
#define AX_REG_WRITE 				4
#define AX_ACTION 					5
#define AX_RESET 					6
#define AX_SYNC_WRITE				131

#define AX_BUFFER_SIZE 	32
static unsigned char ax_buffer[AX_BUFFER_SIZE];

// making these volatile keeps the compiler from optimizing loops of available()
static volatile int ax_buffer_head = 0;
static volatile int ax_buffer_tail = 0;

/** helper functions to emulate half-duplex */
static void setTX(){
	ClearBit(UCSR1B, RXEN1);	
	SetBit(UCSR1B, TXEN1);
	//ClearBit(UCSR1B, RXCIE1);
}
static void setRX(){
	SetBit(UCSR1B, RXEN1);
	ClearBit(UCSR1B, TXEN1);
	// enable interrupt on complete reception of a byte
	//SetBit(UCSR1B, RXCIE1);	
}

/** initializes serial1 transmit at baud, 8-N-1 */
void ax12init(long baud){
	UBRR1H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
	UBRR1L = ((F_CPU / 16 + baud / 2) / baud - 1);
	// enable tx
	setTX();
}

/** Sends a character out the serial port. */
void ax12write(byte data){
    //if(bit_is_clear(UCSR1B, TXEN1)){
		// have to disable RX & enable TX
		setTX();
	//}
	while (bit_is_clear(UCSR1A, UDRE1))
		;
	UDR1 = data;
}

/** = number of bytes available in buffer */
static int ax12Available(){
	return (AX_BUFFER_SIZE + ax_buffer_head - ax_buffer_tail) % AX_BUFFER_SIZE;
}

/** = a byte from the buffer */
unsigned char ax12Read(){
    setRX();	
    // if the head isn't ahead of the tail, we don't have any characters
	while (ax12Available() == 0);
    unsigned char c = ax_buffer[ax_buffer_tail];
	ax_buffer_tail = (ax_buffer_tail + 1) % AX_BUFFER_SIZE;
	return c;
}

/** */
int ax12PacketRead(){
    // 0xFF 0xFF ID LENGTH ERROR PARAM... CHECKSUM
    return 0;
}

/** Write a packet out to the AX-12 bus. */
void ax12PacketWrite(int id, int instruction, int numParams, int * parameters){
    int checksum, i;
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    ax12write(0xFF);
	ax12write(0xFF);
	ax12write(id);
	ax12write(2 + numParams);	// length
    ax12write(instruction);  
    checksum = id + instruction + numParams;   
    for(i=0;i<numParams;i++){
        ax12write(parameters[i]);
        checksum += parameters[i];
    }
	// checksum = 
    checksum = 0xFF - (checksum % 256);
    ax12write(0xFF - (checksum % 256));
    // read back 
    ax12PacketRead();
}

void serial1Flush(){
	ax_buffer_head = ax_buffer_tail;
}

ISR(USART1_RX_vect){
	unsigned char c = UDR1;

	int i = (ax_buffer_head + 1) % AX_BUFFER_SIZE;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if (i != ax_buffer_tail) {
		ax_buffer[ax_buffer_head] = c;
		ax_buffer_head = i;
	}	
}


/* packet: FF FF ID LENGTH INS(0x03) PARAM .. CHECKSUM */
void ax12SetPosition(int id, int pos){	
    // send a packet
	ax12write(0xFF);
	ax12write(0xFF);
	ax12write(id);
	ax12write(5);	// length
	ax12write(AX_WRITE_DATA);
	ax12write(AX_GOAL_POSITION_L);
	ax12write(pos&0xFF);
	ax12write((pos&0xFF00)>>8);
	// checksum = 
	ax12write(0xFF - ((id + 5 + AX_WRITE_DATA + AX_GOAL_POSITION_L + (pos&0xFF) + ((pos&0xFF00)>>8)) % 256) );	
    // read back 
    
}

/* packet: FF FF ID LENGTH INS(0x03) PARAM .. CHECKSUM */
void ax12SetSpeed(int id, int speed){	
    // send a packet
	ax12write(0xFF);
	ax12write(0xFF);
	ax12write(id);
	ax12write(5);	// length
	ax12write(AX_WRITE_DATA);
	ax12write(AX_GOAL_SPEED_L);
	ax12write(speed&0xFF);
	ax12write((speed&0xFF00)>>8);
	// checksum = 
	ax12write(0xFF - ((id + 5 + AX_WRITE_DATA + AX_GOAL_SPEED_L + (speed&0xFF) + ((speed&0xFF00)>>8)) % 256) );	
    // read back 
    
}

#endif
