/******************************************************************************
 * AVRRA: The AVR Robotics API
 * srf08.h - device driver srf08 & srf02 sonar rangers
 *
 * Copyright (c) 2008, 2009 Michael E. Ferguson
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

#ifndef AVRRA_SRF08
#define AVRRA_SRF08

#include "i2c.h"

/* Base address = 0xE0. 0xE0-0xFF valid. addr = base + (2 * device_no) */
#define SRF_ADDR        0xE0

#define SRF_COMMAND     0x00        // command register
#define SRF_LIGHT       0x01        // light sensor register
#define SRF_ECHO_H      0x02        // 1st echo high byte
#define SRF_ECHO_L      0x03        // 1st echo low byte
#define SRF_2ND_H       0x04        // 2nd echo high byte
#define SRF_2ND_L       0x05        // 2nd echo low byte

#define SRF_CMD_CM      0x51        // Ranging mode - result in centimeters

/*********************************srf08 Functions*****************************/

/** Initialize I2C bus, srf08 requires no init */
void srf08Init(void){
	// init I2C
    i2cInit();    
}

/** = version of sonar sonar, device = address */
int srf08GetVersion(unsigned char device){
    int version;
    i2cStart(SRF_ADDR + I2C_WRITE + (2*device));
    i2cWrite(SRF_COMMAND);
    i2cStop();
    i2cStart(SRF_ADDR + I2C_READ + (2*device));
    version = i2cReadNck();
    i2cStop();
    return version;
}

/** We must ping the sonar before we can read it back. */
void srf08Ping(unsigned char device){
    // write commad to read in cm
    i2cStart(SRF_ADDR + I2C_WRITE + (2*device));
    i2cWrite(SRF_COMMAND);
    i2cWrite(SRF_CMD_CM);
    i2cStop();    
}

/** = a reading from a sonar, should call ping first. */
int srf08GetData(unsigned char device){
    int distance;
    // read back data
    i2cStart(SRF_ADDR + I2C_WRITE + (2*device));
    i2cWrite(SRF_ECHO_H);
    //i2cStop();
    i2cStart(SRF_ADDR + I2C_READ + (2*device));
    distance = i2cReadAck();
    distance = distance << 8;
    distance += i2cReadNck();
    i2cStop();
    return distance;
}

/** Change address of SRF 08
int srf08ChangeAddr(unsigned char oldAddr, unsigned char newAddr){
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    i2cWrite(0xA0);
    i2cStop();   
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    i2cWrite(0xAA);
    i2cStop();   
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    i2cWrite(0xA5);
    i2cStop();   
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    // actually send address
    i2cWrite(SRF_ADDR + (2 * newAddr));
    i2cStop(); 
} NOTE THIS CODE WORKS, BUT IS NOT STANDARD TO INCLUDE. */

#endif

/*********************************End of srf08.h*******************************
 * REVISIONS: */

