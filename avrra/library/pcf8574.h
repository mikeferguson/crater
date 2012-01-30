/******************************************************************************
 * AVRRA: The AVR Robotics API
 * pcf8574.h - device driver i2c I/O expander
 *
 * Copyright (c) 2008, Michael E. Ferguson
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

#ifndef AVRRA_PCF8574
#define AVRRA_PCF8574

#include "i2c.h"

/* Base address = 0x70. 0x70-0x7E valid. addr = base + (2 * device_no) */
#define PCF8574_ADDR      0x70

/*******************************pcf8574 Functions*****************************/

/** Initialize I2C bus. pcf8574 needs no initialization  */
void pcf8574Init(void){
	// init I2C
    i2cInit();
}

/** = data port of pcf8574. */
unsigned char pcf8574GetData(unsigned char device){
    unsigned char data;
    // send start, address, read
    i2cStart(PCF8574_ADDR + I2C_READ + (2 * device));
    // read data
    data = i2cReadNck();
    i2cStop();
	return data;
}

/** set the output of the data port */
void pcf8574SetData(unsigned char data, unsigned char device){
    // send start, address, write
    i2cStart(PCF8574_ADDR + I2C_WRITE + (2 * device));
    // write data
    i2cWrite(data);
    i2cStop();
}

#endif

/*******************************End of pcf8574.h*******************************
 * REVISIONS: */
