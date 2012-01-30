/******************************************************************************
 * AVRRA: The AVR Robotics API
 * i2cIR.h - device driver for I2C IR Rangefinders from HVW tech
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

#ifndef AVRRA_I2CIR
#define AVRRA_I2CIR
#include "i2c.h"

/* Base address = 0x20. 0x20-0x2E valid. addr = base + (2 * device_no) */
#define I2CIR_ADDR      0x40
#define I2CIR_CM        0x02

/*********************************i2cIR Functions*****************************/

/** Initialize I2C bus, i2cIR requires no init */
void i2cIRInit(void){
	// init I2C
    i2cInit();    
}
/** = a reading from a i2c connected IR. device = the offset from the base 
  * address, ex: if you are using 0x22 as the i2c address call i2cIRGetData(1) 
  * This function performs no error checking on device numbers... */
int i2cIRGetData(unsigned char device){
    int distance;
    i2cStart(I2CIR_ADDR + I2C_WRITE + (2*device));
    i2cWrite(I2CIR_CM);
    i2cStop();
    i2cStart(I2CIR_ADDR + I2C_READ + (2*device));
    distance = i2cReadNck();
    i2cStop();
    return distance;
}

#endif

/*********************************End of i2cIR.h*******************************
 * REVISIONS: */

