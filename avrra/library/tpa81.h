/******************************************************************************
 * AVRRA: The AVR Robotics API
 * tpa81.h - device driver for Devantech TPA81 Infra-read Thermal Sensor
 *
 * Copyright (c) 2009, Michael E. Ferguson
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

#ifndef AVRRA_TPA81
#define AVRRA_TPA81

#include "i2c.h"

/* Base address = 0xD0. 0xD0-0xDE valid. addr = base + (2 * device_no) */
#define TPA_ADDR        0xD0

#define TPA_VER_REG     0x00
#define TPA_AMBIENT     0x01
// pixels are reg 2-9


/********************************tpa81 Functions******************************/

/** Initialize I2C bus. TPA81 needs no initialization  */
void tpa81Init(void){
	// init I2C
    i2cInit();
}

/** = a reading of ambient temp, and loads pixel values into buffer */
int tpa81GetData(unsigned char device, unsigned char * pixels){
    int ambient;
    // read back data
    i2cStart(TPA_ADDR + I2C_WRITE + (2*device));
    i2cWrite(TPA_AMBIENT);
    i2cStop();
    i2cStart(TPA_ADDR + I2C_READ + (2*device));
    ambient = i2cReadAck();
    pixels[0] = i2cReadAck();
    pixels[1] = i2cReadAck();
    pixels[2] = i2cReadAck();
    pixels[3] = i2cReadAck();
    pixels[4] = i2cReadAck();
    pixels[5] = i2cReadAck();
    pixels[6] = i2cReadAck();
    pixels[7] = i2cReadNck();
    i2cStop();
    return ambient;
}

#endif

/********************************End of tpa81.h********************************
 * REVISIONS: */
