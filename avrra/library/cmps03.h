/******************************************************************************
 * AVRRA: The AVR Robotics API
 * cmps03.h - device driver for I2C compass
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

#ifndef AVRRA_CMPS03
#define AVRRA_CMPS03

#include "i2c.h"

#define CMPS_ADDR       0xC0
#define CMPS_VERSION    0x00
#define CMPS_BEARING    0x01
#define CMPS_BEAR_HI    0x02
#define CMPS_BEAR_LO    0x03
#define CMPS_CALIBRATE  0x0F

/********************************cmps03 Functions*****************************/

/** Initialize I2C bus, compass requires no init */
void cmps03Init(void){
	// init I2C
    i2cInit();    
}

/** = heading from compass (URCP BinaryRADianS 0-255) */
unsigned char cmps03GetData(){
    unsigned char heading;
    // send start, address, write
    i2cStart(CMPS_ADDR + I2C_WRITE);
    // write address to read
    i2cWrite(CMPS_BEARING);
    i2cStop();
    
    // send start, address, read
    i2cStart(CMPS_ADDR + I2C_READ);
    // read heading (BRADS)
    heading = i2cReadNck();
    i2cStop();
     
	return heading;
}

/** = version from compass */
unsigned char cmps03GetVer(){
    unsigned char version;
    // send start, address, write
    i2cStart(CMPS_ADDR + I2C_WRITE);
    // write address to read
    i2cWrite(CMPS_VERSION);
    i2cStop();
    
    // send start, address, read
    i2cStart(CMPS_ADDR + I2C_READ);
    // read version
    version = i2cReadNck();
    i2cStop();
     
	return version;
}

#endif

/********************************End of cmps03.h*******************************
 * REVISIONS: */

