/******************************************************************************
 * AVRRA: The AVR Robotics API
 * sp03.h - device driver for Devantech speech synthesizer (I2C mode)
 *   Currently only speaks pre-defined phrases - no support for buffers...
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

#ifndef AVRRA_SP03
#define AVRRA_SP03

#include "i2c.h"

#define SP03_ADDR       0xC4
// Write nearly everything to CMD_REG. Read it to see if done (0).
#define SP03_CMD_REG    0x00
#define SP03_VER_REG    0x01
// follow NOP with the text to be put in the buffer (85 bytes max)...
// form: Start+Address, Command Reg, NOP, Volume (0 = Max), 
//       speed, pitch, text.... NULL, Stop Bit.
#define SP03_CMD_NOP    0x00
// commands 1=30 are speak pre defined phrase X...
#define SP03_SPEAK_BUF  0x40

/*********************************sp03 Functions******************************/

/** Initialize I2C bus. SP03 needs no initialization  */
void sp03Init(void){
	// init I2C
    i2cInit();
}

/** = status of SP03. 0 = done. */
unsigned char sp03GetData(){
    unsigned char data;
    // send start, address, write
    i2cStart(SP03_ADDR + I2C_WRITE);
    // write address to read
    i2cWrite(SP03_CMD_REG);
    i2cStop();
    
    // send start, address, read
    i2cStart(SP03_ADDR + I2C_READ);
    // read version
    data = i2cReadNck();
    i2cStop();
     
	return data;
}

/** = version of SP03. */
unsigned char sp03GetVer(){
    unsigned char version;
    // send start, address, write
    i2cStart(SP03_ADDR + I2C_WRITE);
    // write address to read
    i2cWrite(SP03_VER_REG);
    i2cStop();
    
    // send start, address, read
    i2cStart(SP03_ADDR + I2C_READ);
    // read version
    version = i2cReadNck();
    i2cStop();
     
	return version;
}

/** = speak a pre-defined phrase, 1-30. */
void sp03Speak(unsigned char phrase){
    // check bounds
    if(phrase > 30 || phrase < 1) return;
    // wait for any previous phrase to finish
    while(sp03GetData() > 0);
    // send start, address, write
    i2cStart(SP03_ADDR + I2C_WRITE);
    // write address to write
    i2cWrite(SP03_CMD_REG);
    // write phrase to speak..
    i2cWrite(phrase);
    i2cStop();
}

// follow NOP with the text to be put in the buffer (85 bytes max)...
// form: Start+Address, Command Reg, NOP, Volume (0 = Max), 
//       speed, pitch, text.... NULL, Stop Bit.

/** = speak phrase at max volume. *
void sp03Say(const char * phrase, unsigned char pitch, unsigned char speed){
    const char *pch;
    // wait for any previous phrase to finish
    while(sp03GetData() > 0);
    // send start, address, write
    i2cStart(SP03_ADDR + I2C_WRITE);
    // write address to write
    i2cWrite(SP03_CMD_REG);
    i2cWrite(SP03_CMD_NOP);
    // volume = max
    i2cWrite(0);
    // speed
    i2cWrite(speed);
    // pitch
    i2cWrite(pitch);
    // write phrase to say
	for (pch = phrase; *pch != 0; ++pch)
		i2cWrite(*pch);
    // pass the null
    i2cWrite(0);
    i2cStop();
    // now speak the buffer
    // wait for any previous phrase to finish
    while(sp03GetData() > 0);
    // send start, address, write
    i2cStart(SP03_ADDR + I2C_WRITE);
    // write address to write
    i2cWrite(SP03_CMD_REG);
    // write speak...!
    i2cWrite(SP03_SPEAK_BUF);
    i2cStop();
}*/

#endif

/*********************************End of sp03.h********************************
 * REVISIONS: */
