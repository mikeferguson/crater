/******************************************************************************
 * AVRRA: The AVR Robotics API
 * i2c.h - device driver for I2C (TWI)
 * 
 * Modified Version of the i2c master library by
 *  Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
 * 
 * Licensing Unknown!
 *****************************************************************************/

/* Todo: implement event driven driver... */
#ifndef AVRRA_I2C
#define AVRRA_I2C

#include <compat/twi.h>
#include "utils.h"

#define SCL_CLOCK 100000L

#define I2C_READ    1
#define I2C_WRITE   0

/**********************************i2c Functions******************************/

/** Initializes onboard TWI */
void i2cInit(void){
    // no prescaler, set clock
    TWSR = 0; 
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
}

/** Issues a start condition and sends address and transfer direction. */
void i2cStart(unsigned char address){
    unsigned char twst;
    while ( 1 ){
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ){    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }
}

/** Sends stop condition to release the I2C bus. */
void i2cStop(void){
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));

}

/** Sends one byte to I2C bus. 0 = success. */
unsigned char i2cWrite( unsigned char data ){	
    unsigned char twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;
}

/** = one byte read form the I2C bus. Requests more data from bus. */
unsigned char i2cReadAck(void){
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
    return TWDR;
}

/** = one byte read from the I2C bus. Issues stop condition... */
unsigned char i2cReadNck(void){
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
    return TWDR;
}

#endif

/**********************************End of i2c.h********************************
 * REVISIONS: */

