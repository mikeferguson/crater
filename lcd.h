/******************************************************************************
 * AVRRA: The AVR Robotics API
 * lcd.h- device file 2-line lcd, implements auto-scrolling and auto linefeed.
 *****************************************************************************/

#ifndef AVRRA_LCD
#define AVRRA_LCD

#include <stdlib.h>
#include "avrra/library/digital.h"

#define RS_DATA     1
#define RS_CMD      0 

#define CHARS_PER_LINE  8
#define NUM_OF_LINES    2

// next place to write a character
static unsigned char lcdPosition;
static char * lcd2ndLine;

void lcdPulseE(){
    digitalSetData(PIN_LCD_E, AVRRA_HIGH);
    asm("nop");
    asm("nop");    
    asm("nop");
    digitalSetData(PIN_LCD_E, AVRRA_LOW);
}

void lcdWrite (unsigned char data, unsigned int rs){
    if (rs) {
        digitalSetData(PIN_LCD_RS, AVRRA_HIGH);
    }else{
        digitalSetData(PIN_LCD_RS, AVRRA_LOW);
    }
    PORTB &= 0x0F;              // clear upper nibble
    PORTB |= (data & 0xF0);     // send upper nibble
    lcdPulseE();
    delayms(1);
    PORTB &= 0x0F;              
    PORTB |= ((data<<4)&0xF0);
    lcdPulseE();    
    delayms(5);                 // wait for 1ms
}

void lcdInit(void){
    unsigned char i;
    digitalSetDirection(PIN_LCD_E,AVRRA_OUTPUT);
    digitalSetData(PIN_LCD_E,AVRRA_LOW);
    digitalSetDirection(PIN_LCD_RS,AVRRA_OUTPUT);
    digitalSetData(PIN_LCD_RS,AVRRA_LOW);
    digitalSetDirection(PIN_LCD_D4,AVRRA_OUTPUT);
    digitalSetDirection(PIN_LCD_D5,AVRRA_OUTPUT);
    digitalSetDirection(PIN_LCD_D6,AVRRA_OUTPUT);
    digitalSetDirection(PIN_LCD_D7,AVRRA_OUTPUT);

    // put us in 4-bit
    PORTB &= 0x0F;              // clear upper nibble
    PORTB |= 0x30;  
    digitalSetData(PIN_LCD_RS,AVRRA_LOW);
    lcdPulseE();
    delayms(6);    
    lcdPulseE();
    delayms(2);   
    lcdPulseE();                // repeat 3rd time
    delayms(2);
    PORTB &= 0x2F;              // ask for 4-bit mode
    lcdPulseE();    
    delayms(2);

    lcdWrite (0x28,RS_CMD);     // 4 data lines, 2 lines
    lcdWrite (0x06,RS_CMD);     // cursor setting
    lcdWrite (0x01,RS_CMD);     // clear LCD memory
    lcdWrite (0x0f,RS_CMD);     // display ON
    delayms (10);               // 10ms delay after clearing LCD   
    lcdPosition = 0; 

    lcd2ndLine = (char *) malloc(sizeof(char) * 9);
    for(i=0;i<=8;i++)
        *(lcd2ndLine+i) = 0;
}

void lcdCursor (char row, char column){
    switch (row){
        case 0: lcdWrite(0x80 + column, RS_CMD); break;
        case 1: lcdWrite(0xc0 + column, RS_CMD); break;
        default: break;
    }
    lcdPosition = row*CHARS_PER_LINE + column;
}

void lcdClear(void){
    unsigned char i;    
    lcdWrite(0x01,RS_CMD);
    lcdCursor(0,0);
    for(i=0;i<=8;i++)
        *(lcd2ndLine+i) = 0;
}

void lcdLineFeed(){
    unsigned char i; 
    char * j;
    if(lcdPosition <= CHARS_PER_LINE){
        lcdCursor(1,0);
        j = lcd2ndLine;  
    }else{
        lcdPosition = CHARS_PER_LINE;
        lcdCursor(0,0);
        lcdWrite(0x01,RS_CMD);
        j = lcd2ndLine;
        while(*j)
            lcdWrite(*j++,RS_DATA);
        lcdCursor(1,0);
    }
    for(i=0; i<=8; i++)
        *(lcd2ndLine+i) = 0;
}

void lcdPrint(const char * text){
    while (*text){
        // line feed if neccesary
        if((lcdPosition%CHARS_PER_LINE == 0) && (lcdPosition != 0))
            lcdLineFeed();
        lcdWrite(*text,RS_DATA);
        if(lcdPosition >= CHARS_PER_LINE)
            *(lcd2ndLine+(lcdPosition%CHARS_PER_LINE)) = *text;
        lcdPosition++;text++;
    }
}

void lcdPrintNumber(int value){
    char data[5] = "     ";
    data[4] = 0;
    if(value < 0){
		data[0] = '-';
        value = -value;
  	}
	if(value > 999){
		value = 999;
	}
	data[1] = value/100 + 48;
	data[2] = (value/10)%10 + 48;
	data[3] = value%10 + 48;
	lcdPrint(data);
}

void lcdPrintLong(int value){
    char data[9] = "         ";
    data[8] = 0;
    if(value < 0){
		data[0] = '-';
        value = -value;
  	}
	if(value > 9999){
		value = 9999;
	}
    data[1] = value/1000 + 48;		
    data[2] = (value/100)%10 + 48;
	data[3] = (value/10)%10 + 48;
	data[4] = value%10 + 48;
	lcdPrint(data);
}

#endif
