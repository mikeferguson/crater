/*
 * Example Main File for AVRRA Mini
 */

#include "../avrra/library/boards/mini.h"

#define PIN_LED		8   # pin PB0
   
int main(){
  // make the LED pin an output
  ditigalSetDirection(PIN_LED, AVRRA_OUTPUT);    
  // do this forever...
  while(1){
    digitalWrite(PIN_LED, AVRRA_HIGH); 
    delayms(500); // half a second delay
    digitalWrite(PIN_LED, AVRRA_LOW);
    delayms(500);     
  }
}    
