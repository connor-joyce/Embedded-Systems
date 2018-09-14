/*
 * main.xc
 *
 *  Created on: Sep 13, 2018
 *      Author: connor
 */
#include <xs1.h>
#define FLASH_DELAY (XS1_TIMER_HZ/4)

out port oLEDs = XS1_PORT_32A;


int main(){
    timer tmr;
    unsigned int t;
    unsigned pattern = 0b01000001010100000000;
    tmr :> t;
    while(1){
        oLEDs <: pattern;
        t += FLASH_DELAY;
        tmr when timerafter(t) :> void;
        pattern = ~pattern;
    }
    return 0;
}
