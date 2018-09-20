/*
 * utility.c
 *
 *  Created on: Sep 20, 2018
 *      Author: root
 */


#include "utility.h"

#define TICKS_PER_SECOND XS1_TIMER_HZ
#define TIMER_MAX_TICKS 0xFFFFFFFF

unsigned int timer_diff(unsigned int t0, unsigned int t1){
    if(t0 > t1){
        t1 += TIMER_MAX_TICKS;
    }

    return t1 - t0;
}

void format_message(char buffer[], unsigned int t0, unsigned int t1){
    sprintf(buffer, "Difference was %f seconds\n",
            ((float)timer_diff(t0, t1))/TICKS_PER_SECOND);
}


