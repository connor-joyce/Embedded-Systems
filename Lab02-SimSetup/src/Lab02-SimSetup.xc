/*
 * Lab02-SimSetup.xc
 *
 *  Created on: Sep 20, 2018
 *      Author: root
 */


#include <xs1.h>
#include <print.h>
#include "utility.h"

#define TICKS_PER_MS (XS1_TIMER_HZ/1000)

in port iButton = XS1_PORT_1C;
out port oButtonSim = XS1_PORT_1B;

void monitor_button(){
    char buffer[64];
    timer tmr;
    unsigned int tmr_val_start, tmr_val_end;
    iButton when pinseq(0) :> void;
    while(1){
        iButton when pinseq(0) :> void;
        tmr :> tmr_val_start;
        iButton when pinseq(1) :> void;
        tmr :> tmr_val_end;
        format_message(buffer, tmr_val_start, tmr_val_end);
        printstr(buffer);
    }
}

void button_simulator(){
    timer tmr;
    unsigned int timer_val;
    oButtonSim <: 1;
    unsigned int iter = 1;
    while(1){
        oButtonSim <: 0;
        tmr :> timer_val;
        tmr when timerafter(timer_val + (TICKS_PER_MS*iter)) :> void;
        oButtonSim <: 1;
        tmr :> timer_val;
        tmr when timerafter(timer_val + (TICKS_PER_MS/2)) :> void;
        iter++;
    }
}

int main(){
    par{
        monitor_button();
        button_simulator();
    }

    return 0;
}

int main_test(){
    char buffer[64];

    //BEGIN TEST CASES
    format_message(buffer, TICKS_PER_MS, 50*TICKS_PER_MS);
    printstr(buffer);

    //i dont think this is working properly
    format_message(buffer, 900*TICKS_PER_MS, TICKS_PER_MS);
    printstr(buffer);



    return 0;
    //END TEST CASES
}
