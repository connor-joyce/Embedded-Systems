/*
 * HW02-HotPotato.xc
 *
 *  Created on: Sep 20, 2018
 *      Author: root
 */


#include <xs1.h>
#include <print.h>
#include <stdio.h>
//ask if we should create our own c file that does this isntead of
//including the std io library

#define TICKS_PER_MICROSECOND (XS1_TIMER_HZ/1000000)
#define TICKS_PER_MILLISECOND (XS1_TIMER_HZ/1000)

in port iButton = XS1_PORT_1C;
out port oButtonSim = XS1_PORT_1A;

void worker(unsigned int worker_id, chanend left, chanend right);
void button_listener_task(chanend left, chanend right);
void button_simulator();

void button_listener_task(chanend left, chanend right){
    timer tmr;
    unsigned int token, tmr_val;
    unsigned int reverse = 0;
    iButton when pinseq(1) :> void;
    while(1){
        tmr :> tmr_val;
        select{
            case iButton when pinseq(0) :> void:
                iButton when pinseq(1) :>void;
                reverse = 1;
                break;

            case left :> token:
                if(reverse){
                    left <: token;
                }
                else{
                    right <: token;
                }
                reverse = 0;
                break;

            case right :> token:
                if(reverse){
                    right <: token;
                }
                else{
                    left <: token;
                }
                reverse = 0;
                break;

            case tmr when timerafter(tmr_val + (2*TICKS_PER_MILLISECOND)) :> void:
                    return;

        }
    }
}

void button_simulator(){
    timer tmr;
    unsigned int tmr_val;
    oButtonSim <: 1;
    tmr :> tmr_val;
    tmr when timerafter(tmr_val + TICKS_PER_MICROSECOND) :> void;
    oButtonSim <: 0;
    tmr :> tmr_val;
    tmr when timerafter(tmr_val + TICKS_PER_MICROSECOND) :> void;
    oButtonSim <: 1;
}

void worker(unsigned int worker_id, chanend left, chanend right){
    unsigned int token;
    timer tmr;
    unsigned int tmr_val;
    char buffer[64];
    if(worker_id == 1){
        right <: 0;
    }
    while(1){
        tmr :> tmr_val;
        select {
            case left :> token:
                token++;
                sprintf(buffer, "Token received by worker %d. Current value is %d\n",
                        worker_id, token);
                printstr(buffer);
                if(token >= 10){
                    return;
                }
                tmr :> tmr_val;
                tmr when timerafter(tmr_val + (10*TICKS_PER_MICROSECOND)) :> void;
                right <: token;
                break;

            case right :> token:
                token++;
                sprintf(buffer, "Token received by worker %d. Current value is %d\n",
                        worker_id, token);
                printstr(buffer);
                if(token >= 10){
                    return;
                }
                tmr :> tmr_val;
                tmr when timerafter(tmr_val + (10*TICKS_PER_MICROSECOND)) :> void;
                left <: token;
                break;

            case tmr when timerafter(tmr_val + TICKS_PER_MILLISECOND) :> void:
                 return;
                 //TIMEOUT

        }
    }
}

int main(){
    chan c0, c1, c2, c3;
    par{
        worker(1, c0, c1);
        worker(2, c1, c2);
        worker(3, c2, c3);
        button_listener_task(c3, c0);
        button_simulator();
    }
    return 0;
}

