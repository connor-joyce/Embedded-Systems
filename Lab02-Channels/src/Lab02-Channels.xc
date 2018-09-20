/*
 * Lab02-Channels.xc
 *
 *  Created on: Sep 20, 2018
 *      Author: root
 */


#include <xs1.h>
#include <print.h>
#include <stdio.h>

#define TICKS_PER_MS (XS1_TIMER_HZ/1000)

in port iButton1 = XS1_PORT_1C;
in port iButton2 = XS1_PORT_1D;

out port oButtonSim1 = XS1_PORT_1A;
out port oButtonSim2 = XS1_PORT_1B;

void display_output(chanend c);
void monitor_buttons(chanend c);
void button_simulator();

int main(){
    chan c1;
    par{
        display_output(c1);
        monitor_buttons(c1);
        button_simulator();
    }

    return 0;
}

void display_output(chanend c){
    char output[64];
    unsigned int in_data = 0;
    while(1)
    {
        c :> in_data;
        sprintf(output, "Button %d was pressed\n", in_data);
        printstr(output);
    }
}

void monitor_buttons(chanend c){
    iButton1 when pinseq(1) :> void;
    iButton2 when pinseq(1) :> void;
    while(1){
        select{
            case iButton1 when pinseq(0) :> void:
                iButton1 when pinseq(1) :> void;
                c <: 1;
                break;

            case iButton2 when pinseq(0) :> void:
                iButton2 when pinseq(1) :> void;
                c <: 2;
                break;
        }
    }
}

void button_simulator(){
    oButtonSim1 <: 1;
    oButtonSim2 <: 1;
    int iter = 1;
    timer tmr;
    unsigned int tmr_val;
    while(1)
    {
        if(iter%2)
        {
            oButtonSim1 <: 0;
        }
        else
        {
            oButtonSim2 <: 0;
        }
        tmr :> tmr_val;
        tmr when timerafter(tmr_val + TICKS_PER_MS) :> void;
        oButtonSim1 <: 1;
        oButtonSim2 <: 1;
        iter++;
    }
}
