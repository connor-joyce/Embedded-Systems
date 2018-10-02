/*
 * HW03-Ping.xc
 *
 *  Created on: Sep 27, 2018
 *      Author: root
 */


#include <xs1.h>
#include <print.h>
#include <stdio.h>

#define TICKS_PER_SECOND XS1_TIMER_HZ
#define TICKS_PER_MS TICKS_PER_SECOND/1000
#define TICKS_PER_US TICKS_PER_SECOND/1000000
#define PING_T_OUT 5*TICKS_PER_US

const unsigned int NUM_SAMPLES = 3;
const unsigned int SAMPLES_MM[] = {1000, 5000, 10000};
const unsigned int SOUND_MM_PER_SECOND = 340290;


port ioPingPort = XS1_PORT_1A;
port ioPingSimulator = XS1_PORT_1B;

void distance_consumer(chanend c);
void ping_task_timeout(port p, chanend c);
void ping_task(port p, chanend c);
void start_ping(port p);
void ping_simulator(port p, const unsigned int mms[],
        unsigned int n_mms,
        unsigned int mm_per_second);
int get_dist(float start, float end);


int main(void){
    chan c;
    par{
        ping_task_timeout(ioPingPort, c);
        distance_consumer(c);
        ping_simulator(ioPingSimulator,
                SAMPLES_MM,
                NUM_SAMPLES,
                SOUND_MM_PER_SECOND);
    }

    return 0;
}

void distance_consumer(chanend c){
    int data = 0;
    char buffer[64];
    for(int i = 0; i<NUM_SAMPLES; i++){
        c :> data;
        //print total meters traveled for the amount of samples used
        sprintf(buffer, "DISTANCE: %im\n", data);
        printstrln(buffer);
    }
    return;
}

void ping_task_timeout(port p, chanend c){
    int dist;
    //picked timeout value randomly
    //simulator takes so long it times out everytime I do it, idk
    //what value I should do
    unsigned int timeout_incr = 10*TICKS_PER_MS;
    timer t;
    unsigned int timeout, start, end;
    t :> timeout;
    for(int i = 0; i<NUM_SAMPLES; i++){
        dist = 0;
        timeout += timeout_incr;
        start_ping(p);
        select{
            case p when pinseq(1) :> void:
                t :> start;
                timeout += timeout_incr;
                select{
                    case p when pinseq(0) :> void:
                        t :> end;
                        break;
                    case t when timerafter(timeout) :> void:
                        dist = -1;
                        break;
                }
                break;
            case t when timerafter(timeout) :> void:
                dist = -1;
                break;
        }

        if(dist != -1){
            dist = get_dist(start, end);
        }
        c <: dist;
    }
    return;
}

void ping_task(port p, chanend c){
    int dist;
    timer t;
    unsigned int start, end;
    for(int i = 0; i<NUM_SAMPLES; i++){
        dist = 0;
        //send a high value to the port for the given amount of time in the hw manual
        //saved as PING_T_OUT
        //wait then send a low signal
        start_ping(p);

        //wait for pin to go hig
        p when pinseq(1) :> void;
        //mark start and wait for pin to go low
        t :> start;
        p when pinseq(0) :> void;
        //mark end
        t :> end;

        //compute meters traveled
        dist = get_dist(start, end);
        c <: dist;

    }
}

int get_dist(float start, float end){
    float seconds;
    if(end < start){
        //deal with overflow
        //division by t_p_s gives us total seconds passed
        printf("working");
        seconds = ((0xFFFFFF - start) + end)/TICKS_PER_SECOND;
    }
    else{
        //division by t_p_s gives us total seconds passed
        seconds = (end - start)/TICKS_PER_SECOND;
    }
    //multiply by speed of sound to get total mm traveled
    //divide by 1000 to get total m traveled
    return (int)((seconds*SOUND_MM_PER_SECOND)/1000);
}


void start_ping(port p){
    timer t;
    unsigned int tmr_val;
    t :> tmr_val;
    p <: 1;
    t when timerafter((tmr_val + PING_T_OUT)) :> tmr_val;
    p <: 0;

    p :> void;
}
