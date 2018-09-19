/*
 * main.xc
 *
 *  Created on: Sep 13, 2018
 *      Author: root
 */

#include <xs1.h>
#include <stdio.h>
#include <stdlib.h>

#define FLASH_NUMBER 3
#define SAMPLE_SIZE 10
#define FLASH_PERIOD XS1_TIMER_HZ/6
#define TICKS_PER_SECOND XS1_TIMER_HZ
#define MAX_TIMER_TICKS TICKS_PER_SECOND*42;
#define TICKS_PER_MILLISECOND TICKS_PER_SECOND*1000

out port led1 = XS1_PORT_1A;
out port led2 = XS1_PORT_1D;

in port ibutton = XS1_PORT_32A;

//the first if statement takes into account that the timer may have initially been %'ed
//down to a number that would be lower than the starting time. By adding 42 seconds of clock
//ticks I am able to find the actual distance between the two. This is because the clock rolls over
//after 42 seconds.
unsigned int compute_difference(unsigned int t0, unsigned int t1){
    if(t1 < t0){
        //also apparently the underflow could just fix itself
        //but I don't trust like that
        t1 += MAX_TIMER_TICKS;
    }

    return t1 - t0;
}

//had to convert ticks to milliseconds 4 times so I made a method
float to_ms(float ticks){
    return (ticks/TICKS_PER_MILLISECOND);
}

//flashes two given 1 bit leds the flash_number of times over a given period
void flash_2_1bit_LEDs(out port led1, out port led2, int period, int flash_number){
    timer tmr;
    unsigned int timer_value;
    for(int i = 0; i<flash_number; ++i){
        tmr :> timer_value;
        timer_value += period;
        led1 <: 1;
        led2 <: 1;

        tmr when timerafter(timer_value) :> timer_value;
        timer_value += period;
        led1 <: 0;
        led2 <: 0;

        tmr when timerafter(timer_value) :> timer_value;
    }
}

//delay function waits for the timer for 1 sec + a random amount less than a sec
//called in between the start up and the exam
void delay(){
    timer tmr;
    unsigned t;
    float delay = TICKS_PER_SECOND + rand()% TICKS_PER_SECOND;
    tmr :> t;
    t+= delay;
    tmr when timerafter(t) :> void;
}

//iterates through the list twice, sorts it using bubble sort, and then once again to get the average value
//returns middle most element post sort, average value, min value, and max value
{float, float, float, float} find_statistics(float arr[]){
    float sum = 0;
    //bubble sort found at
    //https://www.geeksforgeeks.org/bubble-sort/
    for(int i = 0; i<SAMPLE_SIZE-1; i++){
            for(int j = 0; j<SAMPLE_SIZE-i-1; j++){
                if(arr[j] > arr[j+1]){
                    int temp = arr[j];
                    arr[j] = arr[j+1];
                    arr[j+1] = temp;
                }
            }
        }


    //since Bubble sort moves things around, still have to iterate through to get the average
    for(int i = 0; i<SAMPLE_SIZE; ++i){
        sum += arr[i];
    }

    return {arr[(SAMPLE_SIZE/2) - 1], sum/SAMPLE_SIZE, arr[0], arr[SAMPLE_SIZE-1]};

}

void print_stats(float arr[]){
    float med = 0;
    float avg = 0;
    float min = 0;
    float max = 0;
    {med, avg, min, max} = find_statistics(arr);

     printf("AVERAGE: %fms \nMEDIAN: %fms \nHIGHEST: %fms \nLOWEST: %fms \n", avg, med, max, min);
}



int main(){
    float records[SAMPLE_SIZE];
    timer tmr;
    while(1){
        unsigned t;
        unsigned button_value;
        flash_2_1bit_LEDs(led1, led2, FLASH_PERIOD, FLASH_NUMBER);
        delay();
        unsigned start, end;
        for(int i = 0; i<SAMPLE_SIZE; ++i){
            led1 <: 1;
            led2 <: 1;
            tmr :> start;
            ibutton :> button_value;
            ibutton when pinsneq(button_value) :> void;
            if(button_value & 0x1){
                tmr :> end;
                records[i] = to_ms(compute_difference(start, end));
                //manually turn off the lights.
                led1 <: 0;
                led2 <: 0;
            }

            //delay for 1 second
          tmr :> t;
          t += TICKS_PER_SECOND;
          tmr when timerafter(t) :> void;
        }

        //print all the recorded stats at the end and restart the whole thing.
        print_stats(records);
    }

    return 0;

}

