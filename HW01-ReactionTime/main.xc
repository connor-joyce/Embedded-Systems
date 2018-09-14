/*
 * main.xc
 *
 *  Created on: Sep 13, 2018
 *      Author: root
 */

#include <xs1.h>
#include <stdio.h>
#include <stdlib.h>

#define flash_number 3
#define sample_size 10
#define flash_period XS1_TIMER_HZ/2

out port led1 = XS1_PORT_1A;
out port led2 = XS1_PORT_1D;

in port ibutton = XS1_PORT_32A;

//the first if statement takes into account that the timer may have initially been %'ed
//down to a number that would be lower than the starting time. By adding 1 second of clock
//ticks I am able to find the actual distance between the two
unsigned int compute_difference(unsigned int t0, unsigned int t1){
    if(t1 < t0){
        t1 += XS1_TIMER_HZ;
    }

    return t1 - t0;
}

//delay function waits for the timer for 1 sec + a random amount less than a sec
//called in between the start up and the exam
void delay(){
    timer tmr;
    unsigned t;
    float delay = XS1_TIMER_HZ + rand()% XS1_TIMER_HZ;
    tmr :> t;
    t+= delay;
    tmr when timerafter(t) :> void;
}


//find low, iterates through list and finds the lowest number in the array,
//may be replaced as the array gets sorted at some point. Removing the need to
//iterate through the list at all
float find_low(float arr[], int n){
    float low = arr[0];
        for(int i = 0; i<n; ++i){
            if(low > arr[i]){
                low = arr[i];
            }
        }

        return low;
}

//find high iterates through the list and returns the largest number in the array
//may also be replaced as the sorting of the list removes the need to do any iteration
float find_high(float arr[], int n){
    float high = arr[0];
    for(int i = 0; i<n; ++i){
        if(high < arr[i]){
            high = arr[i];
        }
    }

    return high;
}

//sums every element in the array and divides them by the sample_size
//returning the average
float find_avg(float arr[], int n){
    float sum = 0;
    for(int i = 0; i<n; ++i){
        sum += arr[i];
    }

    return sum/n;
}

//returns the middle most value of the array, in terms of size. This uses bubble sort to
//get all the elements in order and then returns the middle most element. This sort
//is what makes the find_min and find_max methods obsolete
float find_med(float arr[], int n){
    for(int i = 0; i<n-1; i++){
        for(int j = 0; j<n-i-1; j++){
            if(arr[j] > arr[j+1]){
                int temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = arr[j];
            }
        }
    }

    return arr[(n/2)-1];
}

//uses all the "find" functions to get a series of variables and then print them to the console

void print_stats(float arr[]){
    int arr_size = sample_size;
    float avg = find_avg(arr, arr_size);
    float med = find_med(arr, arr_size);
    //float high = find_high(arr, arr_size);
    //float low = find_low(arr, arr_size);

    //at this point the array has been sorted so there's no reason
    //not to just pull the highest and lowest from the array directly
    float high = arr[sample_size-1];
    float low = arr[0];
     printf("AVERAGE: %fms \nMEDIAN: %fms \nHIGHEST: %fms \nLOWEST: %fms \n", avg, med, high, low);
}

int main(){
    float records[sample_size];
    timer tmr;
    while(1){
        unsigned t;
        unsigned toggle = 1;
        unsigned button_value;
        for(int i = 0; i<flash_number; ++i){
            tmr :> t;
            //flash period is set to 1/2 seconds worth of ticks
            t+= flash_period;
            tmr when timerafter(t) :> void;

            //toggle both of the lights and then flip toggle, so that the next time around
            //it inverts the current settings
            led1 <: toggle;
            led2 <: ~toggle;
            toggle = ~toggle;
        }
        delay();
        unsigned start, end;
        for(int i = 0; i<sample_size; ++i){
            led1 <: 1;
            led2 <: 1;
            tmr :> start;
            //takes button value (hopefully off at this point) and waits until its on
            ibutton :> button_value;
            ibutton when pinsneq(button_value) :> void;
            //once the button is pressed the difference is computed and put directly into the
            //array
            tmr :> end;
            records[i] = compute_difference(start, end);
            //manually turn off the lights.
            led1 <: 0;
            led2 <: 0;

            //unneccessary section, refuse to let the trial start over until
            //the button has been released
            ibutton :> button_value;
            while(button_value%2 != 1){
                ibutton :> button_value;
            }

            //delay for 1 second
          tmr :> t;
          t += XS1_TIMER_HZ;
          tmr when timerafter(t) :> void;
        }

        //print all the recorded stats at the end and restart the whole thing.
        print_stats(records);
    }

    return 0;

}
