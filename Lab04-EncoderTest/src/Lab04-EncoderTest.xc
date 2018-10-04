/*
 * Lab04-EncoderTest.xc
 *
 *  Created on: Oct 4, 2018
 *      Author: root
 */

#include <xs1.h>

#define TICKS_PER_S XS1_TIMER_HZ
#define TICKS_PER_MS TICKS_PER_S/1000
#define PWM_FRAME_TICKS TICKS_PER_MS

#define BIN1_ON 0b0010
#define BIN2_ON 0b1000
#define AIN1_ON 0b0100
#define AIN2_ON 0b0001

out port oMotorPWMA = XS1_PORT_1P;
out port oMotorPWMB = XS1_PORT_1I;
out port oMotorControl = XS1_PORT_4D;
out port oLED = XS1_PORT_1A;
out port oSTBY = XS1_PORT_1O;
out port oLED2 = XS1_PORT_1D;

in port iEncoder = XS1_PORT_4C;

void motor_task_static(out port oMotorPWM, out port oMotorControl,
        unsigned int control_mask, unsigned int duty_cycle);
void encoder_task(in port iEncoder, out port oLed1, out port oLed2);


int main(){
    par{
        //motor_task_static(oMotorPWMA, oMotorControl, AIN1_ON, 50);
        encoder_task(iEncoder, oLED, oLED2);
    }

    return 0;
}


void motor_task_static(out port oMotorPWM, out port oMotorControl,
        unsigned int control_mask, unsigned int duty_cycle){
    oMotorControl <: control_mask;
    oSTBY <: 1;
    unsigned int up_time = ((float)duty_cycle*PWM_FRAME_TICKS)/100;
    timer tmr;
    unsigned int tmr_val;
    while(1){
        tmr :> tmr_val;
        if(duty_cycle == 0){
            oMotorPWM <: 0;
        }
        else{
            oMotorPWM <: 1;
        }

        tmr when timerafter(tmr_val + up_time) :> void;
        oMotorPWM <: 0;
        tmr when timerafter(tmr_val + PWM_FRAME_TICKS) :> void;
    }
}

void encoder_task(in port iEncoder, out port oLed1, out port oLed2){
    int led;
    while(1){
        iEncoder :> led;
//        led2 = led1;
//        led1 &= 0b1;
//        led2 &= 0b10;
        //led2 >> 1;
        oLed1 <: led&0b1;
        oLed2 <: ((led&0b10) >> 1);
    }
}
