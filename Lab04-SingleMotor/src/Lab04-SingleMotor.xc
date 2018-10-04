/*
 * Lab04-SingleMotor.xc
 *
 *  Created on: Oct 4, 2018
 *      Author: root
 */

#include <xs1.h>

//control pin bit masks
#define BIN1_ON 0b0010
#define BIN2_ON 0b1000
#define AIN1_ON 0b0100
#define AIN2_ON 0b0001

//ports
out port oMotorPWMA = XS1_PORT_1P;
out port oMotorPWMB = XS1_PORT_1I;
out port oMotorControl = XS1_PORT_4D;
out port oLED = XS1_PORT_1A;
out port oSTBY = XS1_PORT_1O;


#define TICKS_PER_S XS1_TIMER_HZ
#define TICKS_PER_MS TICKS_PER_S/1000
#define PWM_FRAME_TICKS TICKS_PER_MS

void toggle_port(out port oLED, unsigned int hz);
void motor_task_static(out port oMotorPWM, out port oMotorControl,
        unsigned int control_mask, unsigned int duty_cycle);
void driver_task(chanend out_motor_cmd_chan, int increment, unsigned int delay_ticks);
void motor_task(out port oMotorPWM,
        out port oMotorControl, unsigned int cw_mask,
        unsigned int ccw_mmask,
        chanend in_motor_cmd_chan);
int main(){
    chan motor_cmd_chan;
    par{
        motor_task(oMotorPWMB, oMotorControl, BIN1_ON, BIN2_ON, motor_cmd_chan);
        driver_task(motor_cmd_chan, 5, TICKS_PER_S/8);
        toggle_port(oLED, 2);
    }
    return 0;
}


void toggle_port(out port oLED, unsigned int hz){
    int toggle = 1;
    unsigned int t_val;
    timer tmr;
    tmr :> t_val;
    while(1){
        t_val += (hz*TICKS_PER_S);
        oLED <: toggle;
        tmr when timerafter(t_val) :> t_val;
        toggle = ~toggle;
    }
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

void driver_task(chanend out_motor_cmd_chan, int increment, unsigned int delay_ticks){
    timer tmr;
    unsigned int tmr_val;
    tmr :> tmr_val;
    int duty_cycle = 0;
    while(1){
        tmr_val += delay_ticks;
        out_motor_cmd_chan <: duty_cycle;
        if(duty_cycle >= 100 || duty_cycle <= -100){
            increment*=-1;
        }
        duty_cycle += increment;
        tmr when timerafter(tmr_val) :> tmr_val;

    }
}

void motor_task(out port oMotorPWM,
        out port oMotorControl, unsigned int cw_mask,
        unsigned int ccw_mask,
        chanend in_motor_cmd_chan){

    //
    int duty_cycle;
    oSTBY <: 1;
    int up_time;
    timer tmr;
    unsigned int tmr_val;
    in_motor_cmd_chan :> duty_cycle;
    int waiting;
    tmr :> tmr_val;
    int ccw = 0;
    while(1){
        waiting = 1;
        if(duty_cycle >= 0){
            oMotorControl <: cw_mask;
        }
        else{
            oMotorControl <: ccw_mask;
        }
        up_time = ((float)duty_cycle*PWM_FRAME_TICKS)/100;
        if(up_time < 0){
            up_time *= -1;
        }
        if(duty_cycle == 0){
            oMotorPWM <: 0;
        }
        else{
            oMotorPWM <: 1;
        }
        while(waiting){
            select{
            case tmr when timerafter(tmr_val + up_time) :> void:
                    oMotorPWM <: 0;
                    waiting = 0;
                    break;
            case in_motor_cmd_chan :> duty_cycle:
                break;
            }
        }
        waiting = 1;
        while(waiting){
            select{
            case tmr when timerafter(tmr_val + PWM_FRAME_TICKS) :> void:
                    waiting = 0;
                    break;
            case in_motor_cmd_chan :> duty_cycle:
                break;
            }
        }
        tmr_val+=PWM_FRAME_TICKS;
    }

}


