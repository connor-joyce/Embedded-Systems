/*
 * HW04-MultiMotor.xc
 *
 *  Created on: Oct 9, 2018
 *      Author: root
 */

#define BIN1_ON 0b0010
#define BIN2_ON 0b1000
#define AIN1_ON 0b0100
#define AIN2_ON 0b0001

typedef struct{
    int left_duty_cycle;
    int right_duty_cycle;
}motor_cmd_t;

void driver_task(chanend out_motor_cmd_chan, int increment, unsigned int delay_ticks);
void multi_motor_task(out port oLeftPWM, out port oRightPWM,
        out port oMotorControl, chanend in_motor_cmd_chan);

void driver_task(chanend out_motor_cmd_chan, int increment, unsigned int delay_ticks){
    timer tmr;
    unsigned int tmr_val;
    tmr :> tmr_val;
    motor_cmd_t cmd;
    cmd.left_duty_cycle = 0;
    cmd.right_duty_cycle = 0;
    while(1){
        tmr_val += delay_ticks;
        out_motor_cmd_chan <: cmd;
        if(duty_cycle >= 100 || duty_cycle <= -100){
            increment*=-1;
        }
        cmd.left_duty_cycle += increment;
        cmd.right_duty_cycle += increment;
        tmr when timerafter(tmr_val) :> tmr_val;

    }
}

void motor_task(out port oLeftPWM, out port oRightPWM,
        out port oMotorControl,
        chanend in_motor_cmd_chan){
    motor_cmd_t cmd;
    unsigned int in_mask = 0b0;
    int right_duty_cycle, left_duty_cycle;
    oSTBY <: 1;
    int r_up_time, r_down_time, l_up_time, l_down_time;
    timer tmr_total, tmr_right, tmr_left;
    unsigned int total_val, right_val, left_val;
    in_motor_cmd_chan :> cmd;
    right_duty_cycle = cmd.right_duty_cycle;
    left_duty_cycle = cmd.left_duty_cycle;
    int waiting;
    tmr :> total_val, right_val, left_val;
    int ccw = 0;
    while(1){
        waiting = 1;
        in_mask = compute_mask(right_duty_cycle, left_duty_cycle);

        r_up_time = ((float)right_duty_cycle*PWM_FRAME_TICKS)/100;
        l_up_time = ((float)left_duty_cycle*PWM_FRAME_TICKS)/100;
        if(r_up_time < 0){
            r_up_time *= -1;
        }
        if(l_up_time < 0){
            l_up_time *= -1;
        }
        if(r_duty_cycle == 0){
            oRightPWM <: 0;
        }
        else{
            oRightPWM <: 1;
        }
        if(l_duty_cycle == 0){
            oLeftPWM <: 0;
        }
        else{
            oRightPWM <: 1;
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

unsigned int compute_mask (unsigned int r_duty, unsigned int l_duty){
    unsigned int ret = 0b0;

    if(right_duty_cycle >= 0){
        ret = in_mask|AIN1_ON;
    }
    else{
        ret = in_mask|AIN2_ON;
    }
    if(left_duty_cycle >= 0){
        ret = in_mask|BIN1_ON;
    }
    else{
        ret = in_mask|BIN2_ON;
    }
    return ret;
}
