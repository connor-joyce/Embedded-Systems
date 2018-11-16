/*
 * Maze.xc
 *
 *  Created on: Nov 8, 2018
 *      Author: root
 */



#include <xs1.h>
#include <string.h>
#include <stdio.h>
#include <platform.h>
#include <stdlib.h>
#include <print.h>
#include <math.h>

#include "mpu6050.h"
//#include "i2c.h"


//could import the other project instead of copying and pasting

//define motor control bitmasks
#define BIN1_ON 0b0010
#define BIN2_ON 0b1000
#define AIN1_ON 0b0100
#define AIN2_ON 0b0001

//define general definitions
#define TICKS_PER_SECOND XS1_TIMER_HZ
#define BAUDRATE 9600
#define BUFFER_LENGTH 128
#define BUFFER_DELAY TICKS_PER_SECOND/4
#define TICKS_PER_MS TICKS_PER_SECOND/1000
#define PWM_FRAME_TICKS TICKS_PER_MS
#define DRIVE_ERROR .5
#define TURN_ARC .7
#define ENCODER_DELTA TICKS_PER_SECOND/2
#define TURN_DEG 3.14/2
#define ENCODER_TURN_CHANGES 11
#define ENCODER_DRIVE_CHANGES 4

//define all used duty cycles

#define FULL_FORWARD_L_DUTY_CYCLE -50
#define FULL_FORWARD_R_DUTY_CYCLE 50

#define HALF_FORWARD_L_DUTY_CYCLE -25
#define HALF_FORWARD_R_DUTY_CYCLE 25

#define FULL_REVERSE_L_DUTY_CYCLE 50
#define FULL_REVERSE_R_DUTY_CYCLE -50

#define HALF_REVERSE_L_DUTY_CYCLE 25
#define HALF_REVERSE_R_DUTY_CYCLE -25

//make these values smaller when running in flash mode,
//the vehicle overshoots the turn if it's going too fast
#define RTURN_R_DUTY_CYCLE 50
#define RTURN_L_DUTY_CYCLE 50

#define LTURN_L_DUTY_CYCLE -50
#define LTURN_R_DUTY_CYCLE -50

out port oLED = XS1_PORT_1A;
out port oWiFiRX = XS1_PORT_1F;
out port oRPWM = XS1_PORT_1P;
out port oLPWM = XS1_PORT_1I;
out port oMotorControl = XS1_PORT_4D;
out port oSTBY = XS1_PORT_1O;

in port iWiFiTX = XS1_PORT_1H;
in port iEncoder = XS1_PORT_4C;
in  port butP = XS1_PORT_32A;

struct IMU imu = {{
        on tile[0]:XS1_PORT_1L,                         //scl
        on tile[0]:XS1_PORT_4E,                         //sda
        400},};                                         //clockticks (1000=i2c@100kHz)

//structure for motor command
typedef struct{
    int left_duty_cycle;
    int right_duty_cycle;
}motor_cmd_t;

typedef struct{
    float yaw;
    float pitch;
    float roll;
}ypr_t;


//structure to carry a string
typedef struct{
    char data[BUFFER_LENGTH];
    motor_cmd_t cmd;
    int turning;
    int driving;
    int until_stop;
    float sens;
} message_t;

void toggle_port(out port oLED, unsigned int hz);
void line(const char buffer[]);
void uart_transmit_byte(out port oPort, char value, unsigned int baudrate);
char uart_receive_byte(in port iPort, unsigned int baudrate);
void uart_transmit_bytes(out port oPort, const char buffer[], unsigned int baudrate);
void uart_to_console_task(chanend trigger_chan);
void output_task(chanend trigger_chan);
void multi_motor_task(out port oLeftPWM, out port oRightPWM,
        out port oMotorControl,
        chanend in_motor_cmd_chan);
unsigned int compute_mask (int right_duty, int left_duty);
void PWM_Frame(int r_up_time, int l_up_time, out port oRightPWM, out port oLeftPWM, chanend in_motor_cmd_chan, motor_cmd_t &cmd);
void move_controller(chanend c, motor_cmd_t t);
void run_wifi_program();
void imu_task(chanend dmp_out);
void filter(chanend imu_chan, chanend motor_chan, chanend console_chan);
void controller(chanend motor_c, in port iEncoder, chanend dmp, chanend input, chanend output);




int main(){
    oWiFiRX <: 1;
    chan output;
    chan motor_c;
    chan input;
    chan dmp;
    par{
        uart_to_console_task(input);
        output_task(output);
        multi_motor_task(oLPWM, oRPWM, oMotorControl, motor_c);
        //imu_task(dmp);
        controller(motor_c, iEncoder, dmp, input, output);
    }

    return 0;
}

void controller(chanend motor_c, in port iEncoder, chanend dmp, chanend input, chanend output){
    unsigned int count = 0;
    //100% of expected time
    float forward_s = 0;
    float backward_s = 0;
    float sensitivity = 0;
    message_t msg;
    timer t;
    unsigned int encoder_in = 0;
    unsigned int prev_encoder_in = 0;
    unsigned int curr_t = 0;
    unsigned int last_change = 0;
    float target_yaw = 0;
    msg.cmd.left_duty_cycle = 0;
    msg.cmd.right_duty_cycle = 0;
    msg.until_stop = 0;
    msg.turning = 0;
    msg.driving = 0;
    ypr_t ypr;
    int turn_count = 0;
    int right_count = 0;
    int left_count = 0;
    int starting_count = 0;
    while(1){
        t :> curr_t;
        //dmp :> ypr;
        if(msg.until_stop){
            if(curr_t > (last_change + ENCODER_DELTA + sensitivity)){
                msg.cmd.left_duty_cycle = 0;
                msg.cmd.right_duty_cycle = 0;
                msg.turning = 0;
                msg.driving = 0;
                msg.until_stop = 0;
                t :> last_change;
            }
        }
        //turning right
        if(msg.turning){
            if((turn_count - starting_count) > ENCODER_TURN_CHANGES){
                msg.cmd.left_duty_cycle = 0;
                msg.cmd.right_duty_cycle = 0;
                msg.turning = 0;
                msg.driving = 0;
                msg.until_stop = 0;
            }

        }
        if(msg.driving){
            //use 3 as a reasonable error
            if((left_count + right_count)%ENCODER_DRIVE_CHANGES == 0){
                if(left_count > right_count){
                    if(msg.cmd.right_duty_cycle < 70 && msg.cmd.right_duty_cycle > 0){
                        msg.cmd.right_duty_cycle += 5;
                    }
                    else if(msg.cmd.right_duty_cycle > -70 && msg.cmd.right_duty_cycle < 0){
                        msg.cmd.right_duty_cycle -= 5;
                    }
                    else if(msg.cmd.left_duty_cycle > 10){
                        msg.cmd.left_duty_cycle -= 5;
                    }
                    else if(msg.cmd.left_duty_cycle < -10){
                        msg.cmd.left_duty_cycle += 5;
                    }
                }
                else if(right_count > left_count){
                    if(msg.cmd.left_duty_cycle < 70 && msg.cmd.left_duty_cycle > 0){
                        msg.cmd.left_duty_cycle += 5;
                    }
                    else if(msg.cmd.left_duty_cycle > -70 && msg.cmd.left_duty_cycle < 0){
                        msg.cmd.left_duty_cycle -= 5;
                    }
                    else if(msg.cmd.right_duty_cycle > 10){
                        msg.cmd.right_duty_cycle -= 5;
                    }
                    else if(msg.cmd.right_duty_cycle < -10){
                        msg.cmd.right_duty_cycle += 5;
                    }
                }
                left_count = 0;
                right_count = 0;
            }
        }
        select{
            case iEncoder :> encoder_in:
                //encoder sending a changed signal
                if((encoder_in&1) != (prev_encoder_in&1)){
                    t :> last_change;
                    left_count++;
                    prev_encoder_in = encoder_in;
                    count++;
                }
                else if((encoder_in&0b10) != (prev_encoder_in&0b10)){
                    right_count++;
                    prev_encoder_in = encoder_in;
                }
                turn_count = right_count + left_count;
                break;

            case input :> msg:
                if(msg.data[0] == '?'){
                    sprintf(msg.data, "COUNTS: %i YAW: %f pitch: %f ROLL: %f\0", count, ypr.yaw, ypr.pitch, ypr.roll);
                    count = 0;
                }
                if(!strncmp(msg.data, "FORWARD UNTIL STOP", 18)){
                    t :> last_change;
                    sensitivity = forward_s;
                }
                if(!strncmp(msg.data, "UPDATE FORWARD SENSITIVITY", 26)){
                    forward_s = msg.sens;
                    sensitivity = forward_s;
                }
                if(!strncmp(msg.data, "REVERSE UNTIL STOP", 18)){
                    t :> last_change;
                    sensitivity = backward_s;
                }
                if(!strncmp(msg.data, "UPDATE REVERSE SENSITIVITY", 26)){
                    backward_s = msg.sens;
                    sensitivity = backward_s;
                }
                if(msg.turning){
                    starting_count = turn_count;
                }
                output <: msg;
                break;
//
            case dmp :> ypr:;
                break;

        }
            motor_c <: msg.cmd;

    }
}

//changed from UART lab, this method will run until a null terminator is found
void uart_transmit_bytes(out port oPort,
                         const char buffer[],
                         unsigned int baudrate){
    int i = 0;
    while(1){
        uart_transmit_byte(oPort, buffer[i], baudrate);

        if(buffer[i] == '\0'){
            return;
        }
        i++;
        //oPort <: 1;
    }
}

//uart receive byte from UART lab
char uart_receive_byte(in port iPort, unsigned int baudrate){
    int BIT_TIME_TICKS = (float)(TICKS_PER_SECOND/baudrate);
    char byte = 0;
    timer t;
    unsigned int time;
    iPort when pinseq(0) :> void;
    t :> time;
    time += BIT_TIME_TICKS/2;
    for(int i = 0; i<8; i++){
        time+=BIT_TIME_TICKS;
        t when timerafter(time) :> void;
        iPort :> >>byte;

    }

    time += BIT_TIME_TICKS;
    t when timerafter(time) :> void;
    iPort :> void;


    return byte;
}

//traditional uart transmission method from the UART lab
void uart_transmit_byte(out port oPort, char value, unsigned int baudrate){
    int BIT_TIME_TICKS = TICKS_PER_SECOND/baudrate;
    timer t;
    unsigned int time;
    t :> time;
    time += BIT_TIME_TICKS;
    oPort <: 0;
    t when timerafter(time) :> void;

    for(int i = 0; i<8; i++){
            oPort <: >> value;
            time += BIT_TIME_TICKS;
            t when timerafter(time) :> void;

    }

    oPort <: 1;
    time += BIT_TIME_TICKS;
    t when timerafter(time) :> void;
}

void toggle_port(out port oLED, unsigned int hz){
    int toggle = 1;
    unsigned int t_val;
    timer tmr;
    tmr :> t_val;
    while(1){
        t_val += (hz*TICKS_PER_SECOND);
        oLED <: toggle;
        tmr when timerafter(t_val) :> t_val;
        toggle = ~toggle;
    }
}

//constantly reads in from the wifi module (uart_receive_byte) builds a string
//once the string is finished process it, look for certain commands like the command to run the wifi module
//commands to move forwards, backwards, etc.
//sends movement commands or sends unknown inputs to console
void uart_to_console_task(chanend test){
    char buffer[BUFFER_LENGTH];
    int index = 0;
    message_t t;
    while(1){
        buffer[index] = uart_receive_byte(iWiFiTX, BAUDRATE);
        //change
        //if end line or carriage return, or out of space, process the buffer
        //out of space check is to prevent array index out of bounds
        //printstr(buffer);
        if(buffer[index] == '\n' || buffer[index] == '\r' || index == BUFFER_LENGTH-1){
            buffer[index] = '\0';
            if(!strncmp(buffer, "lua: cannot open init.lua", 25)){
                strcpy(t.data, "run_wifi_setup");
                t.cmd.left_duty_cycle = 0;
                t.cmd.right_duty_cycle = 0;
                t.turning = 0;
                t.driving = 0;
                t.until_stop = 0;
                test <: t;
            }
            else if(!strncmp(buffer, "\0", 1)){
                //fixes an issue where a null terminator comes through
                //and then gets output as UNKNOWN INPUT:
                //aesthetic fix
            }
            else if(!strncmp(buffer, "init", 4)){
                strcpy(t.data, buffer);
                test <: t;
            }
            else{
                if(!strncmp(buffer, "F", 1)){
                    if(buffer[1] == '\0'){
                        strcpy(t.data, "FULL FORWARD");
                        t.cmd.right_duty_cycle = FULL_FORWARD_R_DUTY_CYCLE;
                        t.cmd.left_duty_cycle = FULL_FORWARD_L_DUTY_CYCLE;
                    }
                    else{
                        char temp[3] = {0, 0, 0};
                        for(int i = 0; i<3; i++){
                            temp[i] = buffer[i+1];
                            if(temp[i] == '\0'){
                                break;
                            }
                        }
                        int d_c = 0;
                        sscanf(temp, "%u", &d_c);
                        t.cmd.right_duty_cycle = d_c;
                        t.cmd.left_duty_cycle = -1 * d_c;
                        sprintf(t.data, "FORWARD AT %u%", d_c);
                    }
                    t.turning = 0;
                    t.driving = 1;
                    t.until_stop = 0;
                }
                else if(!strncmp(buffer, "f\0", 2)){
                    strcpy(t.data, "HALF FORWARD");
                    t.cmd.right_duty_cycle = HALF_FORWARD_R_DUTY_CYCLE;
                    t.cmd.left_duty_cycle = HALF_FORWARD_L_DUTY_CYCLE;
                    t.turning = 0;
                    t.driving = 1;
                    t.until_stop = 0;
                }
                else if(!strncmp(buffer, "R", 1)){
                    if(buffer[1] == '\0'){
                        strcpy(t.data, "FULL REVERSE");
                        t.cmd.right_duty_cycle = FULL_REVERSE_R_DUTY_CYCLE;
                        t.cmd.left_duty_cycle = FULL_REVERSE_L_DUTY_CYCLE;
                    }
                    else{
                        char temp[3] = {0, 0, 0};
                        for(int i = 0; i<3; i++){
                            temp[i] = buffer[i+1];
                            if(temp[i] == '\0'){
                                break;
                            }
                        }
                        int d_c = 0;
                        sscanf(temp, "%u", &d_c);
                        t.cmd.right_duty_cycle = -1 * d_c;
                        t.cmd.left_duty_cycle = d_c;
                        sprintf(t.data, "REVERSE AT %u%", d_c);
                    }
                    t.turning = 0;
                    t.driving = 1;
                    t.until_stop = 0;
                }
                else if(!strncmp(buffer, "r\0", 2)){
                    strcpy(t.data, "HALF REVERSE");
                    t.cmd.right_duty_cycle = HALF_REVERSE_R_DUTY_CYCLE;
                    t.cmd.left_duty_cycle = HALF_REVERSE_L_DUTY_CYCLE;
                    t.turning = 0;
                    t.driving = 1;
                    t.until_stop = 0;
                }
                else if(!strncmp(buffer, "<", 1)){
                    unsigned int i = 0;
                    for(i; i<10; ++i){
                        if(buffer[i] != '<'){
                            break;
                        }
                    }
                    unsigned int d_c = i * 10;
                    sprintf(t.data, "TURN LEFT AT %u", d_c);
                    t.cmd.right_duty_cycle = -d_c;
                    t.cmd.left_duty_cycle = -d_c;
                    t.turning = 0;
                    t.driving = 0;
                    t.until_stop = 0;
                }
                else if(!strncmp(buffer, ">", 1)){
                    unsigned int i = 0;
                    for(i; i<10; ++i){
                        if(buffer[i] != '>'){
                            break;
                        }
                    }
                    unsigned int d_c = i * 10;
                    sprintf(t.data, "TURN RIGHT AT %u", d_c);
                    t.cmd.right_duty_cycle = d_c;
                    t.cmd.left_duty_cycle = d_c;
                    t.turning = 0;
                    t.driving = 0;
                    t.until_stop = 0;
                }
                else if(!strncmp(buffer, "x\0", 2)){
                    strcpy(t.data, "STOP");
                    t.cmd.right_duty_cycle = 0;
                    t.cmd.left_duty_cycle = 0;
                    t.turning = 0;
                    t.driving = 0;
                    t.until_stop = 0;
                }
                else if(!strncmp(buffer, "T", 1)){
                    unsigned int d_c;
                    char temp[3] = {0, 0, 0};
                    for(int i = 0; i<3; ++i){
                        temp[i] = buffer[i+1];
                        if(buffer[i+1] == '\0'){
                            if(i == 0){
                                temp[0] = 0;
                                temp[1] = '\0';
                            }
                            break;
                        }
                    }
                    sscanf(temp, "%u", &d_c);
                    strcpy(t.data, "FORWARD UNTIL STOP");
                    t.cmd.right_duty_cycle = d_c;
                    t.cmd.left_duty_cycle = -1*d_c;
                    t.turning = 0;
                    t.driving = 1;
                    t.until_stop = 1;
                }
                else if(!strncmp(buffer, "S", 1)){
                    float s;
                    char temp[3] = {0, 0, 0};
                    for(int i = 0; i<3; ++i){
                        temp[i] = buffer[i+1];
                        if(buffer[i+1] == '\0'){
                            if(i == 0){
                                temp[0] = 0;
                                temp[1] = '\0';
                            }
                            break;
                        }
                    }
                    sscanf(temp, "%f", &s);
                    strcpy(t.data, "UPDATE FORWARD SENSITIVITY");
                    t.sens = ENCODER_DELTA * (s/100);
                }
                else if(!strncmp(buffer, "s", 1)){
                    float s;
                    char temp[3] = {0, 0, 0};
                    for(int i = 0; i<3; ++i){
                        temp[i] = buffer[i+1];
                        if(buffer[i+1] == '\0'){
                            if(i == 0){
                                temp[0] = 0;
                                temp[1] = '\0';
                            }
                            break;
                        }
                    }
                    sscanf(temp, "%f", &s);
                    strcpy(t.data, "UPDATE REVERSE SENSITIVITY");
                    t.sens = ENCODER_DELTA * (s/100);
                }
                else if(!strncmp(buffer, "t", 1)){
                    unsigned int d_c;
                    char temp[3] = {0, 0, 0};
                    for(int i = 0; i<3; ++i){
                        temp[i] = buffer[i+1];
                        if(buffer[i+1] == '\0'){
                            if(i == 0){
                                temp[0] = 0;
                                temp[1] = '\0';
                            }
                            break;
                        }
                    }
                    sscanf(temp, "%u", &d_c);
                    strcpy(t.data, "REVERSE UNTIL STOP");
                    t.cmd.right_duty_cycle = -1*d_c;
                    t.cmd.left_duty_cycle = d_c;
                    t.turning = 0;
                    t.driving = 1;
                    t.until_stop = 1;
                }
                else if(!strncmp(buffer, ",\0", 2)){
                    strcpy(t.data, "LEFT 90 DEGREES");
                    t.cmd.right_duty_cycle = LTURN_R_DUTY_CYCLE;
                    t.cmd.left_duty_cycle = LTURN_L_DUTY_CYCLE;
                    t.turning = 1;
                    t.driving = 0;
                    t.until_stop = 0;
                }
                else if(!strncmp(buffer, ".\0", 2)){
                    strcpy(t.data, "RIGHT 90 DEGREES");
                    t.cmd.right_duty_cycle = RTURN_R_DUTY_CYCLE;
                    t.cmd.left_duty_cycle = RTURN_L_DUTY_CYCLE;
                    t.turning = 1;
                    t.driving = 0;
                    t.until_stop = 0;
                }
                else if(!strncmp(buffer, "?\0", 2)){
                    strcpy(t.data, "?");
                }
                else{
                  strcpy(t.data, "UNKNOWN INPUT: ");
                  strcat(t.data, buffer);
                }
                test <: t;
            }
            for(int i = 0; i<BUFFER_LENGTH; i++){
                buffer[i] = 0;
                t.data[i] = 0;
            }
            index = 0;
        }
        else{
            index++;
        }
    }
}


void line(const char buffer[]){
    timer t;
    unsigned int t_val;
    t :> t_val;
    t when timerafter(t_val + BUFFER_DELAY) :> void;
    uart_transmit_bytes(oWiFiRX, buffer, BAUDRATE);
    //sending "\r\n" as two bytes was giving me errors,
    //sending one byte at a time fixed it.
    uart_transmit_byte(oWiFiRX, '\r', BAUDRATE);
    uart_transmit_byte(oWiFiRX, '\n', BAUDRATE);

}

void output_task(chanend trigger_chan){
    message_t t;
    while(1){
        trigger_chan :> t;
        //if told to start wifi, start wifi
        //otherwise print out message as a debug
        if(!strncmp(t.data, "run_wifi_setup", 14)){
            line("dofile('wifi.lua')");
        }
        else{
            line(t.data);
        }

    }

}
//
//void run_wifi_program(){
//    line("dofile('wifi.lua')");
//}


//now just takes a command and sends it along,
//maybe should have just sent the commands through the port in the parent method
//but I had this from previous labs and it makes sense to me even though I changed it.
//all of these functions are from the multi motor lab
//none of them have been changed
void PWM_Frame(int r_up_time, int l_up_time, out port oRightPWM, out port oLeftPWM, chanend in_motor_cmd_chan, motor_cmd_t &cmd){
    int t_waiting;
    int l_waiting = 1;
    int r_waiting = 1;
    timer tmr_total, tmr_right, tmr_left;
    unsigned int total_val, right_val, left_val;
    tmr_total :> total_val;
    tmr_right :> right_val;
    tmr_left :> left_val;

    //if 0 percent duty cycle then go to zero, else go to one
    if(r_up_time == 0){
        oRightPWM <: 0;
    }
    else{
        oRightPWM <: 1;
    }
    if(l_up_time == 0){
        oLeftPWM <: 0;
    }
    else{
        oLeftPWM <: 1;
    }

    while(l_waiting || r_waiting){
        select{
        case tmr_right when timerafter(right_val + r_up_time) :> void:
                oRightPWM <: 0;
                r_waiting = 0;
                //can do this because i resample the timer every function call. This prevents only one case from being
                //selected every time
                //theres no way for this timeout to happen twice
                //because both right_val and left_val are less than or equal to PWM_FRAME_TICKs
                right_val += 2*PWM_FRAME_TICKS;
                break;
        case tmr_left when timerafter(left_val + l_up_time) :> void:
                oLeftPWM <: 0;
                l_waiting = 0;
                left_val += 2*PWM_FRAME_TICKS;
                break;
        case in_motor_cmd_chan :> cmd:
            break;
        }
    }
    t_waiting = 1;
    while(t_waiting){
        select{
            //down time
        case tmr_total when timerafter(total_val + PWM_FRAME_TICKS) :> void:
                t_waiting = 0;
                break;
                //keep track of the incoming channel during down time
        case in_motor_cmd_chan :> cmd:
            break;
        }
    }
}

unsigned int compute_mask (int right_duty, int left_duty){
    unsigned int ret = 0b0;

    if(right_duty >= 0){
        ret = ret|AIN1_ON;
    }
    else{
        ret = ret|AIN2_ON;
    }
    if(left_duty >= 0){
        ret = ret|BIN1_ON;
    }
    else{
        ret = ret|BIN2_ON;
    }
    return ret;
}

void multi_motor_task(out port oLeftPWM, out port oRightPWM,
        out port oMotorControl,
        chanend in_motor_cmd_chan){
    motor_cmd_t cmd;
    int in_mask = 0b0;
    int r_duty_cycle, l_duty_cycle;
    oSTBY <: 1;
    int r_up_time, l_up_time;
    in_motor_cmd_chan :> cmd;
    while(1){
        r_duty_cycle = cmd.right_duty_cycle;
        l_duty_cycle = cmd.left_duty_cycle;

        in_mask = compute_mask(r_duty_cycle, l_duty_cycle);
        oMotorControl <: in_mask;

        r_up_time = (PWM_FRAME_TICKS/100)*r_duty_cycle;
        l_up_time = (PWM_FRAME_TICKS/100)*l_duty_cycle;

        //up times need to be positive values, even if the duty cycle is negative
        if(r_up_time < 0){
            r_up_time *= -1;
        }
        if(l_up_time < 0){
            l_up_time *= -1;
        }

        PWM_Frame(r_up_time, l_up_time, oRightPWM, oLeftPWM, in_motor_cmd_chan, cmd);


   }

}

//void imu_task(chanend dmp_out){
//    //does all MPU6050 actions
////    timer tmr;
////    unsigned int t_val;
////    t :> t_val;
//    ypr_t t;
//    int packetsize,mpuIntStatus,fifoCount;
//    int address;
//    unsigned char result[64];                           //holds dmp packet of data
//    //float qtest;
//    float q[4]={0,0,0,0},g[3]={0,0,0},euler[3]={0,0,0},ypr[3]={0,0,0};
//    int but_state;
//    int fifooverflowcount=0;
//    int GO_FLAG=1;
//    float readings[5] = {0, 0, 0, 0, 0};
//    int count = 0;
//    float avg = 0;
//    //printf("Starting MPU6050...\n");
//    mpu_init_i2c(imu);
//    //printf("I2C Initialized...\n");
//    address=mpu_read_byte(imu.i2c, MPU6050_RA_WHO_AM_I);
//    //printf("MPU6050 at i2c address: %.2x\n",address);
//    mpu_dmpInitialize(imu);
//    //printf("ENABLING DMP\n");
//    mpu_enableDMP(imu,1);   //enable DMP
//    //printf("done\n");
//    mpuIntStatus=mpu_read_byte(imu.i2c,MPU6050_RA_INT_STATUS);
//    //printf("MPU Interrupt Status:%d\n",mpuIntStatus);
//    packetsize=42;                  //size of the fifo buffer
//    delay_milliseconds(250);
//
//    //The hardware interrupt line is not used, the FIFO buffer is polled
//    while (GO_FLAG){
//        mpuIntStatus=mpu_read_byte(imu.i2c,MPU6050_RA_INT_STATUS);
//        if (mpuIntStatus >= 2) {
//            fifoCount = mpu_read_short(imu.i2c,MPU6050_RA_FIFO_COUNTH);
//            if (fifoCount>=1024) { //fifo overflow
//                mpu_resetFifo(imu);
//                fifooverflowcount+=1;           //keep track of how often this happens to tweak parameters
//                //printf("FIFO Overflow!\n");
//            }
//            while (fifoCount < packetsize) {    //wait for a full packet in FIFO buffer
//                fifoCount = mpu_read_short(imu.i2c,MPU6050_RA_FIFO_COUNTH);
//            }
//            //printf("fifoCount:%d\n",fifoCount);
//            mpu_getFIFOBytes(imu,packetsize,result);    //retrieve the packet from FIFO buffer
//
//            mpu_getQuaternion(result,q);
//            //qtest=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
//            //if (fabs(qtest-1.0)<0.001){                             //check for fifo corruption - quat should be unit quat
////                dmp_out <: q[0];
////                dmp_out <: q[1];
////                dmp_out <: q[2];
////                dmp_out <: q[3];
////
//                 mpu_getGravity(q,g);
////                dmp_out <: g[0];
////                dmp_out <: g[1];
////                dmp_out <: g[2];
////
//                 mpu_getEuler(euler,q);
////                dmp_out <: euler[0];
////                dmp_out <: euler[1];
////                dmp_out <: euler[2];
//
//                mpu_getYawPitchRoll(q,g,ypr);
//                //dmp_out <: ypr[0];
//                //dmp_out <: ypr[1];
//                //dmp_out <: ypr[2];
//
//                t.yaw = ypr[0];
//                t.pitch = ypr[1];
//                t.roll = ypr[2];
//                readings[count%5] = ypr[0];
//                count++;
//                avg = 0;
//                for(int i = 0; i<5; ++i){
//                    avg+=readings[i];
//                }
//                avg = avg/5.0;
//                t.yaw = avg;
//                //still getting completely random yaw readings
//
//                //average is calculated after every reading.
//
//                //if(count%5 == 0 && count != 0){
//                    dmp_out <: t;
//                //}
//
//       }
//       butP :> but_state;               //check to see if button is pushed to end program, low means pushed
//       but_state &=0x1;
//       if (but_state==0){
//           //printf("Exiting...\n");
//           GO_FLAG=0;
//       }
//    }
//    mpu_Stop(imu);      //reset hardware gracefully and put into sleep mode
//    //printf("Fifo Overflows:%d Fifo Corruptions:%d\n",fifooverflowcount,fifocorrupt);
//    exit(0);
//}

