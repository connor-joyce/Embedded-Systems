/*
 * Lab07-Turn90.xc
 *
 *  Created on: Nov 6, 2018
 *      Author: root
 */



#include <xs1.h>
#include <string.h>
#include <stdio.h>
#include <print.h>
#include <math.h>
#include <platform.h>
#include <stdlib.h>

#include "mpu6050.h"
#include "i2c.h"


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

//define all used duty cycles
#define FULL_FORWARD_L_DUTY_CYCLE -100
#define FULL_FORWARD_R_DUTY_CYCLE 100

#define HALF_FORWARD_L_DUTY_CYCLE -50
#define HALF_FORWARD_R_DUTY_CYCLE 50

#define FULL_REVERSE_L_DUTY_CYCLE 100
#define FULL_REVERSE_R_DUTY_CYCLE -100

#define HALF_REVERSE_L_DUTY_CYCLE 50
#define HALF_REVERSE_R_DUTY_CYCLE -50

#define RTURN_R_DUTY_CYCLE 50
#define RTURN_L_DUTY_CYCLE 50

#define LTURN_L_DUTY_CYCLE -50
#define LTURN_R_DUTY_CYCLE -50

#define IMU_TIMEOUT TICKS_PER_MS*3;


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

typedef struct{
    int vals[3];
    int count;

}encoder_m;

//structure to carry a string
typedef struct{
    char data[BUFFER_LENGTH];
} message_t;

void toggle_port(out port oLED, unsigned int hz);
void line(const char buffer[]);
void uart_transmit_byte(out port oPort, char value, unsigned int baudrate);
char uart_receive_byte(in port iPort, unsigned int baudrate);
void uart_transmit_bytes(out port oPort, const char buffer[], unsigned int baudrate);
void uart_to_console_task(chanend trigger_chan, chanend motor_chan, chanend encoder_chan, chanend encoder_counter, chanend yaw_output);
void output_task(chanend trigger_chan);
void multi_motor_task(out port oLeftPWM, out port oRightPWM,
        out port oMotorControl,
        chanend in_motor_cmd_chan);
unsigned int compute_mask (int right_duty, int left_duty);
void PWM_Frame(int r_up_time, int l_up_time, out port oRightPWM, out port oLeftPWM, chanend in_motor_cmd_chan, motor_cmd_t &cmd);
void move_controller(chanend c, motor_cmd_t t);
void run_wifi_program();
void encoder_task(in port iEncoder, chanend encoder_chan, chanend encoder_count, chanend dmp_in, chanend yaw_output);
void imu_task(chanend dmp_out);
void filter(chanend imu_chan, chanend motor_chan, chanend console_chan);




int main(){
    oWiFiRX <: 1;
    chan c;
    chan motor_c;
    chan encoder_chan;
    chan encoder_counter;
    chan dmp;
    chan yaw_output;
    par{
        toggle_port(oLED, 2);
        uart_to_console_task(c, motor_c, encoder_chan, encoder_counter, yaw_output);
        output_task(c);
        multi_motor_task(oLPWM, oRPWM, oMotorControl, motor_c);
        encoder_task(iEncoder, encoder_chan, encoder_counter, dmp, yaw_output);
        imu_task(dmp);
    }

    return 0;
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
    unsigned int time, timeout;
    t :> timeout;
    //added timeout so that the yaw can be checked without waiting for input through wifi to run the loop.
    //the loop in the output to console task was getting hung up waiting for this method to finish
    //return tilda as a timeout notification (pray nothing else sends it), then just write over the same
    //index after
    select{
        case iPort when pinseq (0) :> void:
            break;
        case t when timerafter (timeout + BIT_TIME_TICKS) :> void:
                return '~';
                break;
    }
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
void uart_to_console_task(chanend trigger_chan, chanend motor_chan, chanend encoder_chan, chanend encoder_change,
        chanend yaw_output){
    char buffer[BUFFER_LENGTH];
    motor_cmd_t cmd;
    unsigned int encoder_output;
    int index = 0;
    message_t t;
    ypr_t ypr;
    int straight_drive = 0;
    float target_yaw_drive = 0;
    int turn = 0;
    float starting_yaw = 0;
    while(1){
        buffer[index] = uart_receive_byte(iWiFiTX, BAUDRATE);
        //get yaw at every loop to determine if turn is done, or car off course
        float yaw = 1;
        yaw_output <: 2;
        yaw_output :> yaw;
        if(turn){

            //if the machine has turned roughly 90 degrees in either direction
            if(yaw >= starting_yaw + TURN_ARC || yaw <= starting_yaw - TURN_ARC){
                //stop and set all modes to 0
                starting_yaw = 0;
                cmd.right_duty_cycle = 0;
                cmd.left_duty_cycle = 0;
                straight_drive = 0;
                turn = 0;
                move_controller(motor_chan, cmd);
            }
        }

        //if end line or carriage return, or out of space, process the buffer
        //out of space check is to prevent array index out of bounds
        if(buffer[index] == '\n' || buffer[index] == '\r' || index == BUFFER_LENGTH-1){
            buffer[index] = '\0';
            if(!strncmp(buffer, "lua: cannot open init.lua", 25)){
                strcpy(t.data, "run_wifi_setup");
                trigger_chan <: t;
            }
            else if(!strncmp(buffer, "\0", 1)){
                //fixes an issue where a null terminator comes through
                //and then gets output as UNKNOWN INPUT:
                //aesthetic fix
            }
            else{
                if(!strncmp(buffer, "F\0", 2)){
                    strcpy(t.data, "FULL FORWARD");
                    cmd.right_duty_cycle = FULL_FORWARD_R_DUTY_CYCLE;
                    cmd.left_duty_cycle = FULL_FORWARD_L_DUTY_CYCLE;
                    straight_drive = 1;
                    turn = 0;
                    target_yaw_drive = yaw;

                }
                else if(!strncmp(buffer, "f\0", 2)){
                    strcpy(t.data, "HALF FORWARD");
                    cmd.right_duty_cycle = HALF_FORWARD_R_DUTY_CYCLE;
                    cmd.left_duty_cycle = HALF_FORWARD_L_DUTY_CYCLE;
                    straight_drive = 1;
                    turn = 0;
                    target_yaw_drive = yaw;
                }
                else if(!strncmp(buffer, "R\0", 2)){
                    strcpy(t.data, "FULL REVERSE");
                    cmd.right_duty_cycle = FULL_REVERSE_R_DUTY_CYCLE;
                    cmd.left_duty_cycle = FULL_REVERSE_L_DUTY_CYCLE;
                    straight_drive = 1;
                    turn = 0;
                    target_yaw_drive = yaw;
                }
                else if(!strncmp(buffer, "r\0", 2)){
                    strcpy(t.data, "HALF REVERSE");
                    cmd.right_duty_cycle = HALF_REVERSE_R_DUTY_CYCLE;
                    cmd.left_duty_cycle = HALF_REVERSE_L_DUTY_CYCLE;
                    straight_drive = 1;
                    turn = 0;
                    target_yaw_drive = yaw;
                }
                else if(!strncmp(buffer, "<\0", 2)){
                    strcpy(t.data, "TURN LEFT");
                    cmd.right_duty_cycle = LTURN_R_DUTY_CYCLE;
                    cmd.left_duty_cycle = LTURN_L_DUTY_CYCLE;
                    straight_drive = 0;
                    turn = 0;
                }
                else if(!strncmp(buffer, ">\0", 2)){
                    strcpy(t.data, "TURN RIGHT");
                    cmd.right_duty_cycle = RTURN_R_DUTY_CYCLE;
                    cmd.left_duty_cycle = RTURN_L_DUTY_CYCLE;
                    straight_drive = 0;
                    turn = 0;
                }
                else if(!strncmp(buffer, "x\0", 2)){
                    strcpy(t.data, "STOP");
                    cmd.right_duty_cycle = 0;
                    cmd.left_duty_cycle = 0;
                    straight_drive = 0;
                    turn = 0;
                }
                else if(!strncmp(buffer, "?\0", 2)){
                    char temp[128];
                    encoder_chan <: 1;
                    encoder_chan :> encoder_output;
                    //encoder continually updates the current yaw pitch roll values and waits to send the
                    //ypr struct down this channel when called in this way
                    encoder_chan :> ypr;
                    sprintf(temp, "ENCODER TICKS SINCE LAST CALLED %u\nYAW: %f PITCH: %f ROLL: %f", encoder_output, ypr.yaw,
                            ypr.pitch, ypr.roll);
                    strcpy(t.data, temp);
                }
                else if(!strncmp(buffer, ",\0", 2)){
                    strcpy(t.data, "90 DEGREE LEFT TURN");
                    cmd.right_duty_cycle = LTURN_R_DUTY_CYCLE;
                    cmd.left_duty_cycle = LTURN_L_DUTY_CYCLE;
                    straight_drive = 0;
                    turn = 1;
                    starting_yaw = yaw;
                }
                else if(!strncmp(buffer, ".\0", 2)){
                    strcpy(t.data, "90 DEGREE RIGHT TURN");
                    cmd.right_duty_cycle = RTURN_R_DUTY_CYCLE;
                    cmd.left_duty_cycle = RTURN_L_DUTY_CYCLE;
                    straight_drive = 0;
                    turn = 1;
                    starting_yaw = yaw;
                }
                else{
                  strcpy(t.data, "UNKNOWN INPUT: ");
                  strcat(t.data, buffer);
                }
                trigger_chan <: t;
            }


            for(int i = 0; i<BUFFER_LENGTH; i++){
                buffer[i] = 0;
                t.data[i] = 0;
            }
            index = 0;
        }
        else{
            //since the receive bytes can now timeout I have to check if it timed out
            //I pass a tilda when I time out
            //only increment if any data was actually received.
            if(buffer[index] != '~'){
                index++;
            }
        }

        move_controller(motor_chan, cmd);
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
            run_wifi_program();
        }
        else{
            line(t.data);
        }

    }

}

void run_wifi_program(){
    line("dofile('wifi.lua')");
}

//encoder task is now my hub for IMU information, since it runs constantly and isn't required to send info down channels a whole lot
//stores most recent ypr struct, and sends yaw info down a channel to the output task, and a ypr struct down another
//use two separate channels so that the count variable isn't reset
void encoder_task(in port iEncoder, chanend encoder_chan, chanend encoder_count, chanend dmp_in, chanend yaw_output){
    unsigned int curr_count = 0;
    int ready, enc_input, prev_input;
    ypr_t t;
    t.yaw = 0;
    t.pitch = 0;
    t.roll = 0;
    while(1){
        select{
            case dmp_in :> t:
                break;

            case encoder_chan :> ready:
                //when an integer is sent down this channel, encoder_task sends back current count
                //and then resets it
                encoder_chan <: curr_count;
                encoder_chan <: t;
                curr_count = 0;
                break;
            case iEncoder :> enc_input:
                //every time the encoder sends a signal, we check to see if the 0th bit has changed
                //this is just the bit I'm counting, if it has changed update the prev_input and update count
                if((enc_input&1) != (prev_input&1)){
                    prev_input = enc_input;
                    curr_count++;
                }
                break;
            case yaw_output :> ready:
                yaw_output <: t.yaw;
                break;

        }
    }
}

//now just takes a command and sends it along,
//maybe should have just sent the commands through the port in the parent method
//but I had this from previous labs and it makes sense to me even though I changed it.
void move_controller(chanend to_motor, motor_cmd_t t){
    to_motor <: t;
}

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

void imu_task(chanend dmp_out){
    //does all MPU6050 actions
//    timer tmr;
//    unsigned int t_val;
//    t :> t_val;
    ypr_t t;
    int packetsize,mpuIntStatus,fifoCount;
    int address;
    unsigned char result[64];                           //holds dmp packet of data
    //float qtest;
    float q[4]={0,0,0,0},g[3]={0,0,0},euler[3]={0,0,0},ypr[3]={0,0,0};
    int but_state;
    int fifooverflowcount=0,fifocorrupt=0;
    int GO_FLAG=1;
    float readings[5] = {0, 0, 0, 0, 0};
    int count = 0;
    float avg = 0;
    //printf("Starting MPU6050...\n");
    mpu_init_i2c(imu);
    //printf("I2C Initialized...\n");
    address=mpu_read_byte(imu.i2c, MPU6050_RA_WHO_AM_I);
    //printf("MPU6050 at i2c address: %.2x\n",address);
    mpu_dmpInitialize(imu);
    mpu_enableDMP(imu,1);   //enable DMP

    mpuIntStatus=mpu_read_byte(imu.i2c,MPU6050_RA_INT_STATUS);
    //printf("MPU Interrupt Status:%d\n",mpuIntStatus);
    packetsize=42;                  //size of the fifo buffer
    delay_milliseconds(250);

    //The hardware interrupt line is not used, the FIFO buffer is polled
    while (GO_FLAG){
        mpuIntStatus=mpu_read_byte(imu.i2c,MPU6050_RA_INT_STATUS);
        if (mpuIntStatus >= 2) {
            fifoCount = mpu_read_short(imu.i2c,MPU6050_RA_FIFO_COUNTH);
            if (fifoCount>=1024) {              //fifo overflow
                mpu_resetFifo(imu);
                fifooverflowcount+=1;           //keep track of how often this happens to tweak parameters
                //printf("FIFO Overflow!\n");
            }
            while (fifoCount < packetsize) {    //wait for a full packet in FIFO buffer
                fifoCount = mpu_read_short(imu.i2c,MPU6050_RA_FIFO_COUNTH);
            }
            //printf("fifoCount:%d\n",fifoCount);
            mpu_getFIFOBytes(imu,packetsize,result);    //retrieve the packet from FIFO buffer

            mpu_getQuaternion(result,q);
            //qtest=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
            //if (fabs(qtest-1.0)<0.001){                             //check for fifo corruption - quat should be unit quat
//                dmp_out <: q[0];
//                dmp_out <: q[1];
//                dmp_out <: q[2];
//                dmp_out <: q[3];
//
                 mpu_getGravity(q,g);
//                dmp_out <: g[0];
//                dmp_out <: g[1];
//                dmp_out <: g[2];
//
                 mpu_getEuler(euler,q);
//                dmp_out <: euler[0];
//                dmp_out <: euler[1];
//                dmp_out <: euler[2];

                mpu_getYawPitchRoll(q,g,ypr);
                //dmp_out <: ypr[0];
                //dmp_out <: ypr[1];
                //dmp_out <: ypr[2];

                t.yaw = ypr[0];
                t.pitch = ypr[1];
                t.roll = ypr[2];
                readings[count%5] = t.yaw;
                count++;
                for(int i = 0; i<5; ++i){
                    avg+=readings[i];
                }
                avg/=5;
                t.yaw = avg;
                //average is calculated after every reading.

                char temp[64];
                sprintf(temp, "%f\n", avg);
                printstr(temp);
                dmp_out <: t;

       }
       butP :> but_state;               //check to see if button is pushed to end program, low means pushed
       but_state &=0x1;
       if (but_state==0){
           //printf("Exiting...\n");
           GO_FLAG=0;
       }
    }
    mpu_Stop(imu);      //reset hardware gracefully and put into sleep mode
    //printf("Fifo Overflows:%d Fifo Corruptions:%d\n",fifooverflowcount,fifocorrupt);
    exit(0);
}

