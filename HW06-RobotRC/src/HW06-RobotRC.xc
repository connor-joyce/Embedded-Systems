/*
 * HW06-RobotRC.xc
 *
 *  Created on: Oct 20, 2018
 *      Author: root
 */

#include <xs1.h>
#include <print.h>
#include <string.h>
#include <stdio.h>


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
#define BUFFER_DELAY TICKS_PER_SECOND/8
#define TICKS_PER_MS TICKS_PER_SECOND/1000
#define PWM_FRAME_TICKS TICKS_PER_MS

//define all used duty cycles
#define FULL_FORWARD_L_DUTY_CYCLE 100
#define FULL_FORWARD_R_DUTY_CYCLE -100

#define HALF_FORWARD_L_DUTY_CYCLE 50
#define HALF_FORWARD_R_DUTY_CYCLE -50

#define FULL_REVERSE_L_DUTY_CYCLE -100
#define FULL_REVERSE_R_DUTY_CYCLE 100

#define HALF_REVERSE_L_DUTY_CYCLE -50
#define HALF_REVERSE_R_DUTY_CYCLE 50

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

//structure for motor command
typedef struct{
    int left_duty_cycle;
    int right_duty_cycle;
}motor_cmd_t;

//structure to carry a string
typedef struct{
    char data[BUFFER_LENGTH];
} message_t;

void toggle_port(out port oLED, unsigned int hz);
void line(const char buffer[]);
void uart_transmit_byte(out port oPort, char value, unsigned int baudrate);
char uart_receive_byte(in port iPort, unsigned int baudrate);
void uart_transmit_bytes(out port oPort, const char buffer[], unsigned int baudrate);
void uart_to_console_task(chanend trigger_chan, chanend motor_chan, chanend encoder_chan, chanend encoder_counter);
void output_task(chanend trigger_chan);
void multi_motor_task(out port oLeftPWM, out port oRightPWM,
        out port oMotorControl,
        chanend in_motor_cmd_chan);
unsigned int compute_mask (int right_duty, int left_duty);
void PWM_Frame(int r_up_time, int l_up_time, out port oRightPWM, out port oLeftPWM, chanend in_motor_cmd_chan, motor_cmd_t &cmd);
void move_controller(chanend c, int r, int l);
void run_wifi_program();
void encoder_task(in port iEncoder, chanend encoder_chan, chanend encoder_count);



int main(){
    oWiFiRX <: 1;
    chan c;
    chan motor_c;
    chan encoder_chan;
    chan encoder_counter;
    par{
        toggle_port(oLED, 2);
        uart_to_console_task(c, motor_c, encoder_chan, encoder_counter);
        output_task(c);
        multi_motor_task(oLPWM, oRPWM, oMotorControl, motor_c);
        encoder_task(iEncoder, encoder_chan, encoder_counter);
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
    unsigned int time;
    iPort when pinseq (0) :> void;
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
void uart_to_console_task(chanend trigger_chan, chanend motor_chan, chanend encoder_chan, chanend encoder_change){
    char buffer[BUFFER_LENGTH];
    int encoder_output = 0;
    int index = 0;
    message_t t;
    while(1){
        buffer[index] = uart_receive_byte(iWiFiTX, BAUDRATE);
        //if end line or carriage return, or out of space, process the buffer
        //out of space check is to prevent array index out of bounds
        if(buffer[index] == '\n' || buffer[index] == '\r' || index == BUFFER_LENGTH-1){
            buffer[index] = '\0';
            if(!strncmp(buffer, "lua: cannot open init.lua", 25)){
                strcpy(t.data, "run_wifi_setup");
                trigger_chan <: t;
                printstrln(buffer);
            }
            else if(!strncmp(buffer, "\0", 1)){
                //fixes an issue where a null terminator comes through
                //and then gets output as UNKNOWN INPUT:
                //aesthetic fix
            }
            else{
                if(!strncmp(buffer, "F\0", 2)){
                    strcpy(t.data, "FULL FORWARD");
                    //full forward until stopped
                    move_controller(motor_chan, FULL_FORWARD_R_DUTY_CYCLE, FULL_FORWARD_L_DUTY_CYCLE);
                }
                else if(!strncmp(buffer, "f\0", 2)){
                    strcpy(t.data, "HALF FORWARD");
                    //half forward until stopped
                    move_controller(motor_chan, HALF_FORWARD_R_DUTY_CYCLE, HALF_FORWARD_L_DUTY_CYCLE);
                }
                else if(!strncmp(buffer, "R\0", 2)){
                    strcpy(t.data, "FULL REVERSE");
                    //full reverse until stopped
                    move_controller(motor_chan, FULL_REVERSE_R_DUTY_CYCLE, FULL_REVERSE_L_DUTY_CYCLE);
                }
                else if(!strncmp(buffer, "r\0", 2)){
                    strcpy(t.data, "HALF REVERSE");
                    //half reverse until stopped
                    move_controller(motor_chan, HALF_REVERSE_R_DUTY_CYCLE, HALF_REVERSE_L_DUTY_CYCLE);
                }
                else if(!strncmp(buffer, "<\0", 2)){
                    strcpy(t.data, "TURN LEFT");
                    //turn left until stopped
                    move_controller(motor_chan, LTURN_R_DUTY_CYCLE, LTURN_L_DUTY_CYCLE);
                }
                else if(!strncmp(buffer, ">\0", 2)){
                    strcpy(t.data, "TURN RIGHT");
                    //turn right until stopped
                    move_controller(motor_chan, RTURN_R_DUTY_CYCLE, RTURN_L_DUTY_CYCLE);
                }
                else if(!strncmp(buffer, "x\0", 2)){
                    strcpy(t.data, "STOP");
                    //stop
                    move_controller(motor_chan, 0, 0);
                }
                else if(!strncmp(buffer, "?\0", 2)){
                    char temp[10];
                    encoder_chan <: 1;
                    encoder_chan :> encoder_output;
                    strcpy(t.data, "ENCODER TICKS SINCE LAST CALLED ");
                    //sprintf(storage, type, value)
                    sprintf(temp, "%u", encoder_output);
                    strcat(t.data, temp);
                }
                else{
                  strcpy(t.data, "UNKNOWN INPUT: ");
                  strcat(t.data, buffer);
                }
                trigger_chan <: t;

            }
            for(int i = 0; i<BUFFER_LENGTH; i++){
                buffer[i] = 0;
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
    char temp[BUFFER_LENGTH];
    unsigned int t_val;
    t :> t_val;
    strcpy(temp, buffer);
    strcat(temp, "\r\n");
    t when timerafter(t_val + BUFFER_DELAY) :> void;
    uart_transmit_bytes(oWiFiRX, temp, BAUDRATE);
    //sending "\r\n" as two bytes was giving me errors,
    //sending one byte at a time fixed it.
    //uart_transmit_byte(oWiFiRX, '\r', BAUDRATE);
    //uart_transmit_byte(oWiFiRX, '\n', BAUDRATE);

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
            //print not line, line sends a command back to the Wifi module
            //which would then  get read as an unknown command and come right back here
            //infinite cycle
            //printstrln(t.data);

            //TEST THIS i think it causes an infinite loop
            //once this works you're done
            //printf("test");
            line(t.data);
        }

    }

}

void run_wifi_program(){
    line("dofile('wifi.lua')");
}

void encoder_task(in port iEncoder, chanend encoder_chan, chanend encoder_count){
    int curr_count = 0;
    int ready, enc_input, prev_input;
    while(1){
        select{
            case encoder_chan :> ready:
                //when an integer is sent down this channel, encoder_task sends back current count
                //and then resets it
                encoder_chan <: curr_count;
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
        }
    }
}

void move_controller(chanend to_motor, int r, int l){
    //puts given duty cycles into a motor command
    //and sends it down the motor channel.
    //This function handles all of the movement in this assignment
    motor_cmd_t t;
    t.right_duty_cycle = r;
    t.left_duty_cycle = l;
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
        //bitwise or's the masks together in the proper way
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

        //handles the rest of the PWM frame, including transitions. Timers are sampled inside
        PWM_Frame(r_up_time, l_up_time, oRightPWM, oLeftPWM, in_motor_cmd_chan, cmd);
   }

}


