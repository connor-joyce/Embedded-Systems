/*
 * Lab03-UART.xc
 *
 *  Created on: Sep 27, 2018
 *      Author: root
 */
#include <xs1.h>
#include <string.h>
#include <print.h>
#include <stdio.h>

#define TICKS_PER_SECOND XS1_TIMER_HZ
#define BAUDRATE 9600


void uart_transmit_byte(out port oPort, char value, unsigned int baudrate);
char uart_receive_byte(in port iPort, unsigned int baudrate);

in port iUART = XS1_PORT_1C;
out port oUART = XS1_PORT_1A;

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

char uart_receive_byte(in port iPort, unsigned int baudrate){
    int BIT_TIME_TICKS = TICKS_PER_SECOND/baudrate;
    char byte = 0;
    timer t;
    unsigned int time;
    iPort when pinseq (0) :> void;
    t :> time;
    time += BIT_TIME_TICKS/2;
    for(int i = 0; i<8; i++){
        time+=BIT_TIME_TICKS;
        t when timerafter(time) :> void;
        iPort :> >> byte;
        //printf("%u\n", byte);
    }

    time += BIT_TIME_TICKS;
    t when timerafter(time) :> void;
    iPort :> void;


    return byte;
}

int main_single(){
    char value;
    oUART <: 1;
    par{
        uart_transmit_byte(oUART, 'H', BAUDRATE);
        value = uart_receive_byte(iUART, BAUDRATE);
    }

    printcharln(value);
    return 0;

}

void uart_transmit_bytes(out port oPort,
                         const char values[],
                         unsigned int n,
                         unsigned int baudrate){
    for(int i = n-1; i>=0; i--){
        uart_transmit_byte(oPort, values[i], baudrate);
        //oPort <: 1;
    }
}

void uart_receive_bytes(in port iPort, char values[], unsigned int n, unsigned int baudrate){
    for(int i = n-1; i>=0; i--){
        values[i] = uart_receive_byte(iPort, baudrate);
    }
}

int main_array(){
    const char message[] = "Hello, Denver.";
    char buffer[64];
    oUART <: 1;
    par{
        uart_transmit_bytes(oUART, message, (strlen(message)+1), BAUDRATE);
        uart_receive_bytes(iUART, buffer, (strlen(message)+1), BAUDRATE);
    }
    printstrln(buffer);
    return 0;
}

int main(){
    return main_array();
}
