/*
 * Lab06-SetupNodeMCU.xc
 *
 *  Created on: Oct 18, 2018
 *      Author: root
 */

#include <xs1.h>
#include <print.h>
#include <string.h>
#include <stdio.h>


#define TICKS_PER_SECOND XS1_TIMER_HZ
#define BAUDRATE 9600
#define BUFFER_LENGTH 300
#define BUFFER_DELAY TICKS_PER_SECOND


out port oLED = XS1_PORT_1A;
out port oWiFiRX = XS1_PORT_1F;

in port iWiFiTX = XS1_PORT_1H;

void toggle_port(out port oLED, unsigned int hz);
void uart_transmit_byte(out port oPort, char value, unsigned int baudrate);
char uart_receive_byte(in port iPort, unsigned int baudrate);
void uart_transmit_bytes(out port oPort, const char buffer[], unsigned int baudrate);
void uart_to_console_task(chanend trigger_chan);
void send_blink_program();
void init_wifi_task(chanend trigger_chan);
void write_blink_program();
void read_blink_program();
void run_blink_program();
void write_wifi_program();
void read_wifi_program();
void run_wifi_program();

void uart_transmit_bytes(out port oPort,
                         const char buffer[],
                         unsigned int baudrate){
    int i = 0;
    while(1){
        uart_transmit_byte(oPort, buffer[i], baudrate);
        i++;
        if(buffer[i] == '\0'){
            break;
        }
        //oPort <: 1;
    }
}

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

void uart_to_console_task(chanend trigger_chan){
    char buffer[BUFFER_LENGTH];
    int index = 0;
    while(1){
        buffer[index] = uart_receive_byte(iWiFiTX, BAUDRATE);
        if(buffer[index] == '\n' || buffer[index] == '\r'){
            buffer[index] = '\0';
            if(!strncmp(buffer, "lua: cannot open init.lua", 25)){
                trigger_chan <: 1;
            }
            printstrln(buffer);
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
    unsigned int t_val;
    t :> t_val;
    t when timerafter(t_val + BUFFER_DELAY) :> void;
    uart_transmit_bytes(oWiFiRX, buffer, BAUDRATE);
    uart_transmit_byte(oWiFiRX, '\r', BAUDRATE);
    uart_transmit_byte(oWiFiRX, '\n', BAUDRATE);
    //try sending terminate character
    //uart_transmit_byte(oWiFiRX, '\0', BAUDRATE);

}

void send_blink_program(){
    line("gpio.mode(3, gpio.OUTPUT)");
    line("while 1 do");
    line("gpio.write(3, gpio.HIGH)");
    line("tmr.delay(1000000)");
    line("gpio.write(3, gpio.LOW)");
    line("tmr.delay(1000000)");
    line("end");
}

void init_wifi_task(chanend trigger_chan){
    int init;
    while(1){
        trigger_chan :> init;
        //send_blink_program();
        //write the program
        write_wifi_program();
        //write_blink_program();
        //read_blink_program();
        //run_blink_program();
        //read the program
        //read_wifi_program();
        //execute the program
        run_wifi_program();
        //program goes off the rails
        //throw the program away
    }

}

void write_blink_program(){
    line("file.remove('blink.lua')");
    line("file.open('blink.lua', 'w')");

    line("file.writeline('gpio.mode(3, gpio.OUTPUT)')");
    line("file.writeline('while 1 do')");
    line("file.writeline('gpio.write(3, gpio.HIGH)')");
    line("file.writeline('tmr.delay(1000000)')");
    line("file.writeline('gpio.write(3, gpio.LOW)')");
    line("file.writeline('tmr.delay(1000000)')");
    line("file.writeline('end')");

    line("file.close()");
}

void read_blink_program(){
    line("if file.open('blink.lua', 'r') then");
    line("repeat");
    line("l = file.readline()");
    line("if l then");
    //try delaying here for like a second?
    //line("tmr.delay(500000)");
    line("print(l)");
    line("end");
    line("until not l");
    line("file.close()");
    line("end");
}

void run_blink_program(){
    line("dofile('blink.lua')");
}

void write_wifi_program(){
    line("file.remove('wifi.lua')");
    line("file.open ('wifi.lua', 'w')");
    ///line("file.writeline('gpio.mode(3, gpio.OUTPUT)')");
    line("file.writeline('wifi.setmode(wifi.SOFTAP)')");
    line("file.writeline('cfg = {}')");
    line("file.writeline('cfg.ssid=\"MiniUGVCRJ\"')");
    line("file.writeline('cfg.pwd=\"kingofkings\"')");
    line("file.writeline('cfg.ip=\"192.168.0.1\"')");
    line("file.writeline('cfg.netmask=\"255.255.255.0\"')");
    line("file.writeline('cfg.gateway=\"192.168.0.1\"')");
    line("file.writeline('port = 9876')");
    line("file.writeline('wifi.ap.setip(cfg)')");
    line("file.writeline('wifi.ap.config(cfg)')");
    line("print(\"SSID: \" .. cfg.ssid .. \"PASS: \" .. cfg.pwd)");
    line("print('RoboRemo app must connect to ' .. cfg.ip .. ':' .. port)");
    line("file.writeline('tmr.alarm(0, 200, 0, function() -- run after a delay')");
    line("file.writeline('srv=net.createServer(net.TCP, 28800)')");
    line("file.writeline('srv:listen(port, function(conn)')");
    line("file.writeline('uart.on(\"data\", 0, function(data)')");
    line("file.writeline('conn:send(data)')");
    line("file.writeline('end, 0)')");
    line("file.writeline('conn:on(\"receive\", function(conn, payload)')");
    line("file.writeline('uart.write(0, payload)')");
    line("file.writeline('end)')");
    line("file.writeline('conn:on(\"disconnection\", function(c)')");
    line("file.writeline('uart.on(\"data\")')");
    line("file.writeline('end)')");
    line("file.writeline('end)')");
    line("file.writeline('end)')");
    line("file.close()");

}

void run_wifi_program(){
    line("dofile('wifi.lua')");
}

void read_wifi_program(){
    line("if file.open('wifi.lua', 'r') then");
    line("repeat");
    line("l = file.readline()");
    line("if l then");
    line("print(l)");
    line("end");
    line("until not l");
    line("file.close()");
    line("end");
}

int main(){
    oWiFiRX <: 1;
    chan c;
    par{
        toggle_port(oLED, 2);
        uart_to_console_task(c);
        init_wifi_task(c);
    }

    return 0;
}
