// Copyright (c) 2011, XMOS Ltd, All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

// I2C master, includes multi master.

#include <xs1.h>
#include <xclib.h>
#include <stdio.h>
#include <stdlib.h>
#include "i2c.h"

void i2c_master_init(struct r_i2c &i2c_master) {
    i2c_master.scl :> void;
    i2c_master.sda :> void;
}

static void waitQuarter(struct r_i2c &i2c) {
    timer gt;
    int time;

    gt :> time;
    time += (i2c.clockTicks+3) >> 2;      // Round time upwards prior to division by 4.
    gt when timerafter(time) :> int _;
}

static void waitHalf(struct r_i2c &i2c) {
    waitQuarter(i2c);
    waitQuarter(i2c);
}

static int highPulse(struct r_i2c &i2c, int doSample) {
    int temp;
    if (doSample) {
        i2c.sda :> int _;
    }
    waitQuarter(i2c);
    i2c.scl when pinseq(1) :> void;
    waitQuarter(i2c);
    if (doSample) {
        i2c.sda :> temp;
    }
    waitQuarter(i2c);
    i2c.scl <: 0;
    waitQuarter(i2c);
    return temp;
}

static void startBit(struct r_i2c &i2c, int waitForQuiet) {
    if (waitForQuiet) {
        timer t;
        int time;
        int done = 0;
        int sdaState = 1;
        int sclState = 1;
        t :> time;
        while(!done) {
            select {
            case i2c.sda when pinsneq(sdaState) :> sdaState:
                t :> time;
                break;
            case i2c.scl when pinsneq(sclState) :> sclState:
                t :> time;
                break;
            case sdaState && sclState => t when timerafter(time+i2c.clockTicks) :> void:
                done = 1;
                break;
            }
        }
    } else {
        waitQuarter(i2c);
    }
    i2c.sda  <: 0;
    waitHalf(i2c);
    i2c.scl  <: 0;
    waitQuarter(i2c);
}

static void stopBit(struct r_i2c &i2c) {
    i2c.sda <: 0;
    waitQuarter(i2c);
    i2c.scl when pinseq(1) :> void;
    waitHalf(i2c);
    i2c.sda :> void;
    waitQuarter(i2c);
}

static int floatWires(struct r_i2c &i2c) {
    i2c.scl :> void;
    waitQuarter(i2c);
    i2c.sda :> void;
    return 0;
}

static int tx8(struct r_i2c &i2c, unsigned data) {
    unsigned CtlAdrsData = ((unsigned) bitrev(data)) >> 24;
    for (int i = 8; i != 0; i--) {
        if (CtlAdrsData & 1) {
            if (highPulse(i2c, 1) == 0) { // Somebody else pulling low.
                return 0;
            }
        } else {
            i2c.sda <: 0;
            highPulse(i2c, 0);
        }
        CtlAdrsData >>= 1;
    }
    return highPulse(i2c, 1) == 0;
}

#ifndef I2C_TI_COMPATIBILITY
int i2c_master_do_rx(int device, unsigned char data[], int nbytes, struct r_i2c &i2c) {
   int i;
   int rdData = 0;

   if (!tx8(i2c, device<<1 | 1)) return floatWires(i2c);
   for(int j = 0; j < nbytes; j++) {
       for (i = 8; i != 0; i--) {
           int temp = highPulse(i2c, 1);
           rdData = (rdData << 1) | temp;
       }
       if (j != nbytes - 1) {
           i2c.sda <: 0;
           highPulse(i2c, 0);
       } else {
           highPulse(i2c, 1);
       }
       data[j] = rdData;
   }
   stopBit(i2c);
   return 1;
}

int i2c_master_rx(int device, unsigned char data[], int nbytes, struct r_i2c &i2c) {
   startBit(i2c, 1);
   return i2c_master_do_rx(device, data, nbytes, i2c);
}

int i2c_master_read_reg(int device, int addr, unsigned char data[], int nbytes, struct r_i2c &i2c) {
    startBit(i2c, 1);
    if (!tx8(i2c, device<<1)) return floatWires(i2c);
    if (!tx8(i2c, addr)) return floatWires(i2c);
//    stopBit(i2c);          // do not stop but restart for multi master.
    i2c.sda :> void;       // stop bit, but not as we know it - restart.
    waitQuarter(i2c);
    i2c.scl :> void;       // stop bit, but not as we know it - restart.
    waitQuarter(i2c);
    startBit(i2c, 0);      // Do not wait on start-bit - just do it.
    return i2c_master_do_rx(device, data, nbytes, i2c);
}

int i2c_master_16bit_read_reg(int device, unsigned int addr, unsigned char data[], int nbytes, struct r_i2c &i2c) {
    startBit(i2c, 1);
    if (!tx8(i2c, device<<1)) return floatWires(i2c);
    if (!tx8(i2c, ((addr >> 8) & 0xFF))) return floatWires(i2c);
    if (!tx8(i2c, (addr & 0xFF))) return floatWires(i2c);
    i2c.sda :> void;       // stop bit, but not as we know it - restart.
    waitQuarter(i2c);
    i2c.scl :> void;       // stop bit, but not as we know it - restart.
    waitQuarter(i2c);
    startBit(i2c, 0);      // Do not wait on start-bit - just do it.
    return i2c_master_do_rx(device, data, nbytes, i2c);
}

#endif

int i2c_master_write_reg(int device, int addr, unsigned char s_data[], int nbytes, struct r_i2c &i2c) {
   startBit(i2c, 1);
   if (!tx8(i2c, device<<1)) return floatWires(i2c);
#ifndef I2C_NO_REGISTER_ADDRESS
#ifdef I2C_TI_COMPATIBILITY
   if (!tx8(i2c, addr << 1 | (s_data[0] >> 8) & 1)) return floatWires(i2c);
#else
   if (!tx8(i2c, addr)) return floatWires(i2c);
#endif
#endif
   for(int j = 0; j < nbytes; j++) {
       if (!tx8(i2c, s_data[j])) return floatWires(i2c);
   }
   stopBit(i2c);
   return 1;
}


int i2c_master_16bit_write_reg(int device, unsigned int addr, unsigned char s_data[], int nbytes, struct r_i2c &i2c) {
   startBit(i2c, 1);
   if (!tx8(i2c, device<<1)) return floatWires(i2c);

   if (!tx8(i2c, ((addr >> 8) & 0xFF))) return floatWires(i2c);
   if (!tx8(i2c, (addr & 0xFF))) return floatWires(i2c);

   for(int j = 0; j < nbytes; j++) {
       if (!tx8(i2c, s_data[j])) return floatWires(i2c);
   }
   stopBit(i2c);
   return 1;
}

int i2c_master_readBit(int device, unsigned int addr, unsigned int bitNum, unsigned char data[], struct r_i2c &i2c){
    unsigned char b[1];
    unsigned int count;
    count = i2c_master_read_reg(device, addr, b,1,i2c);
    data[0] = b[0] & (1 << bitNum);
    return count;
}

int i2c_master_readBits(int device, unsigned int addr, unsigned int bitStart, unsigned int length, unsigned char data[], struct r_i2c &i2c){
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    unsigned char b[1],mask;
    unsigned int count;
    if ((count = i2c_master_read_reg(device, addr, b,1,i2c)) != 0){
        mask = ((1 << length) - 1) << (bitStart - length + 1);
        b[0] &= mask;
        b[0] >>= (bitStart - length + 1);
        data[0] = b[0];
    }
    return count;
}

int i2c_master_writeBit(int device, unsigned int addr, unsigned int bitNum, unsigned char data[], struct r_i2c &i2c) {
    unsigned char b[1];
    int result;
    i2c_master_read_reg(device, addr, b,1,i2c);
    b[0] = (data[0] != 0) ? (b[0] | (1 << bitNum)) : (b[0] & ~(1 << bitNum));
    result=i2c_master_write_reg(device, addr, b, 1, i2c);
    return result;
}

int i2c_master_writeBits(int device, unsigned int addr, unsigned int bitStart, unsigned int length, unsigned char data[], struct r_i2c &i2c){
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    unsigned char b[1],mask;
    if (i2c_master_read_reg(device, addr, b,1,i2c) != 0){
        mask = ((1 << length) - 1) << (bitStart - length + 1);
        data[0] <<= (bitStart - length + 1); // shift data into correct position
        data[0] &= mask; // zero all non-important bits in data
        b[0] &= ~(mask); // zero all important bits in existing byte
        b[0] |= data[0]; // combine data with existing byte
        return i2c_master_write_reg(device, addr, b, 1, i2c);
    } else {
        return 0;
    }
}
