/*
 * mpu6050.h
 *
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c.h"
#include "mpu6050regs.h"

struct IMU {
    struct r_i2c i2c;
    int gyro_x_offset ;
    int gyro_y_offset ;
    int gyro_z_offset ;
};


void mpu_init_i2c(struct IMU &imu);
unsigned char mpu_read_byte(REFERENCE_PARAM(struct r_i2c, mpu), const int reg);
unsigned char mpu_read_bytes(REFERENCE_PARAM(struct r_i2c, mpu), const int reg, unsigned char data[], const int length);
void mpu_dumpregs(REFERENCE_PARAM(struct r_i2c, mpu));
short mpu_read_short(REFERENCE_PARAM(struct r_i2c, mpu), const int reg);
void mpu_writebits(REFERENCE_PARAM(struct r_i2c, mpu), const int reg, const int bitStart, const int length, const unsigned char value);
void mpu_writebit(REFERENCE_PARAM(struct r_i2c, mpu), const int reg, const int bitNum, const unsigned char value);
int mpu_readBits(REFERENCE_PARAM(struct r_i2c, mpu), const int reg, const int bitStart, const int length);
void mpu_writeMemoryBlock(struct IMU &imu, unsigned char data[],int dataSize,int bank,int address,int verify);
int mpu_setMemoryBank(struct IMU &imu, int bank, const int prefetchEnabled, const int userBank);
void mpu_writeDMPConfigurationSet(struct IMU &imu, unsigned char data[], int dataSize, int bank, int address, int verify);
void mpu_getFIFOBytes(struct IMU &imu,int numbytes,unsigned char fifobuf[]);
void mpu_getQuaternion(unsigned char packet[],float quat[]);
void mpu_getGravity(float q[],float g[]);
void mpu_getYawPitchRoll(float q[],float g[],float ypr[]);
void mpu_getEuler(float euler[], float q[]);
void mpu_getAccel(unsigned char packet[],short data[]);
void mpu_getGyro(unsigned char packet[],short data[]);
void mpu_dmpInitialize(struct IMU &imu);
void mpu_Stop(struct IMU &imu);
void mpu_resetFifo(struct IMU &imu);
void mpu_enableDMP(struct IMU &imu,int enable);

float rad2deg(float rad);


#endif /* MPU6050_H_ */
