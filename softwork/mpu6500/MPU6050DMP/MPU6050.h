#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "sys.h"
#include "MPU6050_I2C.h"

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG		  	0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B //电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I	  	0x75	
#define	SlaveAddress	0x68 //IIC地址寄存器0x68

u8   mpu6050_write(u8 addr, u8 reg, u8 len, u8* buf);//返回值 0：读成功  -1：读失败
u8   mpu6050_read (u8 addr, u8 reg, u8 len, u8 *buf);//返回值 0：读成功  -1：读失败
void mpu6050_write_reg(u8 reg, u8 dat);
u8   mpu6050_read_reg (u8 reg);
void MPU6050_Init(void);

#endif
