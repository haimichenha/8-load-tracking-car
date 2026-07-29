#include "JY301P.h"
#include "Delay.h"

#define JY301P_ADDR 0x50 // 维特智能默认 I2C 地址

// 复用 Tracking 的 I2C 宏定义
#define SCL_H GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define SCL_L GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define SDA_H GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define SDA_L GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define SDA_READ GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

JY301P_Data g_jy301p_data;

static void I2C_Delay(void) { delay_us(4); }

static void I2C_Start(void) {
    SDA_H; SCL_H; I2C_Delay();
    SDA_L; I2C_Delay();
    SCL_L; I2C_Delay();
}

static void I2C_Stop(void) {
    SDA_L; SCL_H; I2C_Delay();
    SDA_H; I2C_Delay();
}

static void I2C_SendByte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) SDA_H; else SDA_L;
        I2C_Delay(); SCL_H; I2C_Delay(); SCL_L;
        byte <<= 1;
    }
    SDA_H; I2C_Delay(); SCL_H; I2C_Delay(); SCL_L;
}

static uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t byte = 0;
    SDA_H;
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        SCL_H; I2C_Delay();
        if (SDA_READ) byte |= 0x01;
        SCL_L; I2C_Delay();
    }
    if (ack) SDA_L; else SDA_H;
    I2C_Delay(); SCL_H; I2C_Delay(); SCL_L;
    SDA_H;
    return byte;
}

void JY301P_Init(void) {
    // PB10/11 初始化已在 Tracking_Init 中完成
}

void JY301P_Update(void) {
    uint8_t buf[24];
    
    // 读取加速度、角速度、角度 (寄存器 0x34 开始)
    I2C_Start();
    I2C_SendByte((JY301P_ADDR << 1) | 0);
    I2C_SendByte(0x34); 
    I2C_Start();
    I2C_SendByte((JY301P_ADDR << 1) | 1);
    for (int i = 0; i < 23; i++) buf[i] = I2C_ReadByte(1);
    buf[23] = I2C_ReadByte(0);
    I2C_Stop();

    // 解析数据 (根据维特智能协议)
    short temp;
    for (int i = 0; i < 3; i++) {
        temp = (short)(buf[i*2+1] << 8 | buf[i*2]);
        g_jy301p_data.acc[i] = (float)temp / 32768.0 * 16.0;
        
        temp = (short)(buf[i*2+7] << 8 | buf[i*2+6]);
        g_jy301p_data.gyro[i] = (float)temp / 32768.0 * 2000.0;
        
        temp = (short)(buf[i*2+13] << 8 | buf[i*2+12]);
        g_jy301p_data.angle[i] = (float)temp / 32768.0 * 180.0;
    }
}
