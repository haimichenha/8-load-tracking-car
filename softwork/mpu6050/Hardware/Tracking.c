#include "Tracking.h"
#include "Delay.h"

#define TRACKING_ADDR 0x10

#define SCL_H GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define SCL_L GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define SDA_H GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define SDA_L GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define SDA_READ GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

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
    SDA_H; I2C_Delay(); SCL_H; I2C_Delay(); SCL_L; // Wait for ACK
}

static uint8_t I2C_ReadByte(void) {
    uint8_t byte = 0;
    SDA_H;
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        SCL_H; I2C_Delay();
        if (SDA_READ) byte |= 0x01;
        SCL_L; I2C_Delay();
    }
    SDA_H; I2C_Delay(); SCL_H; I2C_Delay(); SCL_L; // Send NACK
    return byte;
}

void Tracking_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // 开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    SCL_H; SDA_H;
}

uint8_t Tracking_Read(void) {
    uint8_t data = 0;
    I2C_Start();
    I2C_SendByte((TRACKING_ADDR << 1) | 0);
    I2C_SendByte(0x01);
    I2C_Start();
    I2C_SendByte((TRACKING_ADDR << 1) | 1);
    data = I2C_ReadByte();
    I2C_Stop();
    return data;
}
