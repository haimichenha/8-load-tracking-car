#include "stm32f10x.h"
#include "Tracking.h"
#include "Delay.h"
#include "IOI2C.h"

/*
 * 循迹模块驱动 - 使用IOI2C底层函数
 * 
 * 硬件连接：
 * SCL: PB10
 * SDA: PB11
 * 
 * 设备信息：
 * I2C地址: 0x12 (7位地址)
 */

#define TRACKING_ADDR    0x12    // 循迹模块I2C地址（7位）

/*====================================================================================*/
/*                                  初始化函数                                         */
/*====================================================================================*/

/**
  * 函    数：循迹模块初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：初始化I2C总线，并发送模块初始化命令
  */
void Tracking_Init(void)
{
    // I2C引脚初始化（使用IOI2C的初始化函数）
    IIC_Init();
    
    Delay_ms(100);  // 等待模块上电稳定
    
    // 尝试发送初始化命令（根据雅博姆模块协议）
    IIC_Start();
    IIC_Send_Byte(TRACKING_ADDR << 1);      // 发送写地址
    if (IIC_Wait_Ack() == 0) {
        IIC_Send_Byte(0x00);                // 寄存器地址
        IIC_Wait_Ack();
        IIC_Send_Byte(0x01);                // 启用/复位命令
        IIC_Wait_Ack();
    }
    IIC_Stop();
    
    Delay_ms(10);
}

/*====================================================================================*/
/*                                  读取函数                                           */
/*====================================================================================*/

/**
  * 函    数：直接读取循迹数据
  * 参    数：无
  * 返 回 值：8位循迹数据，每位代表一个传感器状态
  * 说    明：直接读取模式，不指定寄存器地址
  */
uint8_t Tracking_ReadDirect(void)
{
    uint8_t data = 0xFF;
    
    IIC_Start();
    IIC_Send_Byte((TRACKING_ADDR << 1) | 1);    // 发送读地址
    if (IIC_Wait_Ack() == 0) {
        data = IIC_Read_Byte(0);                // 读取数据，发送NACK
    }
    IIC_Stop();
    
    return data;
}

/**
  * 函    数：从指定寄存器读取循迹数据
  * 参    数：reg 寄存器地址
  * 返 回 值：读取到的数据
  * 说    明：先写寄存器地址，再读取数据
  */
uint8_t Tracking_ReadReg(uint8_t reg)
{
    uint8_t data = 0xFF;
    
    // 第一步：发送寄存器地址
    IIC_Start();
    IIC_Send_Byte(TRACKING_ADDR << 1);          // 发送写地址
    if (IIC_Wait_Ack() == 0) {
        IIC_Send_Byte(reg);                     // 发送寄存器地址
        IIC_Wait_Ack();
    }
    IIC_Stop();
    
    Delay_us(10);
    
    // 第二步：读取数据
    IIC_Start();
    IIC_Send_Byte((TRACKING_ADDR << 1) | 1);    // 发送读地址
    if (IIC_Wait_Ack() == 0) {
        data = IIC_Read_Byte(0);                // 读取数据，发送NACK
    }
    IIC_Stop();
    
    return data;
}

/**
  * 函    数：使用高级接口读取循迹数据
  * 参    数：reg 寄存器地址
  * 返 回 值：读取到的数据
  * 说    明：使用IOI2C提供的i2cRead高级函数
  */
uint8_t Tracking_ReadByI2CFunc(uint8_t reg)
{
    uint8_t data = 0xFF;
    i2cRead(TRACKING_ADDR, reg, 1, &data);
    return data;
}

/**
  * 函    数：读取循迹数据（默认接口）
  * 参    数：无
  * 返 回 值：8位循迹数据
  * 说    明：默认使用直接读取模式
  */
uint8_t Tracking_Read(void)
{
    return Tracking_ReadDirect();
}
