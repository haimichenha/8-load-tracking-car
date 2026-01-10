#ifndef __TRACKING_H
#define __TRACKING_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * 函    数：循迹模块初始化
  * 参    数：无
  * 返 回 值：无
  */
void Tracking_Init(void);

/**
  * 函    数：读取循迹数据（默认接口）
  * 参    数：无
  * 返 回 值：8位循迹数据
  */
uint8_t Tracking_Read(void);

/**
  * 函    数：直接读取循迹数据
  * 参    数：无
  * 返 回 值：8位循迹数据
  */
uint8_t Tracking_ReadDirect(void);

/**
  * 函    数：从指定寄存器读取循迹数据
  * 参    数：reg 寄存器地址
  * 返 回 值：读取到的数据
  */
uint8_t Tracking_ReadReg(uint8_t reg);

/**
  * 函    数：使用高级接口读取循迹数据
  * 参    数：reg 寄存器地址
  * 返 回 值：读取到的数据
  */
uint8_t Tracking_ReadByI2CFunc(uint8_t reg);

#endif
