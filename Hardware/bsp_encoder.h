/**
 * @file    bsp_encoder.h
 * @brief   双路编码器 (左:TIM2 PA0/PA1, 右:TIM3 PA6/PA7)
 */

#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "stm32f10x.h"

void Encoder_Init(void);
int16_t Encoder_GetDeltaLeft(void);
int16_t Encoder_GetDeltaRight(void);
void Encoder_Reset(void);

#endif /* __BSP_ENCODER_H */
