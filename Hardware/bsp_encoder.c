/**
 * @file    bsp_encoder.c
 * @brief   双路编码器 (左:TIM2 PA0/PA1, 右:TIM3 PA6/PA7)
 */

#include "bsp_encoder.h"

static void Encoder_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* PA0/PA1, PA6/PA7 输入浮空 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void Encoder_TIM2_Init(void)
{
    /* TIM2: 左轮编码器，PA0/PA1 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef tim;
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 0;
    tim.TIM_Period = 0xFFFF;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

static void Encoder_TIM3_Init(void)
{
    /* TIM3: 右轮编码器，PA6/PA7 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef tim;
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 0;
    tim.TIM_Period = 0xFFFF;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tim);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

void Encoder_Init(void)
{
    Encoder_GPIO_Config();
    Encoder_TIM2_Init();
    Encoder_TIM3_Init();
}

int16_t Encoder_GetDeltaLeft(void)
{
    int16_t delta = (int16_t)TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2, 0);
    return delta;
}

int16_t Encoder_GetDeltaRight(void)
{
    int16_t delta = (int16_t)TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM3, 0);
    return delta;
}

void Encoder_Reset(void)
{
    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM3, 0);
}
