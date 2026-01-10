#include "HCSR04.h"
#include "Delay.h"

/*
 * Trig: PA7
 * Echo: PA8 (TIM1_CH1)
 */

void HCSR04_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // Using TIM2 for timing if needed, but we'll use TIM1 for capture
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Trig Pin
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Echo Pin (PA8 - TIM1_CH1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // TIM1 Time Base
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 1MHz (1us per tick)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // TIM1 Input Capture
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    TIM_Cmd(TIM1, ENABLE);
}

float HCSR04_GetValue(void)
{
    uint16_t t1, t2;
    
    GPIO_SetBits(GPIOA, GPIO_Pin_7);
    Delay_us(10);
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    
    // Wait for rising edge
    TIM_SetCounter(TIM1, 0);
    TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
    while (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC1) == RESET);
    t1 = TIM_GetCapture1(TIM1);
    
    // Wait for falling edge
    TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling);
    TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
    while (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC1) == RESET);
    t2 = TIM_GetCapture1(TIM1);
    
    // Reset to rising edge for next time
    TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising);
    
    uint16_t duration = t2 - t1;
    float distance = (float)duration * 0.034 / 2.0; // cm
    
    return distance;
}
