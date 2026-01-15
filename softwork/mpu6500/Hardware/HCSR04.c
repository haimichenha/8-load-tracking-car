#include "HCSR04.h"
#include "Delay.h"

/**
  * Trig: PF0
  * Echo: PA8 (TIM1_CH1)
  */
void HCSR04_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOF | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Trig Pin (PF0)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    // Echo Pin (PA8 - TIM1_CH1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // 下拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // TIM1 时基单元配置 (1us 计数一次)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 72MHz / 72 = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // TIM1 输入捕获配置
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0F; // 滤波
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    TIM_Cmd(TIM1, ENABLE);
    GPIO_ResetBits(GPIOF, GPIO_Pin_0);
}

float HCSR04_GetValue(void)
{
    uint16_t t1, t2;
    
    // 触发信号
    GPIO_SetBits(GPIOF, GPIO_Pin_0);
    delay_us(15);
    GPIO_ResetBits(GPIOF, GPIO_Pin_0);
    
    // 等待上升沿
    TIM_SetCounter(TIM1, 0);
    TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising);
    TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
    while (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC1) == RESET);
    t1 = TIM_GetCapture1(TIM1);
    
    // 等待下降沿
    TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling);
    TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
    while (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC1) == RESET);
    t2 = TIM_GetCapture1(TIM1);
    
    // 计算距离 (声速 340m/s = 0.034cm/us)
    uint16_t duration = t2 - t1;
    float distance = (float)duration * 0.034 / 2.0;
    
    return distance;
}
