#include "Motor.h"

void Motor_Init(void)
{
    // 开启 GPIOA, GPIOE 和 TIM5 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // PWM 引脚 (PA2, PA3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 方向控制引脚 (PE2-PE6)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // TIM5 时基配置 (20kHz PWM)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;    // 分辨率 0-100
    TIM_TimeBaseStructure.TIM_Prescaler = 36 - 1; // 72MHz / 36 / 100 = 20kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    // TIM5 PWM 通道配置
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    TIM_OC3Init(TIM5, &TIM_OCInitStructure); // PWMA
    TIM_OC4Init(TIM5, &TIM_OCInitStructure); // PWMB

    TIM_Cmd(TIM5, ENABLE);
    
    GPIO_SetBits(GPIOE, GPIO_Pin_6); // STBY 置高，使能驱动
}

void Motor_SetSpeed(int16_t speedA, int16_t speedB)
{
    // 电机 A (左)
    if (speedA >= 0) {
        GPIO_SetBits(GPIOE, GPIO_Pin_2);   // IN1
        GPIO_ResetBits(GPIOE, GPIO_Pin_3); // IN2
        TIM_SetCompare3(TIM5, speedA);
    } else {
        GPIO_ResetBits(GPIOE, GPIO_Pin_2);
        GPIO_SetBits(GPIOE, GPIO_Pin_3);
        TIM_SetCompare3(TIM5, -speedA);
    }

    // 电机 B (右)
    if (speedB >= 0) {
        GPIO_SetBits(GPIOE, GPIO_Pin_4);   // IN3
        GPIO_ResetBits(GPIOE, GPIO_Pin_5); // IN4
        TIM_SetCompare4(TIM5, speedB);
    } else {
        GPIO_ResetBits(GPIOE, GPIO_Pin_4);
        GPIO_SetBits(GPIOE, GPIO_Pin_5);
        TIM_SetCompare4(TIM5, -speedB);
    }
}
