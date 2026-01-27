#include "HCSR04.h"
#include "Delay.h"
#include "bsp_systick.h"

#define HCSR04_TIMEOUT_MS 30U

/*
 * Trig: PF0
 * Echo: PA8 (GPIO)
 */

void HCSR04_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOF, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Trig Pin (PF0)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    // Echo Pin (PA8 - GPIO input)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

float HCSR04_GetValue(void)
{
    uint32_t tStart;
    uint32_t tEnd;
    uint32_t deadlineMs;

    GPIO_SetBits(GPIOF, GPIO_Pin_0);
    Delay_us(10);
    GPIO_ResetBits(GPIOF, GPIO_Pin_0);

    deadlineMs = SysTick_GetMs() + HCSR04_TIMEOUT_MS;
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET)
    {
        if ((int32_t)(SysTick_GetMs() - deadlineMs) >= 0)
        {
            return -1.0f;
        }
    }
    tStart = SysTick_GetMs();

    deadlineMs = SysTick_GetMs() + HCSR04_TIMEOUT_MS;
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_SET)
    {
        if ((int32_t)(SysTick_GetMs() - deadlineMs) >= 0)
        {
            return -1.0f;
        }
    }
    tEnd = SysTick_GetMs();

    if (tEnd <= tStart)
    {
        return -1.0f;
    }

    float durationMs = (float)(tEnd - tStart);
    float distance = (durationMs * 34.0f) / 2.0f; // cm

    if (distance < 2.0f || distance > 400.0f)
    {
        return -1.0f;
    }

    return distance;
}
