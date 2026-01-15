/**
  * @file    bsp_key.c
  * @brief   按键驱动 (PC5)
  * @note    支持短按/长按检测，非阻塞式状态机实现
  *          改进版：多次采样消抖
  */

#include "bsp_key.h"
#include "Delay.h"

/*====================================================================================*/
/*                                  私有变量                                           */
/*====================================================================================*/

static KeyState_t s_keyState = KEY_STATE_IDLE;  // 按键状态机
static uint32_t s_pressStartTime = 0;           // 按下开始时间
static uint32_t s_systemTick = 0;               // 系统时间计数 (ms)
static uint8_t s_stableLevel = 1;               // 稳定后的电平值
static uint8_t s_sampleCount = 0;               // 连续相同电平的采样次数
static uint8_t s_lastRawLevel = 1;              // 上次原始电平

/* 消抖参数 - 最快响应 */
#define DEBOUNCE_SAMPLES    1   // 只需要1次采样

/*====================================================================================*/
/*                                  初始化函数                                         */
/*====================================================================================*/

/**
  * @brief  按键初始化
  */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 使能GPIOC时钟 */
    RCC_APB2PeriphClockCmd(KEY_GPIO_CLK, ENABLE);
    
    /* 配置PC5为上拉输入 */
    GPIO_InitStructure.GPIO_Pin = KEY_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStructure);
    
    /* 初始化状态 */
    s_keyState = KEY_STATE_IDLE;
    s_pressStartTime = 0;
    s_systemTick = 0;
    s_stableLevel = 1;
    s_sampleCount = 0;
    s_lastRawLevel = 1;
}

/*====================================================================================*/
/*                                  按键扫描                                           */
/*====================================================================================*/

/**
  * @brief  获取按键当前电平状态（原始值）
  */
uint8_t Key_GetLevel(void)
{
    return GPIO_ReadInputDataBit(KEY_GPIO_PORT, KEY_GPIO_PIN);
}

/**
  * @brief  获取消抖后的稳定电平
  */
static uint8_t Key_GetStableLevel(void)
{
    uint8_t rawLevel = Key_GetLevel();
    
    /* 如果电平与上次相同，增加计数 */
    if (rawLevel == s_lastRawLevel)
    {
        if (s_sampleCount < 255)
            s_sampleCount++;
    }
    else
    {
        /* 电平变化，重新计数 */
        s_sampleCount = 1;
        s_lastRawLevel = rawLevel;
    }
    
    /* 连续多次相同电平才更新稳定值 */
    if (s_sampleCount >= DEBOUNCE_SAMPLES)
    {
        s_stableLevel = rawLevel;
    }
    
    return s_stableLevel;
}

/**
  * @brief  带时间参数的按键扫描
  */
KeyEvent_t Key_ScanWithTime(uint32_t deltaMs)
{
    KeyEvent_t event = KEY_EVENT_NONE;
    uint8_t keyLevel;
    
    /* 更新系统时间 */
    s_systemTick += deltaMs;
    
    /* 获取消抖后的电平 */
    keyLevel = Key_GetStableLevel();
    
    switch (s_keyState)
    {
        case KEY_STATE_IDLE:
            /* 空闲状态：检测按下 */
            if (keyLevel == 0)
            {
                s_keyState = KEY_STATE_PRESSED;
                s_pressStartTime = s_systemTick;
            }
            break;
            
        case KEY_STATE_PRESSED:
            /* 按下状态：等待释放或长按 */
            if (keyLevel == 1)
            {
                /* 释放了 */
                uint32_t pressTime = s_systemTick - s_pressStartTime;
                
                if (pressTime >= KEY_LONG_TIME)
                {
                    /* 已经触发过长按，不再触发 */
                }
                else if (pressTime >= KEY_DEBOUNCE_TIME)
                {
                    /* 短按 */
                    event = KEY_EVENT_SHORT;
                }
                /* 太短的按压忽略 */
                
                s_keyState = KEY_STATE_IDLE;
            }
            else if (s_systemTick - s_pressStartTime >= KEY_LONG_TIME)
            {
                /* 达到长按时间 */
                event = KEY_EVENT_LONG;
                s_keyState = KEY_STATE_LONG_WAIT;
            }
            break;
            
        case KEY_STATE_LONG_WAIT:
            /* 长按等待释放 */
            if (keyLevel == 1)
            {
                s_keyState = KEY_STATE_IDLE;
            }
            break;
            
        default:
            s_keyState = KEY_STATE_IDLE;
            break;
    }
    
    return event;
}

/**
  * @brief  按键扫描 - 使用默认时间间隔
  */
KeyEvent_t Key_Scan(void)
{
    return Key_ScanWithTime(50);
}

/**
  * @brief  按键状态机复位
  */
void Key_Reset(void)
{
    s_keyState = KEY_STATE_IDLE;
    s_pressStartTime = 0;
    s_sampleCount = 0;
    s_stableLevel = 1;
}

/**
  * @brief  获取当前状态机状态（调试用）
  */
uint8_t Key_GetState(void)
{
    return (uint8_t)s_keyState;
}
