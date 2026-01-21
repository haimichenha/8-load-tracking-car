/**
  * @file    bsp_systick.c
  * @brief   系统滴答定时器 - 精准计时模块
   * @note    使用内核 SysTick 实现 1ms 中断，提供精准的时间基准
   *          SysTick 时钟 = HCLK = 72MHz
   *          计数周期     = 72MHz / 1000 = 72,000 -> 1ms
  */

#include "bsp_systick.h"
#include "bsp_key2.h"

/*====================================================================================*/
/*                                  全局变量                                           */
/*====================================================================================*/

volatile SysTick_Time_t g_sysTime = {0, 0, 0, 0, 0};

/*====================================================================================*/
/*                                  初始化函数                                         */
/*====================================================================================*/

/**
   * @brief  初始化系统滴答定时器 (SysTick, 1ms中断)
  */
void SysTick_Init(void)
{
    /* SysTick 计数值 = HCLK / 1000 = 72,000 -> 1ms */
    SysTick_Config(SystemCoreClock / 1000);
    /* 设置最高优先级，保持与原 TIM2 一致 */
    NVIC_SetPriority(SysTick_IRQn, 0);
}

/*====================================================================================*/
/*                                  中断服务函数                                       */
/*====================================================================================*/

/**
   * @brief  SysTick 中断服务函数 - 每1ms执行一次
  */
  void SysTick_Handler(void)
{
    /* 更新总毫秒数 */
    g_sysTime.totalMs++;
    
    /* 更新毫秒 */
    g_sysTime.milliseconds++;
    
    if (g_sysTime.milliseconds >= 1000)
    {
      g_sysTime.milliseconds = 0;
      g_sysTime.seconds++;
        
      if (g_sysTime.seconds >= 60)
      {
        g_sysTime.seconds = 0;
        g_sysTime.minutes++;
            
        if (g_sysTime.minutes >= 60)
        {
          g_sysTime.minutes = 0;
          g_sysTime.hours++;
        }
      }
    }
    
    /* KEY2 按键扫描 (每1ms调用) */
    Key2_Scan();
}

/*====================================================================================*/
/*                                  功能函数                                           */
/*====================================================================================*/

/**
  * @brief  获取当前毫秒数
  * @return 从启动到现在的总毫秒数
  */
uint32_t SysTick_GetMs(void)
{
    return g_sysTime.totalMs;
}

/**
  * @brief  获取格式化的时间字符串
  * @param  buf: 输出缓冲区 (至少6字节: "MM:SS\0")
  */
void SysTick_GetTimeString(char *buf)
{
    uint32_t min = g_sysTime.minutes;
    uint32_t sec = g_sysTime.seconds;
    
    /* 格式化为 "MM:SS" */
    buf[0] = '0' + (min / 10) % 10;
    buf[1] = '0' + min % 10;
    buf[2] = ':';
    buf[3] = '0' + sec / 10;
    buf[4] = '0' + sec % 10;
    buf[5] = '\0';
}

/**
  * @brief  重置计时器
  */
void SysTick_Reset(void)
{
    /* 禁用中断，防止竞态条件 */
    __disable_irq();
    
    g_sysTime.milliseconds = 0;
    g_sysTime.seconds = 0;
    g_sysTime.minutes = 0;
    g_sysTime.hours = 0;
    g_sysTime.totalMs = 0;
    
    __enable_irq();
}
