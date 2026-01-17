#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "bsp_usart.h"
#include "HW_I2C.h"
#include "Tracking.h"
#include "JY301P.h"
#include "IOI2C.h"
#include "bsp_key.h"
#include "bsp_systick.h"
#include "bsp_led.h"
#include "bsp_buzzer.h"
#include "app_ui.h"
#include "app_stats.h"
#include <stdio.h>

/*
 * 测试模式 v11：循迹模块(硬件I2C) + 陀螺仪(IOI2C) + 按键UI切换
 * 
 * 重要说明：
 * OLED、循迹模块、陀螺仪都使用PB10/PB11，需要分时复用
 * 
 * 操作顺序：
 * 1. 使用硬件I2C读取循迹
 * 2. 切换到软件I2C读取陀螺仪
 * 3. 切换到OLED的软件I2C进行显示
 * 4. 循环
 */

#define TRACKING_ADDR   0x12    // 循迹模块I2C地址 (7位)
#define TRACKING_REG    0x30    // 数据寄存器地址（官方）

/* 主循环周期 (ms) */
#define LOOP_PERIOD_MS  50

/* 全局运行时间计数器 */
static volatile uint32_t g_loopCount = 0;

int main(void)
{
    uint8_t track_hw = 0x00;      // 硬件I2C读取的循迹数据
    uint8_t hw_result = 0;        // 硬件I2C返回值
    uint8_t fail_count = 0;       // 连续失败计数
    KeyEvent_t keyEvent;          // 按键事件
    
    /*========== 第一阶段：初始化 ==========*/
    
    /* 初始化系统滴答定时器 (TIM2, 1ms中断) - 必须最先初始化 */
    SysTick_Init();
    
    /* 初始化LED (PB9绿色, PE6红色) */
    LED_Init();
    LED_SetAll(1);  /* 两个LED常亮 */
    
    /* 初始化按键 (PC5) - 独立引脚，不受I2C影响 */
    Key_Init();
    
    /* 初始化统计模块 */
    Stats_Init();
    
    /* 初始化蜂鸣器 (PB8 PWM) */
    Buzzer_Init();
    
    /* 初始化UI模块 */
    UI_Init();
    
    /* 初始化OLED (使用PB10/PB11软件I2C) */
    OLED_Init();
    Delay_ms(100);
    
    /* 显示启动画面 */
    OLED_ShowString(1, 1, "System v11");
    OLED_ShowString(2, 1, "Track+Gyro+Key");
    OLED_ShowString(3, 1, "Init...");
    Delay_ms(800);
    
    /* 切换到IOI2C初始化陀螺仪 */
    IIC_Init();
    Delay_ms(50);
    JY301P_Init();
    Delay_ms(50);
    
    /* 显示初始化完成 */
    OLED_I2C_Init();  // 重新初始化OLED的I2C
    OLED_ShowString(3, 1, "Ready!    ");
    Delay_ms(500);
    OLED_Clear();
    
    /* 初始化硬件I2C（用于循迹模块） */
    HW_I2C_Init();
    Delay_ms(50);
    
    /*========== 第二阶段：主循环 ==========*/
    while (1)
    {
        g_loopCount++;
        
        /*=== 步骤1：按键扫描（独立引脚，随时可用）===*/
        keyEvent = Key_ScanWithTime(LOOP_PERIOD_MS);
        
        if (keyEvent == KEY_EVENT_SHORT)
        {
            UI_NextPage();
        }
        else if (keyEvent == KEY_EVENT_LONG)
        {
            if (UI_IsShowingStats())
            {
                UI_HideStats();
            }
            else
            {
                UI_ShowStats();
            }
        }
        
        /*=== 步骤2：更新统计数据（不依赖I2C）===*/
        Stats_Update(LOOP_PERIOD_MS);
        Stats_UpdateAngularSpeed(g_jy301p_data.gyro[2]);
        
        /*=== 步骤2.5：蜂鸣器状态机更新 ===*/
        Buzzer_Update(LOOP_PERIOD_MS);
        
        /* 检测循迹激活 */
        if (track_hw != 0x00 && track_hw != 0xFF)
        {
            Buzzer_StartTiming();
        }
        
        /* 检测1分钟提醒和终点条件 */
        if (Buzzer_IsTimingStarted())
        {
            if (Buzzer_CheckOneMinute(SysTick_GetMs()))
            {
                Buzzer_BeepTwice();
            }
            
            if (Buzzer_CheckFinish(track_hw, 
                                   g_jy301p_data.angle[0],
                                   g_jy301p_data.angle[1],
                                   g_jy301p_data.angle[2],
                                   SysTick_GetMs()))
            {
                Buzzer_BeepTriple();
            }
        }
        
        /*=== 步骤3：读取循迹模块（硬件I2C）===*/
        /* 确保硬件I2C已启用 */
        hw_result = HW_I2C_ReadByte(TRACKING_ADDR, TRACKING_REG, &track_hw);
        
        if (hw_result != 0) {
            fail_count++;
            if (fail_count >= 3) {
                HW_I2C_BusRecovery();
                fail_count = 0;
            }
        } else {
            fail_count = 0;
        }
        
        /*=== 步骤4：切换到软件I2C，读取陀螺仪 ===*/
        HW_I2C_Disable();
        Delay_us(100);
        
        IIC_Init();
        JY301P_Update();
        IIC_ReleaseBus();
        Delay_us(100);
        
        /*=== 步骤5：切换到OLED的I2C，更新显示 ===*/
        OLED_I2C_Init();
        Delay_us(50);
        
        UI_Display(track_hw, hw_result);
        
        /*=== 步骤6：切换回硬件I2C，准备下次循迹读取 ===*/
        HW_I2C_Enable();
        
        /*=== 步骤7：延时 ===*/
        Delay_ms(30);
    }
}

