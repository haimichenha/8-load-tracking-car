#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "bsp_usart.h"
#include "HW_I2C.h"
#include "Tracking.h"
#include "JY301P.h"
#include "IOI2C.h"
#include "bsp_key.h"
#include "app_ui.h"
#include "app_stats.h"
#include <stdio.h>

/*
 * 版本 v18：综合页面 + 按键切换
 * 
 * 参数配置：
 * - 主循环周期: 5ms
 * - 长按阈值: 16ms
 * - 消抖采样: 1次
 */

#define TRACKING_ADDR   0x12
#define TRACKING_REG    0x30

/* 主循环周期 (ms) */
#define LOOP_PERIOD_MS  5

/* 主循环计数器 */
static volatile uint32_t g_loopCounter = 0;

/* 时钟变量 - 用循环计数当秒位 */
static volatile uint16_t g_seconds = 0;   /* 秒位 0-59，直接显示 */
static volatile uint16_t g_minutes = 0;   /* 分位 0-59 */
static volatile uint16_t g_hours = 0;     /* 时位（可选显示）*/

/* 不需要 g_secCounter 了，g_loopCounter 直接当秒位用 */

/* 长按计时变量 */
static volatile uint8_t g_longPressCounter = 0;  /* 长按计数 0-5 */

/* 当前页面 */
static uint8_t g_currentPage = 0;  /* 0=综合页, 1=循迹页 */

int main(void)
{
    uint8_t track_hw = 0x00;
    uint8_t hw_result = 0;
    uint8_t fail_count = 0;
    KeyEvent_t keyEvent;
    uint8_t keyRaw = 1;
    char minBuf[4];   /* 分钟缓冲 */
    char secBuf[4];   /* 秒钟缓冲 */
    uint8_t i;
    
    /*========== 初始化 ==========*/
    
    /* 初始化按键 */
    Key_Init();
    
    /* 初始化统计模块 */
    Stats_Init();
    
    /* 初始化UI模块 */
    UI_Init();
    
    /* 初始化OLED */
    OLED_Init();
    Delay_ms(100);
    
    /* 显示启动画面 */
    OLED_ShowString(1, 1, "System v18");
    OLED_ShowString(2, 1, "Long:16ms 5ms/lp");
    Delay_ms(800);
    OLED_Clear();
    
    /* 初始化IOI2C和陀螺仪 */
    IIC_Init();
    Delay_ms(50);
    JY301P_Init();
    Delay_ms(50);
    
    /* 初始化硬件I2C */
    HW_I2C_Init();
    Delay_ms(50);
    
    /*========== 主循环 ==========*/
    while (1)
    {
        g_loopCounter++;
        
        /* 时钟逻辑：g_loopCounter 当秒位，满60进位 */
        if (g_loopCounter >= 60)
        {
            g_loopCounter = 0;
            g_minutes++;
            
            /* 分钟满60也重置 */
            if (g_minutes >= 60)
            {
                g_minutes = 0;
                g_hours++;
            }
        }
        
        /*=== 按键扫描 ===*/
        keyRaw = Key_GetLevel();
        keyEvent = Key_ScanWithTime(LOOP_PERIOD_MS);
        
        /* 长按计时：按键按下时计数，松开时重置 */
        if (keyRaw == 0)  /* 按键按下 */
        {
            if (g_longPressCounter < 9)
                g_longPressCounter++;
        }
        else  /* 按键松开 */
        {
            g_longPressCounter = 0;
        }
        
        if (keyEvent == KEY_EVENT_SHORT)
        {
            /* 短按切换页面 */
            g_currentPage = (g_currentPage + 1) % 2;  /* 0 和 1 之间切换 */
            OLED_Clear();  /* 切换页面时清屏 */
        }
        else if (keyEvent == KEY_EVENT_LONG)
        {
            if (UI_IsShowingStats())
                UI_HideStats();
            else
                UI_ShowStats();
        }
        
        /*=== 更新时间 ===*/
        Stats_Update(LOOP_PERIOD_MS);
        
        /*=== 读取循迹 ===*/
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
        
        /*=== 读取陀螺仪 ===*/
        HW_I2C_Disable();
        Delay_us(100);
        IIC_Init();
        JY301P_Update();
        Stats_UpdateAngularSpeed(g_jy301p_data.gyro[2]);
        IIC_ReleaseBus();
        Delay_us(100);
        
        /*=== 更新循迹分析 ===*/
        UI_UpdateTrackingAnalysis(track_hw);
        
        /*=== 显示 ===*/
        OLED_I2C_Init();
        Delay_us(50);
        
        /* 格式化分钟 */
        minBuf[0] = '0' + (g_minutes / 10) % 10;
        minBuf[1] = '0' + g_minutes % 10;
        minBuf[2] = '\0';
        
        /* 秒位直接用 g_loopCounter */
        secBuf[0] = '0' + (g_loopCounter / 10) % 10;
        secBuf[1] = '0' + g_loopCounter % 10;
        secBuf[2] = '\0';
        
        /*=== 根据页面显示不同内容 ===*/
        if (g_currentPage == 0)
        {
            /*========== 页面0：综合信息页 ==========*/
            // 第1行: Roll Pitch
            OLED_ShowString(1, 1, "R:");
            OLED_ShowSignedNum(1, 3, (int32_t)g_jy301p_data.angle[0], 4);
            OLED_ShowString(1, 9, "P:");
            OLED_ShowSignedNum(1, 11, (int32_t)g_jy301p_data.angle[1], 4);
            
            // 第2行: Yaw + 二进制循迹
            OLED_ShowString(2, 1, "Y:");
            OLED_ShowSignedNum(2, 3, (int32_t)g_jy301p_data.angle[2], 4);
            OLED_ShowString(2, 8, " ");
            for (i = 0; i < 8; i++)
            {
                OLED_ShowChar(2, 9 + i, g_trackAnalysis.sensorStatus[i] ? '1' : '0');
            }
            
            // 第3行: 时间 + 速度预留
            OLED_ShowString(3, 1, "T:");
            OLED_ShowString(3, 3, minBuf);
            OLED_ShowString(3, 5, ":");
            OLED_ShowString(3, 6, secBuf);
            OLED_ShowString(3, 9, "V:----");  // 速度预留
            
            // 第4行: 超声波预留 + 按键状态
            OLED_ShowString(4, 1, "US:----");  // 超声波预留
            OLED_ShowString(4, 9, "K:");
            OLED_ShowNum(4, 11, keyRaw, 1);
            OLED_ShowString(4, 13, "S:");
            OLED_ShowNum(4, 15, g_longPressCounter, 1);
        }
        else
        {
            /*========== 页面1：循迹详情页 ==========*/
            // 第1行: 循迹二进制
            OLED_ShowString(1, 1, "TRK:");
            for (i = 0; i < 8; i++)
            {
                OLED_ShowChar(1, 5 + i, g_trackAnalysis.sensorStatus[i] ? '1' : '0');
            }
            OLED_ShowString(1, 14, "  ");  // 清除残留
            
            // 第2行: HEX值 + 偏移量
            OLED_ShowString(2, 1, "HEX:");
            OLED_ShowHexNum(2, 5, track_hw, 2);
            OLED_ShowString(2, 8, " OFF:");
            OLED_ShowSignedNum(2, 13, g_trackAnalysis.offset, 3);
            
            // 第3行: 左右传感器遮挡状态 (0-3左侧, 4-7右侧)
            OLED_ShowString(3, 1, "L:");
            for (i = 0; i < 4; i++)
            {
                OLED_ShowChar(3, 3 + i, g_trackAnalysis.sensorStatus[i] ? '1' : '0');
            }
            OLED_ShowString(3, 8, " R:");
            for (i = 4; i < 8; i++)
            {
                OLED_ShowChar(3, 11 + (i - 4), g_trackAnalysis.sensorStatus[i] ? '1' : '0');
            }
            
            // 第4行: 速度预留 + 按键状态
            OLED_ShowString(4, 1, "V:----");  // 速度预留
            OLED_ShowString(4, 9, "K:");
            OLED_ShowNum(4, 11, keyRaw, 1);
            OLED_ShowString(4, 13, "S:");
            OLED_ShowNum(4, 15, g_longPressCounter, 1);
        }
        
        /*=== 切换回硬件I2C ===*/
        HW_I2C_Enable();
        
        /*=== 延时 ===*/
        Delay_us(500);  // 500us延时
    }
}

