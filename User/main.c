#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "bsp_usart.h"
#include "HW_I2C.h"
#include "Tracking.h"
#include "JY301P.h"
#include "IOI2C.h"
#include "bsp_key.h"
#include "bsp_key2.h"
#include "bsp_systick.h"
#include "bsp_led.h"
#include "bsp_led_pwm.h"
#include "bsp_buzzer.h"
#include "app_ui.h"
#include "app_stats.h"
#include "app_motor.h"
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
static uint32_t s_lastUltrasonicMs = 0;
static float s_cachedUltrasonicCm = -1.0f;
static uint8_t s_lastGyroReady = 0;

static void Main_ToggleObstacleMode(uint8_t gyroReady)
{
    uint8_t obstacleMode = Control_GetObstacleMode();

    if (!obstacleMode && !gyroReady)
    {
        UI_ShowObstacleHint(1);
        return;
    }

    obstacleMode = obstacleMode ? 0 : 1;
    Control_SetObstacleMode(obstacleMode);
    UI_SetObstacleMode(obstacleMode);
    UI_ShowObstacleHint(1);
    LED_StartObstacleEffect();
    Buzzer_BeepObstacle();
}

int main(void)
{
    uint8_t track_hw = 0x00;      // 硬件I2C读取的循迹数据
    uint8_t hw_result = 0;        // 硬件I2C返回值
    uint8_t fail_count = 0;       // 连续失败计数
    KeyEvent_t keyEvent;          // PC5按键事件
    Key2Event_t key2Event;        // PC4按键事件 (亮度调节)
    uint32_t motorTestStartMs = 0;
    uint32_t motorTestPhaseMs = 0;
    
    /*========== 第一阶段：初始化 ==========*/
    
    /* 初始化系统滴答定时器 (TIM2, 1ms中断) - 必须最先初始化 */
    SysTick_Init();
    
    /* 初始化LED PWM模块 (PB9绿色, PE0红色, PE1指示灯) - 使用TIM3 */
    LED_PWM_Init();
    LED_BindUIRefresh(UI_RequestRefresh);
    LED_Switch(LED_GREEN, 1);   /* 绿色LED开启 */
    LED_Switch(LED_RED, 1);     /* 红色LED开启 */
    
    /* 初始化按键 (PC5) - 页面切换 */
    Key_Init();
    
    /* 初始化按键2 (PC4) - 亮度调节 */
    Key2_Init();
    
    /* 初始化统计模块 */
    Stats_Init();
    
    /* 初始化蜂鸣器 (PB8 PWM) */
    Buzzer_Init();
    
    /* 初始化UI模块 */
    UI_Init();
    
    /* 初始化超声波模块 */
    HCSR04_Init();
    
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

    /* 电机与控制初始化 */
    Set_Motor(0);
    Control_Init();
    motorTestStartMs = SysTick_GetMs();
    motorTestPhaseMs = motorTestStartMs;
    s_lastUltrasonicMs = motorTestStartMs;
    s_cachedUltrasonicCm = -1.0f;
    
    /*========== 第二阶段：主循环 ==========*/
    while (1)
    {
        uint32_t nowMs;
        uint8_t gyroReady = s_lastGyroReady;

        g_loopCount++;

        nowMs = SysTick_GetMs();
        
        /*=== 步骤1：按键扫描（独立引脚，随时可用）===*/
        keyEvent = Key_ScanWithTime(LOOP_PERIOD_MS);
        key2Event = Key2_GetEvent();
        Key2_ClearEvent();
        
        /* 亮度调节模式处理 */
        if (LED_GetAdjustMode() == BRIGHTNESS_MODE_ADJUST)
        {
            /* 在亮度调节模式中 */
            if (key2Event == KEY2_EVENT_SHORT)
            {
                /* 短按PC4: 改变占空比 (亮度等级循环) */
                LED_ChangeBrightnessStep();
            }
            
            /* 长按PC4退出，或者PC5任何操作也退出模式 */
            if (key2Event == KEY2_EVENT_LONG || keyEvent == KEY_EVENT_SHORT || keyEvent == KEY_EVENT_LONG)
            {
                LED_ExitAdjustMode();
                LED_IndicatorBlink2();  /* 退出时指示灯闪烁 */
                
                if (keyEvent == KEY_EVENT_SHORT)
                {
                    UI_NextPage();
                }
                else if (keyEvent == KEY_EVENT_LONG)
                {
                    Main_ToggleObstacleMode(gyroReady);
                }
            }
        }
        else
        {
            /* 正常模式 */
            if (key2Event == KEY2_EVENT_LONG)
            {
                /* 长按PC4: 进入亮度调节模式 */
                LED_EnterAdjustMode();
            }
            
            if (keyEvent == KEY_EVENT_SHORT)
            {
                LED_IndicatorBlink2();  /* 指示灯闪烁两次 */
                UI_NextPage();
            }
            else if (keyEvent == KEY_EVENT_LONG)
            {
                LED_IndicatorBlink2();  /* 指示灯闪烁两次 */
                Main_ToggleObstacleMode(gyroReady);
            }
        }
        
        /*=== 步骤1.5：LED亮度调节模式更新 ===*/
        LED_AdjustModeUpdate(LOOP_PERIOD_MS);
        UI_Tick(LOOP_PERIOD_MS);

        /* 超声波采样(低频) */
        if ((nowMs - s_lastUltrasonicMs) >= 120U)
        {
            float distanceCm = HCSR04_GetValue();
            s_cachedUltrasonicCm = distanceCm;
            if (distanceCm >= 0.0f)
            {
                UI_SetUltrasonicDistance(distanceCm);
            }
            s_lastUltrasonicMs = nowMs;
        }

        /* 闭环控制输出 */
        {
            ControlOutput_t controlOut;
            float speedLeft = Encoder_GetSpeedMMS(ENCODER_LEFT);
            float speedRight = Encoder_GetSpeedMMS(ENCODER_RIGHT);
            uint8_t raceStarted = Buzzer_IsTimingStarted();

            uint8_t gyroValid = (JY301P_GetDataUpdateFlag() & (GYRO_UPDATE | ANGLE_UPDATE)) ? 1 : 0;
            if (gyroValid)
            {
                JY301P_ClearDataUpdateFlag(GYRO_UPDATE | ANGLE_UPDATE);
            }

            if (!raceStarted)
            {
                gyroValid = 0;
            }

            gyroReady = gyroValid ? 1 : 0;
            s_lastGyroReady = gyroReady;

            controlOut = Control_Update(track_hw, g_jy301p_data.gyro[2], g_jy301p_data.angle[1], s_cachedUltrasonicCm, speedLeft, speedRight, gyroValid, nowMs);

            /* 探线前安全限速：约15%（避免手持/离地时误动作过猛） */
            if (!raceStarted)
            {
                if (controlOut.vx > 150) controlOut.vx = 150;
                if (controlOut.vz > 150) controlOut.vz = 150;
                if (controlOut.vz < -150) controlOut.vz = -150;
            }

            Motion_Car_Control(controlOut.vx, 0, controlOut.vz);
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
        
        /* 检测1分钟提醒 */
        if (Buzzer_IsTimingStarted())
        {
            if (Buzzer_CheckOneMinute(SysTick_GetMs()))
            {
                Buzzer_BeepTwice();
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

