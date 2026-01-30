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

/*====================================================================================*/
/*                          电机测试模式开关                                           */
/*====================================================================================*/
/* 测试模式选择:
 * 0 = 正常循迹模式
 * 1 = 开环速度测试 (测量不同PWM下的速度)
 * 2 = 闭环速度环测试 (验证速度PID效果)
 */
#define MOTOR_TEST_MODE  2

#if MOTOR_TEST_MODE == 1
/*====================================================================================*/
/*                          电机速度测试模式                                           */
/*====================================================================================*/

/* 已测得的死区参数 */
#define DEADZONE_L_PCT      4       /* 左轮死区 4% */
#define DEADZONE_R_PCT      5       /* 右轮死区 5% */

/* 测试参数 */
#define TEST_PWM_LEVELS     5       /* 测试几个占空比等级 */
#define TEST_HOLD_MS        2000    /* 每个等级保持时间 (ms) */
#define SAMPLE_PERIOD_MS    50      /* 编码器采样周期 (ms) */

/* 测试的占空比列表 */
static const int16_t testPwmList[TEST_PWM_LEVELS] = {10, 20, 30, 50, 80};

/* 测试阶段 */
typedef enum {
    TEST_IDLE = 0,      /* 等待开始 */
    TEST_RUNNING,       /* 测试中 */
    TEST_DONE           /* 完成 */
} TestPhase_t;

int main(void)
{
    TestPhase_t phase = TEST_IDLE;
    uint32_t phaseStartMs = 0;
    uint32_t lastSampleMs = 0;
    uint8_t currentLevel = 0;
    int16_t currentPwm = 0;
    float speedL = 0.0f, speedR = 0.0f;
    float sumSpeedL = 0.0f, sumSpeedR = 0.0f;
    uint16_t sampleCount = 0;
    float avgSpeedL = 0.0f, avgSpeedR = 0.0f;
    char buf[20];
    
    /* 初始化 */
    SysTick_Init();
    Key_Init();
    Key2_Init();
    
    /* 初始化OLED */
    OLED_Init();
    Delay_ms(100);
    
    /* 初始化电机和编码器 */
    Set_Motor(0);
    
    /* 显示启动信息 */
    OLED_Clear();
    OLED_ShowString(1, 1, "Speed Test");
    OLED_ShowString(2, 1, "DeadZone:");
    sprintf(buf, "L%d%% R%d%%", DEADZONE_L_PCT, DEADZONE_R_PCT);
    OLED_ShowString(2, 10, buf);
    OLED_ShowString(3, 1, "PC5=Start");
    OLED_ShowString(4, 1, "Test:10,20,30,50,80%");
    
    phaseStartMs = SysTick_GetMs();
    lastSampleMs = phaseStartMs;
    
    while (1)
    {
        uint32_t nowMs = SysTick_GetMs();
        uint32_t elapsed = nowMs - phaseStartMs;
        KeyEvent_t keyEvent;
        Key2Event_t key2Event;
        
        /* 扫描按键 */
        keyEvent = Key_Scan();
        Key2_Scan();
        key2Event = Key2_GetEvent();
        if (key2Event != KEY2_EVENT_NONE) Key2_ClearEvent();
        
        /* PC4 按下 = 紧急停止 */
        if (key2Event == KEY2_EVENT_SHORT || key2Event == KEY2_EVENT_LONG)
        {
            Motor_SetSpeedBoth(0, 0);
            phase = TEST_IDLE;
            currentLevel = 0;
            OLED_Clear();
            OLED_ShowString(1, 1, "STOPPED!");
            OLED_ShowString(3, 1, "PC5=Restart");
        }
        
        /* PC5 按下 = 开始/重新开始 */
        if (keyEvent == KEY_EVENT_SHORT && phase == TEST_IDLE)
        {
            phase = TEST_RUNNING;
            currentLevel = 0;
            currentPwm = testPwmList[0];
            phaseStartMs = nowMs;
            sumSpeedL = 0.0f;
            sumSpeedR = 0.0f;
            sampleCount = 0;
            OLED_Clear();
        }
        
        /* 采样编码器 */
        if ((nowMs - lastSampleMs) >= SAMPLE_PERIOD_MS)
        {
            Encoder_Update(nowMs - lastSampleMs);
            speedL = Encoder_GetSpeedMMS(ENCODER_LEFT);
            speedR = Encoder_GetSpeedMMS(ENCODER_RIGHT);
            lastSampleMs = nowMs;
            
            /* 累加（跳过前500ms让电机稳定） */
            if (phase == TEST_RUNNING && elapsed > 500)
            {
                sumSpeedL += speedL;
                sumSpeedR += speedR;
                sampleCount++;
            }
        }
        
        /* 状态机 */
        switch (phase)
        {
        case TEST_IDLE:
            break;
            
        case TEST_RUNNING:
            /* 双轮同时转 */
            Motor_SetSpeed(MOTOR_LEFT, currentPwm);
            Motor_SetSpeed(MOTOR_RIGHT, currentPwm);
            
            /* 显示当前状态 */
            sprintf(buf, "PWM:%2d%% [%d/%d]", currentPwm, currentLevel + 1, TEST_PWM_LEVELS);
            OLED_ShowString(1, 1, buf);
            sprintf(buf, "L:%+7.1f mm/s", speedL);
            OLED_ShowString(2, 1, buf);
            sprintf(buf, "R:%+7.1f mm/s", speedR);
            OLED_ShowString(3, 1, buf);
            
            /* 时间到，计算平均并切换 */
            if (elapsed >= TEST_HOLD_MS)
            {
                if (sampleCount > 0)
                {
                    avgSpeedL = sumSpeedL / sampleCount;
                    avgSpeedR = sumSpeedR / sampleCount;
                }
                
                /* 显示结果 */
                sprintf(buf, "%2d%%:L%+.0f R%+.0f", currentPwm, avgSpeedL, avgSpeedR);
                OLED_ShowString(4, 1, buf);
                
                /* 下一个等级 */
                currentLevel++;
                if (currentLevel >= TEST_PWM_LEVELS)
                {
                    phase = TEST_DONE;
                    Motor_SetSpeedBoth(0, 0);
                }
                else
                {
                    currentPwm = testPwmList[currentLevel];
                    phaseStartMs = nowMs;
                    sumSpeedL = 0.0f;
                    sumSpeedR = 0.0f;
                    sampleCount = 0;
                }
            }
            break;
            
        case TEST_DONE:
            Motor_SetSpeedBoth(0, 0);
            OLED_ShowString(1, 1, "=== DONE ===    ");
            OLED_ShowString(2, 1, "Last result:    ");
            sprintf(buf, "L:%+.1f mm/s", avgSpeedL);
            OLED_ShowString(3, 1, buf);
            sprintf(buf, "R:%+.1f mm/s", avgSpeedR);
            OLED_ShowString(4, 1, buf);
            
            if (keyEvent == KEY_EVENT_SHORT)
            {
                phase = TEST_IDLE;
                currentLevel = 0;
                OLED_Clear();
                OLED_ShowString(1, 1, "Speed Test");
                OLED_ShowString(2, 1, "PC5=Start");
            }
            break;
        }
        
        Delay_ms(10);
    }
}

#elif MOTOR_TEST_MODE == 2
/*====================================================================================*/
/*                          闭环速度环测试模式                                         */
/*====================================================================================*/
/* 
 * 测试速度环 PID 效果
 * 使用 Motion_Car_Control 接口，验证速度跟踪性能
 */

/* 测试速度列表 (mm/s 的千分比，1000 = 最大速度) */
#define SPEED_TEST_LEVELS   4
static const int16_t speedTestList[SPEED_TEST_LEVELS] = {200, 400, 600, 200};  /* 对应 20%, 40%, 60%, 20% */
#define SPEED_TEST_HOLD_MS  8000    /* 每个速度保持时间 8秒 */
#define SPEED_SAMPLE_MS     50      /* 采样周期 */

typedef enum {
    STEST_IDLE = 0,
    STEST_RUNNING,
    STEST_DONE
} SpeedTestPhase_t;

int main(void)
{
    SpeedTestPhase_t phase = STEST_IDLE;
    int currentLevel = 0;
    int16_t targetSpeed = 0;
    uint32_t phaseStartMs = 0;
    uint32_t lastSampleMs = 0;
    uint32_t nowMs;
    uint32_t elapsed;
    float speedL, speedR;
    float targetMms;
    float errorL, errorR;
    char buf[20];
    KeyEvent_t keyEvent;
    Key2Event_t key2Event;
    
    /* 初始化 */
    SysTick_Init();
    OLED_Init();
    Key_Init();
    Key2_Init();
    Set_Motor(0);
    Encoder_Init();
    Encoder_Reset();
    
    OLED_Clear();
    OLED_ShowString(1, 1, "Speed Loop Test");
    OLED_ShowString(2, 1, "PC5=Start");
    OLED_ShowString(3, 1, "PC4=Stop");
    OLED_ShowString(4, 1, "Verify PID");
    
    lastSampleMs = SysTick_GetMs();
    
    while (1)
    {
        nowMs = SysTick_GetMs();
        elapsed = nowMs - phaseStartMs;
        
        /* 按键扫描 */
        keyEvent = Key_Scan();
        
        Key2_Scan();
        key2Event = Key2_GetEvent();
        if (key2Event != KEY2_EVENT_NONE) Key2_ClearEvent();
        
        /* PC4 = 紧急停止 */
        if (key2Event == KEY2_EVENT_SHORT || key2Event == KEY2_EVENT_LONG)
        {
            Motion_Car_Control(0, 0, 0);
            phase = STEST_IDLE;
            currentLevel = 0;
            OLED_Clear();
            OLED_ShowString(1, 1, "STOPPED!");
            OLED_ShowString(3, 1, "PC5=Restart");
        }
        
        /* PC5 = 开始 */
        if (keyEvent == KEY_EVENT_SHORT && phase == STEST_IDLE)
        {
            phase = STEST_RUNNING;
            currentLevel = 0;
            targetSpeed = speedTestList[0];
            phaseStartMs = nowMs;
            OLED_Clear();
        }
        
        /* 采样速度：编码器更新在 Motion_Car_Control 内部完成，避免重复清零计数 */
        if ((nowMs - lastSampleMs) >= SPEED_SAMPLE_MS)
        {
            speedL = Encoder_GetSpeedMMS(ENCODER_LEFT);
            speedR = Encoder_GetSpeedMMS(ENCODER_RIGHT);
            lastSampleMs = nowMs;
        }
        
        /* 状态机 */
        switch (phase)
        {
        case STEST_IDLE:
            break;
            
        case STEST_RUNNING:
            /* 调用闭环控制 */
            Motion_Car_Control(targetSpeed, 0, 0);
            
            /* 计算目标速度 (mm/s): 2100 = MOTOR_SPEED_MAX_MMS */
            targetMms = (float)targetSpeed / 1000.0f * 2100.0f;
            errorL = targetMms - speedL;
            errorR = targetMms - speedR;
            
            /* 显示：目标、实际速度、误差 */
            sprintf(buf, "Cmd:%3d Tgt:%4.0f", targetSpeed, targetMms);
            OLED_ShowString(1, 1, buf);
            sprintf(buf, "L:%+5.0f E:%+4.0f", speedL, errorL);
            OLED_ShowString(2, 1, buf);
            sprintf(buf, "R:%+5.0f E:%+4.0f", speedR, errorR);
            OLED_ShowString(3, 1, buf);
            sprintf(buf, "%d/%d  %lus       ", currentLevel + 1, SPEED_TEST_LEVELS, elapsed / 1000);
            OLED_ShowString(4, 1, buf);
            
            /* 切换到下一个速度 */
            if (elapsed >= SPEED_TEST_HOLD_MS)
            {
                currentLevel++;
                if (currentLevel >= SPEED_TEST_LEVELS)
                {
                    phase = STEST_DONE;
                    Motion_Car_Control(0, 0, 0);
                }
                else
                {
                    targetSpeed = speedTestList[currentLevel];
                    phaseStartMs = nowMs;
                }
            }
            break;
            
        case STEST_DONE:
            Motion_Car_Control(0, 0, 0);
            OLED_ShowString(1, 1, "=== DONE ===    ");
            OLED_ShowString(2, 1, "Speed loop OK?  ");
            OLED_ShowString(3, 1, "Check errors    ");
            OLED_ShowString(4, 1, "PC5=Restart     ");
            
            if (keyEvent == KEY_EVENT_SHORT)
            {
                phase = STEST_IDLE;
                currentLevel = 0;
                OLED_Clear();
                OLED_ShowString(1, 1, "Speed Loop Test");
                OLED_ShowString(2, 1, "PC5=Start");
            }
            break;
        }
        
        Delay_ms(10);
    }
}

#else
/*====================================================================================*/
/*                          正常模式 - 原有代码                                        */
/*====================================================================================*/

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

/* 主循环最小步进 (ms) */
#define LOOP_PERIOD_MS  2

/* 任务周期 (ms) */
#define CONTROL_PERIOD_MS   20
#define OLED_REFRESH_MS     200
#define GYRO_REFRESH_MS     20

/* 全局运行时间计数器 */
static volatile uint32_t g_loopCount = 0;
static uint32_t s_lastLoopMs = 0;
static uint32_t s_lastControlMs = 0;
static uint32_t s_lastGyroMs = 0;
static uint32_t s_lastOledMs = 0;
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
    int16_t cmdVx = 0;
    int16_t cmdVz = 0;

    /* 电机标定(临时)实测死区：左4%，右5% (PWM周期=3600) */
    uint16_t motorMinPwmL = 144;
    uint16_t motorMinPwmR = 180;
    float motorSpeedDiffMms = 0.0f;

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
    UI_SetMotorTest(motorMinPwmL, motorMinPwmR, motorSpeedDiffMms);
    
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
    s_lastLoopMs = motorTestStartMs;
    s_lastControlMs = motorTestStartMs;
    s_lastGyroMs = motorTestStartMs;
    s_lastOledMs = motorTestStartMs;
    s_lastUltrasonicMs = motorTestStartMs;
    s_cachedUltrasonicCm = -1.0f;
    
    /*========== 第二阶段：主循环 ==========*/
    while (1)
    {
        uint32_t nowMs;
        uint32_t deltaMs;
        uint8_t needHwI2CEnable = 0;
        uint8_t gyroReady = s_lastGyroReady;

        g_loopCount++;

        nowMs = SysTick_GetMs();
        deltaMs = nowMs - s_lastLoopMs;
        s_lastLoopMs = nowMs;

        /*=== 步骤1：按键扫描（独立引脚，随时可用）===*/
        keyEvent = Key_ScanWithTime(deltaMs);
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
        LED_AdjustModeUpdate(deltaMs);
        UI_Tick(deltaMs);

        /*=== 步骤2：更新统计数据（不依赖I2C）===*/
        Stats_Update(deltaMs);
        Stats_UpdateAngularSpeed(g_jy301p_data.gyro[2]);

        {
            float speedLeft = Encoder_GetSpeedMMS(ENCODER_LEFT);
            float speedRight = Encoder_GetSpeedMMS(ENCODER_RIGHT);
            motorSpeedDiffMms = speedLeft - speedRight;
            UI_SetMotorTest(motorMinPwmL, motorMinPwmR, motorSpeedDiffMms);
        }

        /*=== 步骤2.5：蜂鸣器状态机更新 ===*/
        Buzzer_Update(deltaMs);

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

        /*=== 步骤3：控制周期：读取循迹 + PID输出 ===*/
        if ((nowMs - s_lastControlMs) >= CONTROL_PERIOD_MS)
        {
            ControlOutput_t controlOut;
            float speedLeft = Encoder_GetSpeedMMS(ENCODER_LEFT);
            float speedRight = Encoder_GetSpeedMMS(ENCODER_RIGHT);
            uint8_t raceStarted;
            uint8_t gyroValid;

            s_lastControlMs = nowMs;

            /* 读取循迹模块（硬件I2C） */
            hw_result = HW_I2C_ReadByte(TRACKING_ADDR, TRACKING_REG, &track_hw);
            if (hw_result != 0)
            {
                fail_count++;
                if (fail_count >= 3)
                {
                    HW_I2C_BusRecovery();
                    fail_count = 0;
                }
            }
            else
            {
                fail_count = 0;
            }

            /* 检测循迹激活 */
            if (track_hw != 0x00 && track_hw != 0xFF)
            {
                Buzzer_StartTiming();
            }

            raceStarted = Buzzer_IsTimingStarted();

            gyroValid = (JY301P_GetDataUpdateFlag() & (GYRO_UPDATE | ANGLE_UPDATE)) ? 1 : 0;
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

            cmdVx = controlOut.vx;
            cmdVz = controlOut.vz;
        }

        /* 检测1分钟提醒 */
        if (Buzzer_IsTimingStarted())
        {
            if (Buzzer_CheckOneMinute(nowMs))
            {
                Buzzer_BeepTwice();
            }
        }

        /*=== 步骤4：切换到软件I2C，读取陀螺仪（降频） ===*/
        if ((nowMs - s_lastGyroMs) >= GYRO_REFRESH_MS)
        {
            HW_I2C_Disable();
            Delay_us(100);

            IIC_Init();
            JY301P_Update();
            IIC_ReleaseBus();
            Delay_us(100);

            s_lastGyroMs = nowMs;
            needHwI2CEnable = 1;
        }

        /*=== 步骤5：切换到OLED的I2C，更新显示（降频） ===*/
        if ((nowMs - s_lastOledMs) >= OLED_REFRESH_MS)
        {
            if (!needHwI2CEnable)
            {
                HW_I2C_Disable();
                Delay_us(100);
            }

            OLED_I2C_Init();
            Delay_us(50);

            UI_Display(track_hw, hw_result);

            s_lastOledMs = nowMs;
            needHwI2CEnable = 1;
        }

        /*=== 步骤6：切换回硬件I2C，准备下次循迹读取 ===*/
        if (needHwI2CEnable)
        {
            HW_I2C_Enable();
        }

        /* 电机命令保持，速度环持续运行 */
        Motion_Car_Control(cmdVx, 0, cmdVz);

        /*=== 步骤7：延时 ===*/
        Delay_ms(1);
    }
}

#endif /* MOTOR_TEST_MODE */

