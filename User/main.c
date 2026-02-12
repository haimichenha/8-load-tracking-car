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
#include "bsp_ir_gpio.h"  /* GPIO 循迹驱动 */
#include <stdio.h>

/*====================================================================================*/
/*                          电机测试模式开关                                           */
/*====================================================================================*/
/* 测试模式选择:
 * 0 = 正常循迹模式 (使用 Vx+Vz 混合)
 * 1 = 开环速度测试 (测量不同PWM下的速度)
 * 2 = 闭环速度环测试 (验证速度PID效果)
 * 3 = I2C 循迹 + 位置式 PI (V4)
 * 4 = GPIO 循迹 + 高频闭环 (V5 - 推荐！)
 */
#define MOTOR_TEST_MODE  4

#if MOTOR_TEST_MODE == 1
/*====================================================================================*/
/*                          电机速度测试模式                                           */
/*====================================================================================*/

/* 已测得的死区参数 */
#define DEADZONE_L_PCT      3       /* 左轮死区 4% */
#define DEADZONE_R_PCT      4       /* 右轮死区 5% */

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

#elif MOTOR_TEST_MODE == 3
/*====================================================================================*/
/*                          双环PID循迹控制 (V4 - 简化稳定版)                          */
/*====================================================================================*/
/*
 * 问题诊断：
 * 1. I2C 读取慢（~500μs），但 10ms 周期够用
 * 2. 增量式 PID 参数敏感，改用位置式 PI
 * 3. PWM 突变导致冲刺感，加入平滑
 */

#define TRACKING_ADDR   0x12
#define TRACKING_REG    0x30

/*==================== 控制参数 ====================*/

/* 基础速度 (mm/s) */
#define BASE_SPEED_MMS      400.0f      /* 降低基础速度 */
#define MAX_TURN_SPEED      100.0f      /* 最大转向速度差 */

/* 循迹 PD 参数（外环）- 已移至第793行统一管理 */

/* 速度 PI 参数（内环）- 位置式 */
#define SPEED_KP            0.03f       /* 比例：误差100mm/s → PWM 3% */
#define SPEED_KI            0.005f      /* 积分：消除稳态误差 */
#define INTEGRAL_MAX        200.0f      /* 积分限幅 */
#define PWM_MAX             40.0f       /* PWM 最大值 % */
#define PWM_MIN             8.0f        /* PWM 最小值（死区）*/

/* PWM 平滑 */
#define PWM_SLEW_RATE       2.25f        /* 每 10ms 最大变化 2% */

/* 传感器权重 */
static const int8_t SENSOR_WEIGHTS[8] = {-40, -27, -18, -4, 4, 18, 27, 40};

/*==================== 位置式 PI 结构体 ====================*/
typedef struct {
    float Kp, Ki;
    float integral;     /* 积分累积 */
    float integralMax;  /* 积分限幅 */
    float output;       /* 当前输出 */
    float outMin, outMax;
} PI_Position_t;

/* PI 实例 */
static PI_Position_t s_piL, s_piR;

/* 循迹状态 */
static int8_t s_lastError = 0;
static int8_t s_lastDir = 1;
static float s_lastTrackErr = 0.0f;
static uint32_t s_lostLineMs = 0;

/* PWM 平滑 */
static float s_pwmL = 0, s_pwmR = 0;

/*==================== PI 函数 ====================*/

static void PI_Init(PI_Position_t *pi, float kp, float ki, float intMax, float outMin, float outMax)
{
    pi->Kp = kp;
    pi->Ki = ki;
    pi->integral = 0;
    pi->integralMax = intMax;
    pi->output = 0;
    pi->outMin = outMin;
    pi->outMax = outMax;
}

static float PI_Compute(PI_Position_t *pi, float target, float actual)
{
    float err = target - actual;
    float pOut, iOut;
    
    /* 积分累积 */
    pi->integral += err;
    
    /* 积分限幅 */
    if (pi->integral > pi->integralMax) pi->integral = pi->integralMax;
    if (pi->integral < -pi->integralMax) pi->integral = -pi->integralMax;
    
    /* PI 计算 */
    pOut = pi->Kp * err;
    iOut = pi->Ki * pi->integral;
    pi->output = pOut + iOut;
    
    /* 输出限幅 */
    if (pi->output > pi->outMax) pi->output = pi->outMax;
    if (pi->output < pi->outMin) pi->output = pi->outMin;
    
    return pi->output;
}

static void PI_Reset(PI_Position_t *pi)
{
    pi->integral = 0;
    pi->output = 0;
}

/*==================== PWM 平滑函数 ====================*/
static float smoothPWM(float current, float target, float maxChange)
{
    float diff = target - current;
    if (diff > maxChange) diff = maxChange;
    if (diff < -maxChange) diff = -maxChange;
    return current + diff;
}

/*==================== 循迹误差计算 ====================*/

static int8_t calcTrackError(uint8_t raw)
{
    uint8_t active = (uint8_t)(~raw);
    int16_t sum = 0;
    uint8_t count = 0;
    int8_t error;
    uint8_t i;
    
    for (i = 0; i < 8; i++) {
        if (active & (1U << (7 - i))) {
            sum += SENSOR_WEIGHTS[i];
            count++;
        }
    }
    
    if (count == 0 || count == 8) {
        return s_lastError;
    }
    
    error = (int8_t)(sum / (int16_t)count);
    if (error > 30) error = 30;
    if (error < -30) error = -30;
    
    s_lastError = error;
    if (error > 0) s_lastDir = 1;
    else if (error < 0) s_lastDir = -1;
    
    return error;
}

/*==================== 主函数 ====================*/

int main(void)
{
    uint8_t track_raw = 0xFF;
    uint8_t hw_result = 0;
    uint8_t fail_count = 0;
    uint32_t lastControlMs = 0;
    uint32_t lastEncoderMs = 0;
    uint32_t lastOledMs = 0;
    char buf[24];
    KeyEvent_t keyEvent;
    Key2Event_t key2Event;
    uint8_t running = 0;
    
    /* 速度变量 */
    float speedL = 0, speedR = 0;
    float targetL = 0, targetR = 0;
    float pwmTargetL = 0, pwmTargetR = 0;
    
    /* 初始化 */
    SysTick_Init();
    Key_Init();
    Key2_Init();
    OLED_Init();
    Delay_ms(100);
    
    Set_Motor(0);
    Encoder_Init();
    Encoder_Reset();
    
    /* 初始化硬件 I2C */
    HW_I2C_Init();
    Delay_ms(50);
    
    /* 初始化速度 PI */
    PI_Init(&s_piL, SPEED_KP, SPEED_KI, INTEGRAL_MAX, PWM_MIN, PWM_MAX);
    PI_Init(&s_piR, SPEED_KP, SPEED_KI, INTEGRAL_MAX, PWM_MIN, PWM_MAX);
    
    OLED_Clear();
    OLED_ShowString(1, 1, "V4 Stable PID");
    OLED_ShowString(2, 1, "PC5=Start/Stop");
    OLED_ShowString(3, 1, "Position PI");
    OLED_ShowString(4, 1, "PWM Smoothing");
    
    lastControlMs = SysTick_GetMs();
    lastOledMs = lastControlMs;
    lastEncoderMs = lastControlMs;
    
    while (1)
    {
        uint32_t nowMs = SysTick_GetMs();
        
        /* 按键扫描 */
        keyEvent = Key_Scan();
        Key2_Scan();
        key2Event = Key2_GetEvent();
        if (key2Event != KEY2_EVENT_NONE) Key2_ClearEvent();
        
        /* PC4 = 紧急停止 */
        if (key2Event == KEY2_EVENT_SHORT || key2Event == KEY2_EVENT_LONG) {
            Motor_SetSpeedBoth(0, 0);
            running = 0;
            s_pwmL = 0; s_pwmR = 0;
            PI_Reset(&s_piL);
            PI_Reset(&s_piR);
            OLED_ShowString(4, 1, "STOPPED!        ");
        }
        
        /* PC5 = 开始/停止 */
        if (keyEvent == KEY_EVENT_SHORT) {
            running = running ? 0 : 1;
            if (!running) {
                Motor_SetSpeedBoth(0, 0);
                s_pwmL = 0; s_pwmR = 0;
            } else {
                /* 启动时给初始 PWM */
                s_pwmL = 10.0f;
                s_pwmR = 10.0f;
                s_piL.integral = 100.0f;  /* 预填积分，加速启动 */
                s_piR.integral = 100.0f;
            }
            s_lastError = 0;
            s_lastTrackErr = 0;
            s_lostLineMs = 0;
        }
        
        /* 编码器更新 5ms */
        if ((nowMs - lastEncoderMs) >= 5)
        {
            Encoder_Update(nowMs - lastEncoderMs);
            speedL = Encoder_GetSpeedMMS(ENCODER_LEFT);
            speedR = Encoder_GetSpeedMMS(ENCODER_RIGHT);
            lastEncoderMs = nowMs;
        }
        
        /* 控制周期 10ms */
        if ((nowMs - lastControlMs) >= 10)
        {
            float dt = (float)(nowMs - lastControlMs) / 1000.0f;
            int8_t trackErr;
            float derivative;
            float turnSpeed;
            
            lastControlMs = nowMs;
            
            /* 读取循迹 */
            hw_result = HW_I2C_ReadByte(TRACKING_ADDR, TRACKING_REG, &track_raw);
            if (hw_result != 0) {
                fail_count++;
                if (fail_count >= 3) {
                    HW_I2C_BusRecovery();
                    fail_count = 0;
                }
            } else {
                fail_count = 0;
            }
            
            if (!running) {
                continue;
            }
            
            /* 丢线检测 */
            if (track_raw == 0xFF) {
                if (s_lostLineMs == 0) {
                    s_lostLineMs = nowMs;
                }
                
                if ((nowMs - s_lostLineMs) >= 500) {
                    Motor_SetSpeedBoth(0, 0);
                    s_pwmL = 0; s_pwmR = 0;
                    continue;
                }
                
                /* 丢线：按上次方向转 */
                if (s_lastDir > 0) {
                    targetL = BASE_SPEED_MMS;
                    targetR = BASE_SPEED_MMS * 0.4f;
                } else {
                    targetL = BASE_SPEED_MMS * 0.4f;
                    targetR = BASE_SPEED_MMS;
                }
            }
            else
            {
                s_lostLineMs = 0;
                
                /*========== 外环：循迹 PD ==========*/
                trackErr = calcTrackError(track_raw);
                derivative = ((float)trackErr - s_lastTrackErr) / dt;
                turnSpeed = TRACK_KP * (float)trackErr + TRACK_KD * derivative;
                s_lastTrackErr = (float)trackErr;
                
                if (turnSpeed > MAX_TURN_SPEED) turnSpeed = MAX_TURN_SPEED;
                if (turnSpeed < -MAX_TURN_SPEED) turnSpeed = -MAX_TURN_SPEED;
                
                /* 差速分配 */
                if (turnSpeed >= 0) {
                    targetL = BASE_SPEED_MMS;
                    targetR = BASE_SPEED_MMS - turnSpeed;
                } else {
                    targetR = BASE_SPEED_MMS;
                    targetL = BASE_SPEED_MMS + turnSpeed;
                }
                
                if (targetL < 80.0f) targetL = 80.0f;
                if (targetR < 80.0f) targetR = 80.0f;
            }
            
            /*========== 内环：速度 PI ==========*/
            pwmTargetL = PI_Compute(&s_piL, targetL, speedL);
            pwmTargetR = PI_Compute(&s_piR, targetR, speedR);
            
            /*========== PWM 平滑 ==========*/
            s_pwmL = smoothPWM(s_pwmL, pwmTargetL, PWM_SLEW_RATE);
            s_pwmR = smoothPWM(s_pwmR, pwmTargetR, PWM_SLEW_RATE);
            
            /* 设置电机 */
            Motor_SetSpeedBoth((int16_t)s_pwmL, (int16_t)s_pwmR);
        }
        
        /* OLED 刷新 250ms */
        if ((nowMs - lastOledMs) >= 250)
        {
            lastOledMs = nowMs;
            
            HW_I2C_Disable();
            Delay_us(100);
            OLED_I2C_Init();
            
            sprintf(buf, "E:%+3d T:%02X", s_lastError, track_raw);
            OLED_ShowString(1, 1, buf);
            
            sprintf(buf, "Tgt%3.0f/%3.0f", targetL, targetR);
            OLED_ShowString(2, 1, buf);
            
            sprintf(buf, "Spd%3.0f/%3.0f", speedL, speedR);
            OLED_ShowString(3, 1, buf);
            
            sprintf(buf, "PWM%2.0f/%2.0f %s", s_pwmL, s_pwmR, running ? "RUN" : "STP");
            OLED_ShowString(4, 1, buf);
            
            HW_I2C_Enable();
        }
        
        Delay_ms(1);
    }
}

#elif MOTOR_TEST_MODE == 4
/*====================================================================================*/
/*                          GPIO 循迹 + 高频闭环 (V5)                                  */
/*====================================================================================*/
/*
 * 核心改进：
 * 1. GPIO 直接读取循迹 (< 1μs vs I2C 500μs)
 * 2. 控制周期 5ms (200Hz vs 100Hz)
 * 3. 更激进的速度闭环
 * 4. 左右轮差异补偿
 */

/*====================================================================================*/
/*====================================================================================*/
/*                    【干净版】速度PI闭环 + 循迹PD                                   */
/*====================================================================================*/
/*
 * 调参顺序：
 * 
 * 【第一步】速度环（内环）：先P后I，不要D
 *   1. Ki=0，从Kp=0.01开始翻倍，直到电机能转
 *   2. 继续增加Kp，直到出现"嗡嗡"震动，记录此值
 *   3. 取50%作为最终Kp
 *   4. 加Ki（通常是Kp的1/10~1/100），消除静差
 * 
 * 【第二步】循迹环（外环）：先P后D
 *   1. Kd=0，从小到大调Kp，直到出现明显波动
 *   2. 取50%~60%作为最终Kp
 *   3. 加Kd抑制振荡，让过弯更丝滑
 */

/*==================== 调试模式开关 ====================*/
#define SPEED_LOOP_TUNING   0           /* 1=速度环调试模式, 0=正常循迹模式 */
#define DEBUG_NO_KI         0           /* 1=关闭Ki调试, 0=正常（开启Ki） */
#define DEBUG_OPEN_LOOP     0           /* 1=开环测试(直接给PWM), 0=闭环PI */
#define DEBUG_FIXED_PWM     30.0f       /* 开环测试时的固定PWM值 */

/*==================== 速度环参数（内环）====================*/
/* MG310 电机动力很强，PWM=30 就能跑 650mm/s
 * Kp=0.18/Ki=0.12 在500mm/s时OK但320mm/s时PWM振荡(0~33跳动)
 * Kp=0.12/Ki=0.04 不振荡但太慢：目标364实际220，差速建立不起来
 * 折中：Kp=0.15/Ki=0.06，兼顾平滑与响应
 */
#define SPEED_KP            0.15f       /* 速度P - 折中值 */
#define SPEED_KI            0.06f       /* 速度I - 折中值，比0.12低一半 */
#define SPEED_KD            0.0f        /* 速度D - 不需要 */
#define INTEGRAL_MAX        150.0f      /* 积分限幅 - 适度收紧 */

/*==================== 速度环调试目标速度 ====================*/
/* 注意：PWM=30 对应约 650 mm/s，所以目标速度要合理设置
 * 目标速度太低会导致 PI 输出接近 0，产生间断振荡
 */
#define TEST_TARGET_SPEED   500.0f      /* 测试目标速度 mm/s */

/*==================== 循迹环参数（外环）====================*/
/* 调试步骤：
 * 1. 先设 Kd=0，从小到大调 Kp
 * 2. Kp 太小：转弯迟钝，跟不上线
 * 3. Kp 太大：左右摆动（蛇形）
 * 4. 找到临界点后，加 Kd 抑制摆动
 */
#define TRACK_KP            8.00f        /* 循迹P - 回退到验证过的值，先修内环 */
#define TRACK_KD            0.80f        /* 循迹D - 回退到验证过的值 */

/*==================== 速度设定 ====================*/
/* 注意：PWM=30 对应约 650 mm/s
 * 基础速度要根据实际电机能力设置
 */
#define BASE_SPEED_MMS      320.0f      /* 大幅降速！先跑稳再提速 */
#define MIN_SPEED_MMS        60.0f      /* 最低速度 mm/s */
#define MAX_SPEED_MMS       600.0f      /* 最高速度 mm/s */

/*==================== PWM限制 ====================*/
/* 注意：电机驱动接受 0~100 的百分比值
 * PWM_MAX 设太低会导致 Kp 调不出震动（输出被限幅）
 * 建议设为 80~100，给 PI 足够的调节空间
 */
#define PWM_MAX             100.0f
#define PWM_MIN             0.0f

/*==================== 死区补偿 ====================*/
/* 电机死区：PWM低于此值轮子不动
 * 注意：MG310 电机动力很强，PWM=30 就能跑 650mm/s
 * 死区补偿不能太大，否则会导致间断振荡
 */
#define MOTOR_DEADZONE      1.5f        /* 死区PWM值 - 改小！ */

/*==================== 滤波参数 ====================*/
#define SPEED_FILT_ALPHA    0.3f        /* 速度滤波 - 增大减少滞后 */
#define ERROR_FILT_ALPHA    0.5f        /* 误差滤波 - 增大减少滞后 */

/*==================== 辅助宏 ====================*/
#define ABSF(x)             (((x) >= 0.0f) ? (x) : -(x))

/*==================== 日志缓冲系统 ====================*/
/* 运行时存到RAM，停下来后通过串口导出
 * 每条日志 8 字节，可存约 2000 条（约 20 秒 @ 10ms周期）
 */
#define LOG_ENABLE          1           /* 1=启用日志，0=关闭 */
#define LOG_PERIOD_MS       10          /* 日志记录周期（ms）*/
#define LOG_MAX_ENTRIES     1400        /* 最大日志条数（扩展诊断字段后降低容量以控RAM） */

typedef struct {
    uint16_t timeMs;        /* 相对时间（ms） */
    uint16_t seq;           /* 行序号（导出完整性校验） */
    uint8_t  testId;        /* 每行都带测试编号，避免分片时丢上下文 */

    int8_t   trackError;    /* 循迹误差 */
    uint8_t  sensorRaw;     /* 传感器原始值 */
    int8_t   pwmL;          /* 左轮PWM */
    int8_t   pwmR;          /* 右轮PWM */
    int8_t   speedL;        /* 左轮速度/10（实际=值*10 mm/s） */
    int8_t   speedR;        /* 右轮速度/10 */
    int16_t  tgtL;          /* 目标左轮速度 mm/s */
    int16_t  tgtR;          /* 目标右轮速度 mm/s */

    uint8_t  flags;         /* bit0:edgeTurn, bit1:edgeL, bit2:edgeR,
                               bit3:recovery, bit4:protection,
                               bit5:postEtLock, bit6:kpBoost, bit7:stabilize */
    uint8_t  edgeTurnSlow;  /* 边缘退出缓行剩余帧 */
    uint8_t  satInfo;       /* bit0:turnSat, bit1:pwmLsat, bit2:pwmRsat */
    int8_t   kpScale10;     /* 动态Kp倍率*10（10=1.0x） */
    int8_t   dtMs;          /* 控制周期ms */
    int8_t   errSpdL10;     /* (tgtL-speedL)/10 */
    int8_t   errSpdR10;     /* (tgtR-speedR)/10 */

    int16_t  turnOutput;    /* 转向输出（合成后） */
    int16_t  dynBaseSpd;    /* 动态基础速度 mm/s */
} LogEntry_t;

#if LOG_ENABLE
static LogEntry_t s_logBuffer[LOG_MAX_ENTRIES];
static uint16_t s_logIndex = 0;
static uint16_t s_logSeq = 0;
static uint32_t s_logStartMs = 0;
static uint32_t s_lastLogMs = 0;
static uint8_t s_logRunning = 0;
static uint8_t s_testNum = 0;          /* 当前测试编号（1,2,3...） */

/* 添加一条日志 */
static void Log_Add(uint32_t nowMs, int8_t err, uint8_t raw,
                    float pwmL, float pwmR, float spdL, float spdR,
                    float tgtL, float tgtR,
                    uint8_t flags, uint8_t etSlow,
                    uint8_t satInfo, float kpScale, float dtMs,
                    float errSpdL, float errSpdR,
                    float turnOut, float dynBase)
{
    if (s_logIndex >= LOG_MAX_ENTRIES) return;

    LogEntry_t *entry = &s_logBuffer[s_logIndex];
    float kp10 = kpScale * 10.0f;
    float eL10 = errSpdL / 10.0f;
    float eR10 = errSpdR / 10.0f;
    entry->timeMs = (uint16_t)(nowMs - s_logStartMs);
    entry->seq = s_logSeq++;
    entry->testId = s_testNum;
    entry->trackError = err;
    entry->sensorRaw = raw;
    entry->pwmL = (int8_t)pwmL;
    entry->pwmR = (int8_t)pwmR;
    entry->speedL = (int8_t)(spdL / 10.0f);
    entry->speedR = (int8_t)(spdR / 10.0f);
    entry->tgtL = (int16_t)tgtL;
    entry->tgtR = (int16_t)tgtR;
    entry->flags = flags;
    entry->edgeTurnSlow = etSlow;
    entry->satInfo = satInfo;
    if (kp10 > 127.0f) kp10 = 127.0f;
    if (kp10 < -128.0f) kp10 = -128.0f;
    entry->kpScale10 = (int8_t)kp10;
    if (dtMs > 127.0f) dtMs = 127.0f;
    if (dtMs < -128.0f) dtMs = -128.0f;
    entry->dtMs = (int8_t)dtMs;
    if (eL10 > 127.0f) eL10 = 127.0f;
    if (eL10 < -128.0f) eL10 = -128.0f;
    if (eR10 > 127.0f) eR10 = 127.0f;
    if (eR10 < -128.0f) eR10 = -128.0f;
    entry->errSpdL10 = (int8_t)eL10;
    entry->errSpdR10 = (int8_t)eR10;
    entry->turnOutput = (int16_t)turnOut;
    entry->dynBaseSpd = (int16_t)dynBase;
    s_logIndex++;
}

/* 插入测试分隔标记：trackError=127 表示这是一个标记条目，testNum存在sensorRaw中 */
static void Log_InsertTestMarker(uint32_t nowMs)
{
    if (s_logIndex >= LOG_MAX_ENTRIES) return;
    LogEntry_t *entry = &s_logBuffer[s_logIndex];
    entry->timeMs = 0;
    entry->seq = s_logSeq++;
    entry->testId = s_testNum;
    entry->trackError = 127;        /* 特殊标记值，正常范围不会出现 */
    entry->sensorRaw = s_testNum;   /* 测试编号 */
    entry->pwmL = 0;
    entry->pwmR = 0;
    entry->speedL = 0;
    entry->speedR = 0;
    entry->tgtL = 0;
    entry->tgtR = 0;
    entry->flags = 0xFF;            /* 全1标志，便于识别 */
    entry->edgeTurnSlow = 0;
    entry->satInfo = 0;
    entry->kpScale10 = 10;
    entry->dtMs = 0;
    entry->errSpdL10 = 0;
    entry->errSpdR10 = 0;
    entry->turnOutput = 0;
    entry->dynBaseSpd = 0;
    s_logIndex++;
}

/* 开始记录（追加模式：不清空缓冲区，插入测试标记） */
static void Log_Start(uint32_t nowMs)
{
    s_testNum++;
    Log_InsertTestMarker(nowMs);
    s_logStartMs = nowMs;
    s_lastLogMs = nowMs;
    s_logRunning = 1;
}

/* 停止记录 */
static void Log_Stop(void)
{
    s_logRunning = 0;
}

/* 清空日志缓冲区 */
static void Log_Clear(void)
{
    s_logIndex = 0;
    s_logSeq = 0;
    s_testNum = 0;
    s_logRunning = 0;
}

/* 统计8位中1的个数（active触发点数） */
static uint8_t PopCount8(uint8_t x)
{
    uint8_t c = 0;
    while (x) {
        c += (uint8_t)(x & 1U);
        x >>= 1;
    }
    return c;
}

/* 导出日志到串口（CSV格式，改进版：每行延时防截断，测试标记分隔）*/
static void Log_Export(void)
{
    char buf[180];
    uint16_t i;

    /* 使用 UART4 (PC10/PC11) - DAPLink 连接到这里 */
    UART4_init(115200);
    Delay_ms(50);

    /* 导出标记 */
    sprintf(buf, "#LOG_BEGIN,%u,%u\r\n", s_logIndex, s_testNum);
    UART4_Send_String(buf);
    Delay_ms(5);

    /* 打印表头 */
    UART4_Send_String("seq,test_id,time_ms,error,sensor,active,cnt,pwmL,pwmR,speedL,speedR,tgtL,tgtR,flags,etSlow,sat,kpScale10,dtMs,errSpdL10,errSpdR10,turnOut,dynBase\r\n");
    Delay_ms(5);

    /* 打印数据 */
    for (i = 0; i < s_logIndex; i++) {
        LogEntry_t *e = &s_logBuffer[i];

        /* 测试分隔标记：trackError==127 && flags==0xFF */
        if (e->trackError == 127 && e->flags == 0xFF) {
            sprintf(buf, "#TEST,%u\r\n", e->sensorRaw);
            UART4_Send_String(buf);
            Delay_ms(3);
            continue;
        }

        {
            uint8_t active = (uint8_t)(~e->sensorRaw);
            uint8_t cnt = PopCount8(active);
            sprintf(buf, "%u,%u,%u,%d,0x%02X,0x%02X,%u,%d,%d,%d,%d,%d,%d,0x%02X,%u,0x%02X,%d,%d,%d,%d,%d,%d\r\n",
                    (unsigned)e->seq,
                    (unsigned)e->testId,
                    (unsigned)e->timeMs,
                    e->trackError,
                    e->sensorRaw,
                    active,
                    cnt,
                    e->pwmL,
                    e->pwmR,
                    e->speedL * 10,
                    e->speedR * 10,
                    (int)e->tgtL,
                    (int)e->tgtR,
                    e->flags,
                    e->edgeTurnSlow,
                    e->satInfo,
                    e->kpScale10,
                    e->dtMs,
                    e->errSpdL10,
                    e->errSpdR10,
                    (int)e->turnOutput,
                    (int)e->dynBaseSpd);
            UART4_Send_String(buf);
        }

        /* 每行加2ms延时，降低串口拥塞导致的拼行风险 */
        Delay_ms(2);
    }

    sprintf(buf, "# Total: %u entries, %u tests\r\n", s_logIndex, s_testNum);
    UART4_Send_String(buf);
    UART4_Send_String("#LOG_END\r\n");
}
#endif /* LOG_ENABLE */

/* 传感器权重 - 非线性分布（1,3,9,27模式）
 * 边缘传感器权重更大，让弯道反应更激烈
 * 线在左边 → 误差为负 → 左轮减速
 * 线在右边 → 误差为正 → 右轮减速
 */
static const int8_t SENSOR_WEIGHTS_V5[8] = {
    -12,    /* X1: 最左边 - 超大权重！ */
    -8,    /* X2 */
    -3,    /* X3 */
    -1,     /* X4: 中心左 */
    1,      /* X5: 中心右 */
    3,     /* X6 */
    8,     /* X7 */
    12      /* X8: 最右边 - 超大权重！ */
};

/*==================== 速度PI结构体 ====================*/
typedef struct {
    float Kp, Ki;
    float integral;
    float integralMax;
    float outMin, outMax;
} PI_t;

static PI_t s_piL, s_piR;           /* 左右轮速度PI */
static float s_pwmL = 0, s_pwmR = 0; /* 当前PWM */
static float s_speedL_f = 0, s_speedR_f = 0;  /* 滤波后速度 */
static uint8_t s_speedFiltInit = 0;  /* 速度滤波初始化标志 */
static int8_t s_lastDir = 1;         /* 上次方向：1=右，-1=左 */
static int8_t s_lastErr = 0;         /* 上次误差（显示用） */

/*==================== PI函数 ====================*/
static void PI_Init(PI_t *pi, float kp, float ki, float intMax, float outMin, float outMax)
{
    pi->Kp = kp;
    pi->Ki = ki;
    pi->integral = 0;
    pi->integralMax = intMax;
    pi->outMin = outMin;
    pi->outMax = outMax;
}

static float PI_Compute(PI_t *pi, float target, float actual)
{
    float err = target - actual;
    float out;
    
#if DEBUG_NO_KI
    /* 调试模式：关闭Ki，只用Kp */
    out = pi->Kp * err;
#else
    /* 积分累加 - 注意：这里没有乘以dt，所以Ki要设得很小 */
    /* 控制周期是2ms，每秒累加500次 */
    pi->integral += err;
    
    /* 积分限幅 - 防止积分饱和！ */
    if (pi->integral > pi->integralMax) pi->integral = pi->integralMax;
    if (pi->integral < -pi->integralMax) pi->integral = -pi->integralMax;
    
    /* PI输出 */
    out = pi->Kp * err + pi->Ki * pi->integral;
#endif
    
    /* 死区补偿已移除 - MG310电机动力强，不需要 */
    
    /* 输出限幅 */
    if (out > pi->outMax) out = pi->outMax;
    if (out < pi->outMin) out = pi->outMin;
    
    return out;
}

static void PI_Reset(PI_t *pi)
{
    pi->integral = 0;
}

int main(void)
{
    uint8_t ir_raw = 0xFF;
    uint32_t lastControlMs = 0;
    uint32_t lastEncoderMs = 0;
    uint32_t lastOledMs = 0;
    uint32_t loopCount = 0;
    char buf[24];
    KeyEvent_t keyEvent;
    Key2Event_t key2Event;
    uint8_t running = 0;
    
    float speedL = 0, speedR = 0;
    float targetL = 0, targetR = 0;
    
    /* 初始化 */
    SysTick_Init();
    Key_Init();
    Key2_Init();
    
    /* GPIO 循迹初始化 */
    IR_GPIO_Init();
    
    /* OLED 初始化 */
    OLED_Init();

#if LOG_ENABLE
    /* 启动标记：用于验证PC10日志串口是否有输出 */
    UART4_init(115200);
    Delay_ms(10);
    UART4_Send_String("#BOOT\r\n");
#endif
    Delay_ms(100);
    
    /* 电机和编码器 */
    Set_Motor(0);
    Encoder_Init();
    Encoder_Reset();
    
    /* 速度 PI 初始化 */
    PI_Init(&s_piL, SPEED_KP, SPEED_KI, INTEGRAL_MAX, PWM_MIN, PWM_MAX);
    PI_Init(&s_piR, SPEED_KP, SPEED_KI, INTEGRAL_MAX, PWM_MIN, PWM_MAX);
    
    OLED_Clear();
#if SPEED_LOOP_TUNING
    OLED_ShowString(1, 1, "SpeedLoop Test");
    OLED_ShowString(2, 1, "Kp Ki Tuning");
    OLED_ShowString(3, 1, "PC5=Start/Stop");
    OLED_ShowString(4, 1, "PC4=Emergency");
#else
    OLED_ShowString(1, 1, "V6 Clean");
    OLED_ShowString(2, 1, "SpdPI+TrackPD");
    OLED_ShowString(3, 1, "C5=Run/Stop");
    OLED_ShowString(4, 1, "C4=Stop+Export");
#endif
    
    lastControlMs = SysTick_GetMs();
    lastEncoderMs = lastControlMs;
    lastOledMs = lastControlMs;
    
    while (1)
    {
        uint32_t nowMs = SysTick_GetMs();
        
        loopCount++;
        
        /* 按键扫描 */
        keyEvent = Key_Scan();
        Key2_Scan();
        key2Event = Key2_GetEvent();
        if (key2Event != KEY2_EVENT_NONE) Key2_ClearEvent();
        
        /* PC4 = 紧急停止 + 导出日志 */
        if (key2Event == KEY2_EVENT_SHORT || key2Event == KEY2_EVENT_LONG) {
            Motor_SetSpeedBoth(0, 0);
            running = 0;
            s_pwmL = 0; s_pwmR = 0;
            PI_Reset(&s_piL);
            PI_Reset(&s_piR);
            s_speedFiltInit = 0;
#if LOG_ENABLE
            Log_Stop();
            OLED_Clear();
            {
                char buf[32];
                sprintf(buf, "T%u %u entries", s_testNum, s_logIndex);
                OLED_ShowString(1, 1, buf);
            }
            OLED_ShowString(2, 1, "Exporting...");
            Log_Export();
            OLED_ShowString(3, 1, "Done!");
            OLED_ShowString(4, 1, "C5=NewTest");
            /* 不清空缓冲区：下次C5会追加，下次C4会导出全部 */
#endif
        }
        
        /* PC5 = 开始/停止 */
        if (keyEvent == KEY_EVENT_SHORT) {
            running = running ? 0 : 1;
            if (!running) {
                Motor_SetSpeedBoth(0, 0);
                s_pwmL = 0; s_pwmR = 0;
                PI_Reset(&s_piL);
                PI_Reset(&s_piR);
                s_speedFiltInit = 0;
#if LOG_ENABLE
                Log_Stop();
#endif
            } else {
                /* 启动：预填积分，加速启动 */
                s_piL.integral = 150.0f;
                s_piR.integral = 150.0f;
                s_speedFiltInit = 0;
#if LOG_ENABLE
                Log_Start(nowMs);  /* 追加模式：不清空，插入测试标记 */
#endif
            }
            s_lastErr = 0;
        }
        
        /* 编码器更新 + 控制周期 2ms（高频控制，快速响应）
         * 500mm/s 下每2ms移动1mm，必须快速反应
         */
        if ((nowMs - lastControlMs) >= 2)
        {
            float dt = (float)(nowMs - lastControlMs) / 1000.0f;
            if (dt < 0.002f) dt = 0.002f;
            
            /* 第一时间读取传感器！不要等！ */
            ir_raw = IR_GPIO_Read();
            
            /* 更新编码器 */
            Encoder_Update(nowMs - lastControlMs);
            speedL = Encoder_GetSpeedMMS(ENCODER_LEFT);
            speedR = Encoder_GetSpeedMMS(ENCODER_RIGHT);

            if (!s_speedFiltInit) {
                s_speedL_f = speedL;
                s_speedR_f = speedR;
                s_speedFiltInit = 1;
            } else {
                s_speedL_f += SPEED_FILT_ALPHA * (speedL - s_speedL_f);
                s_speedR_f += SPEED_FILT_ALPHA * (speedR - s_speedR_f);
            }

            lastControlMs = nowMs;
            
            if (!running) {
                Motor_SetSpeedBoth(0, 0);
                continue;
            }
            
#if SPEED_LOOP_TUNING
            /*==================== 速度环调试模式 ====================*/
            /* 屏蔽循迹，只做速度闭环
             * 目标：两轮以相同速度直线行驶
             * 调试：用手挡轮子，观察恢复能力
             */
            {
                float spdL_filt, spdR_filt;
                float pwmL, pwmR;
                
#if DEBUG_OPEN_LOOP
                /* 开环测试：直接给固定PWM，不用PI控制 */
                /* 用于排查是电机驱动问题还是PI控制问题 */
                pwmL = DEBUG_FIXED_PWM;
                pwmR = DEBUG_FIXED_PWM;
                targetL = 0;  /* 开环模式下目标速度无意义 */
                targetR = 0;
#else
                /* 固定目标速度 */
                targetL = TEST_TARGET_SPEED;
                targetR = TEST_TARGET_SPEED;
                
                /* 速度PI闭环 */
                spdL_filt = s_speedFiltInit ? s_speedL_f : speedL;
                spdR_filt = s_speedFiltInit ? s_speedR_f : speedR;
                
                pwmL = PI_Compute(&s_piL, targetL, spdL_filt);
                pwmR = PI_Compute(&s_piR, targetR, spdR_filt);
#endif
                
                /* 设置电机 */
                Motor_SetSpeedBoth((int16_t)pwmL, (int16_t)pwmR);
                
                /* 更新显示变量 */
                s_pwmL = pwmL;
                s_pwmR = pwmR;
            }
#else
            /*==================== 串级控制：循迹PD(外环) + 速度PI(内环) ====================*/
            /* V7 极简版：砍掉所有复杂状态机，回归"看到线就跟"的核心
             * 原则：
             * 1. 传感器有线 → PD 立刻跟，不要任何延迟/锁定
             * 2. 丢线 → 按上次方向低速转，一旦看到线立刻恢复PD
             * 3. 弯道 → 靠非线性Kp + 自动减速，不需要ET模式
             */
            {
                uint8_t active = (uint8_t)(~ir_raw);  /* 1=触发 */
                int16_t sum = 0;
                uint8_t count = 0;
                uint8_t i;
                float trackError = 0;
                float turnOutput;
                float spdL_filt, spdR_filt;
                float pwmL, pwmR;
                float logTurnOutput = 0.0f;
                float logDynBaseSpd = 0.0f;
                uint8_t logFlags = 0;
                uint8_t logSatInfo = 0;
                float logKpScale = 1.0f;
                float errSpdL = 0.0f;
                float errSpdR = 0.0f;
                static float lastTrackError = 0;
                static uint8_t lostLineFrames = 0;  /* 连续丢线帧数 */

                /*---------- 第一步：计算循迹误差 ----------*/
                for (i = 0; i < 8; i++) {
                    if (active & (1U << (7 - i))) {
                        sum += SENSOR_WEIGHTS_V5[i];
                        count++;
                    }
                }

                if (count == 0) {
                    /* 丢线：保持上次方向，给最大误差 */
                    trackError = (s_lastDir > 0) ? 25.0f : -25.0f;
                    if (lostLineFrames < 255) lostLineFrames++;
                    logFlags |= 0x08U; /* bit3: 丢线中 */
                } else {
                    /* 有线！立刻跟踪，不管之前是什么状态 */
                    if (count == 8) {
                        /* 全触发（十字路口）：直行 */
                        trackError = 0;
                    } else {
                        trackError = (float)sum / (float)count;
                    }

                    /* 丢线刚恢复：重置PI积分器防暴冲
                     * 同时把 lastTrackError 对齐到当前误差，避免 D 项因“25→小误差”突变
                     * 造成反向尖峰（T5 出现 +12/-12 来回翻的一种常见诱因）。
                     */
                    if (lostLineFrames > 5) {
                        PI_Reset(&s_piL);
                        PI_Reset(&s_piR);
                        lastTrackError = trackError;
                        logFlags |= 0x80U; /* bit7: 刚从丢线恢复 */
                    }
                    lostLineFrames = 0;

                    /* 更新方向记忆（只在有线时更新） */
                    if (trackError >= 1) s_lastDir = 1;
                    else if (trackError <= -1) s_lastDir = -1;
                }

                /*---------- 第二步：PD计算转向 ----------*/
                {
                    float absErr = ABSF(trackError);
                    float dynamicKp = TRACK_KP;
                    float dynamicBaseSpeed = BASE_SPEED_MMS;
                    float dErr;

                    /* 非线性Kp：误差越大，Kp越大
                     * 基础Kp=8已经很强，非线性倍率要保守
                     * 小误差(0~3)：1.0x — 直线稳定
                     * 中误差(3~8)：1.05~1.8x — 浅弯需要更强响应
                     * 大误差(8~25)：1.8~2.8x — 急弯猛转
                     * T5数据: err=-3时turnOut=-24差速48太小，几乎直行
                     * T2数据: err=+7时turnOut=89差速178不够跟急弯
                     */
                    if (absErr > 8.0f) {
                        dynamicKp = TRACK_KP * (1.8f + (absErr - 8.0f) * 0.06f);
                        if (dynamicKp > TRACK_KP * 2.8f) dynamicKp = TRACK_KP * 2.8f;
                    } else if (absErr > 3.0f) {
                        dynamicKp = TRACK_KP * (1.05f + (absErr - 3.0f) * 0.15f);
                    }
                    logKpScale = dynamicKp / TRACK_KP;

                    /* 弯道自动减速：误差越大速度越低
                     * 弯道3弯峰err=-7~-10时速度280~256仍太快，跟不住弯峰
                     * 加大减速斜率：0.025→0.035，大误差区降速更猛
                     * absErr=0: 100% 速度 (320)
                     * absErr=5: ~89% (286)
                     * absErr=7: ~82% (264)  ← 之前是280，现在更低
                     * absErr=10: ~72% (230)  ← 之前是256
                     * absErr=15: ~54% (174)
                     * absErr=25: ~20% → clamp 25% (80)
                     */
                    if (absErr > 2.0f) {
                        float speedFactor = 1.0f - (absErr - 2.0f) * 0.035f;
                        if (speedFactor < 0.25f) speedFactor = 0.25f;
                        dynamicBaseSpeed *= speedFactor;
                    }

                    /* 丢线时额外降速 + 限制转向 */
                    if (lostLineFrames > 0) {
                        /* 丢线搜索策略：温柔搜索，避免冲过线 */
                        dynamicBaseSpeed = 160.0f;  /* 固定搜索速度 */
                        if (lostLineFrames > 50) {
                            dynamicBaseSpeed = 130.0f; /* 长时间丢线稍降 */
                        }
                        logFlags |= 0x10U; /* bit4: 丢线降速 */
                    }

                    /* PD计算 */
                    dErr = (trackError - lastTrackError) / dt;
                    /* 限制微分项，防止噪声放大 */
                    /* 传感器误差量化步长=1, dt=0.002s, 正常dErr=500
                     * 限幅要小！否则D项尖峰远超P项导致摇摆
                     * clamp=10: 最大D贡献=0.8*10=8, 与P项(8*1=8)匹配
                     */
                    if (dErr > 10.0f) dErr = 10.0f;
                    if (dErr < -10.0f) dErr = -10.0f;

                    turnOutput = dynamicKp * trackError + TRACK_KD * dErr;
                    lastTrackError = trackError;

                    /* 转向限幅：丢线时需要足够转向力搜索 */
                    {
                        float turnLimit = 300.0f;
                        if (lostLineFrames > 0) {
                            turnLimit = 220.0f; /* 丢线搜索: 180太小转不回来，提高到220 */
                        }
                        if (turnOutput > turnLimit) {
                            turnOutput = turnLimit;
                            logSatInfo |= 0x01U;
                        }
                        if (turnOutput < -turnLimit) {
                            turnOutput = -turnLimit;
                            logSatInfo |= 0x01U;
                        }
                    }

                    /* 保存日志数据 */
                    logTurnOutput = turnOutput;
                    logDynBaseSpd = dynamicBaseSpeed;

                    /*---------- 第三步：计算目标速度 ----------*/
                    targetL = dynamicBaseSpeed + turnOutput;
                    targetR = dynamicBaseSpeed - turnOutput;
                }

                /* 目标速度限幅
                 * 关键修复：弯道/丢线时不要强行把内侧轮抬到 MIN_SPEED_MMS。
                 * 否则会“推着走”，转弯半径变大（T2/T5 的典型症状）。
                 */
                {
                    float minSpeed = MIN_SPEED_MMS;

                    /* 丢线搜索/大误差：允许内侧轮降到 0，提高转向半径能力 */
                    if (lostLineFrames > 0 || ABSF(trackError) > 8.0f) {
                        minSpeed = 0.0f;
                    }

                    /* 目标速度不允许为负（本工程不支持反转），负值按 0 处理 */
                    if (targetL < 0.0f) targetL = 0.0f;
                    if (targetR < 0.0f) targetR = 0.0f;

                    if (targetL < minSpeed) targetL = minSpeed;
                    if (targetR < minSpeed) targetR = minSpeed;
                    if (targetL > MAX_SPEED_MMS) targetL = MAX_SPEED_MMS;
                    if (targetR > MAX_SPEED_MMS) targetR = MAX_SPEED_MMS;
                }

                /*---------- 第四步：速度PI闭环 ----------*/
                spdL_filt = s_speedFiltInit ? s_speedL_f : speedL;
                spdR_filt = s_speedFiltInit ? s_speedR_f : speedR;

                errSpdL = targetL - spdL_filt;
                errSpdR = targetR - spdR_filt;

                pwmL = PI_Compute(&s_piL, targetL, spdL_filt);
                pwmR = PI_Compute(&s_piR, targetR, spdR_filt);

                if (pwmL >= (PWM_MAX - 0.5f) || pwmL <= (PWM_MIN + 0.5f)) logSatInfo |= 0x02U;
                if (pwmR >= (PWM_MAX - 0.5f) || pwmR <= (PWM_MIN + 0.5f)) logSatInfo |= 0x04U;

                /* 设置电机 */
                Motor_SetSpeedBoth((int16_t)pwmL, (int16_t)pwmR);

                /* 更新显示变量 */
                s_pwmL = pwmL;
                s_pwmR = pwmR;
                s_lastErr = (int8_t)trackError;

#if LOG_ENABLE
                /* 日志记录 */
                if (s_logRunning && (nowMs - s_lastLogMs) >= LOG_PERIOD_MS) {
                    s_lastLogMs = nowMs;
                    Log_Add(nowMs, (int8_t)trackError, ir_raw, pwmL, pwmR, s_speedL_f, s_speedR_f, targetL, targetR,
                            logFlags, (uint8_t)lostLineFrames, logSatInfo, logKpScale, dt * 1000.0f,
                            errSpdL, errSpdR, logTurnOutput, logDynBaseSpd);
                }
#endif
            }
#endif  /* SPEED_LOOP_TUNING */
        }
        
        /* OLED 刷新 500ms（降低频率，减少对控制的干扰）*/
        if ((nowMs - lastOledMs) >= 1200)
        {
            lastOledMs = nowMs;
#if SPEED_LOOP_TUNING
            /* 速度环调试显示 */
            sprintf(buf, "Tgt:%3.0f mm/s", TEST_TARGET_SPEED);
            OLED_ShowString(1, 1, buf);
            sprintf(buf, "L:%3.0f R:%3.0f", s_speedL_f, s_speedR_f);
            OLED_ShowString(2, 1, buf);
            sprintf(buf, "PWM L%2.0f R%2.0f", s_pwmL, s_pwmR);
            OLED_ShowString(3, 1, buf);
            sprintf(buf, "Kp%.2f Ki%.3f", SPEED_KP, SPEED_KI);
            OLED_ShowString(4, 1, buf);
#else
            sprintf(buf, "E:%+3d Spd%3.0f", s_lastErr, (s_speedL_f + s_speedR_f) / 2);
            OLED_ShowString(1, 1, buf);
            sprintf(buf, "Tgt%3.0f/%3.0f", targetL, targetR);
            OLED_ShowString(2, 1, buf);
            sprintf(buf, "PWM%2.0f/%2.0f", s_pwmL, s_pwmR);
            OLED_ShowString(3, 1, buf);
            {
                uint8_t active = (uint8_t)(~ir_raw);
                char sensorStr[9];
                int i;
                for (i = 0; i < 8; i++) {
                    sensorStr[i] = (active & (1 << (7-i))) ? '*' : '-';
                }
                sensorStr[8] = '\0';
                OLED_ShowString(4, 1, sensorStr);
            }
#endif  /* SPEED_LOOP_TUNING */
        }
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
#define CONTROL_PERIOD_MS   10      /* 控制周期从 20ms 改为 10ms */
#define OLED_REFRESH_MS     500     /* OLED 刷新降频，减少 I2C 占用 */
#define GYRO_REFRESH_MS     50      /* 陀螺仪降频 */

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
