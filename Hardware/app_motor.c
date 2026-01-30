#include "app_motor.h"
#include "bsp_encoder.h"
#include "bsp_systick.h"

/*====================================================================================*/
/*                              速度环参数（根据实测调整）                              */
/*====================================================================================*/

#define MOTOR_SPEED_LOOP_ENABLE 1

/* 电机速度参数 - 根据测试结果 */
/* 80% PWM 实测: L=1676, R=1751 mm/s → 反推100%: L≈2095, R≈2189 mm/s */
/* 取保守值 2100 mm/s 作为最大速度 */
#define MOTOR_SPEED_MAX_MMS     2100.0f     /* 最大速度 (mm/s) */
#define MOTOR_DEADZONE_L_PCT    4           /* 左轮死区 (%) */
#define MOTOR_DEADZONE_R_PCT    5           /* 右轮死区 (%) */

/* 速度分档阈值 (mm/s) */
#define MOTOR_SPEED_LOW_THRESH   500.0f     /* 低速区：<500 mm/s，严格限制 */
#define MOTOR_SPEED_MID_THRESH   1000.0f    /* 中速区：500-1000 mm/s，中等限制 */
                                            /* 高速区：>1000 mm/s，快速响应 */

/* 速度环 PID 参数 (单位: error=mm/s, output=%PWM) */
#define MOTOR_PID_KP            0.015f      /* 比例系数 - 减小避免超调 */
#define MOTOR_PID_KI            0.008f      /* 积分系数 - 减小避免震荡 */
#define MOTOR_PID_KD            0.0f
#define MOTOR_PID_INTEGRAL_LIMIT 1000.0f    /* mm/s*s */
#define MOTOR_PID_OUTPUT_LIMIT  25.0f       /* 输出限幅 (%) */
#define MOTOR_PID_MAX_DT_MS     200U        /* 最大时间间隔 */

/* PWM 变化率限制：升速和降速分开控制 */
/* 升速（加速）：慢一点，避免超调 */
#define MOTOR_PWM_SLEW_UP_LOW   1.5f        /* 低速区升速 */
#define MOTOR_PWM_SLEW_UP_MID   2.25f       /* 中速区升速 */
#define MOTOR_PWM_SLEW_UP_HIGH  5.0f        /* 高速区升速 */
/* 降速（减速）：快一点，快速响应 */
#define MOTOR_PWM_SLEW_DN_LOW   2.25f        /* 低速区降速 */
#define MOTOR_PWM_SLEW_DN_MID   8.0f        /* 中速区降速 */
#define MOTOR_PWM_SLEW_DN_HIGH  10.0f        /* 高速区降速 */

/* 速度滤波系数 (0-1, 越小滤波越强) */
#define MOTOR_SPEED_FILTER_ALPHA 0.5f       /* 加强滤波 */

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prevError;
} MotorPid_t;

static MotorPid_t s_pidLeft;
static MotorPid_t s_pidRight;
static uint32_t s_lastUpdateMs = 0;

/* 速度滤波缓存 */
static float s_filteredSpeedL = 0.0f;
static float s_filteredSpeedR = 0.0f;

/* 上一次 PWM 输出（用于变化率限制） */
static float s_lastPwmL = 0.0f;
static float s_lastPwmR = 0.0f;

static int16_t Motor_Clamp(int16_t value, int16_t min, int16_t max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static float Motor_ClampFloat(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static void Motor_PidInit(MotorPid_t *pid)
{
    pid->kp = MOTOR_PID_KP;
    pid->ki = MOTOR_PID_KI;
    pid->kd = MOTOR_PID_KD;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

static void Motor_PidReset(MotorPid_t *pid)
{
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

static float Motor_PidUpdate(MotorPid_t *pid, float error, float dt)
{
    float derivative;
    float output;

    if (dt < 0.0005f) dt = 0.0005f;

    pid->integral += error * dt;
    pid->integral = Motor_ClampFloat(pid->integral, -MOTOR_PID_INTEGRAL_LIMIT, MOTOR_PID_INTEGRAL_LIMIT);

    derivative = (error - pid->prevError) / dt;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->prevError = error;

    return Motor_ClampFloat(output, -MOTOR_PID_OUTPUT_LIMIT, MOTOR_PID_OUTPUT_LIMIT);
}

static int16_t Motor_CalcSpeedOutput(float targetMms, float actualMms, MotorPid_t *pid, float dt, int isLeft)
{
    float basePwm;
    float error;
    float correction;
    float pwm;
    int deadzone;
    float absTarget;
    float *pFiltered;
    float *pLastPwm;
    float filteredSpeed;
    float lastPwm;
    float deltaPwm;

    if (MOTOR_SPEED_MAX_MMS <= 0.1f)
    {
        return 0;
    }

    /* 获取上一次 PWM */
    pLastPwm = isLeft ? &s_lastPwmL : &s_lastPwmR;
    lastPwm = *pLastPwm;

    /* 目标为0时直接返回0 */
    if (targetMms > -0.1f && targetMms < 0.1f)
    {
        Motor_PidReset(pid);
        /* 重置滤波器和上次PWM */
        if (isLeft) { s_filteredSpeedL = 0.0f; s_lastPwmL = 0.0f; }
        else { s_filteredSpeedR = 0.0f; s_lastPwmR = 0.0f; }
        return 0;
    }

    /* 速度滤波：减少编码器噪声 */
    pFiltered = isLeft ? &s_filteredSpeedL : &s_filteredSpeedR;
    *pFiltered = MOTOR_SPEED_FILTER_ALPHA * actualMms + (1.0f - MOTOR_SPEED_FILTER_ALPHA) * (*pFiltered);
    filteredSpeed = *pFiltered;

    /* 计算基础 PWM (前馈) */
    basePwm = targetMms / MOTOR_SPEED_MAX_MMS * 100.0f;
    basePwm = Motor_ClampFloat(basePwm, -100.0f, 100.0f);

    /* PID 修正 - 使用滤波后的速度 */
    error = targetMms - filteredSpeed;
    correction = Motor_PidUpdate(pid, error, dt);
    pwm = basePwm + correction;
    pwm = Motor_ClampFloat(pwm, -100.0f, 100.0f);

    /* PWM 变化率限制：根据目标速度分三档 */
    deltaPwm = pwm - lastPwm;
    absTarget = (targetMms > 0) ? targetMms : -targetMms;
    
    /* 
     * 三档变化率策略 + 升降速非对称：
     * 升速（加速）：慢一点，避免超调
     * 降速（减速）：快一点，快速响应
     */
    {
        float slewRateUp, slewRateDn, slewRate;
        int isSpeedUp;
        
        /* 判断是升速还是降速：PWM 绝对值增大 = 升速 */
        if (pwm > 0) {
            isSpeedUp = (deltaPwm > 0);
        } else {
            isSpeedUp = (deltaPwm < 0);
        }
        
        /* 根据目标速度选择档位 */
        if (absTarget < MOTOR_SPEED_LOW_THRESH)
        {
            slewRateUp = MOTOR_PWM_SLEW_UP_LOW;
            slewRateDn = MOTOR_PWM_SLEW_DN_LOW;
        }
        else if (absTarget < MOTOR_SPEED_MID_THRESH)
        {
            slewRateUp = MOTOR_PWM_SLEW_UP_MID;
            slewRateDn = MOTOR_PWM_SLEW_DN_MID;
        }
        else
        {
            slewRateUp = MOTOR_PWM_SLEW_UP_HIGH;
            slewRateDn = MOTOR_PWM_SLEW_DN_HIGH;
        }
        
        /* 选择升速或降速的变化率 */
        slewRate = isSpeedUp ? slewRateUp : slewRateDn;
        
        /* 应用变化率限制 */
        if (deltaPwm > slewRate) deltaPwm = slewRate;
        else if (deltaPwm < -slewRate) deltaPwm = -slewRate;
        
        pwm = lastPwm + deltaPwm;
    }
    
    *pLastPwm = pwm;  /* 保存本次 PWM */

    /* 死区补偿 */
    deadzone = isLeft ? MOTOR_DEADZONE_L_PCT : MOTOR_DEADZONE_R_PCT;
    
    /* 低速时增加死区余量 */
    if (absTarget < MOTOR_SPEED_LOW_THRESH)
    {
        deadzone += 2;
    }
    
    /* 最小 PWM 保护 */
    if (pwm > 0.5f)
    {
        float minPwm = (float)deadzone + 2.0f;
        if (pwm < minPwm) pwm = minPwm;
        return (int16_t)(pwm + 0.5f);
    }
    else if (pwm < -0.5f)
    {
        float minPwm = (float)deadzone + 2.0f;
        if (pwm > -minPwm) pwm = -minPwm;
        return (int16_t)(pwm - 0.5f);
    }
    return 0;
}

void Set_Motor(int MOTOR_TYPE)
{
    (void)MOTOR_TYPE;
    Motor_Init();
    Motor_Enable(1);
    Encoder_Init();
    Encoder_Reset();
    Motor_PidInit(&s_pidLeft);
    Motor_PidInit(&s_pidRight);
    s_lastUpdateMs = 0;
}

void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z)
{
    float robot_APB = Car_APB;
    float speed_spin = (float)V_z / 1000.0f * robot_APB;
    float speed_left_cmd = (float)V_x + speed_spin;
    float speed_right_cmd = (float)V_x - speed_spin;

    (void)V_y;

    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motor_SetSpeedBoth(0, 0);
        Motor_PidReset(&s_pidLeft);
        Motor_PidReset(&s_pidRight);
        s_lastUpdateMs = SysTick_GetMs();
        return;
    }

    speed_left_cmd = Motor_ClampFloat(speed_left_cmd, -1000.0f, 1000.0f);
    speed_right_cmd = Motor_ClampFloat(speed_right_cmd, -1000.0f, 1000.0f);

#if MOTOR_SPEED_LOOP_ENABLE
    {
        float targetLeftMms = speed_left_cmd / 1000.0f * MOTOR_SPEED_MAX_MMS;
        float targetRightMms = speed_right_cmd / 1000.0f * MOTOR_SPEED_MAX_MMS;
        uint32_t nowMs = SysTick_GetMs();
        uint32_t deltaMs;
        float dt;
        int16_t pwmLeft;
        int16_t pwmRight;

        if (s_lastUpdateMs == 0)
        {
            s_lastUpdateMs = nowMs;
        }

        deltaMs = nowMs - s_lastUpdateMs;
        if (deltaMs == 0)
        {
            deltaMs = 1;
        }
        if (deltaMs > MOTOR_PID_MAX_DT_MS)
        {
            deltaMs = MOTOR_PID_MAX_DT_MS;
        }
        s_lastUpdateMs = nowMs;

        Encoder_Update(deltaMs);

        dt = (float)deltaMs / 1000.0f;
        pwmLeft = Motor_CalcSpeedOutput(targetLeftMms, Encoder_GetSpeedMMS(ENCODER_LEFT), &s_pidLeft, dt, 1);
        pwmRight = Motor_CalcSpeedOutput(targetRightMms, Encoder_GetSpeedMMS(ENCODER_RIGHT), &s_pidRight, dt, 0);

        Motor_SetSpeedBoth(pwmLeft, pwmRight);
    }
#else
    {
        int16_t speed_left = Motor_Clamp((int16_t)speed_left_cmd, -1000, 1000);
        int16_t speed_right = Motor_Clamp((int16_t)speed_right_cmd, -1000, 1000);

        speed_left /= 10;
        speed_right /= 10;

        Motor_SetSpeedBoth(speed_left, speed_right);
    }
#endif
}

