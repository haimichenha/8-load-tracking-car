#include "app_motor.h"
#include "bsp_encoder.h"
#include "bsp_systick.h"

#define MOTOR_SPEED_LOOP_ENABLE 1
#define MOTOR_SPEED_MAX_MMS 180.0f
#define MOTOR_PID_KP 0.04f
#define MOTOR_PID_KI 0.008f
#define MOTOR_PID_KD 0.0f
#define MOTOR_PID_INTEGRAL_LIMIT 160.0f
#define MOTOR_PID_OUTPUT_LIMIT 25.0f
#define MOTOR_PID_MAX_DT_MS 200U

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

static int16_t Motor_CalcSpeedOutput(float targetMms, float actualMms, MotorPid_t *pid, float dt)
{
    float basePwm;
    float error;
    float correction;
    float pwm;

    if (MOTOR_SPEED_MAX_MMS <= 0.1f)
    {
        return 0;
    }

    basePwm = targetMms / MOTOR_SPEED_MAX_MMS * 100.0f;
    basePwm = Motor_ClampFloat(basePwm, -100.0f, 100.0f);

    error = targetMms - actualMms;
    correction = Motor_PidUpdate(pid, error, dt);
    pwm = basePwm + correction;
    pwm = Motor_ClampFloat(pwm, -100.0f, 100.0f);

    if (pwm >= 0.0f)
    {
        return (int16_t)(pwm + 0.5f);
    }
    return (int16_t)(pwm - 0.5f);
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
        pwmLeft = Motor_CalcSpeedOutput(targetLeftMms, Encoder_GetSpeedMMS(ENCODER_LEFT), &s_pidLeft, dt);
        pwmRight = Motor_CalcSpeedOutput(targetRightMms, Encoder_GetSpeedMMS(ENCODER_RIGHT), &s_pidRight, dt);

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

