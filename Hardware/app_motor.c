#include "app_motor.h"

static int16_t Motor_Clamp(int16_t value, int16_t min, int16_t max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

void Set_Motor(int MOTOR_TYPE)
{
    (void)MOTOR_TYPE;
    Motor_Init();
    Motor_Enable(1);
}

void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z)
{
    float robot_APB = Car_APB;
    int16_t speed_fb = V_x;
    int16_t speed_spin = (int16_t)((V_z / 1000.0f) * robot_APB);
    int16_t speed_left;
    int16_t speed_right;

    (void)V_y;

    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motor_SetSpeedBoth(0, 0);
        return;
    }

    speed_left = speed_fb + speed_spin;
    speed_right = speed_fb - speed_spin;

    speed_left = Motor_Clamp(speed_left, -1000, 1000);
    speed_right = Motor_Clamp(speed_right, -1000, 1000);

    speed_left /= 10;
    speed_right /= 10;

    Motor_SetSpeedBoth(speed_left, speed_right);
}

