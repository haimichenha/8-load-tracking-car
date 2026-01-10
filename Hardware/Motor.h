#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void Motor_Init(void);
void Motor_SetSpeed(int16_t speedA, int16_t speedB);

#endif
