#ifndef _MOTORRUN_H_
#define _MOTORRUN_H_

#include "stm32f10x.h"                  // Device header
#include "sys.h"

//*****电机控制端管脚位定义   输出
#define IN1 PAout(6)  // 左后
#define IN2 PAout(7)  // 左后
#define IN3 PAout(11)  //右后
#define IN4 PAout(12) // 右后
#define IN5 PAout(4)  // 左前
#define IN6 PAout(5)  // 左前
#define IN7 PBout(12) // 右前
#define IN8 PBout(14) // 右前

void MotorRun_Init(void);
void Motor_GPIO_Init(void);
void straightrun(uint16_t RunSpeed);//直走
void backrun(uint16_t BackSpeed);//后退
void rightrun(uint16_t RightSpeed, uint16_t LeftSpeed);//右转
void leftrun(uint16_t LeftSpeed, uint16_t RightSpeed);//左转
void circlerun(uint16_t CircleSpeed);//原地转圈
void stoprun(void);//停止
void rightbackrun( uint16_t LeftSpeed,uint16_t RightSpeed);//右转

void rightChange( uint16_t LeftSpeed,uint16_t RightSpeed);
void leftChange(uint16_t LeftSpeed, uint16_t RightSpeed);


#endif
