#include "stm32f10x.h"                  // Device header
#include "pwm.h"
#include "MotorRun.h"

//#define IN1 PAout(6)  // 左后    PA2
//#define IN2 PAout(7)  // 左后
//#define IN3 PAout(11) // 右后		 PA1
//#define IN4 PAout(12) // 右后
//#define IN5 PAout(4)  // 左前		 PA3
//#define IN6 PAout(5)  // 左前
//#define IN7 PBout(12) // 右前		 PA0
//#define IN8 PBout(14) // 右前

void MotorRun_Init(void)
{
	PWM_Init();
}

void Motor_GPIO_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_4 | GPIO_Pin_5;;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void straightrun(uint16_t RunSpeed)//直走
{  
	TIM_SetCompare1(TIM2, RunSpeed);  // 右前
	TIM_SetCompare2(TIM2, RunSpeed);  // 右后
	TIM_SetCompare3(TIM2, RunSpeed);  // 左后
	TIM_SetCompare4(TIM2, RunSpeed);  // 左前
	IN1 = 1; // 左后
	IN2 = 0; 
	IN3 = 1; // 右后	
	IN4 = 0; 
	IN5 = 1; // 左前
	IN6 = 0; 
	IN7 = 1; // 右前
	IN8 = 0; 
}

void backrun(uint16_t BackSpeed)//后退
{
	TIM_SetCompare1(TIM2, BackSpeed);  // 右前
	TIM_SetCompare2(TIM2, BackSpeed);  // 右后
	TIM_SetCompare3(TIM2, BackSpeed);  // 左后
	TIM_SetCompare4(TIM2, BackSpeed);  // 左前
	IN1 = 0; // 左后
	IN2 = 1; 
	IN3 = 0; // 右后	
	IN4 = 1; 
	IN5 = 0; // 左前
	IN6 = 1; 
	IN7 = 0; // 右前
	IN8 = 1;
}

void rightrun( uint16_t LeftSpeed,uint16_t RightSpeed)//右转
{
	TIM_SetCompare1(TIM2, RightSpeed);// 右前
	TIM_SetCompare2(TIM2, RightSpeed);// 右后
	TIM_SetCompare3(TIM2, LeftSpeed); // 左后
	TIM_SetCompare4(TIM2, LeftSpeed); // 左前
	IN1 = 1; // 左后
	IN2 = 0; 
	IN3 = 0; // 右后	
	IN4 = 1; 
	IN5 = 1; // 左前
	IN6 = 0; 
	IN7 = 0; // 右前
	IN8 = 1;
}

void rightChange( uint16_t LeftSpeed,uint16_t RightSpeed)//右调整
{
	TIM_SetCompare1(TIM2, RightSpeed);// 右前
	TIM_SetCompare2(TIM2, RightSpeed);// 右后
	TIM_SetCompare3(TIM2, LeftSpeed); // 左后
	TIM_SetCompare4(TIM2, LeftSpeed); // 左前
	IN1 = 1; // 左后
	IN2 = 0; 
	IN3 = 1; // 右后	
	IN4 = 0; 
	IN5 = 1; // 左前
	IN6 = 0; 
	IN7 = 1; // 右前
	IN8 = 0;
}

void leftrun(uint16_t LeftSpeed, uint16_t RightSpeed)//左转
{
	TIM_SetCompare1(TIM2, RightSpeed);// 右前
	TIM_SetCompare2(TIM2, RightSpeed);// 右后
	TIM_SetCompare3(TIM2, LeftSpeed); // 左后
	TIM_SetCompare4(TIM2, LeftSpeed); // 左前
	IN1 = 0; // 左后
	IN2 = 1; 
	IN3 = 1; // 右后	
	IN4 = 0; 
	IN5 = 0; // 左前
	IN6 = 1; 
	IN7 = 1; // 右前
	IN8 = 0;
}

void leftChange(uint16_t LeftSpeed, uint16_t RightSpeed)//左调整
{
	TIM_SetCompare1(TIM2, RightSpeed);// 右前
	TIM_SetCompare2(TIM2, RightSpeed);// 右后
	TIM_SetCompare3(TIM2, LeftSpeed); // 左后
	TIM_SetCompare4(TIM2, LeftSpeed); // 左前
	IN1 = 1; // 左后
	IN2 = 0; 
	IN3 = 1; // 右后	
	IN4 = 0; 
	IN5 = 1; // 左前
	IN6 = 0; 
	IN7 = 1; // 右前
	IN8 = 0;
}

void rightbackrun( uint16_t LeftSpeed,uint16_t RightSpeed)//右转
{
	TIM_SetCompare1(TIM2, RightSpeed);// 右前
	TIM_SetCompare2(TIM2, RightSpeed);// 右后
	TIM_SetCompare3(TIM2, LeftSpeed); // 左后
	TIM_SetCompare4(TIM2, LeftSpeed); // 左前
	IN1 = 0; // 左后
	IN2 = 1; 
	IN3 = 1; // 右后	
	IN4 = 0; 
	IN5 = 0; // 左前
	IN6 = 1; 
	IN7 = 1; // 右前
	IN8 = 0;
}

void circlerun(uint16_t CircleSpeed)//原地转圈
{
	TIM_SetCompare1(TIM2, CircleSpeed); // 右前
	TIM_SetCompare2(TIM2, CircleSpeed); // 右后
	TIM_SetCompare3(TIM2, CircleSpeed); // 左后
	TIM_SetCompare4(TIM2, CircleSpeed); // 左前
	IN1 = 1; // 左后
	IN2 = 0; 
	IN3 = 0; // 右后	
	IN4 = 1; 
	IN5 = 1; // 左前
	IN6 = 0; 
	IN7 = 0; // 右前
	IN8 = 1;
}

void stoprun(void)//停止
{
	TIM_SetCompare1(TIM2 , 0); // 右前
	TIM_SetCompare2(TIM2 , 0); // 右后
	TIM_SetCompare3(TIM2 , 0); // 左后
	TIM_SetCompare4(TIM2 , 0); // 左前
	IN1 = 0; // 左后
	IN2 = 0; 
	IN3 = 0; // 右后	
	IN4 = 0; 
	IN5 = 0; // 左前
	IN6 = 0; 
	IN7 = 0; // 右前
	IN8 = 0;
}
