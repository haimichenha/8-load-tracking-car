#include "stm32f10x.h"                  // Device header
#include "CarRun.h"
#include "MotorRun.h"
#include "delay.h"
#include "inv_mpu.h"
#include "OLED.h"

float P,R,Y;
//MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw);

void changeCar(float Yaw1,float Yaw)
{
	
	uint32_t Timeout;
	Timeout = 3000;									//给定超时计数时间
	while(Yaw1!=Yaw)
	{
			if(Y>Yaw1)
		{
			rightChange(60,25);
			delay_ms(100);
		}
		else
		{
			leftChange(25,60);
			delay_ms(100);
		}
		MPU6050_DMP_Get_Data(&P,&R,&Y);
		OLED_ShowString(2,1,"Pitch:");
		OLED_ShowString(3,1,"Roll:");
		OLED_ShowString(4,1,"Yaw:");
		OLED_ShowSignedNum(2,8,P,3);
		OLED_ShowSignedNum(3,8,R,3);
		OLED_ShowSignedNum(4,8,Y,3);
		Yaw=Y;
		Timeout --;										//等待时，计数值自减
		if (Timeout == 0)								//自减到0后，等待超时
		{
			/*超时的错误处理代码，可以添加到此处*/
			break;										//跳出等待，不等了
		}
	}
	stoprun();
	straightrun(40);
	delay_ms(100);
}

void Car_Control(void)
{
	//路口1
	straightrun(25);
	delay_ms(1200);
	stoprun();
//	Delay_ms(10);
	//
	rightrun(80,0);
	delay_ms(800);
	MPU6050_DMP_Get_Data(&P,&R,&Y);
	changeCar(-81,Y);
//	MPU6050_DMP_Get_Data(&P,&R,&Y);
//	changeCar(-81,Y);
	stoprun();
//	Delay_ms();
	
	
//	straightrun(40);
//	delay_ms(5300);
//	stoprun();
	
	backrun(25);
	delay_ms(1200);
	stoprun();
	
	rightbackrun(80,0);
	delay_ms(1200);
	stoprun();
	
	backrun(40);
	delay_ms(1000);
	stoprun();
	
	straightrun(40);
	delay_ms(4300);
	stoprun();
	
	leftrun(30,72);
	delay_ms(1300);
	stoprun();
	
	backrun(40);
	delay_ms(2000);
	stoprun();
	
	straightrun(40);
	delay_ms(3000);
	stoprun();
	
	rightrun(70,30);
	delay_ms(1300);
	stoprun();
	
	straightrun(40);
	delay_ms(800);
	stoprun();
	
	leftrun(30,72);
	delay_ms(1200);
	stoprun();
	
	leftrun(30,72);
	delay_ms(1100);
	stoprun();

		straightrun(40);
		delay_ms(1700);
		stoprun();
		
		rightrun(72,30);
		delay_ms(1100);
		stoprun();
		
		straightrun(25);
		delay_ms(1100);
		stoprun();
		
		leftrun(30,72);
		delay_ms(1100);
		stoprun();
		
		straightrun(40);
		delay_ms(3000);
		stoprun();
}
