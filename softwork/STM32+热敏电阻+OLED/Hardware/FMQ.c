#include "FMQ.h"
#include "stm32f10x.h"   
void mfq_Init(void)	
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

void fmq(float RMDZ_Value)
{
		if(RMDZ_Value < 50)
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_0);//µÍµçÆ½´¥·¢
    }
		else
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_0);
		}
}
