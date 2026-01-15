#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "OLED.h"
#include "OELD_Data.h"
#include "FMQ.h"
#include "Serial.h"
#include "RMDZ.h"	

u16 RMDZ_Value;//adc值
float VCC_Value;//电压值

int main(void)
{
	 Adc_Init();
	 OLED_Init();
	 mfq_Init();
	 Serial_Init();
	

	OLED_ShowChinese(40, 50, "热敏电阻：");

	OLED_Update();

	while(1)
	{
		
		RMDZ_Value=Get_Adc_Average(ADC_Channel_1,10)*100/4095;//模拟热敏电阻的值 0-100  热敏电阻特性 反比例关系
		
		OLED_ShowNum(74,32,RMDZ_Value,3,OLED_8X16);
		OLED_Update();
		
		
		fmq(RMDZ_Value);

		 printf("热敏电阻= %d ",RMDZ_Value);
		 Serial_SendString("\r\n");

	}
}
