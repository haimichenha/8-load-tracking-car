#ifndef __MQ2_H
#define __MQ2_H	 
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
void Adc_Init(void);
u16  Get_Adc(u8 ch); 
u16 Get_Adc_Average(u8 ch,u8 times); 
void MQ2_Check(void);

#endif

