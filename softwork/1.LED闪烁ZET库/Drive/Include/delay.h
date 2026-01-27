#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void Delay_Init(void);				//延时函数初始化
void Delay_ms(u32 nTime);	//毫秒延时函数

#endif /* __SYSTICK_H */
