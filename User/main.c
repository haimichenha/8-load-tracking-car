#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "MyI2C.h"
#include "JY301P.h"
#include "bsp_usart.h"
#include <stdio.h>

int main(void)
{
	uint8_t ack;
	
	OLED_Init();
	Delay_ms(100);
	
	// 初始化 MyI2C（确保在 OLED 初始化后重新配置引脚）
	MyI2C_Init();
	Delay_ms(100);
	
	// 初始化陀螺仪
	JY301P_Init();
	
	OLED_ShowString(1, 1, "WIT Gyro I2C");
	OLED_ShowString(2, 1, "Addr: 0x50");
	Delay_ms(1000);
	
	// 测试陀螺仪是否连接
	MyI2C_Start();
	MyI2C_SendByte(WIT_I2C_ADDR << 1);  // 写地址: 0xA0
	ack = MyI2C_ReceiveAck();
	MyI2C_Stop();
	
	OLED_Clear();
	if (ack == 0) {
		OLED_ShowString(1, 1, "Gyro Found!");
	} else {
		OLED_ShowString(1, 1, "Gyro Not Found");
		OLED_ShowString(2, 1, "Check Wiring!");
	}
	Delay_ms(1000);
	OLED_Clear();
	
	while (1)
	{
		// 读取陀螺仪所有数据
		JY301P_Update();
		
		// 在OLED上显示数据
		JY301P_DisplayOnOLED();
		
		Delay_ms(100);  // 100ms刷新一次
	}
}
