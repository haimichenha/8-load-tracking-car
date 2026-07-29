#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Delay.h"
#include "Tracking.h"

// Step 3(可选)：接入 TB6612 电机后再打开
// #include "Motor.h"
// #define APP_USE_MOTOR 1

int main(void)
{
	delay_init();
	OLED_Init();
	Tracking_Init();

	OLED_ShowString(1, 1, "Track: 0x");

	while (1)
	{
		uint8_t track_data = Tracking_Read();
		OLED_ShowHexNum(1, 10, track_data, 2);

		// Step 3(可选)：根据循迹做最简单的左右纠偏
		// if (APP_USE_MOTOR) {
		// 	if (track_data == 0x18) Motor_SetSpeed(40, 40);      // 中间两路压线 -> 直行
		// 	else if (track_data & 0xE0) Motor_SetSpeed(15, 45);  // 偏左 -> 右修正
		// 	else if (track_data & 0x07) Motor_SetSpeed(45, 15);  // 偏右 -> 左修正
		// 	else Motor_SetSpeed(0, 0);
		// }

		delay_ms(50);
	}
}
