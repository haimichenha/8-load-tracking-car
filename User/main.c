#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "MyI2C.h"
#include "bsp_usart.h"
#include "HW_I2C.h"
#include "Tracking.h"
#include <stdio.h>

/*
 * 测试模式 v8：硬件I2C读取循迹模块
 * 
 * 方案：
 * - 循迹模块：使用硬件I2C2（时序精确）
 * - OLED/陀螺仪：使用软件I2C（已验证正常）
 * - 动态切换GPIO模式
 * 
 * 硬件I2C配置：
 * - I2C2: PB10(SCL), PB11(SDA)
 * - 时钟: 400kHz
 * - 寄存器: 0x30
 */

#define TRACKING_ADDR   0x12    // 循迹模块I2C地址 (7位)
#define TRACKING_REG    0x30    // 数据寄存器地址（官方）

int main(void)
{
    uint8_t track_hw = 0x00;      // 硬件I2C读取
    uint8_t hw_result = 0;        // 硬件I2C返回值
    uint8_t i;
    char str[9];
    
    /* 初始化OLED（使用软件I2C） */
    OLED_Init();
    Delay_ms(100);
    
    OLED_ShowString(1, 1, "Track v8");
    OLED_ShowString(2, 1, "HW I2C Test");
    Delay_ms(1000);
    OLED_Clear();
    
    /* 初始化硬件I2C */
    HW_I2C_Init();
    Delay_ms(100);
    
    while (1)
    {
        /*=== 第一步：使用硬件I2C读取循迹模块 ===*/
        hw_result = HW_I2C_ReadByte(TRACKING_ADDR, TRACKING_REG, &track_hw);
        
        /*=== 第二步：切换回软件I2C，更新OLED ===*/
        HW_I2C_Disable();   // 关闭硬件I2C，释放GPIO
        Delay_us(50);
        MyI2C_ReInit();     // 重新初始化软件I2C
        
        // 第1行：显示硬件I2C读取结果
        OLED_ShowString(1, 1, "HW:");
        OLED_ShowHexNum(1, 4, track_hw, 2);
        OLED_ShowString(1, 8, "R:");
        OLED_ShowNum(1, 10, hw_result, 1);  // 0=成功
        
        // 第2行：二进制显示
        OLED_ShowString(2, 1, "Bin:");
        for (i = 0; i < 8; i++) {
            if (track_hw & (0x80 >> i)) {
                OLED_ShowChar(2, 5 + i, '1');
            } else {
                OLED_ShowChar(2, 5 + i, '0');
            }
        }
        
        // 第3行：可视化（bit7=X1, bit0=X8）
        OLED_ShowString(3, 1, "1");
        for (i = 0; i < 8; i++) {
            if (track_hw & (0x80 >> i)) {
                str[i] = '*';  // 检测到黑线
            } else {
                str[i] = '-';  // 未检测到
            }
        }
        str[8] = '\0';
        OLED_ShowString(3, 2, str);
        OLED_ShowString(3, 10, "8");
        
        // 第4行：标签和状态
        OLED_ShowString(4, 1, "X12345678");
        if (hw_result == 0) {
            OLED_ShowString(4, 12, "OK  ");
        } else {
            OLED_ShowString(4, 12, "FAIL");
        }
        
        /*=== 第三步：切换回硬件I2C，准备下次读取 ===*/
        HW_I2C_Enable();
        
        Delay_ms(100);
    }
}

