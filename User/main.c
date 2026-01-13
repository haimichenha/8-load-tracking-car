#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "bsp_usart.h"
#include "HW_I2C.h"
#include "Tracking.h"
#include "JY301P.h"
#include "IOI2C.h"
#include <stdio.h>

/*
 * 测试模式 v9：循迹模块(硬件I2C) + 陀螺仪(IOI2C)
 * 
 * 显示内容：
 * 第1行：循迹HW数据 + 陀螺仪Roll/Pitch
 * 第2行：循迹Bin数据
 * 第3行：陀螺仪Yaw
 * 第4行：状态
 * 
 * I2C分配：
 * - 循迹模块(0x12): 硬件I2C2
 * - 陀螺仪(0x50): IOI2C (软件I2C)
 * - OLED(0x3C): OLED_I2C (自带)
 */

#define TRACKING_ADDR   0x12    // 循迹模块I2C地址 (7位)
#define TRACKING_REG    0x30    // 数据寄存器地址（官方）

int main(void)
{
    uint8_t track_hw = 0x00;      // 硬件I2C读取的循迹数据
    uint8_t hw_result = 0;        // 硬件I2C返回值
    uint8_t fail_count = 0;       // 连续失败计数
    uint8_t i;
    
    /* 初始化OLED */
    OLED_Init();
    Delay_ms(100);
    
    OLED_ShowString(1, 1, "Track v9");
    OLED_ShowString(2, 1, "HW+Gyro Test");
    Delay_ms(1000);
    OLED_Clear();
    
    /* 初始化IOI2C（用于陀螺仪） */
    IIC_Init();
    Delay_ms(100);
    
    /* 初始化陀螺仪 */
    JY301P_Init();
    Delay_ms(100);
    
    /* 初始化硬件I2C（用于循迹模块） */
    HW_I2C_Init();
    Delay_ms(50);
    
    while (1)
    {
        /*=== 第一步：使用硬件I2C读取循迹模块 ===*/
        hw_result = HW_I2C_ReadByte(TRACKING_ADDR, TRACKING_REG, &track_hw);
        
        // 如果读取失败，尝试恢复
        if (hw_result != 0) {
            fail_count++;
            if (fail_count >= 3) {
                // 连续失败3次，尝试总线恢复
                HW_I2C_BusRecovery();
                fail_count = 0;
            }
        } else {
            fail_count = 0;
        }
        
        /*=== 第二步：切换到IOI2C，读取陀螺仪 ===*/
        HW_I2C_Disable();   // 关闭硬件I2C，释放GPIO
        Delay_ms(1);        // 等待总线稳定
        IIC_Init();         // 初始化IOI2C
        
        // 更新陀螺仪数据
        JY301P_Update();
        
        /*=== 第三步：更新OLED显示 ===*/
        // 第1行：显示循迹HW数据 + Roll/Pitch
        OLED_ShowString(1, 1, "R:");
        OLED_ShowSignedNum(1, 3, (int32_t)g_jy301p_data.angle[0], 3);
        OLED_ShowString(1, 9, "P:");
        OLED_ShowSignedNum(1, 11, (int32_t)g_jy301p_data.angle[1], 3);
        
        // 第2行：显示Yaw角度
        OLED_ShowString(2, 1, "Yaw:");
        OLED_ShowSignedNum(2, 5, (int32_t)g_jy301p_data.angle[2], 3);
        
        // 第3行：循迹HW数据 + 二进制显示
        OLED_ShowString(3, 1, "HW:");
        OLED_ShowHexNum(3, 4, track_hw, 2);
        OLED_ShowString(3, 7, "B:");
        for (i = 0; i < 8; i++) {
            if (track_hw & (0x80 >> i)) {
                OLED_ShowChar(3, 9 + i, '1');
            } else {
                OLED_ShowChar(3, 9 + i, '0');
            }
        }
        
        // 第4行：状态
        if (hw_result == 0) {
            OLED_ShowString(4, 1, "Track:OK  ");
        } else {
            OLED_ShowString(4, 1, "Track:FAIL");
        }
        OLED_ShowString(4, 12, "v9");
        
        /*=== 第四步：切换回硬件I2C，准备下次读取 ===*/
        IIC_ReleaseBus();   // 释放IOI2C总线
        Delay_ms(1);        // 等待总线稳定
        HW_I2C_Enable();    // 切换回硬件I2C（完整重新初始化）
        Delay_ms(1);        // 等待I2C稳定
        
        Delay_ms(25);
    }
}

