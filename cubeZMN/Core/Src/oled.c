/**
  ******************************************************************************
  * @file    oled.c
  * @brief   OLED 驱动 (HAL 库版本，使用硬件 I2C2)
  ******************************************************************************
  */

#include "oled.h"
#include "oled_font.h"

extern I2C_HandleTypeDef hi2c2;

#define OLED_ADDR  0x78  // OLED I2C 地址 (0x3C << 1)

/**
  * @brief  向 OLED 写入命令
  */
static void OLED_WriteCommand(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};  // 0x00 表示命令
    HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, data, 2, 100);
}

/**
  * @brief  向 OLED 写入数据
  */
static void OLED_WriteData(uint8_t dat)
{
    uint8_t data[2] = {0x40, dat};  // 0x40 表示数据
    HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, data, 2, 100);
}

/**
  * @brief  设置光标位置
  * @param  Y: 页地址 (0-7)
  * @param  X: 列地址 (0-127)
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                  // 设置页地址
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));  // 设置列地址高4位
    OLED_WriteCommand(0x00 | (X & 0x0F));         // 设置列地址低4位
}

/**
  * @brief  清屏
  */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
    }
}

/**
  * @brief  在指定位置显示一个字符
  * @param  Line: 行 (1-4)
  * @param  Column: 列 (1-16)
  * @param  Char: 要显示的字符
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
    }
}

/**
  * @brief  在指定位置显示字符串
  * @param  Line: 行 (1-4)
  * @param  Column: 列 (1-16)
  * @param  String: 要显示的字符串
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/**
  * @brief  计算 X 的 Y 次方
  */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/**
  * @brief  在指定位置显示数字（十进制，正整数）
  * @param  Line: 行 (1-4)
  * @param  Column: 列 (1-16)
  * @param  Number: 要显示的数字 (0-4294967295)
  * @param  Length: 显示长度 (1-10)
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  在指定位置显示十六进制数
  * @param  Line: 行 (1-4)
  * @param  Column: 列 (1-16)
  * @param  Number: 要显示的数字
  * @param  Length: 显示长度 (1-8)
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/**
  * @brief  在指定位置显示二进制数
  * @param  Line: 行 (1-4)
  * @param  Column: 列 (1-16)
  * @param  Number: 要显示的数字
  * @param  Length: 显示长度 (1-16)
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/**
  * @brief  OLED 初始化
  */
void OLED_Init(void)
{
    HAL_Delay(100);  // 等待 OLED 上电稳定
    
    OLED_WriteCommand(0xAE);  // 关闭显示
    
    OLED_WriteCommand(0xD5);  // 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80);
    
    OLED_WriteCommand(0xA8);  // 设置多路复用率
    OLED_WriteCommand(0x3F);  // 1/64
    
    OLED_WriteCommand(0xD3);  // 设置显示偏移
    OLED_WriteCommand(0x00);
    
    OLED_WriteCommand(0x40);  // 设置显示开始行
    
    OLED_WriteCommand(0xA1);  // 设置左右方向，0xA1正常 0xA0左右反置
    
    OLED_WriteCommand(0xC8);  // 设置上下方向，0xC8正常 0xC0上下反置

    OLED_WriteCommand(0xDA);  // 设置COM引脚硬件配置
    OLED_WriteCommand(0x12);
    
    OLED_WriteCommand(0x81);  // 设置对比度控制
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9);  // 设置预充电周期
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);  // 设置VCOMH取消选择级别
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);  // 设置整个显示打开/关闭

    OLED_WriteCommand(0xA6);  // 设置正常/倒转显示

    OLED_WriteCommand(0x8D);  // 设置充电泵
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF);  // 开启显示
    
    OLED_Clear();  // 清屏
}
