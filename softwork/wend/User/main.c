#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "OLED.h"
#include "dht11.h"

extern unsigned int rec_data[4];

int main()
{
    OLED_Init();
    
    // --- 静态显示部分 (只刷一次) ---
    // 第1行显示温度标签 "Temp:"
    OLED_ShowString(1, 1, "Temp:"); 
    // 第1行显示小数点 "." (在整数和小数之间)
    OLED_ShowString(1, 9, ".");     
    // 第1行显示单位 "C"
    OLED_ShowString(1, 12, "C");    
    
    // 第3行显示湿度标签 "Humi:"
    OLED_ShowString(3, 1, "Humi:"); 
    // 第3行显示小数点 "."
    OLED_ShowString(3, 9, ".");     
    // 第3行显示单位 "%"
    OLED_ShowString(3, 13, "%");    

    while(1)
    {
        Delay_s(1);
        DHT11_REC_Data(); //接收温度和湿度的数据

        // --- 动态数据显示部分 ---
        
        // 显示温度: rec_data[2] . rec_data[3]
        // 第1行第7列开始显示整数 (2位)
        OLED_ShowNum(1, 7, rec_data[2], 2);  
        // 第1行第10列开始显示小数 (1位)
        OLED_ShowNum(1, 10, rec_data[3], 1); 
        
        // 显示湿度: rec_data[0] . rec_data[1]
        // 第3行第7列开始显示整数 (2位)
        OLED_ShowNum(3, 7, rec_data[0], 2);  
        // 第3行第10列开始显示小数 (2位)
        OLED_ShowNum(3, 10, rec_data[1], 2); 
    }
}