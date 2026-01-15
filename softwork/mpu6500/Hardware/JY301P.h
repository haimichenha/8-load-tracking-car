#ifndef __JY301P_H
#define __JY301P_H

#include "stm32f10x.h"

typedef struct {
    float acc[3];
    float gyro[3];
    float angle[3];
} JY301P_Data;

extern JY301P_Data g_jy301p_data;

void JY301P_Init(void);
void JY301P_Update(void);

#endif
