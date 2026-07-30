#ifndef __APP_LINE_FOLLOW_MISSION_H
#define __APP_LINE_FOLLOW_MISSION_H

#include "stm32f10x.h"

/* Physical start/stop ownership remains with KEY2 on PG10. */
void LineFollowMission_Init(uint32_t nowMs);
void LineFollowMission_Update(uint32_t nowMs);
void LineFollowMission_HandleCommand(char command, uint32_t nowMs);

#endif /* __APP_LINE_FOLLOW_MISSION_H */
