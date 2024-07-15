#ifndef __THROWBALL_H__
#define __THROWBALL_H__
#include "main.h"
#include "tim.h"
typedef struct
{
	int flag[100];

} PhotogateSpd;

void Set_servo(TIM_HandleTypeDef *htim, uint32_t Channel, double angle, uint32_t countPeriod, uint32_t CycleTime);
#endif


