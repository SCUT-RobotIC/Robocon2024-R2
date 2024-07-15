#include "throwball.h"


#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "bsp_can.h"
#include "motorctrl.h"
#include "stdio.h"
#include "CALCULATE.h"
#include "math.h"
#include "delay.h"
#include "throwball.h"
PhotogateSpd Spds;
extern double LowAng,HighAng;
extern motor_measure_t   *motor_data[8];
void Set_servo(TIM_HandleTypeDef *htim, uint32_t Channel, double angle, uint32_t countPeriod, uint32_t CycleTime)
{
  uint16_t compare_value = 0;
  if (angle <= 180)
  {
    compare_value = 0.5 * countPeriod / CycleTime + angle * countPeriod / CycleTime / 90;
    __HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
  }
}
