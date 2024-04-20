/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pidctl.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "motorctrl.h"
#include "stdio.h"
#include "CALCULATE.h"
#include "math.h"
#include "delay.h"
#include "throwball.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define VEL      1
#define ANG      2
double LowAng,HighAng;
extern int condition[10];
extern motor_measure_t   *motor_data[8];
extern motor_measure_t   *motor_data1[8];
extern ang_dir MotorSignal[4];
extern PhotogateAng ANGs;
extern PhotogateSpd Spds;
typedef struct __FILE FILE;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


extern int angtemp[4];
extern int flagf[4];
extern double Vx,Vy,omega;
extern int receivefactor;
extern double theta[4];


extern DataPacket DataRe;
extern int16_t lx,ly,rx,ry,lp,rp;
extern uint8_t B1,B2;
extern uint8_t Cal_Parity;
	
int i;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ThrowBall */
osThreadId_t ThrowBallHandle;
const osThreadAttr_t ThrowBall_attributes = {
  .name = "ThrowBall",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Rise */
osThreadId_t RiseHandle;
const osThreadAttr_t Rise_attributes = {
  .name = "Rise",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Throw(void *argument);
void RiseBall(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ThrowBall */
  ThrowBallHandle = osThreadNew(Throw, NULL, &ThrowBall_attributes);

  /* creation of Rise */
  RiseHandle = osThreadNew(RiseBall, NULL, &Rise_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for(;;)
  {	
		for(int i=0;i<4;i++){
		if(ANGs.flag[i]!=1&&receivefactor==1){
				MotorSignal[i].thetas=MotorSignal[i].thetas+0.3;

		}
}
					
    delay_us(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Throw */
/**
* @brief Function implementing the ThrowBall thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Throw */
void Throw(void *argument)
{
  /* USER CODE BEGIN Throw */
  /* Infinite loop */
  for(;;)
  {		if(condition[0]==1){//按下按键1
		set_getball(3390);
		movement_take();
		motor_off();



	}
  //  osDelay(1);
  }
  /* USER CODE END Throw */
}

/* USER CODE BEGIN Header_RiseBall */
/**
* @brief Function implementing the Rise thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RiseBall */
void RiseBall(void *argument)
{
  /* USER CODE BEGIN RiseBall */
  /* Infinite loop */
  for(;;)
  {
		if(condition[1]==1){
				rtU.yaw_target4=-1500;
				while(1)
					{
if(fabs((motor_data[4]->ecd+motor_data[4]->circle*8191)-HighAng)<8191)
{
	delay_us(500000);
	throw_ball();
	  rtU.yaw_target4=2000;

	break;
}
}
		}	
				//	Spds.flag[5]=0;
osDelay(1);
  }
  /* USER CODE END RiseBall */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

