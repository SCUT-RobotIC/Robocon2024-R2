/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "PID_MODEL.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "motorctrl.h"
#include "stdio.h"
#include "calculate.h"
#include "math.h"
#include "delay.h"
#include "solve.h"
#include "throwball.h"
#include "sbus.h"
// #include "R1_Ball_MODEL.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VEL      1
#define ANG      2

#define USART_REC_LEN  			100  	//
#define RXBUFFERSIZE   			1 		//

#define CatchBallInit 90

uint8_t USART2_RX_BUF[USART_REC_LEN]; 
uint16_t USART2_RX_STA = 0; 
uint8_t aRxBuffer2[RXBUFFERSIZE];		  
UART_HandleTypeDef UART2_Handler; 


uint8_t USART6_RX_BUF[USART_REC_LEN]; 
uint16_t USART6_RX_STA = 0; 
uint8_t aRxBuffer6[RXBUFFERSIZE];		  
UART_HandleTypeDef UART6_Handler; 


TGT_COOR TC;
REAL_COOR RC;

extern int condition;
extern motor_measure_t   *motor_data[8];
extern motor_measure_t   *motor_data1[8];
extern ang_dir MotorSignal[4];
extern PhotogateAng ANGs;
extern PhotogateSpd Spds;

typedef struct __FILE FILE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_REC_LEN 100 //
#define RXBUFFERSIZE 1    //
#define VEL      1
#define ANG      2


int receivefactor[4];
//int factor=0;
//int factor1=0;


//int flag=0;
//int flag1=0;
int dir[4];
int ang[4];
int angtemp[4];
int flagf[4];
double Vx,Vy,omega=0;

double theta[4];


DataPacket DataRe;
int16_t lx,ly,rx,ry,lp,rp,LX,LY,RX,RY;
uint8_t B1,B2;
uint8_t Cal_Parity;
	
	
double a,b,c,d=0;
uint8_t USART_FLAG=0;
extern double LowAng,HighAng;


int ML_CH3 =  1224  ; // left_spd
int ML_CH4 =  1003 ; // rotate 
int MR_CH2 =  996 ; //  right_y  max 1400 min 600
int MR_CH1 =  1000 ;  // right_x


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM10_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  // Controler Receive Init
  HAL_UART_Receive_DMA(&huart3, (uint8_t *)&DataRe, 1);
  HAL_UART_Receive_DMA(&huart2, aRxBuffer2, 1);
  HAL_UART_Receive_DMA(&huart6, aRxBuffer6, 1);
  // MATLAB Init
  PID_MODEL_initialize();
  HAL_TIM_Base_Start_IT(&htim10);

  // CAN Init
  can_filter_init();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  // Chassis PID Init
	
  PID_Speed_Para_Init(1, 1, 10 , 3 , 0.01);
  PID_Speed_Para_Init(1, 2, 10 , 3 , 0.01);
  PID_Speed_Para_Init(1, 3, 10 , 3 , 0.01);
  PID_Speed_Para_Init(1, 4, 10 , 3 , 0.01);
  PID_Angle_S_Para_Init(1, 1 , 1.5 , 0.3 , 0.01);
  PID_Angle_A_Para_Init(1, 1 , 0.5 , 0.1 , 0.01);
  PID_Angle_S_Para_Init(1, 2 , 1.5 , 0.3 , 0.01);
  PID_Angle_A_Para_Init(1, 2 , 0.5 , 0.1 , 0.01);
  PID_Angle_S_Para_Init(1, 3 , 1.5 , 0.3 , 0.01);
  PID_Angle_A_Para_Init(1, 3 , 0.5 , 0.1 , 0.01);
  PID_Angle_S_Para_Init(1, 4 , 1.5 , 0.3 , 0.01);
  PID_Angle_A_Para_Init(1, 4 , 0.5 , 0.1 , 0.01);
		
  PID_Speed_Para_Init(2, 1, 1.5 , 0.3 , 0.01);
  PID_Speed_Para_Init(2, 2, 1.5 , 0.3 , 0.01);
  PID_Speed_Para_Init(2, 3, 1.5 , 0.3 , 0.01);
  PID_Speed_Para_Init(2, 4, 1.5 , 0.3 , 0.01);		
																				 
	PID_Angle_S_Para_Init(2, 1 , 1.5 , 0.3 , 0.01);
  PID_Angle_A_Para_Init(2, 1 , 0.5 , 0.1 , 0.01);
  PID_Angle_S_Para_Init(2, 2 , 1.5 , 0.3 , 0.01);
  PID_Angle_A_Para_Init(2, 2 , 0.5 , 0.1 , 0.01);
  PID_Angle_S_Para_Init(2, 3 , 1.5 , 0.3 , 0.01);
  PID_Angle_A_Para_Init(2, 3 , 0.5 , 0.1 , 0.01);
  PID_Angle_S_Para_Init(2, 4 , 1.5 , 0.3 , 0.01);
  PID_Angle_A_Para_Init(2, 4 , 0.5 , 0.1 , 0.01);
																				 
	//Get ball PID init
	PID_Speed_Para_Init(1, 5, 10 , 3 , 0.01);
	PID_Speed_Para_Init(1, 6, 10 , 3 , 0.01);
	//throw ball PID init
	PID_Speed_Para_Init(1, 7, 10 , 3 , 0.01);
	PID_Angle_S_Para_Init(1, 7 , 1.5 , 0.3 , 0.01);
  PID_Angle_A_Para_Init(1, 7 , 0.5 , 0.1 , 0.01);
	rtP.TRANS_CH1_7 = 0.7;
	
	//get ball away
	PID_Speed_Para_Init(2, 5, 1.5 , 0.3 , 0.01);
  PID_Speed_Para_Init(2, 6, 1.5 , 0.3 , 0.01);	
	
	set_mode(VEL, VEL, VEL, VEL, VEL, VEL, VEL,
             VEL, VEL, VEL, VEL, VEL, VEL, VEL); 
  // Clamp TIM Init
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	Set_servo(&htim3, TIM_CHANNEL_1, 10, 20000, 20);//fai(�������ʾ�Ķ��)
	Set_servo(&htim3, TIM_CHANNEL_2,0, 20000, 20);//theta
//	Set_servo(&htim5, TIM_CHANNEL_3,CatchBallInit,20000,20);
  // Clamp PID Init
  // PID_Angle_S_Para_Init(2, 3, 1.5, 0.3, 0.01);
  // PID_Angle_S_Para_Init(2, 4, 1.5, 0.3, 0.01);

  // PID_Angle_A_Para_Init(2, 3, 0.5, 0.1, 0.01);
  // PID_Angle_A_Para_Init(2, 4, 0.5, 0.1, 0.01);
//  PID_Angle_S_Para_Init(2, 3, 0.6901 * 0.75, 2.3727 * 0.17, 0.01);
//  PID_Angle_S_Para_Init(2, 4, 0.6901 * 0.7, 2.3727 * 0.11, 0.0126);

//  PID_Angle_A_Para_Init(2, 3, 0.83005 * 0.85, 0.38548 * 0.02, 0.04    );
//  PID_Angle_A_Para_Init(2, 4, 0.83005 * 0.6 , 0.38548 * 0.02, 0.051858);

//  rtP.TRANS_CH2_3 = 1.1;
//  rtP.TRANS_CH2_4 = 0.4;

//  rtP.DEADBAND_CH2_3 = 800;
//  rtP.DEADBAND_CH2_4 = 800;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//		if(DataRe.header==0xAA)
//		{
//			CAL_MESSAGE();
//			HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataRe + 1, sizeof(DataPacket) - 1);
//		}
//		else if(DataRe.header!=0xAA)
//		{
//			HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataRe, 1);
//		}
// }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    if (DataRe.header == 0xAA && USART_FLAG == 0)
    {
      receivefactor[0] = 1;
      CAL_MESSAGE();
      HAL_UART_Receive_DMA(&huart3, (uint8_t *)&DataRe + 1, sizeof(DataPacket) - 1);
      USART_FLAG = 1;
    }
    if (DataRe.header == 0xAA && USART_FLAG == 1)
    {
      receivefactor[0] = 1;
      CAL_MESSAGE();
      HAL_UART_Receive_DMA(&huart3, (uint8_t *)&DataRe, sizeof(DataPacket));
    }
    else
    {
      HAL_UART_Receive_DMA(&huart3, (uint8_t *)&DataRe, 1);
      USART_FLAG = 0;
    }
  }

  while (huart->Instance == USART2) // uart2 ?
  {
    USART2_RX_BUF[USART2_RX_STA] = aRxBuffer2[0];
    if (USART2_RX_STA == 0 && USART2_RX_BUF[USART2_RX_STA] != 0x0F)
    {
      HAL_UART_Receive_DMA(&huart2, aRxBuffer2, 1);
      break; //
    }
    USART2_RX_STA++;
    HAL_UART_Receive_DMA(&huart2, aRxBuffer2, 1);
    if (USART2_RX_STA > USART_REC_LEN)
      USART2_RX_STA = 0;                                                              //
    if (USART2_RX_BUF[0] == 0x0F && USART2_RX_BUF[15] == 0xAA && USART2_RX_STA == 16) // �?测包头包尾以及数据包长度
    {
      Receive();
      receivefactor[1] = 1;
      Reach_TGT();
      //			for(int i=0;i<14;i++)
      //			  USART2_RX_BUF[i] = 0;//清除数据
      USART2_RX_STA = 0;
    }
    else if (!(USART2_RX_BUF[0] == 0x0F && USART2_RX_BUF[15] == 0xAA) && USART2_RX_STA == 16)
    {
      for (int i = 0; i < 16; i++)
        USART2_RX_BUF[i] = 0; // 清除数据
      USART2_RX_STA = 0;
    }

    break;
  }
	
	
		while (huart->Instance == USART6) //uart2 ?
	{
		USART6_RX_BUF[USART6_RX_STA] = aRxBuffer6[0];
		if (USART6_RX_STA == 0 && USART6_RX_BUF[USART6_RX_STA] != 0x0F) 	
		{			HAL_UART_Receive_DMA(&huart6,aRxBuffer6,1);	
		break; //
			}
		USART6_RX_STA++;
			HAL_UART_Receive_DMA(&huart6,aRxBuffer6,1);
		if (USART6_RX_STA > USART_REC_LEN) USART6_RX_STA = 0;  //
		if (USART6_RX_BUF[0] == 0x0F && USART6_RX_BUF[24] == 0x00 && USART6_RX_STA == 25)	//�?测包头包尾以及数据包长度
		{
			update_sbus(USART6_RX_BUF);
		  RX =  SBUS_CH.CH1 - MR_CH1;
		  RY =  SBUS_CH.CH2 - MR_CH2; 
      LX =  SBUS_CH.CH4 - ML_CH4; 
			if(receivefactor[1]==0){
			receivefactor[0]=1;
			}
			for(int i=0;i<25;i++)
			  USART6_RX_BUF[i] = 0;//清除数据
			USART6_RX_STA = 0;
		}
		else if(!(USART6_RX_BUF[0] == 0x0F && USART6_RX_BUF[24] == 0x00) && USART6_RX_STA == 25){
			for(int i=0;i<25;i++)
			  USART6_RX_BUF[i] = 0;//清除数据
			USART6_RX_STA = 0;
		}

		break;
	}
}

// 错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)&DataRe, 1);
    USART_FLAG = 0;
  }
  if (huart->Instance == USART2)
  {
    HAL_UART_Receive_DMA(&huart2, aRxBuffer2, 1);
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim == &htim10)
  {

				

		
    assign_output();
    PID_MODEL_step();
    motor_state_update();
    motor_state_update1();
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
