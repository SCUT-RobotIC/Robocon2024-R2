#include "throwball.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "pidctl.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "motorctrl.h"
#include "stdio.h"
#include "CALCULATE.h"
#include "math.h"
#include "delay.h"
#include "throwball.h"
PhotogateSpd Spds;

void RiseBall_Init(){
	rtU.yaw_target4=-1000;
	delay_us(500000);
	Spds.flag[4]=1;
		rtU.yaw_target4=1000;
				Spds.flag[2]=0;
				Spds.flag[3]=0;
	}
void set_getball(uint16_t speed){
		rtU.yaw_target5 = speed;
		rtU.yaw_target6 = -(speed);
}
void motor_off(){
		rtU.yaw_target5 = 0;
		rtU.yaw_target6  = 0;
}


//R2坡度上升下降连贯，间隔3s
void movement_take(){
		__HAL_TIM_SET_COMPARE (&htim2,TIM_CHANNEL_2,1890);
		HAL_Delay(10000);
		__HAL_TIM_SET_COMPARE (&htim2,TIM_CHANNEL_2,1928);

}

//R2坡度下降
void ramp_down(){
		__HAL_TIM_SET_COMPARE (&htim2,TIM_CHANNEL_2,1884);
}
	

//R2坡度抬起
void ramp_up(){
		__HAL_TIM_SET_COMPARE (&htim2,TIM_CHANNEL_2,1928);
}




//R2上方舵机扔球
void throw_ball(){	
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,1750);//左抬
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,1950);//右抬
		delay_us(2000000);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,1950);//左降
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,1750);//右降
	
	
}

