#include "stepmoto.h"
#include "tim.h"
uint8_t stepMotoDir;
uint16_t stepMotoSpeed=1000;
uint16_t stepMotoPluse;


void startMoto(uint16_t Pluse)
{
	HAL_TIMEx_OCN_Start_IT(&htim1,TIM_CHANNEL_2);
	stepMotoPluse = Pluse;
}
uint16_t stopMoto(void)
{
	HAL_TIMEx_OCN_Stop_IT(&htim1,TIM_CHANNEL_2);
	return stepMotoPluse;
}

void setMotoSpeed(uint16_t Speed)
{
	stepMotoSpeed = Speed;
}
//获取当前计数 加上速度 然后脉冲数自减

/**
  * @brief  PWM Pulse finished callback in non blocking mode 
  * @param  htim : TIM handle
  * @retval None
  */
uint16_t cnt ;
 void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(stepMotoPluse>0)
	{
		stepMotoPluse--;
		  cnt = __HAL_TIM_GET_COUNTER(&htim1);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,cnt+stepMotoSpeed);
	}
	else
	{
			HAL_TIMEx_OCN_Stop_IT(&htim1,TIM_CHANNEL_2);
	}
	
		
}