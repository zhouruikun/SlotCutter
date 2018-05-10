#include "stepmoto.h"
#include "tim.h"
#include "comm.h"
uint8_t stepMotoDir;
uint16_t stepMotoSpeed=1000;
uint32_t stepMotoPluse;
uint32_t stepMotoSlope;
uint32_t stepMotoSlope_set;
uint32_t stepMotoSlope_speed;
uint32_t stepMotoPluse_has;
uint32_t get_moto_pluse(void)
{
	return stepMotoPluse/2;
}
uint32_t get_moto_pluse_has(void)
{
	return stepMotoPluse_has/2;
}

void runMoto(uint32_t Pluse,uint32_t slope , uint8_t dir)
{
//电机启动 Pluse脉冲   ，slope 斜率 dir  方向
  startMoto(  Pluse,  slope ,  dir);
	while(stepMotoPluse/2!=0)
	{
		osDelay(1);
	}
 
}
void set_slope(uint32_t slope)
{
 
	
		stepMotoSlope=0;
		stepMotoSlope_set = slope-1;
		if(stepMotoPluse<stepMotoSlope_set)
			{
				stepMotoSlope_set=stepMotoPluse;
			}
		stepMotoSlope_set=stepMotoSlope_set*2+1;
}
void continueMoto(void)
{
	HAL_TIMEx_OCN_Start_IT(&htim1,TIM_CHANNEL_2);
}
//电机启动 Pluse脉冲   ，slope 斜率 dir  方向
void startMoto(uint32_t Pluse,uint32_t slope , uint8_t dir)
{


  stepMotoPluse_has=0;
	if(dir){
		setMotoDir(Setting.SettingStruct.motoDirection[INDEX_VALUE]);
	}
	else
	{
		setMotoDir(!Setting.SettingStruct.motoDirection[INDEX_VALUE]);
	}
	 
	//MX_TIM1_Init();
	
	TIM1->CNT=0;
	TIM1->CCR2 =stepMotoSpeed;
	

	
	HAL_TIMEx_OCN_Start_IT(&htim1,TIM_CHANNEL_2);
	
	
	if(Pluse>1)
	stepMotoPluse = Pluse-1;
	else
	{
		stepMotoPluse=0;
	}
	if(slope>1)
	{	
		stepMotoSlope=0;
		stepMotoSlope_set = slope-1;
		if(stepMotoPluse<stepMotoSlope_set)
			{
				stepMotoSlope_set=stepMotoPluse;
			}
	}
	else
	{
		stepMotoSlope_set=0;
		stepMotoSlope=0;
	}
	stepMotoSlope_set=stepMotoSlope_set*2+1;
	stepMotoPluse=stepMotoPluse*2+1;	

}
uint32_t stopMoto(void)
{
	HAL_TIMEx_OCN_Stop_IT(&htim1,TIM_CHANNEL_2);
	return stepMotoPluse/2;
}
void setMotoDir(uint16_t dir)
{
	if(dir)
		HAL_GPIO_WritePin(Step_Motor_Dir_GPIO_Port,Step_Motor_Dir_Pin,GPIO_PIN_SET);
	else
			HAL_GPIO_WritePin(Step_Motor_Dir_GPIO_Port,Step_Motor_Dir_Pin,GPIO_PIN_RESET);
}
void setMotoSpeed(uint32_t Speed)
{
	Speed = 1000000/Speed;
	stepMotoSpeed = Speed-1;
}
//一个脉冲计算一次  一个脉冲增加1.6%
void speedCal(void)
{
	uint32_t index_k =30;
	
	
		int32_t inc = 1000000/Setting.SettingStruct.stepMotoInitSpeed[INDEX_VALUE]-1000000/Setting.SettingStruct.stepMotoRunSpeed[INDEX_VALUE];
	if(inc<0)
	{
		stepMotoSlope_speed =0;
		return ;
	}
		inc/=index_k;
		if(stepMotoSlope_set/2<index_k)
			{
				index_k=stepMotoSlope_set/2;
			}
	
	if(stepMotoSlope_set>stepMotoSlope)
		{
			stepMotoSlope++;
			if(stepMotoSlope<index_k) 
			{
				stepMotoSlope_speed = stepMotoSlope*inc;
			}
			else if(stepMotoSlope>(stepMotoSlope_set-index_k))
			{
				stepMotoSlope_speed = (stepMotoSlope_set-stepMotoSlope)*inc;
			}
			else
			{
				stepMotoSlope_speed=stepMotoSlope_speed;
			}
		}
		else
		{
			stepMotoSlope_speed=0;
		}
	 
}
//获取当前计数 加上速度 然后脉冲数自减

/**
  * @brief  PWM Pulse finished callback in non blocking mode 
  * @param  htim : TIM handle
  * @retval None
  */
 void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

	if(stepMotoPluse>0)
	{

		TIM1->CCR2 =TIM1->CNT+stepMotoSpeed-stepMotoSlope_speed;
		speedCal();
		if(stepMotoPluse<UINT32_MAX)
		{
			stepMotoPluse--;
			stepMotoPluse_has++;
		}
		
	}
	else
	{
		   /* Disable the TIM Output Compare interrupt */
    HAL_TIMEx_OCN_Stop_IT(&htim1,TIM_CHANNEL_2); 
					 /* Disable the Capture compare channel N */
	}
	
		
}