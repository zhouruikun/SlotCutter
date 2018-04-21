/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "HT1628.h"
#include "comm.h"
#include "key.h"
#include "stm_flash.h"
#include "tim.h"
#include "stepmoto.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId LEDTaskHandle;
osThreadId keyTaskHandle;
osThreadId stepMotoTaskHandle;

/* USER CODE BEGIN Variables */
osThreadId keyTaskHandle;

QueueHandle_t led_key_queue; 
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartLEDTask(void const * argument);
void StartKeyTask(void const * argument);
void StartStepMotoTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
	void arg_read(void);
	//增加一个数字位
	uint16_t decNumBit(uint16_t num,uint8_t bit);
	uint16_t incNumBit(uint16_t num,uint8_t bit);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

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

  /* Create the thread(s) */
  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, StartLEDTask, osPriorityNormal, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of keyTask */
  osThreadDef(keyTask, StartKeyTask, osPriorityIdle, 0, 128);
  keyTaskHandle = osThreadCreate(osThread(keyTask), NULL);

  /* definition and creation of stepMotoTask */
  osThreadDef(stepMotoTask, StartStepMotoTask, osPriorityIdle, 0, 128);
  stepMotoTaskHandle = osThreadCreate(osThread(stepMotoTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartLEDTask function */
void StartLEDTask(void const * argument)
{

  /* USER CODE BEGIN StartLEDTask */
	
  /* Infinite loop */
	
	uint8_t ledTaskStatus = STATUS_LED_INIT;
	uint8_t funcNum = 0;
	uint8_t showIndex = 0;
	uint8_t lightFlag = 0;
	KEY_MSG_t keyGet ;
	lint_val_display();
	arg_read();
	led_key_queue= xQueueCreate(4, sizeof(KEY_MSG_t) );
  for(;;)
  {
		switch(ledTaskStatus)
		{
			case STATUS_LED_INIT : 
						HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE],0,Setting.SettingStruct.slotNumber[INDEX_SIZE],Setting.SettingStruct.slotNumber[INDEX_POINT]);
						HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE],3,Setting.SettingStruct.slotNumber[INDEX_SIZE],Setting.SettingStruct.slotNumber[INDEX_POINT]);
						display();
						ledTaskStatus = STATUS_LED_IDLE;
				break;
			case STATUS_LED_IDLE :
					if(xQueueReceive( led_key_queue,&keyGet,pdMS_TO_TICKS(1000) )==pdPASS)
						{
							if(keyGet.key == Key_START && keyGet.status == KEY_HOLD)
							{
								ledTaskStatus = STATUS_LED_MOD_PARA;
							}
							if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
							{
							 startMoto(500);
							}
						} 
				break;
			case STATUS_LED_MOD_PARA :
						if(xQueueReceive( led_key_queue,&keyGet,pdMS_TO_TICKS(300) )==pdPASS)
						{
							if(keyGet.status == KEY_UP)
							{
								switch(keyGet.key)
								{
									case Key_START:
										funcNum++;
										showIndex = 0;
										if(funcNum>=17)
										{
											funcNum = 0;
										}
										break;
									case Key_R:
										showIndex++;
										if(showIndex>=Setting.SettingArray[funcNum][INDEX_SIZE])
										{
											showIndex=Setting.SettingArray[funcNum][INDEX_SIZE]-1;
										}
										break;
									case Key_L:
										if(showIndex>0)
										{
											showIndex--;
										}
										else
										{
											showIndex = 0;
										}
										break;
									case Key_UP:
												Setting.SettingArray[funcNum][INDEX_VALUE]=incNumBit(Setting.SettingArray[funcNum][INDEX_VALUE],Setting.SettingArray[funcNum][INDEX_SIZE]-showIndex);
												if(Setting.SettingArray[funcNum][INDEX_VALUE]>Setting.SettingArray[funcNum][INDEX_MAX])
												{
													Setting.SettingArray[funcNum][INDEX_VALUE]=Setting.SettingArray[funcNum][INDEX_MAX];
												}
											break;
									case Key_DOWN:
												Setting.SettingArray[funcNum][INDEX_VALUE]=decNumBit(Setting.SettingArray[funcNum][INDEX_VALUE],Setting.SettingArray[funcNum][INDEX_SIZE]-showIndex);
												if(Setting.SettingArray[funcNum][INDEX_VALUE]<Setting.SettingArray[funcNum][INDEX_MIN])
												{
													Setting.SettingArray[funcNum][INDEX_VALUE]=Setting.SettingArray[funcNum][INDEX_MIN];
												}
										break;
									case Key_STOP:
												ledTaskStatus = STATUS_LED_INIT;
											break;
								}
							}
							
						}
						clear();
						HT1621_dis_num(5,funcNum);
						HT1621_dis_float(Setting.SettingArray[funcNum][INDEX_VALUE],0,Setting.SettingArray[funcNum][INDEX_SIZE],Setting.SettingArray[funcNum][INDEX_POINT]);
						if(lightFlag == 0)
						{
							HT1621_dis_num(showIndex,17);
							lightFlag = 1;
						}
						else
						{
							lightFlag = 0;
						}
						display();

				break;
			case STATUS_LED_STANDBY :
				break;
			default:break;
		}
     
  }
  /* USER CODE END StartLEDTask */
}

/* StartKeyTask function */
void StartKeyTask(void const * argument)
{
  /* USER CODE BEGIN StartKeyTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(50);
		key_IRQHandler();
  }
  /* USER CODE END StartKeyTask */
}

/* StartStepMotoTask function */
void StartStepMotoTask(void const * argument)
{
  /* USER CODE BEGIN StartStepMotoTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartStepMotoTask */
}

/* USER CODE BEGIN Application */

 
//增加一个数字位
uint16_t decNumBit(uint16_t num,uint8_t bit)
{
	uint16_t power = 1;
	while(bit-1)
	{
		power*=10;
		bit--;
	}
	if(num>power)
		return num - power;
	else
		return 0;
}
//增加一个数字位
uint16_t incNumBit(uint16_t num,uint8_t bit)
{
	uint16_t power = 1;
	while(bit-1)
	{
		power*=10;
		bit--;
	}
	return num + power;
}
	//参数读出函数 如果第一次则初始化
	void arg_read(void)
	{
		STMFLASH_Read( FLASH_BASE+60*1024, (uint16_t *) &Setting, sizeof(Setting)/2 );           //从指定地址开始读出指定长度的数据	
		if(Setting.SettingStruct.diameter[INDEX_VALUE]==0xffff)
			{
				Setting.SettingStruct.diameter[INDEX_VALUE] = 3;Setting.SettingStruct.diameter[INDEX_POINT] = 0;
				Setting.SettingStruct.slotNumber[INDEX_VALUE] = 3;Setting.SettingStruct.slotNumber[INDEX_POINT] = 0;
				Setting.SettingStruct.micaWidth[INDEX_VALUE] = 15;Setting.SettingStruct.micaWidth[INDEX_POINT] = 2;
				Setting.SettingStruct.micaPreTrace[INDEX_VALUE] = 0;Setting.SettingStruct.micaPreTrace[INDEX_POINT] = 0;
				Setting.SettingStruct.motoCompensation[INDEX_VALUE] = 0;Setting.SettingStruct.motoCompensation[INDEX_POINT] = 0;
				Setting.SettingStruct.millingMethod[INDEX_VALUE] = 0;Setting.SettingStruct.millingMethod[INDEX_POINT] = 0;
				Setting.SettingStruct.motoDirection[INDEX_VALUE] = 0;Setting.SettingStruct.motoDirection[INDEX_POINT] = 0;
				Setting.SettingStruct.feedDelayTime[INDEX_VALUE] = 1;Setting.SettingStruct.feedDelayTime[INDEX_POINT] = 2;
				Setting.SettingStruct.feedActionTime[INDEX_VALUE] = 0;Setting.SettingStruct.feedActionTime[INDEX_POINT] = 0;
				Setting.SettingStruct.productDeviation[INDEX_VALUE] = 0;Setting.SettingStruct.productDeviation[INDEX_POINT] = 0;
				Setting.SettingStruct.feedPreCutTime[INDEX_VALUE] = 1;Setting.SettingStruct.feedPreCutTime[INDEX_POINT] = 2;
				Setting.SettingStruct.millingPreBackTime[INDEX_VALUE] = 1;Setting.SettingStruct.millingPreBackTime[INDEX_POINT] = 2;
				Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE] = 1000;Setting.SettingStruct.plusNumberOfMoto[INDEX_POINT] = 0;
				Setting.SettingStruct.stepMotoSlopeTime[INDEX_VALUE] = 16;Setting.SettingStruct.stepMotoSlopeTime[INDEX_POINT] = 1;
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_VALUE] = 0;Setting.SettingStruct.stepMotoRunSpeed[INDEX_POINT] = 0;
				Setting.SettingStruct.stepMotoFinishTime[INDEX_VALUE] = 0;Setting.SettingStruct.stepMotoFinishTime[INDEX_POINT] = 0;
				Setting.SettingStruct.compareThreshold[INDEX_VALUE] = 0;Setting.SettingStruct.compareThreshold[INDEX_POINT] = 1;
				Setting.SettingStruct.qualifiedRate[INDEX_VALUE] = 60;Setting.SettingStruct.qualifiedRate[INDEX_POINT] = 0;
				
				Setting.SettingStruct.diameter[INDEX_MAX] = 200;Setting.SettingStruct.diameter[INDEX_MIN] = 3;
				Setting.SettingStruct.slotNumber[INDEX_MAX] = 256;Setting.SettingStruct.slotNumber[INDEX_MIN] = 3;
				Setting.SettingStruct.micaWidth[INDEX_MAX] = 800;Setting.SettingStruct.micaWidth[INDEX_MIN] = 15;
				Setting.SettingStruct.micaPreTrace[INDEX_MAX] = 0;Setting.SettingStruct.micaPreTrace[INDEX_MIN] = 0;
				Setting.SettingStruct.motoCompensation[INDEX_MAX] = 0;Setting.SettingStruct.motoCompensation[INDEX_MIN] = 0;
				Setting.SettingStruct.millingMethod[INDEX_MAX] = 2;Setting.SettingStruct.millingMethod[INDEX_MIN] = 0;
				Setting.SettingStruct.motoDirection[INDEX_MAX] = 1;Setting.SettingStruct.motoDirection[INDEX_MIN] = 0;
				Setting.SettingStruct.feedDelayTime[INDEX_MAX] = 900;Setting.SettingStruct.feedDelayTime[INDEX_MIN] = 0;
				Setting.SettingStruct.feedActionTime[INDEX_MAX] = 0;Setting.SettingStruct.feedActionTime[INDEX_MIN] = 0;
				Setting.SettingStruct.productDeviation[INDEX_MAX] = 1;Setting.SettingStruct.productDeviation[INDEX_MIN] = 0;
				Setting.SettingStruct.feedPreCutTime[INDEX_MAX] = 900;Setting.SettingStruct.feedPreCutTime[INDEX_MIN] = 1;
				Setting.SettingStruct.millingPreBackTime[INDEX_MAX] = 900;Setting.SettingStruct.millingPreBackTime[INDEX_MIN] = 1;
				Setting.SettingStruct.plusNumberOfMoto[INDEX_MAX] = 65535;Setting.SettingStruct.plusNumberOfMoto[INDEX_MIN] = 1000;
				Setting.SettingStruct.stepMotoSlopeTime[INDEX_MAX] = 1000;Setting.SettingStruct.stepMotoSlopeTime[INDEX_MIN] = 16;
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_MAX] = 0;Setting.SettingStruct.stepMotoRunSpeed[INDEX_MIN] = 0;
				Setting.SettingStruct.stepMotoFinishTime[INDEX_MAX] = 9999;Setting.SettingStruct.stepMotoFinishTime[INDEX_MIN] = 0;
				Setting.SettingStruct.compareThreshold[INDEX_MAX] = 50;Setting.SettingStruct.compareThreshold[INDEX_MIN] = -50;
				Setting.SettingStruct.qualifiedRate[INDEX_MAX] = 100;Setting.SettingStruct.qualifiedRate[INDEX_MIN] = 0;
				
				Setting.SettingStruct.diameter[INDEX_SIZE] = 3; 
				Setting.SettingStruct.slotNumber[INDEX_SIZE] = 3; 
				Setting.SettingStruct.micaWidth[INDEX_SIZE] = 3; 
				Setting.SettingStruct.micaPreTrace[INDEX_SIZE] = 1; 
				Setting.SettingStruct.motoCompensation[INDEX_SIZE] = 1; 
				Setting.SettingStruct.millingMethod[INDEX_SIZE] = 1; 
				Setting.SettingStruct.motoDirection[INDEX_SIZE] = 1; 
				Setting.SettingStruct.feedDelayTime[INDEX_SIZE] = 3; 
				Setting.SettingStruct.feedActionTime[INDEX_SIZE] = 1; 
				Setting.SettingStruct.productDeviation[INDEX_SIZE] = 1; 
				Setting.SettingStruct.feedPreCutTime[INDEX_SIZE] = 3; 
				Setting.SettingStruct.millingPreBackTime[INDEX_SIZE] = 3; 
				Setting.SettingStruct.plusNumberOfMoto[INDEX_SIZE] = 5; 
				Setting.SettingStruct.stepMotoSlopeTime[INDEX_SIZE] = 2;
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_SIZE] = 1; 
				Setting.SettingStruct.stepMotoFinishTime[INDEX_SIZE] = 4; 
				Setting.SettingStruct.compareThreshold[INDEX_SIZE] = 2; 
				Setting.SettingStruct.qualifiedRate[INDEX_SIZE] = 3; 
				STMFLASH_Write( FLASH_BASE+60*1024, (uint16_t *) &Setting, sizeof(Setting)/2 ); 
			}
	}
	
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
