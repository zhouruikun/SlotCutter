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
#include "gpio.h"
#include "stepmoto.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId LEDTaskHandle;
osThreadId keyTaskHandle;
osThreadId stepMotoTaskHandle;
osMutexId CCD_MutexHandle;
osSemaphoreId CCD_BinaryHandle;
osSemaphoreId Key_Sensor_BinaryHandle;
osSemaphoreId Key_KeyTask_BinaryHandle;

/* USER CODE BEGIN Variables */
osThreadId keyTaskHandle;
uint8_t moto_msg[2];
uint16_t motoTestPluse=0;
KEY_MSG_t keyGet ;
QueueHandle_t led_key_queue; 
QueueHandle_t led_moto_queue;
uint8_t ledTaskStatus = STATUS_LED_INIT;
uint16_t mica = 0,cu = 0,mica_last=0;
uint32_t pluse_last;
uint8_t tempj=0,step;
uint8_t pause_flag;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartLEDTask(void const * argument);
void StartKeyTask(void const * argument);
void StartStepMotoTask(void const * argument);
void startMoto(uint32_t Pluse,uint32_t slope , uint8_t dir);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void para_to_set(void);
void arg_read(void);
	//增加一个数字位
	uint16_t decNumBit(uint16_t num,uint8_t bit);
	uint16_t incNumBit(uint16_t num,uint8_t bit);
	uint8_t wait_input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
	uint8_t check_pass(uint16_t width);
	uint16_t absi(int16_t i);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of CCD_Mutex */
  osMutexDef(CCD_Mutex);
  CCD_MutexHandle = osMutexCreate(osMutex(CCD_Mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of CCD_Binary */
  osSemaphoreDef(CCD_Binary);
  CCD_BinaryHandle = osSemaphoreCreate(osSemaphore(CCD_Binary), 1);

  /* definition and creation of Key_Sensor_Binary */
  osSemaphoreDef(Key_Sensor_Binary);
  Key_Sensor_BinaryHandle = osSemaphoreCreate(osSemaphore(Key_Sensor_Binary), 1);

  /* definition and creation of Key_KeyTask_Binary */
  osSemaphoreDef(Key_KeyTask_Binary);
  Key_KeyTask_BinaryHandle = osSemaphoreCreate(osSemaphore(Key_KeyTask_Binary), 1);

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
uint32_t get_moto_pluse(void);
/* StartLEDTask function */
void StartLEDTask(void const * argument)
{

  /* USER CODE BEGIN StartLEDTask */
	
  /* Infinite loop */

	uint8_t funcNum = 0;
	uint8_t showIndex = 0;
	uint8_t lightFlag = 0;
	uint8_t slot_count=0;
	uint8_t temp_i = 0;
	uint32_t width = 0;

	lint_val_display();
	arg_read();
	led_key_queue= xQueueCreate(4, sizeof(KEY_MSG_t) );
	led_moto_queue= xQueueCreate(4, 2 );
	
  for(;;)
  {
		switch(ledTaskStatus)
		{
			
			case STATUS_LED_INIT : 
						clear();
						HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE],0,Setting.SettingStruct.slotNumber[INDEX_SIZE],Setting.SettingStruct.slotNumber[INDEX_POINT]);
						HT1621_dis_float(Setting.SettingStruct.percentOfPassPreCut[INDEX_VALUE],3,Setting.SettingStruct.percentOfPassPreCut[INDEX_SIZE],Setting.SettingStruct.percentOfPassPreCut[INDEX_POINT]);
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
							if(keyGet.key == Key_OUT_START && keyGet.status == KEY_UP)
							{
								 ledTaskStatus = STATUS_MOTO_INIT;
							}
							if(keyGet.key == Key_START && keyGet.status == KEY_UP)
							{
								//测试模式
								 HT1621_dis_point(5,1);	display();
								 while(1)
								 {
									 xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY);
									 if(keyGet.key == Key_R && keyGet.status == KEY_UP)
										{
											//进入测试模式2
											ledTaskStatus = STATUS_MOTO_TEST2;
											clear();
											startMoto(UINT32_MAX,0,1);
											break;
										}
										if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
										{
											//进入测试模式1
											ledTaskStatus = STATUS_MOTO_TEST;
											startMoto(UINT32_MAX,0,1);
											break;
										}
										else if(keyGet.key == Key_START && keyGet.status == KEY_UP)
										{
											HT1621_dis_point(5,0);display();
											break;
										}
								 }
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
													STMFLASH_Write( FLASH_BASE+60*1024, (uint16_t *) &Setting, sizeof(Setting)/2 ); 
													para_to_set();
												break;
									}
								}
								
							}
							clear();
							HT1621_dis_num(5,funcNum+1);
							if(funcNum == 0x10)//电压比较是正负的
							{
								if(Setting.SettingStruct.compareThreshold[INDEX_VALUE]>=50)
								{
										HT1621_dis_num(0,17);
										HT1621_dis_float(Setting.SettingStruct.compareThreshold[INDEX_VALUE]-50,1,Setting.SettingStruct.compareThreshold[INDEX_SIZE],Setting.SettingStruct.compareThreshold[INDEX_POINT]);
								}
								else
								{
										HT1621_dis_num(0,18);
										HT1621_dis_float(50 - Setting.SettingStruct.compareThreshold[INDEX_VALUE],1,Setting.SettingStruct.compareThreshold[INDEX_SIZE],Setting.SettingStruct.compareThreshold[INDEX_POINT]);
								}
							}
							else
							{
									HT1621_dis_float(Setting.SettingArray[funcNum][INDEX_VALUE],0,Setting.SettingArray[funcNum][INDEX_SIZE],Setting.SettingArray[funcNum][INDEX_POINT]);
							}
							 
							if(lightFlag == 0)
							{
								if(funcNum == 0x10)
									HT1621_dis_num(showIndex+1,17);
								else
									HT1621_dis_num(showIndex,17);
								lightFlag = 1;
							}
							else
							{
								lightFlag = 0;
							}
							display();
				break;	
			case STATUS_MOTO_INIT:
							motoInit();
				break;
			case STATUS_MOTO_RUN_MODE1:			
						
				break;
							
			case STATUS_MOTO_RUN_MODE2:
						
				break;
			case STATUS_MOTO_RUN_MODE3:
									
				break;
																					
			case STATUS_MOTO_OUT:
						runBack();				
					break;
			case STATUS_MOTO_ERR:
							motoErr();
					break;
			case STATUS_MOTO_TEST://扫描云母
							modeTest();
				  		
				break;
			case STATUS_MOTO_TEST2://一直运行				
						mica=0;cu=0;
			fuck:	while(ledTaskStatus == STATUS_MOTO_TEST2)
						{
							pluse_last = UINT32_MAX-1;
							startMoto(UINT32_MAX-1,0,1);
							while((UINT32_MAX-1-get_moto_pluse())<=Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE])//一圈
								{
									//找下降沿
									while(HAL_GPIO_ReadPin(CCD_Input_GPIO_Port,CCD_Input_Pin)!=GPIO_PIN_RESET)
									{
										xSemaphoreTake( Key_KeyTask_BinaryHandle,pdMS_TO_TICKS(1));
										if(xQueueReceive( led_key_queue,&keyGet,0 )==pdPASS)
										{
											if(keyGet.key == Key_STOP && keyGet.status == KEY_UP)
											{
												stopMoto();
												if(xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY )==pdPASS)
												{
														if(keyGet.key == Key_STOP && keyGet.status == KEY_DOWN)
																	{		
																		ledTaskStatus = STATUS_LED_INIT;
																		goto fuck;
																	}
																	else if(keyGet.key == Key_FUN && keyGet.status == KEY_DOWN)
																	{
																		continueMoto();
																	}
												}
											}
										}
									}
									cu = pluse_last-get_moto_pluse();
									pluse_last = get_moto_pluse();
									//找上升沿
									while(HAL_GPIO_ReadPin(CCD_Input_GPIO_Port,CCD_Input_Pin)!=GPIO_PIN_SET)
									{
										xSemaphoreTake( Key_KeyTask_BinaryHandle,pdMS_TO_TICKS(1));
										if(xQueueReceive( led_key_queue,&keyGet,0 )==pdPASS)
										{
											if(keyGet.key == Key_STOP && keyGet.status == KEY_UP)
											{
												stopMoto();
												if(xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY )==pdPASS)
												{
													if(keyGet.key == Key_STOP && keyGet.status == KEY_DOWN)
														{		
															ledTaskStatus = STATUS_LED_INIT;
															goto fuck;
														}
														else if(keyGet.key == Key_FUN && keyGet.status == KEY_DOWN)
														{
															continueMoto();
														}
												}
											}
										}
									}
									mica = pluse_last-get_moto_pluse();
									pluse_last = get_moto_pluse();
									motoTestPluse++;
									if(pluse_last>0)
										HT1621_dis_float((uint32_t)mica*100.0/(cu+mica),3,3,0);
									display();
								}
								HT1621_dis_float(motoTestPluse,0,3,0);
 								motoTestPluse=0;
						}
						break;						
			case STATUS_MOTO_STOP:
						stopMoto();
						slot_count=0;
						ledTaskStatus = STATUS_LED_INIT;
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
		xSemaphoreTake( Key_Sensor_BinaryHandle ,pdMS_TO_TICKS(50));
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
		//1ms计算一次速度偏执
	//	speedCal();
		osDelay(1);
   }
  /* USER CODE END StartStepMotoTask */
}

/* USER CODE BEGIN Application */
 
uint16_t absi(int16_t i)
{
	return i>0?i:-i;
}
//输入槽间距脉冲数 然后根据槽数和一圈脉冲数判断
uint8_t check_pass(uint16_t width)
{
	return 1;
	float widht_ratio =  Setting.SettingStruct.percentOfPassPreCut[INDEX_VALUE]/100.0;
	uint16_t widht_set = Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]/Setting.SettingStruct.slotNumber[INDEX_VALUE];
	if((width > widht_set*widht_ratio) && (width < widht_set/widht_ratio))
		return 1;
	else
		return 0;
	
}//等待外部输入信号  等待成功返回1 等待10ms
uint8_t wait_input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)

{
		if(xSemaphoreTake( Key_Sensor_BinaryHandle, pdMS_TO_TICKS(10))==pdPASS)
		{
			if(HAL_GPIO_ReadPin(Key_Stop_GPIO_Port,Key_Stop_Pin)==GPIO_PIN_RESET)
				{
					ledTaskStatus = STATUS_MOTO_STOP;
					return 1;
				}	
			if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==PinState)
				{
					return 0;
				}	

	}
}

//设置参数到外设
void para_to_set(void)
{
	//速度
	setMotoSpeed(Setting.SettingStruct.stepMotoRunSpeed[INDEX_VALUE]);
	//比较电压
	X9C103_Set(Setting.SettingStruct.compareThreshold[INDEX_VALUE]-50);
}

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
 
				Setting.SettingStruct.slotNumber[INDEX_VALUE] = 24;Setting.SettingStruct.slotNumber[INDEX_POINT] = 0;
				Setting.SettingStruct.micaWidth[INDEX_VALUE] = 15;Setting.SettingStruct.micaWidth[INDEX_POINT] = 2;
				Setting.SettingStruct.percentOfPassPreCut[INDEX_VALUE] = 100;Setting.SettingStruct.percentOfPassPreCut[INDEX_POINT] = 0;
				Setting.SettingStruct.motoCompensation[INDEX_VALUE] = 200;Setting.SettingStruct.motoCompensation[INDEX_POINT] = 0;
				Setting.SettingStruct.millingMethod[INDEX_VALUE] = 0;Setting.SettingStruct.millingMethod[INDEX_POINT] = 0;
				Setting.SettingStruct.motoDirection[INDEX_VALUE] = 0;Setting.SettingStruct.motoDirection[INDEX_POINT] = 0;
				Setting.SettingStruct.feedOnTime[INDEX_VALUE] = 1;Setting.SettingStruct.feedOnTime[INDEX_POINT] = 2;
				Setting.SettingStruct.percentOfPassOnCut[INDEX_VALUE] = 150;Setting.SettingStruct.percentOfPassOnCut[INDEX_POINT] = 0;
				Setting.SettingStruct.footerOnTime[INDEX_VALUE] = 1;Setting.SettingStruct.footerOnTime[INDEX_POINT] = 2;
				Setting.SettingStruct.feedOffTime[INDEX_VALUE] = 1;Setting.SettingStruct.feedOffTime[INDEX_POINT] = 2;
				Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE] = 4000;Setting.SettingStruct.plusNumberOfMoto[INDEX_POINT] = 0;
				Setting.SettingStruct.mode0DirSwitchTime[INDEX_VALUE] = 1;Setting.SettingStruct.mode0DirSwitchTime[INDEX_POINT] = 2;
				Setting.SettingStruct.stepMotoInitSpeed[INDEX_VALUE] = 1000;Setting.SettingStruct.stepMotoInitSpeed[INDEX_POINT] = 0;
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_VALUE] = 2000;Setting.SettingStruct.stepMotoRunSpeed[INDEX_POINT] = 0;
				Setting.SettingStruct.compareThreshold[INDEX_VALUE] = 50;Setting.SettingStruct.compareThreshold[INDEX_POINT] = 1;
				Setting.SettingStruct.stepMotoFinishTime[INDEX_VALUE] = 1;Setting.SettingStruct.stepMotoFinishTime[INDEX_POINT] = 0;
				Setting.SettingStruct.micaPreTrace[INDEX_VALUE] = 1;Setting.SettingStruct.micaPreTrace[INDEX_POINT] = 0;
				
 
				Setting.SettingStruct.slotNumber[INDEX_MAX] = 256;Setting.SettingStruct.slotNumber[INDEX_MIN] = 3;
				Setting.SettingStruct.micaWidth[INDEX_MAX] = 800;Setting.SettingStruct.micaWidth[INDEX_MIN] = 15;
				Setting.SettingStruct.percentOfPassPreCut[INDEX_MAX] = 200;Setting.SettingStruct.percentOfPassPreCut[INDEX_MIN] = 0;
				Setting.SettingStruct.motoCompensation[INDEX_MAX] = 200;Setting.SettingStruct.motoCompensation[INDEX_MIN] = 1;
				Setting.SettingStruct.millingMethod[INDEX_MAX] = 3;Setting.SettingStruct.millingMethod[INDEX_MIN] = 0;
				Setting.SettingStruct.motoDirection[INDEX_MAX] = 1;Setting.SettingStruct.motoDirection[INDEX_MIN] = 0;
				Setting.SettingStruct.feedOnTime[INDEX_MAX] = 900;Setting.SettingStruct.feedOnTime[INDEX_MIN] = 1;
				Setting.SettingStruct.percentOfPassOnCut[INDEX_MAX] = 200;Setting.SettingStruct.percentOfPassOnCut[INDEX_MIN] = 0;
				Setting.SettingStruct.footerOnTime[INDEX_MAX] = 900;Setting.SettingStruct.footerOnTime[INDEX_MIN] = 1;
				Setting.SettingStruct.feedOffTime[INDEX_MAX] = 900;Setting.SettingStruct.feedOffTime[INDEX_MIN] = 1;
				Setting.SettingStruct.plusNumberOfMoto[INDEX_MAX] = 65000;Setting.SettingStruct.plusNumberOfMoto[INDEX_MIN] = 1000;
				Setting.SettingStruct.mode0DirSwitchTime[INDEX_MAX] = 900;Setting.SettingStruct.mode0DirSwitchTime[INDEX_MIN] = 1;
				Setting.SettingStruct.stepMotoInitSpeed[INDEX_MAX] = 65000;Setting.SettingStruct.stepMotoInitSpeed[INDEX_MIN] = 100;
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_MAX] = 65000;Setting.SettingStruct.stepMotoRunSpeed[INDEX_MIN] = 1000;
				Setting.SettingStruct.compareThreshold[INDEX_MAX] = 100;Setting.SettingStruct.compareThreshold[INDEX_MIN] = 0;
				Setting.SettingStruct.stepMotoFinishTime[INDEX_MAX] = 65000;Setting.SettingStruct.stepMotoFinishTime[INDEX_MIN] = 0;
				Setting.SettingStruct.micaPreTrace[INDEX_MAX] = Setting.SettingStruct.micaWidth[INDEX_VALUE];Setting.SettingStruct.micaPreTrace[INDEX_MIN] = 0;
				
	 
				Setting.SettingStruct.slotNumber[INDEX_SIZE] = 3; 
				Setting.SettingStruct.micaWidth[INDEX_SIZE] = 3; 
				Setting.SettingStruct.percentOfPassPreCut[INDEX_SIZE] = 3; 
				Setting.SettingStruct.motoCompensation[INDEX_SIZE] = 5; 
				Setting.SettingStruct.millingMethod[INDEX_SIZE] = 1; 
				Setting.SettingStruct.motoDirection[INDEX_SIZE] = 1; 
				Setting.SettingStruct.feedOnTime[INDEX_SIZE] = 3; 
				Setting.SettingStruct.percentOfPassOnCut[INDEX_SIZE] = 3; 
				Setting.SettingStruct.footerOnTime[INDEX_SIZE] = 3; 
				Setting.SettingStruct.feedOffTime[INDEX_SIZE] = 43; 
				Setting.SettingStruct.plusNumberOfMoto[INDEX_SIZE] = 5; 
				Setting.SettingStruct.mode0DirSwitchTime[INDEX_SIZE] = 3; 
				Setting.SettingStruct.stepMotoInitSpeed[INDEX_SIZE] = 5; 
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_SIZE] = 5; 
				Setting.SettingStruct.compareThreshold[INDEX_SIZE] = 2; 
				Setting.SettingStruct.stepMotoFinishTime[INDEX_SIZE] = 4; 
				Setting.SettingStruct.micaPreTrace[INDEX_SIZE] = 3; 
				STMFLASH_Write( FLASH_BASE+60*1024, (uint16_t *) &Setting, sizeof(Setting)/2 ); 
			}
			para_to_set();
	}
	
/* USER CODE END Application */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
