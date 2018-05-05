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
uint8_t tempj=0;
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
						HT1621_dis_float(Setting.SettingStruct.qualifiedRate[INDEX_VALUE],3,Setting.SettingStruct.qualifiedRate[INDEX_SIZE],Setting.SettingStruct.qualifiedRate[INDEX_POINT]);
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
							HT1621_dis_num(5,funcNum);
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
							pause_flag=0;
						 slot_count=0;
						 HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
						 HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//	尾顶左移动(A5使能，A4释放)
						 if(wait_input(Footer_Sensor_End_GPIO_Port, Footer_Sensor_End_Pin, GPIO_PIN_RESET)){break;}//尾顶是否到达工件位置（A15）
						 
						 else
							{
								HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
								//暂停判断
								if(pause_flag == 1)
								{
									pause_flag =0;
									while(1)
									{ 
										xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
										if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
											{
												break;
											}
									}
								}
								//暂停判断结束		
								osDelay(Setting.SettingStruct.feedDelayTime[INDEX_VALUE]*10);//根据设定时间延时
							 switch(Setting.SettingStruct.millingMethod[INDEX_VALUE])
							 {
								 case 0:ledTaskStatus = STATUS_MOTO_RUN_MODE1;
									 break;
								 case 1:ledTaskStatus = STATUS_MOTO_RUN_MODE2;
									 break;
								 case 2:ledTaskStatus = STATUS_MOTO_RUN_MODE3;
									 break;
								 default:break;
							 }
						 }
						 
				break;
			case STATUS_MOTO_RUN_MODE1:			
						 width =0 ;
						 for(temp_i = 0 ;temp_i<3;temp_i++)
							{
								startMoto(65534,0,1);
								if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){break;};//是否检测云母(B12)								
								startMoto(65534,0,1);
								if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){break;};//是否检测铜片(B12)
								width += (65534-stopMoto());
								//暂停判断
								if(pause_flag == 1)
								{
									pause_flag =0;
									while(1)
									{ 
										xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
										if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
											{
												break;
											}
									}
								}
								//暂停判断结束		
							}
						 width = width/3;
						 if(check_pass(width)!=1)
							{
								stopMoto();
								ledTaskStatus = STATUS_MOTO_ERR;
							}							
							cut_next:
							//暂停判断
								if(pause_flag == 1)
								{
									pause_flag =0;
									while(1)
									{ 
										xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
										if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
											{
												break;
											}
									}
								}
								//暂停判断结束	
							slot_count++;
							startMoto(65534,0,1);//步进电机运行(B0)
							if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){break;};//是否检测yunmu(B12)
							width = (65534-stopMoto());
								//云母是否合格
							if(check_pass(width)==1)
								{
									//是
									//往回走一半脉冲
									startMoto(width/2,0,0);//步进电机运行(B0)
									HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
									HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_SET);//铣刀气缸右移切割（B10使能，B1释放）
									if(wait_input(Cutter_Sensor_End_GPIO_Port, Cutter_Sensor_End_Pin, GPIO_PIN_RESET)){break;};//铣刀是否到右端（A10）
									//是否是最后一条云母
									HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);
									if(slot_count == Setting.SettingStruct.slotNumber[INDEX_VALUE])
											{//是
													ledTaskStatus = STATUS_MOTO_OUT;
											}
											else{
												//不是
												HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
												HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//铣刀气缸左移动(B10释放，B1使能)
												if(wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET)){break;};//是否到达中（A9）
												HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
												goto cut_next;
											}
								}
								else{
											//不合格
											stopMoto();
											ledTaskStatus = STATUS_MOTO_ERR;
									}											
				break;
							
			case STATUS_MOTO_RUN_MODE2:
							wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET);//是否检测云母(B12)
							startMoto(65534,0,1);//步进电机运行(B0)
							wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET);//是否检测铜片(B12)
							width = (65534-stopMoto());
											//暂停判断
								if(pause_flag == 1)
								{
									pause_flag =0;
									while(1)
									{ 
										xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
										if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
											{
												break;
											}
									}
								}
								//暂停判断结束		
								//云母是否合格
							if( check_pass(width)==1)
								{
									//往回走一半脉冲
				
									startMoto(width/2,0,0);//步进电机运行(B0
									slot_count =0;
									while(1)
									{
										 //暂停判断
										if(pause_flag == 1)
										{
											pause_flag =0;
											while(1)
												{ 
													xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
												if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
													{
														break;
													}
											}
										}
										//暂停判断结束		
										slot_count++;
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_SET);//铣刀气缸右移切割（B10使能，B1释放）
										if(wait_input(Cutter_Sensor_End_GPIO_Port, Cutter_Sensor_End_Pin, GPIO_PIN_RESET)){break;};//铣刀是否到右端（A10）
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);
										if(slot_count>=(Setting.SettingStruct.slotNumber[INDEX_VALUE]))
										{
											ledTaskStatus = STATUS_MOTO_OUT;
											break;
										}
										runMoto(Setting.SettingStruct.motoCompensation[INDEX_VALUE]);
											//不是
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//铣刀气缸左移动(B10释放，B1使能)
										if(wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET)){break;};//是否到达中（A9）			
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
									}
									
								}
							else
								{	//不合格
									stopMoto();
									ledTaskStatus = STATUS_MOTO_ERR;
									}		
						
				break;
			case STATUS_MOTO_RUN_MODE3:
									while(1)
									{
											//暂停判断
											if(pause_flag == 1)
											{
												pause_flag =0;
												while(1)
												{ 
													xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
													if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
														{
															break;
														}
												}
											}
											//暂停判断结束									
										slot_count++;
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_SET);//铣刀气缸右移切割（B10使能，B1释放）
										if(wait_input(Cutter_Sensor_End_GPIO_Port, Cutter_Sensor_End_Pin, GPIO_PIN_RESET)){break;}//铣刀是否到右端（A10）
										
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);
										if(slot_count>=(Setting.SettingStruct.slotNumber[INDEX_VALUE]))
										{
											ledTaskStatus = STATUS_MOTO_OUT;
											break;
										}
										runMoto(Setting.SettingStruct.motoCompensation[INDEX_VALUE]);
											//不是
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//铣刀气缸左移动(B10释放，B1使能)
										if(wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET)){break;};//是否到达中（A9）
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);										
									}
				break;
																					
			case STATUS_MOTO_OUT:
							HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);	//铣刀气缸左移动(B10释放，B1使能)
							if(wait_input(Cutter_Sensor_Start_GPIO_Port, Cutter_Sensor_Start_Pin, GPIO_PIN_RESET)){break;};//铣刀左移是否到位（A8）
							HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
							runMoto(Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]*3);//步进电机正转3圈
							HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//尾顶退（A4使能，A5释放）
							if(wait_input(Footer_Sensor_Start_GPIO_Port, Footer_Sensor_Start_Pin, GPIO_PIN_RESET)){break;};//尾顶是否退出到右边（A11）
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_SET);	//退料气缸右移（A0使能）
							if(wait_input(Material_R_Sensor_End_GPIO_Port, Material_R_Sensor_End_Pin, GPIO_PIN_RESET)){break;};//退料气缸是否到达右边（B4）
							HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_RESET);//退料气缸左移（A0释放）
							if(wait_input(Material_R_Sensor_Start_GPIO_Port, Material_R_Sensor_Start_Pin, GPIO_PIN_RESET))
							{break;}//退料是否完成（B3）
							else
							{
								ledTaskStatus = STATUS_MOTO_INIT;
							}
							if(wait_input(Key_Start_GPIO_Port, Key_Start_Pin,GPIO_PIN_RESET)){break;};//外部启动开关是否闭合 (B5)										
					break;
			case STATUS_MOTO_ERR:
							//停机并显示E1
							clear();
							HT1621_dis_num(1,0x0e);HT1621_dis_num(2,1);
							wait_input(Key_Stop_GPIO_Port, Key_Stop_Pin, GPIO_PIN_RESET);//是否按停止按键（B6）
							HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//尾顶退（A4使能，A5释放）
							if(wait_input(Footer_Sensor_Start_GPIO_Port, Footer_Sensor_Start_Pin, GPIO_PIN_RESET)){break;};//尾顶是否到右边（A11）
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
							HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE],0,Setting.SettingStruct.slotNumber[INDEX_SIZE],Setting.SettingStruct.slotNumber[INDEX_POINT]);
							HT1621_dis_float(Setting.SettingStruct.qualifiedRate[INDEX_VALUE],3,Setting.SettingStruct.qualifiedRate[INDEX_SIZE],Setting.SettingStruct.qualifiedRate[INDEX_POINT]);
							display();
							ledTaskStatus = STATUS_MOTO_INIT;
					break;
			case STATUS_MOTO_TEST://扫描云母
				  slot_count = Setting.SettingStruct.slotNumber[INDEX_VALUE];
					mica=0;cu=0;
//			
//			while(1)
//			{
 
//					
//					xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );	
//							if(keyGet.status == KEY_UP)
//											{
//												
//												startMoto(100,50,1);
//											}
//			}
//			
				
					while(slot_count>0)//一圈
							{
									slot_count--;

									//找上升沿
										startMoto(UINT32_MAX-1,cu,1);
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
															goto fuckass;
														}
														else if(keyGet.key == Key_FUN && keyGet.status == KEY_DOWN)
														{
															continueMoto();
														}
												}
												
											}
										}
									}
									cu = UINT32_MAX-1-stopMoto();
																		//开启次while 每次停下后等待按键
									while(1)
									{
										if(xQueueReceive( led_key_queue,&keyGet,100 )==pdPASS)
										{
											if(keyGet.status == KEY_UP)
													break;
										}
									}
									//找下降沿
									startMoto(UINT32_MAX-1,mica,1);
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
																		goto fuckass;
																	}
																	else if(keyGet.key == Key_FUN && keyGet.status == KEY_DOWN)
																	{
																		continueMoto();
																	}
												}
											}
										}
									}
									mica = UINT32_MAX-1-stopMoto();
																		//开启次while 每次停下后等待按键
									while(1)
									{
										if(xQueueReceive( led_key_queue,&keyGet,100 )==pdPASS)
										{
											if(keyGet.status == KEY_UP)
													break;
										}
									}
									//往回走一半mica
									startMoto(mica/2,0,0);
									while(get_moto_pluse())
									{
										osDelay(1);
									}
									osDelay(300);//停300ms
									//开启次while 每次停下后等待按键
									while(1)
									{
										if(xQueueReceive( led_key_queue,&keyGet,100 )==pdPASS)
										{
											if(keyGet.status == KEY_UP)
													break;
										}
									}
									startMoto(UINT32_MAX-1,cu,1);
									while(HAL_GPIO_ReadPin(CCD_Input_GPIO_Port,CCD_Input_Pin)!=GPIO_PIN_RESET);
									motoTestPluse++;
									HT1621_dis_float((uint32_t)absi(mica-mica_last) *100.0/(mica),3,3,0);
									HT1621_dis_float(slot_count,0,3,0);
									display();
									mica_last = mica;
								} 
				fuckass:		
								ledTaskStatus = STATUS_LED_INIT;
								stopMoto();
				break;
			case STATUS_MOTO_TEST2://一直运行
			
//					startMoto(Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]);//步进电机正转3圈
//					while(1)
//					{
//					xSemaphoreTake( Key_KeyTask_BinaryHandle,pdMS_TO_TICKS(1));
//										if(xQueueReceive( led_key_queue,&keyGet,0 )==pdPASS)
//										{
//											if(keyGet.key == Key_STOP && keyGet.status == KEY_UP)
//											{
//												stopMoto();
//												if(xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY )==pdPASS)
//												{
//														if(keyGet.key == Key_STOP && keyGet.status == KEY_DOWN)
//																	{		
//																		ledTaskStatus = STATUS_LED_INIT;
//																		break;
//																	}
//																	else if(keyGet.key == Key_FUN && keyGet.status == KEY_DOWN)
//																	{
//																		continueMoto();
//																	}
//												}
//											}
//										}
//														
//					}
				
															
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
	float widht_ratio =  Setting.SettingStruct.qualifiedRate[INDEX_VALUE]/100.0;
	uint16_t widht_set = Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]/Setting.SettingStruct.slotNumber[INDEX_VALUE];
	if((width > widht_set*widht_ratio) && (width < widht_set/widht_ratio))
		return 1;
	else
		return 0;
	
}
//等待外部输入信号 如果是停机则返回1 如果是暂停则返回2 等待成功返回0
uint8_t wait_input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	while(1)
	{
		
		if(xSemaphoreTake( Key_Sensor_BinaryHandle, pdMS_TO_TICKS(100))==pdPASS)
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
				Setting.SettingStruct.diameter[INDEX_VALUE] = 3;Setting.SettingStruct.diameter[INDEX_POINT] = 0;
				Setting.SettingStruct.slotNumber[INDEX_VALUE] = 24;Setting.SettingStruct.slotNumber[INDEX_POINT] = 0;
				Setting.SettingStruct.micaWidth[INDEX_VALUE] = 15;Setting.SettingStruct.micaWidth[INDEX_POINT] = 2;
				Setting.SettingStruct.micaPreTrace[INDEX_VALUE] = 0;Setting.SettingStruct.micaPreTrace[INDEX_POINT] = 0;
				Setting.SettingStruct.motoCompensation[INDEX_VALUE] = 200;Setting.SettingStruct.motoCompensation[INDEX_POINT] = 0;
				Setting.SettingStruct.millingMethod[INDEX_VALUE] = 0;Setting.SettingStruct.millingMethod[INDEX_POINT] = 0;
				Setting.SettingStruct.motoDirection[INDEX_VALUE] = 0;Setting.SettingStruct.motoDirection[INDEX_POINT] = 0;
				Setting.SettingStruct.feedDelayTime[INDEX_VALUE] = 1;Setting.SettingStruct.feedDelayTime[INDEX_POINT] = 2;
				Setting.SettingStruct.feedActionTime[INDEX_VALUE] = 0;Setting.SettingStruct.feedActionTime[INDEX_POINT] = 0;
				Setting.SettingStruct.productDeviation[INDEX_VALUE] = 0;Setting.SettingStruct.productDeviation[INDEX_POINT] = 0;
				Setting.SettingStruct.feedPreCutTime[INDEX_VALUE] = 1;Setting.SettingStruct.feedPreCutTime[INDEX_POINT] = 2;
				Setting.SettingStruct.millingPreBackTime[INDEX_VALUE] = 1;Setting.SettingStruct.millingPreBackTime[INDEX_POINT] = 2;
				Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE] = 4000;Setting.SettingStruct.plusNumberOfMoto[INDEX_POINT] = 0;
				Setting.SettingStruct.stepMotoSlopeTime[INDEX_VALUE] = 16;Setting.SettingStruct.stepMotoSlopeTime[INDEX_POINT] = 2;
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_VALUE] = 1000;Setting.SettingStruct.stepMotoRunSpeed[INDEX_POINT] = 0;
				Setting.SettingStruct.stepMotoFinishTime[INDEX_VALUE] = 0;Setting.SettingStruct.stepMotoFinishTime[INDEX_POINT] = 0;
				Setting.SettingStruct.compareThreshold[INDEX_VALUE] = 50;Setting.SettingStruct.compareThreshold[INDEX_POINT] = 1;
				Setting.SettingStruct.qualifiedRate[INDEX_VALUE] = 60;Setting.SettingStruct.qualifiedRate[INDEX_POINT] = 0;
				
				Setting.SettingStruct.diameter[INDEX_MAX] = 200;Setting.SettingStruct.diameter[INDEX_MIN] = 3;
				Setting.SettingStruct.slotNumber[INDEX_MAX] = 256;Setting.SettingStruct.slotNumber[INDEX_MIN] = 3;
				Setting.SettingStruct.micaWidth[INDEX_MAX] = 800;Setting.SettingStruct.micaWidth[INDEX_MIN] = 15;
				Setting.SettingStruct.micaPreTrace[INDEX_MAX] = 0;Setting.SettingStruct.micaPreTrace[INDEX_MIN] = 0;
				Setting.SettingStruct.motoCompensation[INDEX_MAX] = 6553;Setting.SettingStruct.motoCompensation[INDEX_MIN] = 0;
				Setting.SettingStruct.millingMethod[INDEX_MAX] = 2;Setting.SettingStruct.millingMethod[INDEX_MIN] = 0;
				Setting.SettingStruct.motoDirection[INDEX_MAX] = 1;Setting.SettingStruct.motoDirection[INDEX_MIN] = 0;
				Setting.SettingStruct.feedDelayTime[INDEX_MAX] = 900;Setting.SettingStruct.feedDelayTime[INDEX_MIN] = 0;
				Setting.SettingStruct.feedActionTime[INDEX_MAX] = 0;Setting.SettingStruct.feedActionTime[INDEX_MIN] = 0;
				Setting.SettingStruct.productDeviation[INDEX_MAX] = 1;Setting.SettingStruct.productDeviation[INDEX_MIN] = 0;
				Setting.SettingStruct.feedPreCutTime[INDEX_MAX] = 900;Setting.SettingStruct.feedPreCutTime[INDEX_MIN] = 1;
				Setting.SettingStruct.millingPreBackTime[INDEX_MAX] = 900;Setting.SettingStruct.millingPreBackTime[INDEX_MIN] = 1;
				Setting.SettingStruct.plusNumberOfMoto[INDEX_MAX] = 65535;Setting.SettingStruct.plusNumberOfMoto[INDEX_MIN] = 1000;
				Setting.SettingStruct.stepMotoSlopeTime[INDEX_MAX] = 10000;Setting.SettingStruct.stepMotoSlopeTime[INDEX_MIN] = 16;
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_MAX] = 9999;Setting.SettingStruct.stepMotoRunSpeed[INDEX_MIN] = 10;
				Setting.SettingStruct.stepMotoFinishTime[INDEX_MAX] = 9999;Setting.SettingStruct.stepMotoFinishTime[INDEX_MIN] = 0;
				Setting.SettingStruct.compareThreshold[INDEX_MAX] = 100;Setting.SettingStruct.compareThreshold[INDEX_MIN] = 0;
				Setting.SettingStruct.qualifiedRate[INDEX_MAX] = 100;Setting.SettingStruct.qualifiedRate[INDEX_MIN] = 0;
				
				Setting.SettingStruct.diameter[INDEX_SIZE] = 3; 
				Setting.SettingStruct.slotNumber[INDEX_SIZE] = 3; 
				Setting.SettingStruct.micaWidth[INDEX_SIZE] = 3; 
				Setting.SettingStruct.micaPreTrace[INDEX_SIZE] = 1; 
				Setting.SettingStruct.motoCompensation[INDEX_SIZE] = 4; 
				Setting.SettingStruct.millingMethod[INDEX_SIZE] = 1; 
				Setting.SettingStruct.motoDirection[INDEX_SIZE] = 1; 
				Setting.SettingStruct.feedDelayTime[INDEX_SIZE] = 3; 
				Setting.SettingStruct.feedActionTime[INDEX_SIZE] = 1; 
				Setting.SettingStruct.productDeviation[INDEX_SIZE] = 1; 
				Setting.SettingStruct.feedPreCutTime[INDEX_SIZE] = 3; 
				Setting.SettingStruct.millingPreBackTime[INDEX_SIZE] = 3; 
				Setting.SettingStruct.plusNumberOfMoto[INDEX_SIZE] = 5; 
				Setting.SettingStruct.stepMotoSlopeTime[INDEX_SIZE] = 3	;
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_SIZE] = 4; 
				Setting.SettingStruct.stepMotoFinishTime[INDEX_SIZE] = 4; 
				Setting.SettingStruct.compareThreshold[INDEX_SIZE] = 2; 
				Setting.SettingStruct.qualifiedRate[INDEX_SIZE] = 3; 
				STMFLASH_Write( FLASH_BASE+60*1024, (uint16_t *) &Setting, sizeof(Setting)/2 ); 
			}
			para_to_set();
	}
	
/* USER CODE END Application */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
