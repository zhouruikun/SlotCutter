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
osThreadId backTask1Handle;
osThreadId backTask2Handle;
osThreadId keyTaskHandle;
uint8_t moto_msg[2];
uint16_t motoTestPluse=0;
KEY_MSG_t keyGet ;
QueueHandle_t led_key_queue; 
QueueHandle_t led_moto_queue;
uint8_t ledTaskStatus = STATUS_LED_INIT;
uint16_t mica = 0,cu = 0,mica_last=0;
uint32_t pluse_last;
uint8_t tempj=0,test_flag=0;
uint8_t pause_flag;
uint8_t backTask1Finish=0;
float per_pluse= 0.0;
uint8_t backTask2Finish=0;
uint8_t footer_flag=0,stop_flag=0;
uint8_t  feed_flag=0;
	uint8_t slot_count=0;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartLEDTask(void const * argument);
void StartKeyTask(void const * argument);
void StartStepMotoTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

void backTake1(void const * argument);
void backTake2(void const * argument);

void para_to_set(void);
void arg_read(void);
	//???????
	uint16_t decNumBit(uint16_t num,uint8_t bit);
	uint16_t incNumBit(uint16_t num,uint8_t bit);
	uint8_t wait_input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
	uint8_t check_pass_onCut(uint16_t width);
		uint8_t check_pass_preCut(uint16_t width);
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
  osThreadDef(keyTask, StartKeyTask, osPriorityIdle, 0, 32);
  keyTaskHandle = osThreadCreate(osThread(keyTask), NULL);

  /* definition and creation of stepMotoTask */
  osThreadDef(stepMotoTask, StartStepMotoTask, osPriorityIdle, 0, 32);
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

	uint8_t funcNum = 0;
	uint8_t showIndex = 0;
	uint8_t lightFlag = 0;

	uint16_t temp_i = 0,temp_j_feed=0,temp_j = 0;
	uint32_t width = 0;
  
	lint_val_display();
	arg_read();
	led_key_queue= xQueueCreate(4, sizeof(KEY_MSG_t) );
	led_moto_queue= xQueueCreate(4, 2 );
	
  for(;;)
  {
	status_start:	switch(ledTaskStatus)
		{
			
			case STATUS_LED_INIT : 
						clear();
						HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE],0,Setting.SettingStruct.slotNumber[INDEX_SIZE],Setting.SettingStruct.slotNumber[INDEX_POINT]);
						HT1621_dis_float(Setting.SettingStruct.percentOfPassOnCut[INDEX_VALUE],3,Setting.SettingStruct.percentOfPassOnCut[INDEX_SIZE],Setting.SettingStruct.percentOfPassOnCut[INDEX_POINT]);
						display();
						ledTaskStatus = STATUS_LED_IDLE;
				break;	
			case STATUS_LED_IDLE :
						osDelay(1);
					if(HAL_GPIO_ReadPin(Key_Start_GPIO_Port,Key_Start_Pin)==GPIO_PIN_RESET)
						{
							osDelay(10);
							if(HAL_GPIO_ReadPin(Key_Start_GPIO_Port,Key_Start_Pin)==GPIO_PIN_RESET)
							{
								 ledTaskStatus = STATUS_MOTO_INIT;test_flag = 0;
							}
						}
					if(xQueueReceive( led_key_queue,&keyGet,pdMS_TO_TICKS(1000) )==pdPASS)
						{
							if(keyGet.key == Key_START && keyGet.status == KEY_HOLD)
							{
								ledTaskStatus = STATUS_LED_MOD_PARA;
							}
							if(keyGet.key == Key_R && keyGet.status == KEY_HOLD)
							{
								ledTaskStatus = STATUS_FOOTER;
							}
							if(keyGet.key == Key_START && keyGet.status == KEY_UP)
							{
								//????
								 HT1621_dis_point(5,1);	display();
								 while(1)
								 {
									 xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY);
									 if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
										{
											//??????2
											test_flag = 1;
											 switch(Setting.SettingStruct.millingMethod[INDEX_VALUE])
											 {
												 case 0:ledTaskStatus = STATUS_MOTO_RUN_MODE0;
													 break;
												 case 1:ledTaskStatus = STATUS_MOTO_RUN_MODE1;
													 break;
												 case 2:ledTaskStatus = STATUS_MOTO_RUN_MODE2;
													 break;
												 case 3:ledTaskStatus = STATUS_MOTO_RUN_MODE3;
													 break;
												 default:break;
											 }									 
											clear();
											slot_count =0;
											pause_flag=0;
											break;
										}
										if(keyGet.key == Key_R && keyGet.status == KEY_UP)
										{
											//??????1
											slot_count =0;
											pause_flag=0;
											ledTaskStatus = STATUS_MOTO_TEST2;
											startMoto(INT32_MAX,0,1);
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
						
			case STATUS_FOOTER : 
				
			
						if(footer_flag==0)
						{
							footer_flag =1;
							HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//	?????(A5??,A4??)
							osDelay(Setting.SettingStruct.footerOnTime[INDEX_VALUE]*100);
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);					
						}
						else
						{
							footer_flag=0;
							HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);//	????(A4??,A5??)
							osDelay(Setting.SettingStruct.footerExitTime[INDEX_VALUE]*100);
							HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);			
						}
						ledTaskStatus = STATUS_LED_IDLE;
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
											if(funcNum>=20)
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
													Setting.SettingStruct.micaPreTrace[INDEX_MAX] = (Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]/Setting.SettingStruct.slotNumber[INDEX_VALUE])-
														Setting.SettingStruct.micaWidth[INDEX_VALUE];										
													STMFLASH_Write( FLASH_BASE+60*1024, (uint16_t *) &Setting, sizeof(Setting)/2 ); 
													para_to_set();
												break;
									}
								}
							}
							clear();
							HT1621_dis_num(5,funcNum);
							if(funcNum == 0x0f)//????????
							{
								if(Setting.SettingStruct.compareThreshold[INDEX_VALUE]>=50)
								{
										HT1621_dis_num(0,SMG_NONE);
										HT1621_dis_float(Setting.SettingStruct.compareThreshold[INDEX_VALUE]-50,1,Setting.SettingStruct.compareThreshold[INDEX_SIZE],Setting.SettingStruct.compareThreshold[INDEX_POINT]);
								}
								else
								{
										HT1621_dis_num(0,SMG_NEG);
										HT1621_dis_float(50 - Setting.SettingStruct.compareThreshold[INDEX_VALUE],1,Setting.SettingStruct.compareThreshold[INDEX_SIZE],Setting.SettingStruct.compareThreshold[INDEX_POINT]);
								}
							}
							else
							{
									HT1621_dis_float(Setting.SettingArray[funcNum][INDEX_VALUE],0,Setting.SettingArray[funcNum][INDEX_SIZE],Setting.SettingArray[funcNum][INDEX_POINT]);
							}
							if(lightFlag == 0)
							{
								if(funcNum == 0x0f)
									HT1621_dis_num(showIndex+1,SMG_NONE);
								else
									HT1621_dis_num(showIndex,SMG_NONE);
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
							stop_flag=0;
						HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);//(B10¨º¡ì?¨¹??B1¨º1?¨¹)
						if(footer_flag==0)
							{
							 footer_flag=1;
							 HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
							 HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//	?????(A5??,A4??)
							}
							osDelay(Setting.SettingStruct.footerOnTime[INDEX_VALUE]*100);
							{
							 
								//????
								if(pause_flag == 1)
								{
								  osDelay(100);
									while(1)
									{ xQueueReset(led_key_queue);
										xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
										if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
											{	pause_flag =0;
												break;
											}
									}
								}
								//??????		
								//osDelay(Setting.SettingStruct.feedDelayTime[INDEX_VALUE]*10);//????????
							 switch(Setting.SettingStruct.millingMethod[INDEX_VALUE])
							 {
								 case 0:ledTaskStatus = STATUS_MOTO_RUN_MODE0;
									 break;
								 case 1:ledTaskStatus = STATUS_MOTO_RUN_MODE1;
									 break;
								 case 2:ledTaskStatus = STATUS_MOTO_RUN_MODE2;
									 break;
								 case 3:ledTaskStatus = STATUS_MOTO_RUN_MODE3;
									 break;
								 default:break;
							 }
						 }
				break;
			case STATUS_MOTO_RUN_MODE0:
										width =0 ;
										mica=0;
										cu=0;
										startMoto(65534,0,1);
										if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){goto status_start;};//??????(B12)
										if(test_flag==0)
										{
										 for(temp_i = 0 ,slot_count=0;slot_count<Setting.SettingStruct.slotNumber[INDEX_VALUE];slot_count++)
											{
												startMoto(65534,0,1);
												if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){goto status_start;};//??????(B12)
												width = (65534-stopMoto());
												startMoto(65534,0,1);
												while(get_moto_pluse_has()<Setting.SettingStruct.micaPreTrace[INDEX_VALUE]){osDelay(1);};//??????
												if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){goto status_start;};//??????(B12)								

												 if(check_pass_preCut(width)!=1)
													{
														temp_i = 0;
													}
													else
													{
														temp_i++;
														if(temp_i>=3)
														{
																break;
														}
													}
												//????
												if(pause_flag == 1)
												{
													osDelay(100);
													while(1)
													{ xQueueReset(led_key_queue);
														xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
														if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
															{pause_flag =0;
																break;
															}
														
													}
												}
												//??????		
											}
											if(slot_count == Setting.SettingStruct.slotNumber[INDEX_VALUE])
											{
												stopMoto();
												ledTaskStatus = STATUS_MOTO_ERR;
											}
										 
										}			
											slot_count =0 ;										
											cut_nextmode4:
											//????
											if(pause_flag == 1)
												{
													osDelay(100);
													while(1)
													{ xQueueReset(led_key_queue);
														xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
														if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
															{
																pause_flag =0;
																break;
															}
														if(keyGet.key == Key_STOP && keyGet.status == KEY_DOWN&&test_flag==1)
														{
															goto stop;
														}	
													}
												}
												//??????	
						 
										
										HT1621_dis_float((uint32_t)absi(mica-mica_last) *100/(mica),3,3,0);mica_last = mica;	
												//??????
											if(check_pass_onCut(width)==1)
												{
													//?
													//????
												 	runMoto(Setting.SettingStruct.motoCompensation[INDEX_VALUE],0,1);//??????(B0)
													if(test_flag==0)
													{
														HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
														HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_SET);//????????(B10??,B1??)
														if(wait_input(Cutter_Sensor_End_GPIO_Port, Cutter_Sensor_End_Pin, GPIO_PIN_RESET)){goto status_start;};//???????(A10)
														HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
														HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//???????(B10??,B1??)
														if(wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET)){goto status_start;};//?????(A9)
														HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
													}
													else{
														HT1621_dis_point(5,1);	
														osDelay(300);
													}
													HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE]-slot_count,0,3,0);	display();		
													if(slot_count == Setting.SettingStruct.slotNumber[INDEX_VALUE])						//?????????
															{//?
																	ledTaskStatus = STATUS_MOTO_OUT;
															}
															else{

																//????

																startMoto(INT32_MAX-1,0,1);
																if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){goto status_start;};;//?????(B12)
																mica = INT32_MAX-1-stopMoto();	
																												
																//????
																	startMoto(INT32_MAX-1,cu,1);
																	while(get_moto_pluse_has()<((Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]/Setting.SettingStruct.slotNumber[INDEX_VALUE])-
																		Setting.SettingStruct.micaWidth[INDEX_VALUE]-Setting.SettingStruct.micaPreTrace[INDEX_VALUE])){osDelay(1);};//??????
																while(HAL_GPIO_ReadPin(CCD_Input_GPIO_Port,CCD_Input_Pin)!=GPIO_PIN_SET)
																{
																	osDelay(1);
																	if(pause_flag == 1)
																		{stopMoto();
																			osDelay(100);
																			while(1)
																			{ xQueueReset(led_key_queue);
																				xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
																				if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
																					{
																						pause_flag=0;
																						continueMoto();
																						break;
																					}
																				if(keyGet.key == Key_STOP && keyGet.status == KEY_DOWN&&test_flag==1)
																				{
																					goto stop;
																				}	
																			}
																		}
																}
																cu = INT32_MAX-1-stopMoto();
																width = mica+Setting.SettingStruct.motoCompensation[INDEX_VALUE];
																slot_count++;
																goto cut_nextmode4;
															}
												
												}
												else{
															//???
															stopMoto();
															ledTaskStatus = STATUS_MOTO_ERR;
													}											
									 break;
			case STATUS_MOTO_RUN_MODE1:			
									width =0 ;
									mica=0;
									cu=0;
									startMoto(65534,0,1);
									if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){goto status_start;};//??????(B12)
									if(test_flag==0)
										{
										 for(temp_i = 0 ,slot_count=0;slot_count<Setting.SettingStruct.slotNumber[INDEX_VALUE];slot_count++)
											{
												startMoto(65534,0,1);
												while(get_moto_pluse_has()<Setting.SettingStruct.micaPreTrace[INDEX_VALUE]){osDelay(1);};//??????		
												if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){goto status_start;};//??????(B12)								
												startMoto(65534,0,1);
												if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){goto status_start;};//??????(B12)
												width = (65534-stopMoto());
												 if(check_pass_preCut(width)!=1)
													{
														temp_i = 0;
													}
													else
													{
														temp_i++;
														if(temp_i>=3)
														{
																break;
														}
													}
												//????
												if(pause_flag == 1)
												{
													osDelay(100);
													while(1)
													{ xQueueReset(led_key_queue);
														xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
														if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
															{pause_flag =0;
																break;
															}
														
													}
												}
												//??????		
											}
											if(slot_count == Setting.SettingStruct.slotNumber[INDEX_VALUE])
											{
												stopMoto();
												ledTaskStatus = STATUS_MOTO_ERR;
											}
											else {
											slot_count=0;
											}
										 
										}						
							cut_next:
							//????
							if(pause_flag == 1)
								{
									osDelay(100);
									while(1)
									{ xQueueReset(led_key_queue);
										xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
										if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
											{
												pause_flag =0;
												break;
											}
											if(keyGet.key == Key_STOP && keyGet.status == KEY_UP&&test_flag==1)
														{
															goto stop;
														}				
									}
								}
								//??????	

						
						HT1621_dis_float((uint32_t)absi(mica-mica_last) *100/(mica),3,3,0);mica_last = mica;	
								//??????
							if(check_pass_onCut(width)==1)
								{
									//?
									//???????
									osDelay(Setting.SettingStruct.mode0DirSwitchTime[INDEX_VALUE]*10);
									runMoto(width/2,0,0);//??????(B0)
									if(test_flag==0)
									{
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_SET);//????????(B10??,B1??)
										if(wait_input(Cutter_Sensor_End_GPIO_Port, Cutter_Sensor_End_Pin, GPIO_PIN_RESET)){goto status_start;};//???????(A10)
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//???????(B10??,B1??)
										if(wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET)){goto status_start;};//?????(A9)
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);										
									}
									else{
										HT1621_dis_point(5,1);	
										osDelay(300);
									}
										
									if(slot_count == Setting.SettingStruct.slotNumber[INDEX_VALUE])										//?????????
											{//?
													ledTaskStatus = STATUS_MOTO_OUT;
											}
											else{
												//??

													startMoto(INT32_MAX-1,0,1);
													if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){goto status_start;};//??????(B12)
													//????
													startMoto(INT32_MAX-1,cu,1);
													while(get_moto_pluse_has()<((Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]/Setting.SettingStruct.slotNumber[INDEX_VALUE])-
														Setting.SettingStruct.micaWidth[INDEX_VALUE]-Setting.SettingStruct.micaPreTrace[INDEX_VALUE])){osDelay(1);};//??????
													if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){goto status_start;};//??????(B12)
													cu = INT32_MAX-1-stopMoto();
													
													//????
													startMoto(INT32_MAX-1,0,1);
													while(HAL_GPIO_ReadPin(CCD_Input_GPIO_Port,CCD_Input_Pin)!=GPIO_PIN_RESET)
													{
														osDelay(1);
																		if(pause_flag == 1)
																			{
																				pause_flag =0;
																				osDelay(100);
																				stopMoto();
																				while(1)
																				{ xQueueReset(led_key_queue);
																					xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
																					if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
																						{
																							pause_flag=0;
																							continueMoto();
																							break;
																						}
																							if(keyGet.key == Key_STOP && keyGet.status == KEY_DOWN&&test_flag==1)
																					{
																						goto stop;
																					}	
																				}
																			}
													}
													mica = INT32_MAX-1-stopMoto();	
														
													width = mica;
													HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE]-slot_count,0,3,0);	display();												
												slot_count++;
												goto cut_next;
											}
							
								}
								else{
											//???
											stopMoto();
											ledTaskStatus = STATUS_MOTO_ERR;
									}											
				break;
							
			case STATUS_MOTO_RUN_MODE3:
							startMoto(65534,0,1);//??????(B0)
							if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){goto status_start;};;//??????(B12)
							startMoto(65534,0,1);//??????(B0)
							while(get_moto_pluse_has()<Setting.SettingStruct.micaPreTrace[INDEX_VALUE]){osDelay(1);};//??????
							if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){goto status_start;};;//??????(B12)
							startMoto(65534,0,1);//??????(B0)
							if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){goto status_start;};;//??????(B12)
							width = (65534-stopMoto());
											//????
								if(pause_flag == 1)
								{
									osDelay(100);
									while(1)
									{ xQueueReset(led_key_queue);
										xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
										if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
											{pause_flag =0;
												break;
											}
									}
								}
								//??????		
								//??????
							if(check_pass_onCut(width)==1)
								{
									//????
									runMoto(Setting.SettingStruct.motoCompensation[INDEX_VALUE],0,1);//??????(B0)
									slot_count =0;
									per_pluse=(float)Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]/Setting.SettingStruct.slotNumber[INDEX_VALUE];
								  startMoto(Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE],0,1);
									stopMoto();
									while(get_moto_pluse())
									{
										 //????
										if(pause_flag == 1)
										{
											osDelay(100);
											while(1)
												{ xQueueReset(led_key_queue);
													xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
													 if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
													{
														pause_flag =0;
														break;
													}
													if(keyGet.key == Key_STOP && keyGet.status == KEY_DOWN)
														{
															goto stop;
														}
											}
										}
										//??????		

										if(test_flag==0)
										{
											HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
											HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_SET);//????????(B10??,B1??)
											if(wait_input(Cutter_Sensor_End_GPIO_Port, Cutter_Sensor_End_Pin, GPIO_PIN_RESET)){goto status_start;};//???????(A10)
											HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
											HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//???????(B10??,B1??)
											if(wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET)){goto status_start;};//?????(A9)			
											HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
										}
										else
										{
											HT1621_dis_point(5,1);	
											osDelay(300);
										}
										set_slope((uint32_t)per_pluse);
									  continueMoto();	
										slot_count++;
										while((Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]-get_moto_pluse())<per_pluse*slot_count)
										{
											if(get_moto_pluse()==0||slot_count == Setting.SettingStruct.slotNumber[INDEX_VALUE])
												break;
											osDelay(1);
										}
								
										stopMoto();
										HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE]-slot_count+1,0,3,0);	display();		
									}
									stopMoto();
									ledTaskStatus = STATUS_MOTO_OUT;
									
								}
							else
								{	//???
									stopMoto();
									ledTaskStatus = STATUS_MOTO_ERR;
									}		
				break;
			case STATUS_MOTO_RUN_MODE2:
									per_pluse=(float)Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]/Setting.SettingStruct.slotNumber[INDEX_VALUE];
								  startMoto(Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE],0,1);
									stopMoto();
									while(get_moto_pluse())
									{
											//????
											if(pause_flag == 1)
											{
												osDelay(100);
												while(1)
												{ xQueueReset(led_key_queue);
													xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY );
													
													if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
														{pause_flag =0;
															break;
														}
													if(keyGet.key == Key_STOP && keyGet.status == KEY_DOWN)
														{
															goto stop;
														}
												}
											}
											//??????									

										if(test_flag==0)
										{
											HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
											HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_SET);//????????(B10??,B1??)
											if(wait_input(Cutter_Sensor_End_GPIO_Port, Cutter_Sensor_End_Pin, GPIO_PIN_RESET)){goto status_start;}//???????(A10)
											HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
											HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//???????(B10??,B1??)
											if(wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET)){goto status_start;};//?????(A9)
											HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);		
										}
										else
										{
											HT1621_dis_point(5,1);	
											osDelay(300);
										}

											set_slope((uint32_t)per_pluse);
										continueMoto();
										slot_count++;
										while((Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]-get_moto_pluse())<=per_pluse*slot_count)
										{
												if(get_moto_pluse()==0||slot_count == Setting.SettingStruct.slotNumber[INDEX_VALUE])
												break;
											osDelay(1);
										}
										stopMoto();
										HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE]-slot_count+1,0,3,0);	display();		
									}
							stopMoto();
							ledTaskStatus = STATUS_MOTO_OUT;
			
				break;
																					
			case STATUS_MOTO_OUT:

							if(test_flag==1)
							{
								ledTaskStatus = STATUS_LED_INIT;
								break;
							}
							
							HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//???????(B10??,B1??)
							wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET);
							wait_input(Cutter_Sensor_Start_GPIO_Port, Cutter_Sensor_Start_Pin, GPIO_PIN_RESET);//?????(A8)			
							HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
							temp_i = 0,temp_j_feed=0,temp_j = 0;
							feed_flag = 0;
							backTask1Finish=0;
							backTask2Finish=0;
							while(backTask1Finish!=1 || backTask2Finish!=1)
							{
								if(temp_i==Setting.SettingStruct.footerDelayExitTime[INDEX_VALUE]*10)
								{	
									HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_SET);
									HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);//???(A4??,A5??)
									footer_flag=0;
								}
								if(footer_flag==0)
								{
									temp_j++;
									if(temp_j==Setting.SettingStruct.footerExitTime[INDEX_VALUE]*10)
									{
									  HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);//A4??
										backTask1Finish=1;
									}
								}
								
								
								if(temp_i == Setting.SettingStruct.feedOnDelayTime[INDEX_VALUE])
								{
									feed_flag =1;
									HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_SET);	//??????(A0??)
								}
								if(feed_flag ==1 )
								{
									temp_j_feed++;
									if(temp_j_feed==Setting.SettingStruct.feedOnTime[INDEX_VALUE])
										{
											HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_RESET);//??????(A0???			
											backTask2Finish=1;									
										}
								}
								osDelay(10);
								temp_i++;
								if(HAL_GPIO_ReadPin(Key_Stop_GPIO_Port,Key_Stop_Pin)==GPIO_PIN_RESET)
								{
									stop_flag = 1;	
									
								}
							}
							wait_input(Material_R_Sensor_Start_GPIO_Port, Material_R_Sensor_Start_Pin, GPIO_PIN_RESET);
							if(stop_flag==1)
							{
								ledTaskStatus = STATUS_MOTO_STOP;
							}
							else
							{
								ledTaskStatus = STATUS_MOTO_STOP;
							}
						
							
					break;
			case STATUS_MOTO_ERR:
							//?????E1
							clear();
							HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);	
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);	
							HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_RESET);	
							HT1621_dis_num(1,0x0e);HT1621_dis_num(2,1);display();
							while(HAL_GPIO_ReadPin(Key_Stop_GPIO_Port,Key_Stop_Pin)==GPIO_PIN_SET);//???????(B6)
							while(HAL_GPIO_ReadPin(Key_Stop_GPIO_Port,Key_Stop_Pin)==GPIO_PIN_RESET);//???????(B6)
							HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//???????(B10??,B1??)
							wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET);//(A9)	
							wait_input(Cutter_Sensor_Start_GPIO_Port, Cutter_Sensor_Start_Pin, GPIO_PIN_RESET);//?????(A8)		
							HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);			
							HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//???(A4??,A5??)
							footer_flag =0 ;
							osDelay(Setting.SettingStruct.footerExitTime[INDEX_VALUE]*100);	
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
							HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE],0,Setting.SettingStruct.slotNumber[INDEX_SIZE],Setting.SettingStruct.slotNumber[INDEX_POINT]);
							HT1621_dis_float(Setting.SettingStruct.percentOfPassOnCut[INDEX_VALUE],3,Setting.SettingStruct.percentOfPassOnCut[INDEX_SIZE],Setting.SettingStruct.percentOfPassOnCut[INDEX_POINT]);
							ledTaskStatus = STATUS_LED_INIT;
					break;
			case STATUS_MOTO_TEST://????
 
				 	ledTaskStatus = STATUS_LED_INIT;
				break;
			case STATUS_MOTO_TEST2://????
			

			fuck:	while(ledTaskStatus == STATUS_MOTO_TEST2)
						{
							pluse_last = Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE];
							//runMoto(100,0,1);
							startMoto(Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE],0,1);
							while(get_moto_pluse()>0)//??
								{
									
									//  ???
									while( HAL_GPIO_ReadPin(CCD_Input_GPIO_Port,CCD_Input_Pin)!=GPIO_PIN_RESET )
									{
										if(get_moto_pluse()==0)
													goto end_test2;
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
//									
									}
								  
									mica = pluse_last-get_moto_pluse();
									pluse_last = get_moto_pluse();
									osDelay(1);
									//????
									while(HAL_GPIO_ReadPin(CCD_Input_GPIO_Port,CCD_Input_Pin)!=GPIO_PIN_SET)
									{
										if(get_moto_pluse()==0)
													goto end_test2;
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
									osDelay(1);
//									stopMoto();
//									continueMoto();
									motoTestPluse++;
									if(pluse_last>0)
										HT1621_dis_float((uint32_t)mica*100/(cu+mica),3,3,0);
								  HT1621_dis_point(5,1);
									display();
								}
							end_test2:	
								stopMoto();
								HT1621_dis_float(motoTestPluse,0,3,0);
								display();
 								motoTestPluse=0;	
								osDelay(1000);
						}
						break;						
			case STATUS_MOTO_STOP:
				stop:	if(test_flag==1)
							{
								ledTaskStatus = STATUS_LED_INIT;
								break;
							}
							HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);	
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);	
							HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_RESET);							
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
		//1ms????????
	//	speedCal();
		osDelay(1);
   }
  /* USER CODE END StartStepMotoTask */
}

/* USER CODE BEGIN Application */
 

//void backTake1(void const * argument)
//{
//  /* USER CODE BEGIN StartStepMotoTask */

//  /* Infinite loop */
//  for(;;)
//  {

// vTaskDelete(NULL);
//   }
//  /* USER CODE END StartStepMotoTask */
//}

void backTake2(void const * argument)
{
  /* USER CODE BEGIN StartStepMotoTask */

  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_SET);	//??????(A0??)
		osDelay(Setting.SettingStruct.feedOnTime[INDEX_VALUE]*10);
		HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_RESET);//??????(A0??)		
		backTask2Finish=1;
		vTaskDelete(NULL);
   }
  /* USER CODE END StartStepMotoTask */
}

uint16_t absi(int16_t i)
{
	return i>0?i:-i;
}
//???????? ??????????????
uint8_t check_pass_preCut(uint16_t width)
{
	if(test_flag ==1)
		return 1;
	float widht_ratio =  Setting.SettingStruct.percentOfPassPreCut[INDEX_VALUE]/200.0;
	uint16_t widht_set = Setting.SettingStruct.micaWidth[INDEX_VALUE];
	if((width > widht_set*(1-widht_ratio)) && (width < widht_set*(1+widht_ratio)))
		return 1;
	else
		return 0;
	
}
//???????? ??????????????
uint8_t check_pass_onCut(uint16_t width)
{
	if(test_flag ==1)
		return 1;
	float widht_ratio = Setting.SettingStruct.percentOfPassOnCut[INDEX_VALUE]/200.0;
	uint16_t widht_set = Setting.SettingStruct.micaWidth[INDEX_VALUE];
	if((width > widht_set*(1-widht_ratio)) && (width < widht_set*(1+widht_ratio)))
		return 1;
	else
		return 0;
	
}
//???????? ????????1 ????????2 ??????0
uint8_t wait_input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	while(1)
	{
 
		osDelay(1);
		//if(xSemaphoreTake( Key_Sensor_BinaryHandle, pdMS_TO_TICKS(1))==pdPASS)
		{
			
			if(HAL_GPIO_ReadPin(Key_Stop_GPIO_Port,Key_Stop_Pin)==GPIO_PIN_RESET)
				{
					
					stop_flag = 1;
					if(ledTaskStatus != STATUS_MOTO_OUT)
					{
						ledTaskStatus = STATUS_MOTO_OUT;
						return 1;
					}
				}	
			if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==PinState)
				{
					return 0;
				}	
		}		
	}
}

//???????
void para_to_set(void)
{
	//??
	setMotoSpeed(Setting.SettingStruct.stepMotoInitSpeed[INDEX_VALUE]);
	//????
	X9C103_Set(Setting.SettingStruct.compareThreshold[INDEX_VALUE]-50);
 
}

//???????
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
//???????
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
	//?????? ?????????
 
	void arg_read(void)
	{
		STMFLASH_Read( FLASH_BASE+60*1024, (uint16_t *) &Setting, sizeof(Setting)/2 );           //????????????????	
		if(Setting.SettingStruct.diameter[INDEX_VALUE]==0xffff)
			{
				Setting.SettingStruct.diameter[INDEX_VALUE] = 10;Setting.SettingStruct.diameter[INDEX_POINT] = 2;
				Setting.SettingStruct.footerExitTime[INDEX_VALUE] = 1;Setting.SettingStruct.footerExitTime[INDEX_POINT] = 1;
				Setting.SettingStruct.slotNumber[INDEX_VALUE] = 24;Setting.SettingStruct.slotNumber[INDEX_POINT] = 0;
				Setting.SettingStruct.micaWidth[INDEX_VALUE] = 15;Setting.SettingStruct.micaWidth[INDEX_POINT] = 0;
				Setting.SettingStruct.percentOfPassPreCut[INDEX_VALUE] = 100;Setting.SettingStruct.percentOfPassPreCut[INDEX_POINT] = 0;
				Setting.SettingStruct.motoCompensation[INDEX_VALUE] = 10;Setting.SettingStruct.motoCompensation[INDEX_POINT] = 0;
				Setting.SettingStruct.millingMethod[INDEX_VALUE] = 0;Setting.SettingStruct.millingMethod[INDEX_POINT] = 0;
				Setting.SettingStruct.motoDirection[INDEX_VALUE] = 0;Setting.SettingStruct.motoDirection[INDEX_POINT] = 0;
				Setting.SettingStruct.feedOnTime[INDEX_VALUE] = 1;Setting.SettingStruct.feedOnTime[INDEX_POINT] = 2;
				Setting.SettingStruct.feedOnDelayTime[INDEX_VALUE] = 1;Setting.SettingStruct.feedOnDelayTime[INDEX_POINT] = 2;
				Setting.SettingStruct.percentOfPassOnCut[INDEX_VALUE] = 150;Setting.SettingStruct.percentOfPassOnCut[INDEX_POINT] = 0;
				Setting.SettingStruct.footerOnTime[INDEX_VALUE] = 1;Setting.SettingStruct.footerOnTime[INDEX_POINT] = 1;
				Setting.SettingStruct.footerDelayExitTime[INDEX_VALUE] = 1;Setting.SettingStruct.footerDelayExitTime[INDEX_POINT] = 1;
				Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE] = 4000;Setting.SettingStruct.plusNumberOfMoto[INDEX_POINT] = 0;
				Setting.SettingStruct.mode0DirSwitchTime[INDEX_VALUE] = 20;Setting.SettingStruct.mode0DirSwitchTime[INDEX_POINT] = 2;
				Setting.SettingStruct.stepMotoInitSpeed[INDEX_VALUE] = 600;Setting.SettingStruct.stepMotoInitSpeed[INDEX_POINT] = 0;
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_VALUE] = 1000;Setting.SettingStruct.stepMotoRunSpeed[INDEX_POINT] = 0;
				Setting.SettingStruct.compareThreshold[INDEX_VALUE] = 50;Setting.SettingStruct.compareThreshold[INDEX_POINT] = 1;
				Setting.SettingStruct.stepMotoFinishTime[INDEX_VALUE] = 1;Setting.SettingStruct.stepMotoFinishTime[INDEX_POINT] = 0;
				Setting.SettingStruct.micaPreTrace[INDEX_VALUE] = 1;Setting.SettingStruct.micaPreTrace[INDEX_POINT] = 0;
				 
				Setting.SettingStruct.diameter[INDEX_MAX] = 800;Setting.SettingStruct.diameter[INDEX_MIN] = 10;
				Setting.SettingStruct.footerExitTime[INDEX_MAX] = 999;Setting.SettingStruct.footerExitTime[INDEX_MIN] = 0;
				Setting.SettingStruct.slotNumber[INDEX_MAX] = 256;Setting.SettingStruct.slotNumber[INDEX_MIN] = 3;
				Setting.SettingStruct.micaWidth[INDEX_MAX] = 800;Setting.SettingStruct.micaWidth[INDEX_MIN] = 5;
				Setting.SettingStruct.percentOfPassPreCut[INDEX_MAX] = 200;Setting.SettingStruct.percentOfPassPreCut[INDEX_MIN] = 0;
				Setting.SettingStruct.motoCompensation[INDEX_MAX] = 65000;Setting.SettingStruct.motoCompensation[INDEX_MIN] = 1;
				Setting.SettingStruct.millingMethod[INDEX_MAX] = 3;Setting.SettingStruct.millingMethod[INDEX_MIN] = 0;
				Setting.SettingStruct.motoDirection[INDEX_MAX] = 1;Setting.SettingStruct.motoDirection[INDEX_MIN] = 0;
				Setting.SettingStruct.feedOnTime[INDEX_MAX] = 900;Setting.SettingStruct.feedOnTime[INDEX_MIN] = 1;
				Setting.SettingStruct.feedOnDelayTime[INDEX_MAX] = 900;Setting.SettingStruct.feedOnDelayTime[INDEX_MIN] = 1;
				Setting.SettingStruct.percentOfPassOnCut[INDEX_MAX] = 200;Setting.SettingStruct.percentOfPassOnCut[INDEX_MIN] = 0;
				Setting.SettingStruct.footerOnTime[INDEX_MAX] = 900;Setting.SettingStruct.footerOnTime[INDEX_MIN] = 1;
				Setting.SettingStruct.footerDelayExitTime[INDEX_MAX] = 900;Setting.SettingStruct.footerDelayExitTime[INDEX_MIN] = 1;
				Setting.SettingStruct.plusNumberOfMoto[INDEX_MAX] = 65000;Setting.SettingStruct.plusNumberOfMoto[INDEX_MIN] = 1000;
				Setting.SettingStruct.mode0DirSwitchTime[INDEX_MAX] = 900;Setting.SettingStruct.mode0DirSwitchTime[INDEX_MIN] = 1;
				Setting.SettingStruct.stepMotoInitSpeed[INDEX_MAX] = 65000;Setting.SettingStruct.stepMotoInitSpeed[INDEX_MIN] = 600;
				Setting.SettingStruct.stepMotoRunSpeed[INDEX_MAX] = 65000;Setting.SettingStruct.stepMotoRunSpeed[INDEX_MIN] = 600;
				Setting.SettingStruct.compareThreshold[INDEX_MAX] = 100;Setting.SettingStruct.compareThreshold[INDEX_MIN] = 0;
				Setting.SettingStruct.stepMotoFinishTime[INDEX_MAX] = 65000;Setting.SettingStruct.stepMotoFinishTime[INDEX_MIN] = 0;
				Setting.SettingStruct.micaPreTrace[INDEX_MAX] = (Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]/Setting.SettingStruct.slotNumber[INDEX_VALUE])-
														Setting.SettingStruct.micaWidth[INDEX_VALUE];
														Setting.SettingStruct.micaPreTrace[INDEX_MIN] = 0;
				Setting.SettingStruct.diameter[INDEX_SIZE] = 3; 
				Setting.SettingStruct.footerExitTime[INDEX_SIZE] = 2; 
				Setting.SettingStruct.slotNumber[INDEX_SIZE] = 3; 
				Setting.SettingStruct.micaWidth[INDEX_SIZE] = 3; 
				Setting.SettingStruct.percentOfPassPreCut[INDEX_SIZE] = 3; 
				Setting.SettingStruct.motoCompensation[INDEX_SIZE] = 5; 
				Setting.SettingStruct.millingMethod[INDEX_SIZE] = 1; 
				Setting.SettingStruct.motoDirection[INDEX_SIZE] = 1; 
				Setting.SettingStruct.feedOnTime[INDEX_SIZE] = 3; 
				Setting.SettingStruct.feedOnDelayTime[INDEX_SIZE] = 3;
				Setting.SettingStruct.percentOfPassOnCut[INDEX_SIZE] = 3; 
				Setting.SettingStruct.footerOnTime[INDEX_SIZE] = 2; 
				Setting.SettingStruct.footerDelayExitTime[INDEX_SIZE] = 2; 
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
//???'2??¨¹????
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
