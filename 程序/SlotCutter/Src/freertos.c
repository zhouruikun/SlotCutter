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
	//����һ������λ
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
								//����ģʽ
								 HT1621_dis_point(5,1);	display();
								 while(1)
								 {
									 xQueueReceive( led_key_queue,&keyGet,portMAX_DELAY);
									 if(keyGet.key == Key_R && keyGet.status == KEY_UP)
										{
											//�������ģʽ2
											ledTaskStatus = STATUS_MOTO_TEST2;
											clear();
											startMoto(UINT32_MAX,0,1);
											break;
										}
										if(keyGet.key == Key_FUN && keyGet.status == KEY_UP)
										{
											//�������ģʽ1
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
							if(funcNum == 0x10)//��ѹ�Ƚ���������
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
						 HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//	β�����ƶ�(A5ʹ�ܣ�A4�ͷ�)
						 if(wait_input(Footer_Sensor_End_GPIO_Port, Footer_Sensor_End_Pin, GPIO_PIN_RESET)){break;}//β���Ƿ񵽴﹤��λ�ã�A15��
						 
						 else
							{
								HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
								//��ͣ�ж�
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
								//��ͣ�жϽ���		
								osDelay(Setting.SettingStruct.feedDelayTime[INDEX_VALUE]*10);//�����趨ʱ����ʱ
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
								if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){break;};//�Ƿ�����ĸ(B12)								
								startMoto(65534,0,1);
								if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){break;};//�Ƿ���ͭƬ(B12)
								width += (65534-stopMoto());
								//��ͣ�ж�
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
								//��ͣ�жϽ���		
							}
						 width = width/3;
						 if(check_pass(width)!=1)
							{
								stopMoto();
								ledTaskStatus = STATUS_MOTO_ERR;
							}							
							cut_next:
							//��ͣ�ж�
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
								//��ͣ�жϽ���	
							slot_count++;
							startMoto(65534,0,1);//�����������(B0)
							if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){break;};//�Ƿ���yunmu(B12)
							width = (65534-stopMoto());
								//��ĸ�Ƿ�ϸ�
							if(check_pass(width)==1)
								{
									//��
									//������һ������
									startMoto(width/2,0,0);//�����������(B0)
									HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
									HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_SET);//ϳ�����������иB10ʹ�ܣ�B1�ͷţ�
									if(wait_input(Cutter_Sensor_End_GPIO_Port, Cutter_Sensor_End_Pin, GPIO_PIN_RESET)){break;};//ϳ���Ƿ��Ҷˣ�A10��
									//�Ƿ������һ����ĸ
									HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);
									if(slot_count == Setting.SettingStruct.slotNumber[INDEX_VALUE])
											{//��
													ledTaskStatus = STATUS_MOTO_OUT;
											}
											else{
												//����
												HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
												HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//ϳ���������ƶ�(B10�ͷţ�B1ʹ��)
												if(wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET)){break;};//�Ƿ񵽴��У�A9��
												HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
												goto cut_next;
											}
								}
								else{
											//���ϸ�
											stopMoto();
											ledTaskStatus = STATUS_MOTO_ERR;
									}											
				break;
							
			case STATUS_MOTO_RUN_MODE2:
							wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET);//�Ƿ�����ĸ(B12)
							startMoto(65534,0,1);//�����������(B0)
							wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET);//�Ƿ���ͭƬ(B12)
							width = (65534-stopMoto());
											//��ͣ�ж�
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
								//��ͣ�жϽ���		
								//��ĸ�Ƿ�ϸ�
							if( check_pass(width)==1)
								{
									//������һ������
				
									startMoto(width/2,0,0);//�����������(B0
									slot_count =0;
									while(1)
									{
										 //��ͣ�ж�
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
										//��ͣ�жϽ���		
										slot_count++;
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_SET);//ϳ�����������иB10ʹ�ܣ�B1�ͷţ�
										if(wait_input(Cutter_Sensor_End_GPIO_Port, Cutter_Sensor_End_Pin, GPIO_PIN_RESET)){break;};//ϳ���Ƿ��Ҷˣ�A10��
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);
										if(slot_count>=(Setting.SettingStruct.slotNumber[INDEX_VALUE]))
										{
											ledTaskStatus = STATUS_MOTO_OUT;
											break;
										}
										runMoto(Setting.SettingStruct.motoCompensation[INDEX_VALUE]);
											//����
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//ϳ���������ƶ�(B10�ͷţ�B1ʹ��)
										if(wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET)){break;};//�Ƿ񵽴��У�A9��			
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
									}
									
								}
							else
								{	//���ϸ�
									stopMoto();
									ledTaskStatus = STATUS_MOTO_ERR;
									}		
						
				break;
			case STATUS_MOTO_RUN_MODE3:
									while(1)
									{
											//��ͣ�ж�
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
											//��ͣ�жϽ���									
										slot_count++;
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_SET);//ϳ�����������иB10ʹ�ܣ�B1�ͷţ�
										if(wait_input(Cutter_Sensor_End_GPIO_Port, Cutter_Sensor_End_Pin, GPIO_PIN_RESET)){break;}//ϳ���Ƿ��Ҷˣ�A10��
										
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);
										if(slot_count>=(Setting.SettingStruct.slotNumber[INDEX_VALUE]))
										{
											ledTaskStatus = STATUS_MOTO_OUT;
											break;
										}
										runMoto(Setting.SettingStruct.motoCompensation[INDEX_VALUE]);
											//����
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
										HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);//ϳ���������ƶ�(B10�ͷţ�B1ʹ��)
										if(wait_input(Cutter_Sensor_Mid_GPIO_Port, Cutter_Sensor_Mid_Pin, GPIO_PIN_RESET)){break;};//�Ƿ񵽴��У�A9��
										HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);										
									}
				break;
																					
			case STATUS_MOTO_OUT:
							HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);	//ϳ���������ƶ�(B10�ͷţ�B1ʹ��)
							if(wait_input(Cutter_Sensor_Start_GPIO_Port, Cutter_Sensor_Start_Pin, GPIO_PIN_RESET)){break;};//ϳ�������Ƿ�λ��A8��
							HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
							runMoto(Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]*3);//���������ת3Ȧ
							HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//β���ˣ�A4ʹ�ܣ�A5�ͷţ�
							if(wait_input(Footer_Sensor_Start_GPIO_Port, Footer_Sensor_Start_Pin, GPIO_PIN_RESET)){break;};//β���Ƿ��˳����ұߣ�A11��
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_SET);	//�����������ƣ�A0ʹ�ܣ�
							if(wait_input(Material_R_Sensor_End_GPIO_Port, Material_R_Sensor_End_Pin, GPIO_PIN_RESET)){break;};//���������Ƿ񵽴��ұߣ�B4��
							HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_RESET);//�����������ƣ�A0�ͷţ�
							if(wait_input(Material_R_Sensor_Start_GPIO_Port, Material_R_Sensor_Start_Pin, GPIO_PIN_RESET))
							{break;}//�����Ƿ���ɣ�B3��
							else
							{
								ledTaskStatus = STATUS_MOTO_INIT;
							}
							if(wait_input(Key_Start_GPIO_Port, Key_Start_Pin,GPIO_PIN_RESET)){break;};//�ⲿ���������Ƿ�պ� (B5)										
					break;
			case STATUS_MOTO_ERR:
							//ͣ������ʾE1
							clear();
							HT1621_dis_num(1,0x0e);HT1621_dis_num(2,1);
							wait_input(Key_Stop_GPIO_Port, Key_Stop_Pin, GPIO_PIN_RESET);//�Ƿ�ֹͣ������B6��
							HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//β���ˣ�A4ʹ�ܣ�A5�ͷţ�
							if(wait_input(Footer_Sensor_Start_GPIO_Port, Footer_Sensor_Start_Pin, GPIO_PIN_RESET)){break;};//β���Ƿ��ұߣ�A11��
							HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
							HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE],0,Setting.SettingStruct.slotNumber[INDEX_SIZE],Setting.SettingStruct.slotNumber[INDEX_POINT]);
							HT1621_dis_float(Setting.SettingStruct.qualifiedRate[INDEX_VALUE],3,Setting.SettingStruct.qualifiedRate[INDEX_SIZE],Setting.SettingStruct.qualifiedRate[INDEX_POINT]);
							display();
							ledTaskStatus = STATUS_MOTO_INIT;
					break;
			case STATUS_MOTO_TEST://ɨ����ĸ
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
				
					while(slot_count>0)//һȦ
							{
									slot_count--;

									//��������
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
																		//������while ÿ��ͣ�º�ȴ�����
									while(1)
									{
										if(xQueueReceive( led_key_queue,&keyGet,100 )==pdPASS)
										{
											if(keyGet.status == KEY_UP)
													break;
										}
									}
									//���½���
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
																		//������while ÿ��ͣ�º�ȴ�����
									while(1)
									{
										if(xQueueReceive( led_key_queue,&keyGet,100 )==pdPASS)
										{
											if(keyGet.status == KEY_UP)
													break;
										}
									}
									//������һ��mica
									startMoto(mica/2,0,0);
									while(get_moto_pluse())
									{
										osDelay(1);
									}
									osDelay(300);//ͣ300ms
									//������while ÿ��ͣ�º�ȴ�����
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
			case STATUS_MOTO_TEST2://һֱ����
			
//					startMoto(Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]);//���������ת3Ȧ
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
							while((UINT32_MAX-1-get_moto_pluse())<=Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE])//һȦ
								{
									//���½���
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
									//��������
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
		//1ms����һ���ٶ�ƫִ
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
//����ۼ�������� Ȼ����ݲ�����һȦ�������ж�
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
//�ȴ��ⲿ�����ź� �����ͣ���򷵻�1 �������ͣ�򷵻�2 �ȴ��ɹ�����0
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

//���ò���������
void para_to_set(void)
{
	//�ٶ�
	setMotoSpeed(Setting.SettingStruct.stepMotoRunSpeed[INDEX_VALUE]);
	//�Ƚϵ�ѹ
	X9C103_Set(Setting.SettingStruct.compareThreshold[INDEX_VALUE]-50);
}

//����һ������λ
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
//����һ������λ
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
//������������ �����һ�����ʼ��
	void arg_read(void)
	{
		STMFLASH_Read( FLASH_BASE+60*1024, (uint16_t *) &Setting, sizeof(Setting)/2 );           //��ָ����ַ��ʼ����ָ�����ȵ�����	
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
