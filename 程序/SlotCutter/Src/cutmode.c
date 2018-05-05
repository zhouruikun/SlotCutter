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

//运行初始化函数

void motoInit(void )
{
		 if(step ==0 )
						 {
							 pause_flag=0;
							 slot_count=0;
							 HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
							 HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//	尾顶左移动(A5使能，A4释放) 
							 step=1;
						 }else if(step ==1 )
						 {
							  if(wait_input(Footer_Sensor_End_GPIO_Port, Footer_Sensor_End_Pin, GPIO_PIN_RESET))
								{
									step =2;
								}
								//尾顶是否到达工件位置（A15）
						 }
						 else if(step ==2 )
							{
								HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
									osDelay(Setting.SettingStruct.feedOnTime[INDEX_VALUE]*10);//根据设定时间延时
								 switch(Setting.SettingStruct.millingMethod[INDEX_VALUE])
								 {
									 case 0:ledTaskStatus = STATUS_MOTO_RUN_MODE1;
										 break;
									 case 1:ledTaskStatus = STATUS_MOTO_RUN_MODE2;
										 break;
									 case 2:ledTaskStatus = STATUS_MOTO_RUN_MODE3;
										 break;
									 case 3:ledTaskStatus = STATUS_MOTO_RUN_MODE4;
										 break;
									 default:break;
								 }
						 }
}
//运行模式退料函数

void runBack(void)
{
	if(step == 0)
	{
		HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);	//铣刀气缸左移动(B10释放，B1使能)
		step =1;
	}
	else 	if(step == 1)
	{
		if(wait_input(Cutter_Sensor_Start_GPIO_Port, Cutter_Sensor_Start_Pin, GPIO_PIN_RESET)){
		step = 2;
		}//铣刀左移是否到位（A8）
	}
	else 	if(step == 2)
		{
		HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
		runMoto(Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]*3);//步进电机正转3圈
		step = 3 ;
	}	else 	if(step == 3)
	{
		HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//尾顶退（A4使能，A5释放）
		if(wait_input(Footer_Sensor_Start_GPIO_Port, Footer_Sensor_Start_Pin, GPIO_PIN_RESET)){
		step = 4 ;
		}//尾顶是否退出到右边（A11）
	}		

	else if(step == 4)
		{
			HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_SET);	//退料气缸右移（A0使能）
			if(wait_input(Material_R_Sensor_End_GPIO_Port, Material_R_Sensor_End_Pin, GPIO_PIN_RESET)){
			step = 5 ;
			}//退料气缸是否到达右边（B4）
		}	
	else if(step == 5)
		{
			HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_RESET);//退料气缸左移（A0释放）
				if(wait_input(Material_R_Sensor_Start_GPIO_Port, Material_R_Sensor_Start_Pin, GPIO_PIN_RESET))
				{
					step = 6 ;
				}//退料是否完成（B3） 
		}
		else{
		if(wait_input(Key_Start_GPIO_Port, Key_Start_Pin,GPIO_PIN_RESET)){
				step = 0 ;	
			}//外部启动开关是否闭合 (B5)			
		}


 
					
}

//运行模式函数
//
void runModeCut(void)
{
	
	
	if(step ==0)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	else if(step ==1)
	{
	}
	
	
	
	
	
	
	
		 width =0 ;
	 for(temp_i = 0 ;temp_i<3;temp_i++)
		{
			startMoto(65534,0,1);
			if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){break;};//是否检测云母(B12)								
			startMoto(65534,0,1);
			if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){break;};//是否检测铜片(B12)
			width += (65534-stopMoto());
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
}

//测试1模式函数
void modeTest(void)
{
	

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
}
void motoErr(void)
{
		//停机并显示E1
	if(step == 0)
	{
		clear();
		HT1621_dis_num(1,0x0e);HT1621_dis_num(2,1);
		step =1;

	}
	else if(step == 1)
	{
	if(wait_input(Key_Stop_GPIO_Port, Key_Stop_Pin, GPIO_PIN_RESET){
			step =2;
			}//是否按停止按键（B6）
	}
	else if(step == 2)
	{
		HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//尾顶退（A4使能，A5释放）
		if(wait_input(Footer_Sensor_Start_GPIO_Port, Footer_Sensor_Start_Pin, GPIO_PIN_RESET)){
		step =0;
		HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
		HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE],0,Setting.SettingStruct.slotNumber[INDEX_SIZE],Setting.SettingStruct.slotNumber[INDEX_POINT]);
		HT1621_dis_float(Setting.SettingStruct.percentOfPassPreCut[INDEX_VALUE],3,Setting.SettingStruct.percentOfPassPreCut[INDEX_SIZE],Setting.SettingStruct.percentOfPassPreCut[INDEX_POINT]);
		display();
		ledTaskStatus = STATUS_MOTO_INIT;	
		};//尾顶是否到右边（A11）
	}
}