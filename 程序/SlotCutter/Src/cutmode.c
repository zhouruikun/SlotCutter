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

//���г�ʼ������

void motoInit(void )
{
		 if(step ==0 )
						 {
							 pause_flag=0;
							 slot_count=0;
							 HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
							 HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//	β�����ƶ�(A5ʹ�ܣ�A4�ͷ�) 
							 step=1;
						 }else if(step ==1 )
						 {
							  if(wait_input(Footer_Sensor_End_GPIO_Port, Footer_Sensor_End_Pin, GPIO_PIN_RESET))
								{
									step =2;
								}
								//β���Ƿ񵽴﹤��λ�ã�A15��
						 }
						 else if(step ==2 )
							{
								HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
									osDelay(Setting.SettingStruct.feedOnTime[INDEX_VALUE]*10);//�����趨ʱ����ʱ
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
//����ģʽ���Ϻ���

void runBack(void)
{
	if(step == 0)
	{
		HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Cutter_Push_GPIO_Port, Cutter_Push_Pin, GPIO_PIN_RESET);	//ϳ���������ƶ�(B10�ͷţ�B1ʹ��)
		step =1;
	}
	else 	if(step == 1)
	{
		if(wait_input(Cutter_Sensor_Start_GPIO_Port, Cutter_Sensor_Start_Pin, GPIO_PIN_RESET)){
		step = 2;
		}//ϳ�������Ƿ�λ��A8��
	}
	else 	if(step == 2)
		{
		HAL_GPIO_WritePin(Cutter_Pull_GPIO_Port, Cutter_Pull_Pin, GPIO_PIN_RESET);
		runMoto(Setting.SettingStruct.plusNumberOfMoto[INDEX_VALUE]*3);//���������ת3Ȧ
		step = 3 ;
	}	else 	if(step == 3)
	{
		HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//β���ˣ�A4ʹ�ܣ�A5�ͷţ�
		if(wait_input(Footer_Sensor_Start_GPIO_Port, Footer_Sensor_Start_Pin, GPIO_PIN_RESET)){
		step = 4 ;
		}//β���Ƿ��˳����ұߣ�A11��
	}		

	else if(step == 4)
		{
			HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_SET);	//�����������ƣ�A0ʹ�ܣ�
			if(wait_input(Material_R_Sensor_End_GPIO_Port, Material_R_Sensor_End_Pin, GPIO_PIN_RESET)){
			step = 5 ;
			}//���������Ƿ񵽴��ұߣ�B4��
		}	
	else if(step == 5)
		{
			HAL_GPIO_WritePin(Material_R_Push_GPIO_Port, Material_R_Push_Pin, GPIO_PIN_RESET);//�����������ƣ�A0�ͷţ�
				if(wait_input(Material_R_Sensor_Start_GPIO_Port, Material_R_Sensor_Start_Pin, GPIO_PIN_RESET))
				{
					step = 6 ;
				}//�����Ƿ���ɣ�B3�� 
		}
		else{
		if(wait_input(Key_Start_GPIO_Port, Key_Start_Pin,GPIO_PIN_RESET)){
				step = 0 ;	
			}//�ⲿ���������Ƿ�պ� (B5)			
		}


 
					
}

//����ģʽ����
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
			if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_SET)){break;};//�Ƿ�����ĸ(B12)								
			startMoto(65534,0,1);
			if(wait_input(CCD_Input_GPIO_Port, CCD_Input_Pin, GPIO_PIN_RESET)){break;};//�Ƿ���ͭƬ(B12)
			width += (65534-stopMoto());
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
}

//����1ģʽ����
void modeTest(void)
{
	

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
}
void motoErr(void)
{
		//ͣ������ʾE1
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
			}//�Ƿ�ֹͣ������B6��
	}
	else if(step == 2)
	{
		HAL_GPIO_WritePin(Footer_Pull_GPIO_Port, Footer_Pull_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_SET);//β���ˣ�A4ʹ�ܣ�A5�ͷţ�
		if(wait_input(Footer_Sensor_Start_GPIO_Port, Footer_Sensor_Start_Pin, GPIO_PIN_RESET)){
		step =0;
		HAL_GPIO_WritePin(Footer_Push_GPIO_Port, Footer_Push_Pin, GPIO_PIN_RESET);
		HT1621_dis_float(Setting.SettingStruct.slotNumber[INDEX_VALUE],0,Setting.SettingStruct.slotNumber[INDEX_SIZE],Setting.SettingStruct.slotNumber[INDEX_POINT]);
		HT1621_dis_float(Setting.SettingStruct.percentOfPassPreCut[INDEX_VALUE],3,Setting.SettingStruct.percentOfPassPreCut[INDEX_SIZE],Setting.SettingStruct.percentOfPassPreCut[INDEX_POINT]);
		display();
		ledTaskStatus = STATUS_MOTO_INIT;	
		};//β���Ƿ��ұߣ�A11��
	}
}