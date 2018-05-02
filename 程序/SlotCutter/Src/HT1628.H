#ifndef __HT1628_H
#define __HT1628_H
 //启蒙电子 LED模块驱动程序STM32 库 头文件  
 //	www.51c51.net 0531-85817017 
#include "main.h"
#include "stm32f1xx_hal.h"
 #include "comm.h"
//-----------------HT1628端口定义---------------- 
/********************************************************************************/
 //SDO	 PE11	
 

#define LED_GPIO_SDO_PORT              TM1628_DIO_GPIO_Port
#define LED_GPIO_SDO_PIN               TM1628_DIO_Pin
 //SCK	 PE12
#define LED_GPIO_SCK_PORT              TM1628_CLK_GPIO_Port
#define LED_GPIO_SCK_PIN               TM1628_CLK_Pin
 //STB	 PE13
#define LED_GPIO_STB_PORT              TM1628_STB_GPIO_Port
#define LED_GPIO_STB_PIN               TM1628_STB_Pin

 
#define SCK_LED1  {HAL_GPIO_WritePin(LED_GPIO_SCK_PORT,LED_GPIO_SCK_PIN,GPIO_PIN_SET);}
#define SCK_LED0  {HAL_GPIO_WritePin(LED_GPIO_SCK_PORT,LED_GPIO_SCK_PIN,GPIO_PIN_RESET);} 

#define STB_LED1  {HAL_GPIO_WritePin(LED_GPIO_STB_PORT,LED_GPIO_STB_PIN,GPIO_PIN_SET);}
#define STB_LED0  {HAL_GPIO_WritePin(LED_GPIO_STB_PORT,LED_GPIO_STB_PIN,GPIO_PIN_RESET);}

#define SDO_LED1  {HAL_GPIO_WritePin(LED_GPIO_SDO_PORT,LED_GPIO_SDO_PIN,GPIO_PIN_SET);}
#define SDO_LED0  {HAL_GPIO_WritePin(LED_GPIO_SDO_PORT,LED_GPIO_SDO_PIN,GPIO_PIN_RESET);}
#define READ_SDO_LED (LED_GPIO_SDO_PORT->IDR&LED_GPIO_SDO_PIN)==LED_GPIO_SDO_PIN

#define SDO_IN {LED_GPIO_SDO_PORT->CRH &=0XFF0FFFFF;LED_GPIO_SDO_PORT->CRH |= 0X00400000;}
#define SDO_OUT {LED_GPIO_SDO_PORT->CRH &=0XFF0FFFFF;LED_GPIO_SDO_PORT->CRH |= 0X00200000;}

 
void sendcmddata(unsigned char databye);

void senddata(unsigned char databye);

void lint_val_display(void);
void showStandby(void);
void show_all_led(void);
void clear(void);
void display(void);//led显示函数
void readKey(void);
uint8_t checkKey(uint16_t keyNum);
void HT1621_dis_num(uint8_t pos,uint8_t num);
//显示带小数点的数字  最大99999 
void HT1621_dis_float(uint32_t num,uint8_t startPos ,uint8_t len,uint8_t pointPos);
///以上程序为LED驱动程序
void HT1621_dis_point(uint8_t pos,uint8_t enable);
#endif  
	 
