#ifndef __HT1628_H
#define __HT1628_H
 //���ɵ��� LEDģ����������STM32 �� ͷ�ļ�  
 //	www.51c51.net 0531-85817017 
#include "main.h"
#include "stm32f1xx_hal.h"
void startMoto(uint16_t Pluse); 
uint16_t stopMoto(void);
void setMotoSpeed(uint16_t Speed);
#endif  