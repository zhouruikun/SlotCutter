#ifndef __HT1628_H
#define __HT1628_H
 //启蒙电子 LED模块驱动程序STM32 库 头文件  
 //	www.51c51.net 0531-85817017 
#include "main.h"
#include "stm32f1xx_hal.h"
void runMoto(uint32_t Pluse,uint32_t slope , uint8_t dir);
void startMoto(uint32_t Pluse,uint32_t slope , uint8_t dir);
uint32_t stopMoto();
void setMotoDir(uint16_t dir);
void setMotoSpeed(uint32_t Speed);
uint32_t get_moto_pluse(void);
void speedCal(void);
void continueMoto(void);
#endif  
