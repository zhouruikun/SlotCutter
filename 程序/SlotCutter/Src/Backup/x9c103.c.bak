/*-------------------------------------------------------------------------- 
X9C103.H 
X9C103 functions. 
Copyright (c) 2007 DC. By Delphifx 2007-8-11. 
All rights reserved. 
--------------------------------------------------------------------------*/ 
 #include "main.h"
#include "stm32f1xx_hal.h"
 #define X_CS_Pin GPIO_PIN_1
#define X_CS_GPIO_Port GPIOA
#define X_UD_Pin GPIO_PIN_2
#define X_UD_GPIO_Port GPIOA
#define X_INC_Pin GPIO_PIN_3
#define X_INC_GPIO_Port GPIOA

 
#define   SETB_X9C103_CS1   {HAL_GPIO_WritePin(X_CS_GPIO_Port,X_CS_Pin,GPIO_PIN_SET);}
#define   CLRB_X9C103_CS1  {HAL_GPIO_WritePin(X_CS_GPIO_Port,X_CS_Pin,GPIO_PIN_RESET);} 
														
#define   SETB_X9C103_INC    {HAL_GPIO_WritePin(X_INC_GPIO_Port,X_INC_Pin,GPIO_PIN_SET);}
#define   CLRB_X9C103_INC   {HAL_GPIO_WritePin(X_INC_GPIO_Port,X_INC_Pin,GPIO_PIN_RESET);} 
#define   SETB_X9C103_UD    {HAL_GPIO_WritePin(X_UD_GPIO_Port,X_UD_Pin,GPIO_PIN_SET);}
#define   CLRB_X9C103_UD   {HAL_GPIO_WritePin(X_UD_GPIO_Port,X_UD_Pin,GPIO_PIN_RESET);} 
 
 
 
 
void X9C103_Inc_N_Step(unsigned char Sel,unsigned char N); 
void X9C103_Dec_N_Step(unsigned char Sel,unsigned char N); 
void Delay(unsigned int t) ; 
void X9C103_Init(unsigned char Sel);//???????? 
 int8_t x9cpos = 0;
//??us???  
void Delay(unsigned int t)  
{  
	unsigned int i;  
	for(i=0;i<t;i++) ; 
	 
}  
 
//************************************************************************ 
// ?????????? 
// ?????100???,???99? 
//************************************************************************ 
void X9C103_Inc_N_Step(unsigned char Sel,unsigned char N) 
{ 
   unsigned char i=0;  
   SETB_X9C103_UD;   // U/D ??   ????INC??,??UP?? 
   Delay(3);         // ????2us 
    
  switch(Sel) 
  { 
    case 1: 
 CLRB_X9C103_CS1;   
    break; 
    case 2: 
 
    break; 
    default:break;     
   }  
//   CLRB_X9C103_CS;   // CS  ?? 
 
   for(i=N;i>0;i--)  
  {  
     SETB_X9C103_INC;  // ??INC    ??INC????? 
     Delay(2);         // ??2us?? 
     CLRB_X9C103_INC;   // INC ??;  // ?????? 
     Delay(600);       // ????500us, Rw?????? 
   } 
  SETB_X9C103_INC;//???? 
  switch(Sel) 
  { 
    case 1: 
 SETB_X9C103_CS1;   
    break; 
    case 2: 
   
    break; 
    default:break;     
   } // ??CS?? //store 
} 
//************************************************************************ 
// ?????????? 
// ?????100???,???99? 
//************************************************************************ 
void X9C103_Dec_N_Step(unsigned char Sel,unsigned char N) 
{ 
   unsigned char i=0;  
   CLRB_X9C103_UD;   // U/D ?0,  ????INC??,??Down?? 
    
   Delay(3);         // ????2us 
    
   switch(Sel) 
  { 
    case 1: 
 CLRB_X9C103_CS1;   
    break; 
    case 2: 
   
    break; 
    default:break;     
   }  
   for(i=N;i>0;i--)  
  {  
     SETB_X9C103_INC;  // ??INC    ??INC????? 
     Delay(2);         // ??2us?? 
     CLRB_X9C103_INC;   // INC ??;  // ?????? 
     Delay(600);       // ????500us, Rw?????? 
  } 
  SETB_X9C103_INC;//???? 
   switch(Sel) 
  { 
    case 1: 
		SETB_X9C103_CS1;   
    break; 
    case 2: 
   
    break;  
    default:break;     
   } // ??CS?? // store  
} 
 
void X9C103_Init(unsigned char Sel) 
{ 
  X9C103_Dec_N_Step(Sel,99);  
	X9C103_Inc_N_Step(Sel,49);  
	x9cpos = 0;
}  
//µÁ—π-50 µΩ50  
void X9C103_Set(int8_t voltage)
{
	if(voltage<x9cpos)
	{
		X9C103_Dec_N_Step(1,x9cpos-voltage);  
	}
	else if(voltage>x9cpos)
	{
		X9C103_Inc_N_Step(1,voltage-x9cpos);  
	}
	x9cpos = voltage;
}

