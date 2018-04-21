/************************* (C) COPYRIGHT 2013 DZXH ************************
* 文  件  名      : KEY.H
* 作      者      : DZXH@Dream Creater
* 适      用      ：DZXH 开发板V2.1/V1.0。
* 淘  宝  店      : http://shop102062100.taobao.com/
* 版      本      : V1.0
* 日      期      : 2013/11
* 主  控  MCU     : STM32F103VET6
* 编  译  器      : Keil ARM 4.54
* 描      述      : 按键初始化头文件
**************************************************************************/
#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f1xx_hal.h"
#include "main.h"

 

 

//下面是定义按键的时间，单位为 ： 50ms
#define KEY_DOWN_TIME   1
#define KEY_HOLD_TIME   20         //最多253，否则需要修改keytime的类型

#define KEY_OFFSET 4096
#define KEY_ 200
 
//按键端口的枚举
typedef enum
{
   Key_START   =0 ,
   Key_STOP  ,
   Key_FUN  ,
   Key_UP  ,
   Key_DOWN,
   Key_L ,
   Key_R  , 
   KEY_MAX,
} KEY_e;



//key状态宏定义
typedef enum
{
    KEY_DOWN  =   0,         //按键按下时对应电平
    KEY_UP    =   1,         //按键弹起时对应电平
    KEY_HOLD,

} KEY_STATUS_e;

//按键及其状态结构体
typedef struct
{
    uint16_t           key;
    KEY_STATUS_e    status;
} KEY_MSG_t;

void    KEY_init(KEY_e key);                    // KEY初始化    函数
void    KEY_ALL_init(void);                     //全部Key初始化
KEY_STATUS_e key_check(uint8_t key);              //检测key状态
 

//定时扫描按键
uint8_t get_key_msg(KEY_MSG_t *keymsg);                  //获取按键消息，返回1表示有按键消息，0为无按键消息
void key_IRQHandler(void);                      //
 

#endif 
//*****************************END**********************************
