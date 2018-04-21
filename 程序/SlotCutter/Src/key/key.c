/************************* (C) COPYRIGHT 2013 DZXH ************************
* 文  件  名      : KEY.C
* 作      者      : DZXH@Dream Creater
* 适      用      ：DZXH 开发板V2.1/V1.0。
* 淘  宝  店      : http://shop102062100.taobao.com/
* 版      本      : V1.0
* 日      期      : 2013/11
* 主  控  MCU     : STM32F103VET6
* 编  译  器      : Keil ARM 4.54
* 描      述      : 按键初始化
**************************************************************************/


#include "key.h"
#include "FreeRTOS.h"
 
 #include "queue.h"
 #include "../HT1628.h"

 /************************************************************************
* 函  数  名  : key_check
* 描      述  : 检测key状态
* 输      入  : 按键标号（1-KEY_MAX），目前KEY_MAX=10
* 返      回  : 无.
************************************************************************/
KEY_STATUS_e key_check(uint8_t key)
{
	
			if(checkKey(key))   //KEY_为采样最大误差
				{
					return KEY_DOWN;             //按键被按下
				}
			else
				{
					 return KEY_UP;
				}
}


//定义按键状态结构体
typedef enum
{
    KEY_MSG_EMPTY,      //没有按键消息
    KEY_MSG_NORMAL,     //正常，有按键消息，但不满
    KEY_MSG_FULL,       //按键消息满
} key_msg_e;

#define KEY_MSG_FIFO_SIZE   20      //最多 255，否则需要修改key_msg_front/key_msg_rear类型
KEY_MSG_t       key_msg[KEY_MSG_FIFO_SIZE];
volatile uint8_t     key_msg_front = 0, key_msg_rear = 0; //接收FIFO的指针
volatile uint8_t     key_msg_flag = KEY_MSG_EMPTY;
extern QueueHandle_t led_key_queue; 
 /************************************************************************
* 函  数  名  : send_key_msg
* 描      述  : 
* 输      入  : 
* 返      回  : 无.
************************************************************************/
void send_key_msg(KEY_MSG_t keymsg)
{
 xQueueSendToFront(led_key_queue,&keymsg,0);
}
 /************************************************************************
* 函  数  名  : get_key_msg
* 描      述  : 
* 输      入  : 
* 返      回  : 无.
************************************************************************/
//uint8_t get_key_msg(KEY_MSG_t *keymsg)
//{
//    if(key_msg_flag == KEY_MSG_EMPTY)   //空，直接返回0
//    {
//        return 0;
//    }
//    keymsg->key = key_msg[key_msg_front].key;
//    keymsg->status = key_msg[key_msg_front].status;

//    key_msg_front++;                //由于非空，所以可以直接出队列

//    if(key_msg_front >= KEY_MSG_FIFO_SIZE)
//    {
//        key_msg_front = 0;          //重头开始
//    }

//    if(key_msg_front == key_msg_rear)       //追到屁股了，接收队列空
//    {
//        key_msg_flag = KEY_MSG_EMPTY;
//    }

//    return 1;
//}

 static uint16_t keytime[KEY_MAX];
//判断是否弹起
uint8_t checkIdle(uint8_t keyNum)
{
	if(keytime[keyNum]>0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

 /************************************************************************
* 函  数  名  : key_IRQHandler
* 描      述  : 定时检测key状态
* 输      入  : 无.
* 返      回  : 无.
************************************************************************/
void key_IRQHandler(void)
{

    uint8_t    keynum;

    KEY_MSG_t keymsg;
		static uint8_t hold_trig=0;;
		readKey();
    for(keynum = 0 ; keynum < KEY_MAX; keynum ++)
    {

        if(key_check( keynum)  == KEY_DOWN)
        {
            keytime[keynum]++;

            if(keytime[keynum] <= KEY_DOWN_TIME)
            {
                continue;
            }
            else if(keytime[keynum] == KEY_DOWN_TIME + 1 )        //有效按下
            {
                keymsg.key = keynum;
                keymsg.status = KEY_DOWN;
                send_key_msg(keymsg);
            }
            else if(keytime[keynum] <= KEY_HOLD_TIME)
            {
                continue;
            }
            else if(keytime[keynum]  == KEY_HOLD_TIME + 1)    //有效按住
            {
                keymsg.key = keynum;;
                keymsg.status = KEY_HOLD;
                send_key_msg(keymsg);
								hold_trig = 1;
              //  keytime[keynum] = 0;
            }
            else
            {
                keytime[keynum] = KEY_DOWN_TIME + 1;
            }
        }
        else
        {
            if(keytime[keynum] > KEY_DOWN_TIME)
            {		
								if(hold_trig==0)
								{
                keymsg.key = keynum;;
                keymsg.status = KEY_UP;
                send_key_msg(keymsg);           //按键弹起
								}
								else
								{
									hold_trig = 0;
								}
            }
            keytime[keynum] = 0;
        }
    }
}
/************************************************************************
* 函  数  名  : absi
* 描      述  : 求绝对值函数
* 输      入  : 无.
* 返      回  : 无.
************************************************************************/
uint16_t absi(int16_t data_s)
{
	return data_s>=0?data_s:-data_s;
}
//*****************************END**********************************
