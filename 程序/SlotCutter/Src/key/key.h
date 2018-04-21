/************************* (C) COPYRIGHT 2013 DZXH ************************
* ��  ��  ��      : KEY.H
* ��      ��      : DZXH@Dream Creater
* ��      ��      ��DZXH ������V2.1/V1.0��
* ��  ��  ��      : http://shop102062100.taobao.com/
* ��      ��      : V1.0
* ��      ��      : 2013/11
* ��  ��  MCU     : STM32F103VET6
* ��  ��  ��      : Keil ARM 4.54
* ��      ��      : ������ʼ��ͷ�ļ�
**************************************************************************/
#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f1xx_hal.h"
#include "main.h"

 

 

//�����Ƕ��尴����ʱ�䣬��λΪ �� 50ms
#define KEY_DOWN_TIME   1
#define KEY_HOLD_TIME   20         //���253��������Ҫ�޸�keytime������

#define KEY_OFFSET 4096
#define KEY_ 200
 
//�����˿ڵ�ö��
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



//key״̬�궨��
typedef enum
{
    KEY_DOWN  =   0,         //��������ʱ��Ӧ��ƽ
    KEY_UP    =   1,         //��������ʱ��Ӧ��ƽ
    KEY_HOLD,

} KEY_STATUS_e;

//��������״̬�ṹ��
typedef struct
{
    uint16_t           key;
    KEY_STATUS_e    status;
} KEY_MSG_t;

void    KEY_init(KEY_e key);                    // KEY��ʼ��    ����
void    KEY_ALL_init(void);                     //ȫ��Key��ʼ��
KEY_STATUS_e key_check(uint8_t key);              //���key״̬
 

//��ʱɨ�谴��
uint8_t get_key_msg(KEY_MSG_t *keymsg);                  //��ȡ������Ϣ������1��ʾ�а�����Ϣ��0Ϊ�ް�����Ϣ
void key_IRQHandler(void);                      //
 

#endif 
//*****************************END**********************************
