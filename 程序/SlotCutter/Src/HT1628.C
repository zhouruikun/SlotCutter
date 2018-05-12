 //启蒙电子 LED模块驱动程序STM32 库 
 //	www.51c51.net 0531-85817017 
#include "HT1628.h"
#include "key.h"

unsigned char lum=0x88;//控制显示：显示控制位：1 0 * *  b3 b2 b1 b0 
//b3=0(显示关)；b3=1(显示开)； b2 b1 b0 有8中选择 000 001 010 011 100 101 110 111
//所有lum的值有：0x88   0x89   0x8a  0x8b   0x8c   0x8d   0x8e   0x8f
unsigned char bitdisplay[6]={0x02,0x08,0x01,0x04,0x10,0x20}; //代表哪位数码管 02是第一位08是第二位.....
unsigned char LEDseg[21]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71,0x70,0x72,0x73,0x00,0x40}; //共阴极0-9
unsigned char displaydata[14];//显示缓冲区最大地址00H--0DH ;
unsigned char keyBuff[5];
/*****************发送1个字节数据子程序***********/
void delay(void);
void sendcmddata(unsigned char databye)
{

	STB_LED1;
	delay();
	STB_LED0;
	delay();
	senddata(databye);
}

/*****************数据传输子程序 ***********/
void senddata(unsigned char databye)
{
	unsigned char i;
	for(i=8;i>=1;i--)
	{
		SCK_LED0;
		delay();
		if(databye&0x01)
			{
				SDO_LED1;
			}
		else 
			SDO_LED0;
		databye=databye>>1;
		delay();
		SCK_LED1;
		delay();
	}
}
//初始化子程序：地址增加模式：设置显示模式、设置数据、设置地址、传输显示数据、控制显示
void lint_val_display(void)
{
unsigned char i;
sendcmddata(0x03);//设为七段码11位显示模式
__nop();
sendcmddata(0x40);
__nop();
sendcmddata(0xc0);
__nop();
for(i=0;i<14;i++)
{
senddata(0x00);
}
sendcmddata(lum);
__nop();
} 
/**************显示程序STAR***********************/
void show_all_led(void)
{
unsigned char i;
sendcmddata(0x03);//设为七段码11位显示模式
__nop();
sendcmddata(0x40);
__nop();
sendcmddata(0xc0);
__nop();
for(i=0;i<14;i++)
{
senddata(displaydata[i]);
}
sendcmddata(lum);
}

//判断按键
//	//解析按键返回
//	if(keyBuff[3]==0x01)
//		return KEY_FUN;
//	if(keyBuff[2]==0x08)
//		return KEY_STOP;
//	if(keyBuff[2]==0x01)
//		return KEY_START;
//	if(keyBuff[0]==0x01)
//		return KEY_UP;
//	if(keyBuff[0]==0x08)
//		return KEY_DOWN;
//	if(keyBuff[1]==0x01)
//		return KEY_LEFT;
//	if(keyBuff[1]==0x08)
//		return KEY_RIGHT;
//	return KEY_NULL;
uint8_t checkKey(uint16_t keyNum)
{

	switch(keyNum)
	{
		case Key_START:
					if((keyBuff[2]&0x01)==0x01)
								return 1;
							else
								return 0;
			 
		case Key_STOP:
					if((keyBuff[2])==0x08)
					{
						pause_flag=1;return 1;
					}
							else
								return 0;
		 
		case Key_FUN:
					if((keyBuff[3]&0x01)==0x01)
								return 1;
							else
								return 0;
		 
		case Key_UP:
					if((keyBuff[0]&0x01)==0x01)
								return 1;
							else
								return 0;
				 
		case Key_DOWN:
					if((keyBuff[0]&0x08)==0x08)
								return 1;
							else
								return 0;
		 						
		case Key_L:
					if((keyBuff[1]&0x01)==0x01)
								return 1;
							else
								return 0;
					 
		case Key_R:
						if((keyBuff[1]&0x08)==0x08)
								return 1;
							else
								return 0;
		case Key_OUT_STOP:
						if(HAL_GPIO_ReadPin(Key_Stop_GPIO_Port,Key_Stop_Pin)==GPIO_PIN_RESET)
						{
								 
								return 1;
						}
						else
								return 0;	
		case Key_OUT_START:
						if(HAL_GPIO_ReadPin(Key_Start_GPIO_Port,Key_Start_Pin)==GPIO_PIN_RESET)
								return 1;
						else
								return 0;	
		case Key_OUT_SHUTDOWN:
						if(HAL_GPIO_ReadPin(Key_Shutdown_GPIO_Port,Key_Shutdown_Pin)==GPIO_PIN_RESET)
								return 1;
						else
								return 0;											
		default:
			break;
		
	}
	return 0;
} 

/*****************按键读取子程序 ***********/

void readKey(void)
{
	unsigned char i,j;
	sendcmddata(0x42);
	//设置上拉输入
	SDO_IN;
	delay();
	for(i=0;i<5;i++)
	{
		delay();
		for(j=0;j<8;j++)
		{
			delay();
			keyBuff[i] = keyBuff[i]>>1;
			SCK_LED0;
			delay();
			SCK_LED1;
			delay();
			if(READ_SDO_LED)
			{
				keyBuff[i] = keyBuff[i]|0X80;
			}
		}
	}
  //设置推晚输出
	SDO_OUT;
	
//	//解析按键返回
//	if(keyBuff[3]==0x01)
//		return KEY_FUN;
//	if(keyBuff[2]==0x08)
//		return KEY_STOP;
//	if(keyBuff[2]==0x01)
//		return KEY_START;
//	if(keyBuff[0]==0x01)
//		return KEY_UP;
//	if(keyBuff[0]==0x08)
//		return KEY_DOWN;
//	if(keyBuff[1]==0x01)
//		return KEY_LEFT;
//	if(keyBuff[1]==0x08)
//		return KEY_RIGHT;
//	return KEY_NULL;
}


void delay(void)
{
			__nop();__nop();__nop();__nop();__nop();
			__nop();__nop();__nop();__nop();__nop();
			__nop();__nop();__nop();
			__nop();__nop();__nop();__nop();__nop();
			__nop();__nop();__nop();__nop();__nop();
			__nop();__nop();__nop();
}
///以上程序为LED驱动程序
void HT1621_dis_point(uint8_t pos,uint8_t enable)
{
	if(enable)
	{
		displaydata[pos*2]|=0x80;
	}
	else
	{
		displaydata[pos*2]&= ~0x80;
	}
}

void HT1621_dis_num(uint8_t pos,uint8_t num)
{
	displaydata[pos*2] = LEDseg[num];
 
}
//显示带小数点的数字  最大99999 
void HT1621_dis_float(uint32_t num,uint8_t startPos ,uint8_t len,uint8_t pointPos)
{
	uint8_t disBuff[6];
	uint8_t startPos1;
	disBuff[0] = num%100000/10000;
	disBuff[1] = num%10000/1000;
	disBuff[2] = num%1000/100;
	disBuff[3] = num%100/10;
	disBuff[4] = num%10;
	startPos1=startPos;
	for(uint8_t i = 5-len ; i<5;i++,startPos++)
	{
		displaydata[startPos*2] = LEDseg[disBuff[i]];
	}
	if(pointPos>0)
		displaydata[startPos1*2+(len-pointPos-1)*2]|=0x80;
 
}
void showStandby(void)
{
		displaydata[0] = LEDseg[18];
		displaydata[2] = LEDseg[18];
		displaydata[4] = LEDseg[18];
		displaydata[6] = LEDseg[18];
		displaydata[8] = LEDseg[18];
		display();
}
void display(void)//led显示函数
{
 	show_all_led();	
}
void clear(void)
{
	for(uint8_t i=0;i<14 ;i++)
	{
		displaydata[i] = 0;
	}
 
}
