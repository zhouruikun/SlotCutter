#ifndef __COMM_H
#define __COMM_H
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#define INDEX_VALUE 0
#define INDEX_POINT 1
#define INDEX_MAX 2
#define INDEX_MIN 3
#define INDEX_SIZE 4
#define STATUS_LED_IDLE 1
#define STATUS_LED_INIT 2
#define STATUS_LED_MOD_PARA 3
#define STATUS_LED_STANDBY 4

#define STATUS_MOTO_TEST 5
#define STATUS_MOTO_IDLE 6
#define STATUS_MOTO_RUN 7

#define STATUS_MOTO_ERR 8
#define STATUS_MOTO_INIT 9
#define STATUS_MOTO_STOP 10
#define STATUS_MOTO_TEST2 11
#define STATUS_MOTO_RUN_MODE1 12
#define STATUS_MOTO_RUN_MODE2 13
#define STATUS_MOTO_RUN_MODE3 14
#define STATUS_MOTO_OUT 15	
#define STANDBY_TIME 100

typedef struct {
//参数读出函数 如果第一次则初始化
 
	uint16_t diameter[5];
	uint16_t slotNumber[5];
	uint16_t micaWidth[5];
	uint16_t percentOfPassPreCut[5];
	uint16_t motoCompensation[5];
	uint16_t millingMethod[5];
	uint16_t motoDirection[5];
	uint16_t feedOnTime[5];
	uint16_t percentOfPassOnCut[5];
	uint16_t footerOnTime[5];
	uint16_t feedOffTime[5];
	uint16_t plusNumberOfMoto[5];
	uint16_t mode0DirSwitchTime[5];
	uint16_t stepMotoInitSpeed[5];
	uint16_t stepMotoRunSpeed[5];	
	uint16_t compareThreshold[5];
	uint16_t stepMotoFinishTime[5];
	uint16_t micaPreTrace[5];

}SettingType;
union SettingUnion{
	SettingType SettingStruct;
	uint16_t SettingArray[16][5];
};
extern uint8_t pause_flag;
extern osSemaphoreId Key_KeyTask_BinaryHandle;
extern osSemaphoreId CCD_BinaryHandle;
extern osSemaphoreId Key_Sensor_BinaryHandle;
extern uint8_t moto_msg[2];
extern QueueHandle_t led_key_queue; 
extern QueueHandle_t led_moto_queue; 
extern union SettingUnion Setting ;
#endif

