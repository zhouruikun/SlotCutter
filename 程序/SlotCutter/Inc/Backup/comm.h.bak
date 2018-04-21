#ifndef __COMM_H
#define __COMM_H
#include "main.h"
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

#define STANDBY_TIME 100

typedef struct {
	uint16_t diameter[5];
	uint16_t slotNumber[5];
	uint16_t micaWidth[5];
	uint16_t micaPreTrace[5];
	uint16_t motoCompensation[5];
	uint16_t millingMethod[5];
	uint16_t motoDirection[5];
	uint16_t feedDelayTime[5];
	uint16_t feedActionTime[5];
	uint16_t productDeviation[5];
	uint16_t feedPreCutTime[5];
	uint16_t millingPreBackTime[5];
	uint16_t plusNumberOfMoto[5];
	uint16_t stepMotoSlopeTime[5];
	uint16_t stepMotoRunSpeed[5];
	uint16_t stepMotoFinishTime[5];	
	short compareThreshold[5];
	uint16_t qualifiedRate[5];
}SettingType;
union SettingUnion{
	SettingType SettingStruct;
	uint16_t SettingArray[16][5];
};
extern union SettingUnion Setting ;
#endif

