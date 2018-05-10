/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Cutter_Motor_Pin GPIO_PIN_13
#define Cutter_Motor_GPIO_Port GPIOC
#define Saved_Output0_Pin GPIO_PIN_14
#define Saved_Output0_GPIO_Port GPIOC
#define Saved_Output1_Pin GPIO_PIN_15
#define Saved_Output1_GPIO_Port GPIOC
#define Material_R_Push_Pin GPIO_PIN_0
#define Material_R_Push_GPIO_Port GPIOA
#define X_CS_Pin GPIO_PIN_1
#define X_CS_GPIO_Port GPIOA
#define X_UD_Pin GPIO_PIN_2
#define X_UD_GPIO_Port GPIOA
#define X_INC_Pin GPIO_PIN_3
#define X_INC_GPIO_Port GPIOA
#define Footer_Pull_Pin GPIO_PIN_4
#define Footer_Pull_GPIO_Port GPIOA
#define Footer_Push_Pin GPIO_PIN_5
#define Footer_Push_GPIO_Port GPIOA
#define Step_Motor_Plus_Pin GPIO_PIN_0
#define Step_Motor_Plus_GPIO_Port GPIOB
#define Cutter_Pull_Pin GPIO_PIN_1
#define Cutter_Pull_GPIO_Port GPIOB
#define Cutter_Push_Pin GPIO_PIN_10
#define Cutter_Push_GPIO_Port GPIOB
#define Step_Motor_Dir_Pin GPIO_PIN_11
#define Step_Motor_Dir_GPIO_Port GPIOB
#define CCD_Input_Pin GPIO_PIN_12
#define CCD_Input_GPIO_Port GPIOB
#define TM1628_DIO_Pin GPIO_PIN_13
#define TM1628_DIO_GPIO_Port GPIOB
#define TM1628_CLK_Pin GPIO_PIN_14
#define TM1628_CLK_GPIO_Port GPIOB
#define TM1628_STB_Pin GPIO_PIN_15
#define TM1628_STB_GPIO_Port GPIOB
#define Cutter_Sensor_Start_Pin GPIO_PIN_8
#define Cutter_Sensor_Start_GPIO_Port GPIOA
#define Cutter_Sensor_Start_EXTI_IRQn EXTI9_5_IRQn
#define Cutter_Sensor_Mid_Pin GPIO_PIN_9
#define Cutter_Sensor_Mid_GPIO_Port GPIOA
#define Cutter_Sensor_Mid_EXTI_IRQn EXTI9_5_IRQn
#define Cutter_Sensor_End_Pin GPIO_PIN_10
#define Cutter_Sensor_End_GPIO_Port GPIOA
#define Cutter_Sensor_End_EXTI_IRQn EXTI15_10_IRQn
#define Footer_Sensor_Start_Pin GPIO_PIN_11
#define Footer_Sensor_Start_GPIO_Port GPIOA
#define Footer_Sensor_Start_EXTI_IRQn EXTI15_10_IRQn
#define Footer_Sensor_End_Pin GPIO_PIN_15
#define Footer_Sensor_End_GPIO_Port GPIOA
#define Footer_Sensor_End_EXTI_IRQn EXTI15_10_IRQn
#define Material_R_Sensor_Start_Pin GPIO_PIN_3
#define Material_R_Sensor_Start_GPIO_Port GPIOB
#define Material_R_Sensor_Start_EXTI_IRQn EXTI3_IRQn
#define Material_R_Sensor_End_Pin GPIO_PIN_4
#define Material_R_Sensor_End_GPIO_Port GPIOB
#define Material_R_Sensor_End_EXTI_IRQn EXTI4_IRQn
#define Key_Start_Pin GPIO_PIN_5
#define Key_Start_GPIO_Port GPIOB
#define Key_Start_EXTI_IRQn EXTI9_5_IRQn
#define Key_Stop_Pin GPIO_PIN_6
#define Key_Stop_GPIO_Port GPIOB
#define Key_Stop_EXTI_IRQn EXTI9_5_IRQn
#define Key_Shutdown_Pin GPIO_PIN_7
#define Key_Shutdown_GPIO_Port GPIOB
#define Key_Shutdown_EXTI_IRQn EXTI9_5_IRQn
#define Saved_Input0_Pin GPIO_PIN_8
#define Saved_Input0_GPIO_Port GPIOB
#define Saved_Input1_Pin GPIO_PIN_9
#define Saved_Input1_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
