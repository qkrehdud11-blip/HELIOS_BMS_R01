/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_temp_Pin GPIO_PIN_2
#define ADC_temp_GPIO_Port GPIOC
#define ADC_gas_Pin GPIO_PIN_3
#define ADC_gas_GPIO_Port GPIOC
#define DC_PWM1_Pin GPIO_PIN_0
#define DC_PWM1_GPIO_Port GPIOA
#define DC_PWM2_Pin GPIO_PIN_1
#define DC_PWM2_GPIO_Port GPIOA
#define Ultra_Trig_C_Pin GPIO_PIN_1
#define Ultra_Trig_C_GPIO_Port GPIOB
#define Ultra_Trig_L_Pin GPIO_PIN_2
#define Ultra_Trig_L_GPIO_Port GPIOB
#define CAN_CS_Pin GPIO_PIN_12
#define CAN_CS_GPIO_Port GPIOB
#define Ultra_Trig_R_Pin GPIO_PIN_15
#define Ultra_Trig_R_GPIO_Port GPIOB
#define Ultra_ECHO_Right_Pin GPIO_PIN_6
#define Ultra_ECHO_Right_GPIO_Port GPIOC
#define CAN_INT_Pin GPIO_PIN_7
#define CAN_INT_GPIO_Port GPIOC
#define CAN_INT_EXTI_IRQn EXTI9_5_IRQn
#define Ultra_ECHO_Center_Pin GPIO_PIN_8
#define Ultra_ECHO_Center_GPIO_Port GPIOC
#define Ultra_ECHO_Left_Pin GPIO_PIN_9
#define Ultra_ECHO_Left_GPIO_Port GPIOC
#define DCMotor_in4_Pin GPIO_PIN_10
#define DCMotor_in4_GPIO_Port GPIOC
#define DCMotor_in2_Pin GPIO_PIN_11
#define DCMotor_in2_GPIO_Port GPIOC
#define DCMotor_in3_Pin GPIO_PIN_12
#define DCMotor_in3_GPIO_Port GPIOC
#define DCMotor_in1_Pin GPIO_PIN_2
#define DCMotor_in1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
