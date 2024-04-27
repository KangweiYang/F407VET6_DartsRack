/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
void SystemClock_Config(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RS485_1_RX_TX_CONTROL_Pin GPIO_PIN_6
#define RS485_1_RX_TX_CONTROL_GPIO_Port GPIOE
#define STEPPER1_DIR_Pin GPIO_PIN_0
#define STEPPER1_DIR_GPIO_Port GPIOC
#define STEPPER2_DIR_Pin GPIO_PIN_1
#define STEPPER2_DIR_GPIO_Port GPIOC
#define STEPPER3_DIR_Pin GPIO_PIN_2
#define STEPPER3_DIR_GPIO_Port GPIOC
#define STEPPER4_DIR_Pin GPIO_PIN_3
#define STEPPER4_DIR_GPIO_Port GPIOC
#define RS485_2_RX_TX_CONTROL_Pin GPIO_PIN_15
#define RS485_2_RX_TX_CONTROL_GPIO_Port GPIOE
#define DART_STOP_SW_Pin GPIO_PIN_14
#define DART_STOP_SW_GPIO_Port GPIOB
#define HALL_BACK_SW_Pin GPIO_PIN_15
#define HALL_BACK_SW_GPIO_Port GPIOD
#define HALL_RIGHT_SW_Pin GPIO_PIN_6
#define HALL_RIGHT_SW_GPIO_Port GPIOC
#define HALL_LEFT_SW_Pin GPIO_PIN_7
#define HALL_LEFT_SW_GPIO_Port GPIOC
#define SW10_Pin GPIO_PIN_9
#define SW10_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
//stepper
#define STEPPER1    &htim1, TIM_CHANNEL_1   //PA8
#define STEPPER2    &htim3, TIM_CHANNEL_1   //PA6
#define STEPPER4    &htim4, TIM_CHANNEL_1   //PD12
#define STEPPER3    &htim9, TIM_CHANNEL_1   //PE5
#define MOTOR_NUM  5

#define USE_CAN1    1
#define USE_CAN2    0

#define HALL_DETECTED   GPIO_PIN_RESET

//print define
#define ADC_DMA_INFO    0

//stepper1,2 dir
#define STEPPER_INFO    1

#define STEPPER1_2_MAX_PUL  1200
#define STEPPER1_2_MIN_CHANGE   (1200 / 20)

#define STEPPER1_2_DIR  -1

#define STEPPER1_Kp   20 * (tension1 - targetTen[0])

//rs485
#define HRS485_1_USART  &huart2
#define HRS485_2_USART  &huart3

#define RS485_INFO  0

//shoot progress
#define SHOOT_INFO  1

#define TENSION1_DATA_ADDRESS    4
#define TENSION2_DATA_ADDRESS    3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
