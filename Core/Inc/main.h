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
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOE
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
#define HALL_BACK_SW_Pin GPIO_PIN_4
#define HALL_BACK_SW_GPIO_Port GPIOA
#define HALL_LEFT_SW_Pin GPIO_PIN_5
#define HALL_LEFT_SW_GPIO_Port GPIOA
#define HALL_RIGHT_SW_Pin GPIO_PIN_5
#define HALL_RIGHT_SW_GPIO_Port GPIOC
#define RS485_2_RX_TX_CONTROL_Pin GPIO_PIN_15
#define RS485_2_RX_TX_CONTROL_GPIO_Port GPIOE
#define DART_STOP_SW_Pin GPIO_PIN_14
#define DART_STOP_SW_GPIO_Port GPIOB
#define HALL_RIGHT_SW__Pin GPIO_PIN_13
#define HALL_RIGHT_SW__GPIO_Port GPIOD
#define HALL_LEFT_SW__Pin GPIO_PIN_14
#define HALL_LEFT_SW__GPIO_Port GPIOD
#define HALL_BACK_SW__Pin GPIO_PIN_15
#define HALL_BACK_SW__GPIO_Port GPIOD
#define SW10_Pin GPIO_PIN_9
#define SW10_GPIO_Port GPIOC
#define SONIC_RANGE_TRIG1_Pin GPIO_PIN_8
#define SONIC_RANGE_TRIG1_GPIO_Port GPIOB
#define SONIC_RANGE_TRIG2_Pin GPIO_PIN_9
#define SONIC_RANGE_TRIG2_GPIO_Port GPIOB
#define SONIC_RANGE_ECHO1_Pin GPIO_PIN_0
#define SONIC_RANGE_ECHO1_GPIO_Port GPIOE
#define SONIC_RANGE_ECHO2_Pin GPIO_PIN_1
#define SONIC_RANGE_ECHO2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

//stepper
#define STEPPER1    &htim1, TIM_CHANNEL_1   //PA8
#define STEPPER2    &htim3, TIM_CHANNEL_1   //PA6
#define STEPPER3    &htim4, TIM_CHANNEL_1   //PD12
#define STEPPER4    &htim9, TIM_CHANNEL_1   //PE5
#define MOTOR_NUM  6
#define RX_BUFF_LENGTH  10000

#define USE_CAN1    1
#define USE_CAN2    0

#define HALL_DETECTED   GPIO_PIN_RESET

//print define
#define ADC_DMA_INFO    0

//stepper1,2 dir
#define STEPPER_INFO    1

#define STEPPER1_2_MAX_PUL  3600
#define STEPPER1_2_MIN_CHANGE   (1200 / 20)

#define STEPPER1_2_DIR  -1

extern double lastBias;
#define STEPPER1_Kp   (20 * ((double) tension1 - targetTen[0]) - 10 * ((double) tension1 - targetTen[0] - lastBias))
#define STEPPER2_Kp   (60 * (targetTen[1] - (double) tensionLL) - 15 * (targetTen[1] - (double) tensionLL - lastBias))

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
