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
#define SW3_Pin GPIO_PIN_13
#define SW3_GPIO_Port GPIOD
#define SW2_Pin GPIO_PIN_15
#define SW2_GPIO_Port GPIOD
#define SW0_Pin GPIO_PIN_6
#define SW0_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_7
#define SW1_GPIO_Port GPIOC
#define SW20_Pin GPIO_PIN_8
#define SW20_GPIO_Port GPIOC
#define SW10_Pin GPIO_PIN_9
#define SW10_GPIO_Port GPIOC
#define CAN0_RX_Pin GPIO_PIN_0
#define CAN0_RX_GPIO_Port GPIOD
#define CAN1_TX_Pin GPIO_PIN_1
#define CAN1_TX_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
//stepper
#define STEPPER1    &htim1, TIM_CHANNEL_1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
