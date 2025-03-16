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
#define HALL_BACK_SW___Pin GPIO_PIN_4
#define HALL_BACK_SW___GPIO_Port GPIOA
#define HALL_LEFT_SW___Pin GPIO_PIN_5
#define HALL_LEFT_SW___GPIO_Port GPIOA
#define RELAY_CONTROL_Pin GPIO_PIN_4
#define RELAY_CONTROL_GPIO_Port GPIOC
#define HALL_FEED_BOTTOM_SW_Pin GPIO_PIN_5
#define HALL_FEED_BOTTOM_SW_GPIO_Port GPIOC
#define RS485_2_RX_TX_CONTROL_Pin GPIO_PIN_15
#define RS485_2_RX_TX_CONTROL_GPIO_Port GPIOE
#define DART_STOP_SW_Pin GPIO_PIN_14
#define DART_STOP_SW_GPIO_Port GPIOB
#define YAW_STEPPER_PUL_Pin GPIO_PIN_12
#define YAW_STEPPER_PUL_GPIO_Port GPIOD
#define HALL_RIGHT_SW_Pin GPIO_PIN_13
#define HALL_RIGHT_SW_GPIO_Port GPIOD
#define HALL_RIGHT_SW_EXTI_IRQn EXTI15_10_IRQn
#define HALL_LEFT_SW_Pin GPIO_PIN_14
#define HALL_LEFT_SW_GPIO_Port GPIOD
#define HALL_LEFT_SW_EXTI_IRQn EXTI15_10_IRQn
#define HALL_BACK_SW_Pin GPIO_PIN_15
#define HALL_BACK_SW_GPIO_Port GPIOD
#define HALL_BACK_SW_EXTI_IRQn EXTI15_10_IRQn
#define SW10_Pin GPIO_PIN_9
#define SW10_GPIO_Port GPIOC
#define SONIC_RANGE_TRIG1_Pin GPIO_PIN_8
#define SONIC_RANGE_TRIG1_GPIO_Port GPIOB
#define SONIC_RANGE_TRIG2_Pin GPIO_PIN_9
#define SONIC_RANGE_TRIG2_GPIO_Port GPIOB
#define SONIC_RANGE_ECHO1_Pin GPIO_PIN_0
#define SONIC_RANGE_ECHO1_GPIO_Port GPIOE
#define SONIC_RANGE_ECHO1_EXTI_IRQn EXTI0_IRQn
#define SONIC_RANGE_ECHO2_Pin GPIO_PIN_1
#define SONIC_RANGE_ECHO2_GPIO_Port GPIOE
#define SONIC_RANGE_ECHO2_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

#define CONT_TO_READY_TO_SHOOT  10         //100ms
#define SHOOT_BREAK  300         //100ms
#define START_TENSION   70

//stepper
#define NO_STEPPER_TEST 0
#define STEPPER1    &htim1, TIM_CHANNEL_1   //PA8
#define STEPPER2    &htim3, TIM_CHANNEL_1   //PA6
#define STEPPER2_VS_1   1
#define STILL_RATE  1.4
#define STEPPER_CHANGE_TO_SMALL_K   14
#define STEPPER1BIGKP 450
#define STEPPER1BIGKD 150
#define STEPPER2BIGKP STEPPER1BIGKP*STEPPER2_VS_1
#define STEPPER2BIGKD STEPPER1BIGKD*STEPPER2_VS_1
#define STEPPER1SMALLKP 120
#define STEPPER1SMALLKD 80
#define STEPPER2SMALLKP STEPPER1SMALLKP*STEPPER2_VS_1
#define STEPPER2SMALLKD STEPPER1SMALLKD*STEPPER2_VS_1
#define STEPPER1SMALLSMALLKP 80
#define STEPPER1SMALLSMALLKD 80
#define STEPPER2SMALLSMALLKP STEPPER1SMALLSMALLKP*STEPPER2_VS_1
#define STEPPER2SMALLSMALLKD STEPPER1SMALLSMALLKD*STEPPER2_VS_1
#define STEPPER1SMALLSMALLSMALLKP 30
#define STEPPER1SMALLSMALLSMALLKD 70
#define STEPPER2SMALLSMALLSMALLKP STEPPER1SMALLSMALLSMALLKP*STEPPER2_VS_1
#define STEPPER2SMALLSMALLSMALLKD STEPPER1SMALLSMALLSMALLKD*STEPPER2_VS_1
#define STEPPER1SMALLSMALLSMALLSMALLKP 16
#define STEPPER1SMALLSMALLSMALLSMALLKD 70
#define STEPPER2SMALLSMALLSMALLSMALLKP 16
#define STEPPER2SMALLSMALLSMALLSMALLKD 70

//servo
#define SERVO_UP_DOWN   1
#define SERVO_GRASP     2
#define SERVO_UP_DOWN_UP    20//SERVO_TRIGGER_MIDDLE//20
#define SERVO_UP_DOWN_DOWN  120
#define SERVO_GRASP_GRASP   104
#define SERVO_GRASP_RELEASE 120//SERVO_TRIGGER_MIDDLE//120
#define SERVO_TRIGGER_SHOOT 20
#define SERVO_TRIGGER_MIDDLE    40
#define SERVO_TRIGGER_RESET 65

//#define STEPPER3    &htim4, TIM_CHANNEL_1   //PD12
#define STEPPER4    &htim9, TIM_CHANNEL_1   //PE5
#define MOTOR_NUM  6
#define RX_BUFF_LENGTH  10000

#define USE_CAN1    1
#define USE_CAN2    0

#define HALL_DETECTED   GPIO_PIN_RESET

//print define
#define CAN_INFO    0
#define MOTOR_INFO  0
#define ADC_DMA_INFO    0
#define HALL_INFO   1
#define TEN_INFO    1

//stepper1,2 dir
#define STEPPER_INFO    1

#define STEPPER1_MAX_PUL  1900
#define STEPPER2_MAX_PUL  STEPPER1_MAX_PUL*STEPPER2_VS_1
#define STEPPER1_2_MIN_CHANGE   (1200 / 20)

#define STEPPER1_2_DIR  -1

extern double lastBias;
extern double posKpStepper0, posKiStepper0, posKdStepper0;
extern double posKpStepper1, posKiStepper1, posKdStepper1;
#define STEPPER1_Kp   (-posKpStepper0 * ((double) tension1 - targetTen[0]) + posKdStepper0 * ((double) tension1 - targetTen[0] - lastBias))
#define STEPPER2_Kp   (-posKpStepper1 * (targetTen[1] - (double) tensionLL) + posKdStepper1 * (targetTen[1] - (double) tensionLL - lastBias))

//rs485
#define HRS485_1_USART  &huart2
#define HRS485_2_USART  &huart3

#define RS485_INFO  0
#define RS485_LIGHT_INFO    1

//shoot progress
#define SHOOT_INFO  1
#define RESET_SPEED 700
#define LOAD_SPEED  4000
#define LOAD_DELAY  400
#define RELEASE_SPEED   -3000
#define ERROR_RELEASE_SPEED   -2000
#define RELEASE_DELAY_TENION_CONTROL    1000    //释放后相隔多少ms后开始拉力闭环控制
#define SHOOT_SPEED -3000
#define WAIT_TIMES  2           //当连续出现几次目标拉力值时发射
#define LOAD_BACK_TIME_100MS    1       //2是100ms，1是不后退
#define BALANCE_OFFSET_SPEED    1900    //弓轮补偿速度
#define BALANCE_OFFSET_MS   4200     //弓轮平衡补偿时间

//judge system uart
#define JUDGE_INFO  0
#define JUDGE020A_INFO 1
#define JUDGE0105_INFO 1
#define JUDGE0001_INFO 1
#define RX6_BUFF_LENGTH 10000

//remote
#define UART5_INFO  0

//sonic
#define SONIC_ENABLE    0

#define TENSION1_DATA_ADDRESS    4
#define TENSION2_DATA_ADDRESS    3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
