//
// Created by root on 24-2-29.
//

#include "../Inc/Servo.h"
#include "../../STEPPER/Inc/Stepper.h"
#include "tim.h"
#include "usart.h"
#include "main.h"

#define SERVO_UP_DOWN   1
#define SERVO_GRASP     2
#define DART_STOP_SW_PORT   SW1_GPIO_Port
#define DART_STOP_SW_PIN    SW1_Pin

void ServoInit(void) {
    HAL_TIM_PWM_Init(&htim2);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void ServoSet(int channel, int angle, int delay) {
    switch (channel) {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, angle);
            HAL_Delay(delay);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angle);
            HAL_Delay(delay);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, angle);
            HAL_Delay(delay);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, angle);
            HAL_Delay(delay);
            break;
    }
}

void ServoGraspDart(void) {
//    ServoSet(SERVO_UP_DOWN, 65, 200);
    ServoSet(SERVO_UP_DOWN, 18, 0);                         //STOP up

    StepperStart(STEPPER1);
    uint16_t freq = 50, cont = 0;
    while (HAL_GPIO_ReadPin(DART_STOP_SW_PORT, DART_STOP_SW_PIN) == GPIO_PIN_RESET){
        if(freq < 1000) {
//            cont++;
//            if (cont == 1) {
//                cont = 0;
                freq++;
//            }
            StepperSetSpeed(STEPPER1, freq);
        }
        printf("freq= %d\n", freq);
    }
    if (HAL_GPIO_ReadPin(DART_STOP_SW_PORT, DART_STOP_SW_PIN) == GPIO_PIN_SET) {
        StepperStop(STEPPER1);
        ServoSet(SERVO_GRASP, 99, 300);                         //Grasp
        ServoSet(SERVO_UP_DOWN, 102, 2900);                      //Start down
        ServoSet(SERVO_GRASP, 109, 500);                        //Release
        ServoSet(SERVO_UP_DOWN, 18, 2500);                      //Start up

    }
}
