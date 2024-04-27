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

extern double targetVel[4];

void DartLoad1(uint16_t delayTime){
    targetVel[1] = 3000;
    targetVel[3] = 3000;
//    HAL_Delay(delayTime); //3700
    while(((HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin) == HALL_DETECTED) ||
           HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin) == HALL_DETECTED) &&
          (targetVel[1] != 0 || targetVel[3] != 0)){

        if(HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin) != HALL_DETECTED)   targetVel[3] = 0;

        if(HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin) != HALL_DETECTED) targetVel[1] = 0;
    }
    targetVel[1] = 3000;
    targetVel[3] = 3000;
    HAL_Delay(delayTime);
    targetVel[1] = 0;
    targetVel[3] = 0;
}

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
    ServoSet(SERVO_UP_DOWN, 20, 0);                         //STOP up

    StepperStart(STEPPER3);
    uint16_t freq = 50, cont = 0;
    StepperStart(STEPPER3);
    while (HAL_GPIO_ReadPin(DART_STOP_SW_GPIO_Port, DART_STOP_SW_Pin) == GPIO_PIN_SET){
        if(freq < 850) {
//            cont++;
//            if (cont == 1) {
//                cont = 0;
                freq += 10;

//            }
            StepperSetSpeed(STEPPER3, freq);
        }
//        printf("freq= %d\n", freq);
    }
    if (HAL_GPIO_ReadPin(DART_STOP_SW_GPIO_Port, DART_STOP_SW_Pin) == GPIO_PIN_RESET) {
        StepperSetSpeed(STEPPER3, -500);
#if SHOOT_INFO
        printf("GRASP\n");
#endif
        ServoSet(SERVO_GRASP, 97, 300);                         //Grasp
        ServoSet(SERVO_UP_DOWN, 102, 1500);                      //Start down
        StepperStop(STEPPER3);
        ServoSet(SERVO_GRASP, 119, 2000);                        //Release
        ServoSet(SERVO_UP_DOWN, 18, 1);                      //Start up

    }
}
