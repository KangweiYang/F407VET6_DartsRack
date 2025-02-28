//
// Created by root on 24-2-29.
//

#include "../Inc/Servo.h"
#include "../../STEPPER/Inc/Stepper.h"
#include "tim.h"
#include "usart.h"
#include "main.h"
#include "../../RS485/Inc/RS485.h"
#include "../../SHOOT_PROCESS/Inc/ShootProcess.h"

extern double targetVel[4];
extern int32_t tension1, tensionL;
extern double targetTen[2];
extern int motor0Flag, motor1Flag, motor2Flag, motor3Flag, stepper0Flag, stepper1Flag;
extern int shootFlag;

void DartLoad1(uint16_t delayTime){
    targetVel[1] = 3000;
    targetVel[2] = 3000;
//    HAL_Delay(delayTime); //3700
    while(((HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin) == HALL_DETECTED) ||
           HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin) == HALL_DETECTED) &&
          (targetVel[1] != 0 || targetVel[2] != 0)){

        if(HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin) != HALL_DETECTED)   targetVel[2] = 0;

        if(HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin) != HALL_DETECTED) targetVel[1] = 0;
    }
    targetVel[1] = 3000;
    targetVel[2] = 3000;
    HAL_Delay(delayTime);
    targetVel[1] = 0;
    targetVel[2] = 0;
}

void ServoInit(void) {
    HAL_TIM_PWM_Init(&htim2);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void Delay(int32_t delayTime){
    while(delayTime > 0){
        delayTime -= 58;
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
    }
//    HAL_Delay(delayTime);
}

void ServoSet(int channel, int angle, int delay) {
    switch (channel) {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, angle);
            Delay(delay);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angle);
            Delay(delay);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, angle);
            Delay(delay);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, angle);
            Delay(delay);
            break;
    }
}

void ServoGraspDart(void) {
//    ServoSet(SERVO_UP_DOWN, 65, 200);
    ServoSet(SERVO_UP_DOWN, SERVO_UP_DOWN_UP, 0);                         //STOP up

//    if(IsDartReadyToLoad() == 0) {
//        while (!IsDartReadyToLoad()) {
//            tension1 = RS485_1_GetTension();
//            tensionL = RS485_2_GetTension();
//        }
//    }
    while (FeedFSMState() != 3){
            tension1 = RS485_1_GetTension();
            tensionL = RS485_2_GetTension();
    }
    DartFeedLoading();
#if SHOOT_INFO
        printf("GRASP\n");
#endif
        ServoSet(SERVO_GRASP, SERVO_GRASP_GRASP, 300);                         //Grasp
        ServoSet(SERVO_UP_DOWN, SERVO_UP_DOWN_DOWN, 500);                      //Start down
        DartReset();
        /*
        while((tension1 != targetTen[0]) || (tensionL != targetTen[1])){
            tension1 = RS485_1_GetTension();
            tensionL = RS485_2_GetTension();
            printf("Ten1: %d, Ten2: %d\n", tension1, tensionL);
        }
        stepper0Flag = 0;
        stepper1Flag = 0;
         */
        ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, 2500);                        //Release
        ServoSet(SERVO_UP_DOWN, SERVO_UP_DOWN_UP, 1);                      //Start up
        DartFeedLoadingEnd();
        if(shootFlag < 4)
            DartFeedStartUp();
        else
            StepperSetSpeed(STEPPER4, 0);
}
/*
void ServoGraspDart(void) {
//    ServoSet(SERVO_UP_DOWN, 65, 200);
    ServoSet(SERVO_UP_DOWN, 20, 0);                         //STOP up

    StepperStart(STEPPER4);
    while (HAL_GPIO_ReadPin(DART_STOP_SW_GPIO_Port, DART_STOP_SW_Pin) == GPIO_PIN_SET){
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
        StepperSetSpeed(STEPPER4, 850);
    }
    if (HAL_GPIO_ReadPin(DART_STOP_SW_GPIO_Port, DART_STOP_SW_Pin) == GPIO_PIN_RESET) {
        StepperSetSpeed(STEPPER4, -500);
#if SHOOT_INFO
        printf("GRASP\n");
#endif
        ServoSet(SERVO_GRASP, 97, 300);                         //Grasp
        ServoSet(SERVO_UP_DOWN, 102, 1);                      //Start down
        StepperStop(STEPPER4);
        DartReset();
        ServoSet(SERVO_GRASP, 119, 500);                        //Release
        ServoSet(SERVO_UP_DOWN, 18, 1);                      //Start up

    }
}
*/
