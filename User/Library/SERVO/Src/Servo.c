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
extern int servo5comp;

void DartLoad1(uint16_t delayTime) {
    targetVel[1] = 3000;
    targetVel[2] = 3000;
//    HAL_Delay(delayTime); //3700
    while (((HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin) == HALL_DETECTED) ||
            HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin) == HALL_DETECTED) &&
           (targetVel[1] != 0 || targetVel[2] != 0)) {

        if (HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin) != HALL_DETECTED) targetVel[2] = 0;

        if (HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin) != HALL_DETECTED) targetVel[1] = 0;
    }
    targetVel[1] = 3000;
    targetVel[2] = 3000;
    HAL_Delay(delayTime);
    targetVel[1] = 0;
    targetVel[2] = 0;
}

void ServoInit(void) {
    HAL_TIM_PWM_Init(&htim2);
    HAL_TIM_PWM_Init(&htim5);
    HAL_TIM_PWM_Init(&htim8);
    HAL_TIM_PWM_Init(&htim12);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

}

void Delay(int32_t delayTime) {
    while (delayTime > 0) {
        delayTime -= 58;
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
    }
//    HAL_Delay(delayTime);
}

void ServoSet(int channel, int angle, int delay) {
    switch (channel) {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, angle);
            Delay(delay);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angle);
            Delay(delay);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, angle);
            Delay(delay);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, angle);
            Delay(delay);
            break;
        case 5:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, angle);
            Delay(delay);
            break;
//            servo5comp == angle;
//            Delay(delay);
//            break;
    }
}

void ServoGraspDart(void) {
#if OLD_FEED
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
            ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, SERVO_PUTDOWN_DELAY);                        //Release
            ServoSet(SERVO_UP_DOWN, SERVO_UP_DOWN_UP, 1);                      //Start up
            DartFeedLoadingEnd();
            if(shootFlag < 4)
                DartFeedStartUp();
            else
                StepperSetSpeed(STEPPER4, 0);
#endif
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

//NEW FEED
void ServoLR_ToNextDart(int dartSerial) {
    if(dartSerial == 5) dartSerial = TEST_SHOOT_TO_TEST_FEED;
    switch (dartSerial) {
        case 3:
            ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, 0);
            ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_LEFT, 0);
            break;
        case 4:
            ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, 0);
            ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_RIGHT, 0);
            break;
    }
}

void ServoUpDown_DownToGrasp(int dartSerial) {
    if(dartSerial == 5) dartSerial = TEST_SHOOT_TO_TEST_FEED;
    switch (dartSerial) {
        case 3:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_DOWN, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_DOWN, 0);
            break;
        case 4:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_DOWN, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_DOWN, 0);
            break;
    }
}

void ServoGrasp_GraspNextDart(int dartSerial) {
    if(dartSerial == 5) dartSerial = TEST_SHOOT_TO_TEST_FEED;
    switch (dartSerial) {
        case 3:
            ServoSet(SERVO_GRASP, SERVO_GRASP_GRASP, 0);
            break;
        case 4:
            ServoSet(SERVO_GRASP, SERVO_GRASP_GRASP, 0);
            break;
    }
}

void ServoUD_UpToMove(int dartSerial) {
    if(dartSerial == 5) dartSerial = TEST_SHOOT_TO_TEST_FEED;
    switch (dartSerial) {
        case 3:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_UP, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_UP, 0);
            break;
        case 4:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_UP, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_UP, 0);
            break;
    }
}

void ServoLR_ToMiddle(int dartSerial) {
    if(dartSerial == 5) dartSerial = TEST_SHOOT_TO_TEST_FEED;
    switch (dartSerial) {
        case 2:
            ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_MIDDLE, 0);
            break;
        case 3:
            ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_MIDDLE, 0);
            break;
        case 4:
            ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_MIDDLE, 0);
            break;
    }
}

void ServoUD_DownToRelease(int dartSerial) {
    if(dartSerial == 5) dartSerial = TEST_SHOOT_TO_TEST_FEED;
    switch (dartSerial) {
        case 2:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_SHOOT_DOWN, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_SHOOT_DOWN, 0);
            break;
        case 3:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_SHOOT_DOWN, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_SHOOT_DOWN, 0);
            break;
        case 4:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_SHOOT_DOWN, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_SHOOT_DOWN, 0);
            break;
    }
}

void ServoGrasp_Realease(int dartSerial) {
    if(dartSerial == 5) dartSerial = TEST_SHOOT_TO_TEST_FEED;
    switch (dartSerial) {
        case 2:
            ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, 0);
            break;
        case 3:
            ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, 0);
            break;
        case 4:
            ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, 0);
            break;
    }
}

void ServoUD_UpToAvoidCrash(int dartSerial) {
    if(dartSerial == 5) dartSerial = TEST_SHOOT_TO_TEST_FEED;
    switch (dartSerial) {
        case 1:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_UP, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_UP, 0);
            break;
        case 2:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_UP, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_UP, 0);
            break;
        case 3:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_UP, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_UP, 0);
            break;
        case 4:
            ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_UP, 0);
            ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_UP, 0);
            break;
    }
}

void ServoLR_ToNotEdge(int dartSerial) {
    if(dartSerial == 5) dartSerial = TEST_SHOOT_TO_TEST_FEED;
    printf("TO NOT EDGE, dartSerial = %d\n", dartSerial);
    switch (dartSerial) {
        case 1:
            ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_NOT_EDGE, 0);
            break;
        case 2:
            ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_NOT_EDGE, 0);
            break;
        case 3:
            ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_NOT_EDGE, 0);
            break;
        case 4:
            ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_NOT_EDGE, 0);
            break;
    }
}
