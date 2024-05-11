//
// Created by 17200 on 2024/5/1.
//

#include "../Inc/ShootProcess.h"
#include "main.h"
#include "../../RS485/Inc/RS485.h"
#include "usart.h"

extern int motor0Flag, motor1Flag, motor2Flag, motor3Flag, stepper0Flag, stepper1Flag;
extern double targetVel[4];
extern int32_t tension1;
extern int32_t tensionL;

extern int furTarTen[4];
extern int furTarYaw[4];

void DartLoad(void) {
    motor0Flag = 0;
    motor1Flag = 1;
    motor2Flag = 0;
    motor3Flag = 1;
    targetVel[1] = LOAD_SPEED;
    targetVel[3] = LOAD_SPEED;
    targetVel[0] = 0;
//    HAL_Delay(2000); //3700
    while (HAL_GPIO_ReadPin(HALL_BACK_SW_GPIO_Port, HALL_BACK_SW_Pin) != HALL_DETECTED) {
        targetVel[0] = 0;
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
    }
#if SHOOT_INFO
    printf("DART LOAD OK!\n");
#endif
    HAL_Delay(100);
    motor0Flag = 0;
    motor1Flag = 0;
    motor2Flag = 0;
    motor3Flag = 0;
    targetVel[1] = 0;
    targetVel[3] = 0;
}

void DartRelease(void) {
    motor0Flag = 0;
    motor1Flag = 1;
    motor2Flag = 0;
    motor3Flag = 1;
    targetVel[1] = RELEASE_SPEED;
    targetVel[3] = RELEASE_SPEED;
    targetVel[0] = 0;
//    HAL_Delay(delayTime); //2100
    while (((HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin) != HALL_DETECTED) ||
            HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin) != HALL_DETECTED) &&
           (targetVel[1] != 0 || targetVel[3] != 0)) {
        if (HAL_GPIO_ReadPin(HALL_BACK_SW_GPIO_Port, HALL_BACK_SW_Pin) != HALL_DETECTED) {
            motor0Flag = 1;
            motor1Flag = 0;
            motor2Flag = 0;
            motor3Flag = 0;
            targetVel[1] = 0;
            targetVel[3] = 0;
            targetVel[0] = SHOOT_SPEED;
            tension1 = RS485_1_GetTension();
            tensionL = RS485_2_GetTension();
            targetVel[0] = 0;
            motor0Flag = 0;
            DartLoad();
            motor0Flag = 0;
            motor1Flag = 1;
            motor2Flag = 0;
            motor3Flag = 1;
            targetVel[1] = RELEASE_SPEED;
            targetVel[3] = RELEASE_SPEED;
            targetVel[0] = 0;
        }

        targetVel[0] = 0;
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
        if (HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin) == HALL_DETECTED) targetVel[3] = 0;

        if (HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin) == HALL_DETECTED) targetVel[1] = 0;
    }
#if SHOOT_INFO
    printf("DART RELEASE OK!\n");
#endif
    motor0Flag = 0;
    motor1Flag = 0;
    motor2Flag = 0;
    motor3Flag = 0;
    targetVel[1] = 0;
    targetVel[3] = 0;
}

void DartShoot(void) {
    motor0Flag = 1;
    motor1Flag = 0;
    motor2Flag = 0;
    motor3Flag = 0;
    targetVel[0] = SHOOT_SPEED;
//    HAL_Delay(delayTime);
    while (HAL_GPIO_ReadPin(HALL_BACK_SW_GPIO_Port, HALL_BACK_SW_Pin) == HALL_DETECTED){
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
    }
#if SHOOT_INFO
    printf("DART SHOOT OK!\n");
#endif
    HAL_Delay(100);
    targetVel[0] = 0;
    motor0Flag = 0;
}
