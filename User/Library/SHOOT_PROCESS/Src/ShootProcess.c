//
// Created by 17200 on 2024/5/1.
//

#include "../Inc/ShootProcess.h"
#include "main.h"
#include "../../RS485/Inc/RS485.h"
#include "../../STEPPER/Inc/Stepper.h"
#include "tim.h"
#include "../../SERVO/Inc/Servo.h"
#include "usart.h"
#include "../../MOTOR/Inc/MotorUnit.h"

extern int motor0Flag, motor1Flag, motor2Flag, motor3Flag, stepper0Flag, stepper1Flag, triggerResetFlag;
extern double targetVel[4];
extern int32_t tension1;
extern int32_t tensionL;
extern int left3508StopCont, right3508StopCont, releaseFlag;

extern int furTarTen[4];
extern int furTarYaw[4];

int feedFSMstate = 0;
int backCont = 0;

int FeedFSMState(void){
    return feedFSMstate;
}

void DartFeedFSM(void){
    switch (feedFSMstate) {
        case -1:                //downward moving
            StepperStart(STEPPER4);
            StepperSetSpeed(STEPPER4, -500);
            break;
        case 0:                 //not move
            StepperSetSpeed(STEPPER4, 0);
            break;
        case 1:                 //upward moving
            StepperStart(STEPPER4);
            StepperSetSpeed(STEPPER4, 500);
            ServoSet(SERVO_UP_DOWN, SERVO_UP_DOWN_UP, 0);
            break;
        case 2:                 //dart loading
            StepperSetSpeed(STEPPER4, 0);
            break;
        case 3:                 //dart is ready to load
            StepperSetSpeed(STEPPER4, 0);
            break;
    }
}

void DartFeedStartDown(void){
    feedFSMstate = -1;
    DartFeedFSM();
}

void DartFeedResetUntilHallDetected(void){
    if(feedFSMstate == -1) {
        if (HAL_GPIO_ReadPin(HALL_FEED_BOTTOM_SW_GPIO_Port, HALL_FEED_BOTTOM_SW_Pin) == GPIO_PIN_RESET) {
            feedFSMstate = 0;
            DartFeedFSM();
        }
    }
}

void DartFeedStartUp(void){
    if(feedFSMstate != -1 && feedFSMstate != 2 && feedFSMstate != 3) {
        feedFSMstate = 1;
        DartFeedFSM();
    }
}

void DartFeedUpUntilSWDetected(void){
    if(feedFSMstate == 1){
        if(HAL_GPIO_ReadPin(DART_STOP_SW_GPIO_Port, DART_STOP_SW_Pin) == GPIO_PIN_RESET){
            feedFSMstate = -1;
            backCont = LOAD_BACK_TIME_100MS;
            DartFeedFSM();
        }
    }
}

void DartFeedLoading(void){
    feedFSMstate = 2;
    DartFeedFSM();
}

void DartFeedLoadingEnd(void){
    feedFSMstate = 0;
    DartFeedFSM();
}

int IsDartReadyToLoad(void){
    if(feedFSMstate == 3) {
        feedFSMstate = 2;
        DartFeedFSM();
        printf("retrun1\n");
        return 1;
    }
    return 0;
}

void DartFeedStopDown(void){
    if(backCont <= 1){
        feedFSMstate = 3;
        DartFeedFSM();
        return;
    }
    feedFSMstate = 0;
    DartFeedFSM();
}

void DartReset(void){
    motor0Flag = 0;
    motor1Flag = 1;
    motor2Flag = 1;
    motor3Flag = 0;
    targetVel[1] = RESET_SPEED;
    targetVel[2] = RESET_SPEED;
    targetVel[0] = 0;
    while(targetVel[1] != 0 || targetVel[2] != 0){
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
    }
}

extern int servoTriggerCont;
void TriggerReset(void){
    ServoSet(SERVO_TRIGGER, SERVO_TRIGGER_RESET, 0);
    ServoSet(SERVO_4, SERVO_TRIGGER_RESET, 0);
    printf("RESET TRIGGE\n");
    servoTriggerCont = 0;
}

void TriggerShoot(void){
    ServoSet(SERVO_TRIGGER, SERVO_TRIGGER_SHOOT, 0);
    ServoSet(SERVO_4, SERVO_TRIGGER_SHOOT, 0);
    printf("RESET TRIGGE\n");
    servoTriggerCont = 0;
}

void DartLoad(int loadSpeed) {
    motor0Flag = 0;
    motor1Flag = 1;
    motor2Flag = 1;
    motor3Flag = 0;
    targetVel[1] = loadSpeed;
    targetVel[2] = loadSpeed;
    targetVel[0] = 0;
//    HAL_Delay(2000); //3700
    while (HAL_GPIO_ReadPin(HALL_BACK_SW_GPIO_Port, HALL_BACK_SW_Pin) != HALL_DETECTED) {
        targetVel[0] = 0;
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
    }
    HAL_Delay(LOAD_DELAY);
    TriggerReset();
#if SHOOT_INFO
    printf("DART LOAD OK!\n");
#endif
//    HAL_Delay(LOAD_DELAY);
    motor0Flag = 0;
    motor1Flag = 0;
    motor2Flag = 0;
    motor3Flag = 0;
    targetVel[1] = 0;
    targetVel[2] = 0;
}

void DartRelease(void) {
    motor0Flag = 1;
    triggerResetFlag = 1;
#if OLD_TRIGGER
    //老扳机转到即将触发的位置
    PWM_Renew(0, TRIGGER_FIRST_RESET_PWM);
#endif
    motor1Flag = 1;
    motor2Flag = 1;
    motor3Flag = 0;
    targetVel[1] = RELEASE_SPEED;
    targetVel[2] = RELEASE_SPEED;
    targetVel[0] = 0;
    tension1 = RS485_1_GetTension();
    motor0Flag = 1;
    triggerResetFlag = 1;
    PWM_Renew(0, TRIGGER_RESET_PWM);
    tensionL = RS485_2_GetTension();
    releaseFlag = 1;
//    HAL_Delay(delayTime); //2100
    uint32_t time = HAL_GetTick();
    while (targetVel[1] != 0 || targetVel[2] != 0) {
        if(stepper0Flag == 0 && HAL_GetTick() - time > RELEASE_DELAY_TENION_CONTROL){    //释放后相隔多少ms后开始拉力闭环控制
            stepper0Flag = 1;
            stepper1Flag = 1;
        }
#if OLD_TRIGGER
        motor0Flag = 1;
        triggerResetFlag = 1;
        PWM_Renew(0, TRIGGER_RESET_PWM);
        if (HAL_GPIO_ReadPin(HALL_BACK_SW_GPIO_Port, HALL_BACK_SW_Pin) != HALL_DETECTED) {
            motor0Flag = 1;
            triggerResetFlag = 0;
            motor1Flag = 0;
            motor2Flag = 0;
            motor3Flag = 0;
            targetVel[1] = 0;
            targetVel[2] = 0;
            targetVel[0] = -2000;
            HAL_Delay(500);
            targetVel[0] = 0;
            HAL_Delay(500);
            motor0Flag = 0;
            DartLoad(ERROR_LOAD_SPEED);
            motor0Flag = 1;
            triggerResetFlag = 1;
            motor1Flag = 1;
            motor2Flag = 1;
            motor3Flag = 0;
            targetVel[1] = RELEASE_SPEED;
            targetVel[2] = RELEASE_SPEED;
            targetVel[0] = 0;
            time = HAL_GetTick();
        }

#endif
        targetVel[0] = 0;
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
        if(right3508StopCont == 0){
            targetVel[2] = 0;
        }
        if(left3508StopCont == 0) {
            targetVel[1] = 0;
        }
    }
    releaseFlag = 0;
    targetVel[2] = 0;
    right3508StopCont = -1;
    targetVel[1] = 0;
    left3508StopCont = -1;
    triggerResetFlag = 0;
    HAL_Delay(200);
#if SHOOT_INFO
    printf("DART RELEASE OK!\n");
#endif
    motor0Flag = 0;
    motor1Flag = 0;
    motor2Flag = 0;
    motor3Flag = 0;
    targetVel[1] = 0;
    targetVel[2] = 0;
}

void DartShoot(void) {
#if OLD_TRIGGER
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
#endif

#if !OLD_TRIGGER
    TriggerShoot();
#endif

#if SHOOT_INFO
    printf("DART SHOOT OK!    system time: %ld ms\n", HAL_GetTick());
#endif

#if OLD_TRIGGER
    HAL_Delay(190);
    targetVel[0] = 0;
    motor0Flag = 1;
    HAL_Delay(500);
    motor0Flag = 0;
#endif
}
