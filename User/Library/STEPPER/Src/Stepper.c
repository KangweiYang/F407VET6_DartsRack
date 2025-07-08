//
// Created by root on 24-3-6.
//

#include "../Inc/Stepper.h"
#include "../../RS485/Inc/RS485.h"
#include "main.h"
#include "usart.h"
#include "tim.h"

static int16_t lastFreq[4] = {1, 1, 1, 1};
extern int32_t tension1;
extern int32_t tension2;
extern int16_t stepper0Speed;
extern int16_t stepper1Speed;
extern double targetTen[2];
extern int stepper0WaitingToStopFlag, stepper1WaitingToStopFlag;

int StepperGetChannel(TIM_HandleTypeDef *htim){
    if (htim == &htim1) return 0;
    else if (htim == &htim3) return 1;
    else if (htim == &htim4) return 2;
    else if (htim == &htim9) return 3;
}

void StepperInit(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t prescaler){
    HAL_TIM_PWM_Init(htim);
    HAL_TIM_PWM_Start(htim, channel);
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
    __HAL_TIM_SET_PRESCALER(htim, prescaler);
}

void StepperStart(TIM_HandleTypeDef *htim, uint32_t channel){
    __HAL_TIM_SET_COMPARE(htim, channel, HALF_COMPARE);
}

void StepperSetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, int16_t frequency){
    if(lastFreq[StepperGetChannel(htim)] * frequency < 0) {
        if (htim == &htim1) HAL_GPIO_TogglePin(STEPPER1_DIR_GPIO_Port, STEPPER1_DIR_Pin);
        else if (htim == &htim3) HAL_GPIO_TogglePin(STEPPER2_DIR_GPIO_Port, STEPPER2_DIR_Pin);
        else if (htim == &htim4) HAL_GPIO_TogglePin(STEPPER3_DIR_GPIO_Port, STEPPER3_DIR_Pin);
#if OLD_FEED
        else if (htim == &htim9) HAL_GPIO_TogglePin(STEPPER4_DIR_GPIO_Port, STEPPER4_DIR_Pin);
#endif
    }
    if(frequency != 0)  lastFreq[StepperGetChannel(htim)] = frequency;
#if STEPPER_INFO
//    printf("prescaler = %d \n",  APB2_FREQ / frequency / HALF_COMPARE / 2);
#endif
    if(frequency < 0)   frequency = -frequency;
    if(htim == &htim3)   __HAL_TIM_SET_PRESCALER(htim, APB2_FREQ / frequency / HALF_COMPARE / 2);
    if(htim == &htim1)   __HAL_TIM_SET_PRESCALER(htim, APB2_FREQ / frequency / HALF_COMPARE / 2);
    else    __HAL_TIM_SET_PRESCALER(htim, APB2_FREQ / frequency / HALF_COMPARE / 2);
}

void StepperStop(TIM_HandleTypeDef *htim, uint32_t channel){
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
}

void StepperTensionControlStart(int channel) {
    if(channel == 1) {
        for (int i = 0; i < 30; ++i) {
            tension1 = RS485_1_GetTension();
#if STEPPER_INFO
            printf("%ld, %ld\n", tension1, stepper0Speed);
#endif
        }
        if (tension1 > targetTen[0] + STEPPER1_2_MIN_CHANGE) {
            stepper0Speed = STEPPER1_MAX_PUL;
            stepper0WaitingToStopFlag = 1;
#if STEPPER_INFO
            printf("set:%ld, %d\n", tension1, stepper0Speed);
            printf("speed=%d\n", STEPPER1_MAX_PUL);
#endif
        } else if (tension1 < targetTen[0] - STEPPER1_2_MIN_CHANGE) {
            stepper0Speed = -STEPPER1_MAX_PUL;
            stepper0WaitingToStopFlag = 1;
#if STEPPER_INFO
            printf("set:%ld, %d\n", tension1, stepper0Speed);
            printf("speed=-%d\n", STEPPER1_MAX_PUL);
#endif
        }
        else{
            stepper0Speed = STEPPER1_Kp;
            stepper0WaitingToStopFlag = 0;
#if STEPPER_INFO
            printf("set:%ld, %d\n", tension1, stepper0Speed);
            printf("speed=-%d\n", STEPPER1_MAX_PUL);
#endif
        }
        if (stepper0Speed < 0) {
            for (int i = 0; i > stepper0Speed; i -= 10) {
                HAL_Delay(4);
                StepperSetSpeed(STEPPER1, i);
#if STEPPER_INFO
                printf("accelerate:%ld, %d, %d\n", tension1, i, stepper0Speed);
#endif
            }
        } else {
            for (int i = 0; i < stepper0Speed; i += 10) {
                HAL_Delay(4);
                StepperSetSpeed(STEPPER1, i);
#if STEPPER_INFO
                printf("accelerate:%ld, %d, %d\n", tension1, i, stepper0Speed);
#endif
            }
        }
    }
}

void StepperTensionControl(int channel) {
    if(channel == 1){
        StepperSetSpeed(STEPPER1, STEPPER1_Kp);
        stepper0Speed = STEPPER1_Kp;
    }
}
