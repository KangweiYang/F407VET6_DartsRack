//
// Created by root on 24-3-6.
//

#include "../Inc/Stepper.h"
#include "main.h"
#include "tim.h"

void StepperChangeDir(void){

}

void StepperInit(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t prescaler){
    HAL_TIM_PWM_Start(htim, channel);
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
    __HAL_TIM_SET_PRESCALER(htim, prescaler);
}

void StepperStart(TIM_HandleTypeDef *htim, uint32_t channel){
    __HAL_TIM_SET_COMPARE(htim, channel, HALF_COMPARE);
}

void StepperSetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, int16_t frequency){
    if(frequency < 0){
        frequency = - frequency;
        StepperChangeDir();
    }
    __HAL_TIM_SET_PRESCALER(htim, APB2_FREQ / frequency / HALF_COMPARE / 2);
}

void StepperStop(TIM_HandleTypeDef *htim, uint32_t channel){
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
}
