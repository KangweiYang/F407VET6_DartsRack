//
// Created by root on 24-3-6.
//

#ifndef STEPPER_H
#define STEPPER_H

#include "main.h"

#define HALF_COMPARE    50
#define APB2_FREQ       42000000

void StepperInit(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t prescaler);

void StepperStart(TIM_HandleTypeDef *htim, uint32_t channel);

void StepperSetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, int16_t frequency);

void StepperStop(TIM_HandleTypeDef *htim, uint32_t channel);

void StepperTensionControlStart(int channel);

void StepperTensionControl(int channel);

#endif //STEPPER_H
