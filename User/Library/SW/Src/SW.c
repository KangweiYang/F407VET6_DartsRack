//
// Created by root on 23-12-5.
//

#include "../Inc/SW.h"
#include "main.h"

//hardware define
#define SW1     SW1
#define SW2     SW2
#define SW3     SW3
#define SW4     SW4

uint8_t SW_Get(void) {
    uint8_t swState[4];
    swState[0] = (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) & 0x1);
    swState[1] = (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) & 0x1);
    swState[2] = (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) & 0x1);
    swState[3] = (HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) & 0x1);
    uint8_t result = (uint8_t) ((swState[3] << 3) | (swState[2] << 2) | (swState[1] << 1) | (swState[0]));
    return result;
}
