//
// Created by root on 23-12-4.
//

#include "../Inc/LED.h"
#include "main.h"

//Hardware define begin
//Hardware define end

/**
    * @breif    RGB LED turn on. EX. 0b011 means turn on green and blue lights.
    * @note     None
    * @param    Which LED:0b000-0b111
    *           0b RED GREEN BLUE
    * @retval   None
    */
void TurnOnLED(char whichLED){
    if(whichLED & 0b001) HAL_GPIO_WritePin(BLUE_LED_PORT, BLUE_LED_PIN, LED_ON);
    if(whichLED & 0b010) HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, LED_ON);
    if(whichLED & 0b100) HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, LED_ON);
}

/**
    * @breif    RGB LED turn off. EX. 0b110 means turn off red and green lights.
    * @note     None
    * @param    Which LED:0b000-0b111
    *           0b RED GREEN BLUE
    * @retval   None
    */
void TurnOffLED(char whichLED){
    if(whichLED & 0b001) HAL_GPIO_WritePin(BLUE_LED_PORT, BLUE_LED_PIN, LED_OFF);
    if(whichLED & 0b010) HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, LED_OFF);
    if(whichLED & 0b100) HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, LED_OFF);
}
