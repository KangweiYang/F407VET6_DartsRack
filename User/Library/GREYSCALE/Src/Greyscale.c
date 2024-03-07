//
// Created by root on 23-12-4.
//

#include "../Inc/Greyscale.h"
#include "main.h"
#include "usart.h"

const uint8_t greyscaleSign = 0x57;

/**
    * @breif    Init greyscale's uart
    * @note     None
    * @param    None
    * @retval   None
    */
void GreyscaleInit(void){
}

/**
    * @breif    Get value of greyscale
    * @note     None
    * @param    None
    * @retval   None
    */
uint16_t GetGreyscale(void){
    HAL_UART_Transmit(&huart2, &greyscaleSign, 1, 10);

    return 0xFF;
}
