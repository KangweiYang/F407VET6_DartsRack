//
// Created by JustinWCola on 24-5-9.
//

#include "../Inc/remote.h"
#include "main.h"
#include "usart.h"
#include "memory.h"

cRemote remote(&huart5);

void cRemote::init()
{
    HAL_UART_Receive_DMA(_huart, _rx_buf, 18);
}

void cRemote::update(void (*function)(sRemoteInfo))
{
    HAL_UART_DMAStop(_huart);
    if (_rx_buf == NULL)
        return;

    _info.ch[0] = ((int16_t)_rx_buf[0] | ((int16_t)_rx_buf[1] << 8)) & 0x07FF;        //!< Channel 0  right horizontal
    _info.ch[1] = (((int16_t)_rx_buf[1] >> 3) | ((int16_t)_rx_buf[2] << 5)) & 0x07FF; //!< Channel 1  right vertical
    _info.ch[2] = (((int16_t)_rx_buf[2] >> 6) | ((int16_t)_rx_buf[3] << 2) |
                         ((int16_t)_rx_buf[4] << 10)) & 0x07FF;                                //!< Channel 2  left horizontal
    _info.ch[3] = (((int16_t)_rx_buf[4] >> 1) | ((int16_t)_rx_buf[5] << 7)) & 0x07FF; //!< Channel 3  left vertical
    _info.s[0] = ((_rx_buf[5] >> 4) & 0x0003);                                          //!< Switch right
    _info.s[1] = ((_rx_buf[5] >> 4) & 0x000C) >> 2;                                     //!< Switch left
    _info.ch[4] = ((int16_t)_rx_buf[16]) | ((int16_t)_rx_buf[17] << 8);               //NULL

    _info.ch[0] -= RC_CH_VALUE_OFFSET;
    _info.ch[1] -= RC_CH_VALUE_OFFSET;
    _info.ch[2] -= RC_CH_VALUE_OFFSET;
    _info.ch[3] -= RC_CH_VALUE_OFFSET;
    _info.ch[4] -= RC_CH_VALUE_OFFSET;
    function(_info);
    HAL_UART_Receive_DMA(_huart, _rx_buf, 18);
}

cRemote::sRemoteInfo cRemote::getInfo()
{
    return _info;
}

void MainControl(cRemote::sRemoteInfo info)
{
    switch (info.s[1])
    {
        case 1:
        {
            break;
        }
        case 3:
        {
            break;
        }
        case 2:
        {
            break;
        }
    }
    switch (info.s[0])
    {
        case 1:
        {
            break;
        }
        case 3:
        {
            break;
        }
        case 2:
        {
            break;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5)
    {
        remote.update(MainControl);
    }
}
