//
// Created by JustinWCola on 24-5-9.
//

#include "../Inc/remote.h"
#include "main.h"
#include "usart.h"
#include "memory.h"

uint8_t _rx_buf[18];
sRemoteInfo _info;
extern int sonicRangeUpCloseFlag, sonicRangeDownOpenFlag;

void RemoteInit(void) {
    HAL_UART_Receive_DMA(&huart5, _rx_buf, 18);
}

void MainControl()
{
    switch (_info.s[1])
    {
        case 1:                     //left s
        {
            printf("s[1]: 1\n");    //left: up
            break;
        }
        case 3:
        {
            printf("s[1]: 3\n");    //left: middle
            sonicRangeDownOpenFlag = 0;
            break;
        }
        case 2:
        {
            printf("s[1]: 2\n");    //left: down
            sonicRangeDownOpenFlag = 1;
            break;
        }
    }
    switch (_info.s[0])             //right s
    {
        case 1:
        {
            printf("s[0]: 1\n");    //right: down
            break;
        }
        case 3:
        {
            printf("s[0]: 3\n");    //right: middle
            break;
        }
        case 2:
        {
            printf("s[0]: 2\n");    //right: up
            break;
        }
    }
}

void RemoteControl(void) {
    if (_rx_buf == NULL)
        return;

    _info.ch[0] = ((int16_t) _rx_buf[0] | ((int16_t) _rx_buf[1] << 8)) & 0x07FF;        //!< Channel 0  right horizontal
    _info.ch[1] = (((int16_t) _rx_buf[1] >> 3) | ((int16_t) _rx_buf[2] << 5)) & 0x07FF; //!< Channel 1  right vertical
    _info.ch[2] = (((int16_t) _rx_buf[2] >> 6) | ((int16_t) _rx_buf[3] << 2) |
                   ((int16_t) _rx_buf[4] << 10)) &
                  0x07FF;                                //!< Channel 2  left horizontal
    _info.ch[3] = (((int16_t) _rx_buf[4] >> 1) | ((int16_t) _rx_buf[5] << 7)) & 0x07FF; //!< Channel 3  left vertical
    _info.s[0] = ((_rx_buf[5] >> 4) & 0x0003);                                          //!< Switch right
    _info.s[1] = ((_rx_buf[5] >> 4) & 0x000C) >> 2;                                     //!< Switch left
    _info.ch[4] = ((int16_t) _rx_buf[16]) | ((int16_t) _rx_buf[17] << 8);               //NULL

    _info.ch[0] -= RC_CH_VALUE_OFFSET;
    _info.ch[1] -= RC_CH_VALUE_OFFSET;
    _info.ch[2] -= RC_CH_VALUE_OFFSET;
    _info.ch[3] -= RC_CH_VALUE_OFFSET;
    _info.ch[4] -= RC_CH_VALUE_OFFSET;
    MainControl();
    HAL_UART_Receive_DMA(&huart5, _rx_buf, 18);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart5.Instance)
    {
    }
}
