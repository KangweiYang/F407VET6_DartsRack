//
// Created by root on 24-4-17.
//

#include "../Inc/RS485.h"
#include "usart.h"
#include "main.h"
#include "usart.h"

uint8_t rs4851GetTensionBuffer[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
uint8_t rs4852GetTensionBuffer[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
//uint8_t rs4852GetTensionBuffer[8] = {0x04, 0x03, 0x00, 0x00, 0x00, 0x02};

uint8_t tension1DataAddress = 0, tension2DataAddress = 0;

extern int tensionControlFlag;

void CRC16Calc(int8_t dataBuff[], int dataLen)
{
    int16_t CRCResult = 0xFFFF;
    if (dataLen < 2)
    {
        return;
    }
    for (int i = 0; i < (dataLen - 2); i++)
    {
        CRCResult = CRCResult ^ dataBuff[i];
        for (int j = 0; j < 8; j++)
        {
            if ((CRCResult & 1) == 1)
                CRCResult = (CRCResult >> 1) ^ 0xA001;
            else CRCResult >>= 1;
        }
    }
    dataBuff[dataLen - 1] = CRCResult >> 8;
    dataBuff[dataLen - 2] = CRCResult & 0xff;
}

void RS485Init(void){
    HAL_UART_Init(HRS485_1_USART);
    HAL_UART_Init(HRS485_2_USART);
}

void RS485ReceiveState(int channel){
    if(channel == 1) HAL_GPIO_WritePin(RS485_1_RX_TX_CONTROL_GPIO_Port, RS485_1_RX_TX_CONTROL_Pin, GPIO_PIN_RESET);
    else if(channel == 2)   HAL_GPIO_WritePin(RS485_2_RX_TX_CONTROL_GPIO_Port, RS485_2_RX_TX_CONTROL_Pin, GPIO_PIN_RESET);
}

void RS485SendState(int channel){
    if(channel == 1) HAL_GPIO_WritePin(RS485_1_RX_TX_CONTROL_GPIO_Port, RS485_1_RX_TX_CONTROL_Pin, GPIO_PIN_SET);
    else if(channel == 2)   HAL_GPIO_WritePin(RS485_2_RX_TX_CONTROL_GPIO_Port, RS485_2_RX_TX_CONTROL_Pin, GPIO_PIN_SET);
}

int32_t RS485_1_GetTension(void){
    uint8_t rs4851data[11];
#if RS485_INFO
    printf("send: ");
    for (int i = 0; i < 8; ++i)
        printf("%x,", rs4851GetTensionBuffer[i]);
    printf("\n");
#endif
    RS485SendState(1);
    HAL_UART_Transmit(HRS485_1_USART, rs4851GetTensionBuffer, 8, 50);
    RS485ReceiveState(1);
    HAL_UART_Receive(HRS485_1_USART, rs4851data, 11, 20);
    if(rs4851data[0] == 0x03 && rs4851data[1] == 0x04) tension1DataAddress = 2;
    else if(rs4851data[0] == 0x01 && rs4851data[1] == 0x03 && rs4851data[2] == 0x04) tension1DataAddress = 3;
    else if(rs4851data[1] == 0x01 && rs4851data[2] == 0x03 && rs4851data[3] == 0x04) tension1DataAddress = 4;
    else if(rs4851data[2] == 0x01 && rs4851data[3] == 0x03 && rs4851data[4] == 0x04) tension1DataAddress = 5;
    else{
        printf("rs485 1 error!!!");
        for (int i = 0; i < 11; ++i)
            printf("%x,", rs4851data[i]);
//        printf("\n");
    }
#if RS485_INFO
    printf("1receive: ");
    for (int i = 0; i < 11; ++i)
        printf("%x,", rs4851data[i]);
    printf("\n");
#endif
    if(tensionControlFlag){
        printf("1receive: ");
        for (int i = 0; i < 11; ++i)
            printf("%x,", rs4851data[i]);
//        printf("\n");
    }
    HAL_Delay(10);
    return (rs4851data[tension1DataAddress] << 24) | (rs4851data[tension1DataAddress + 1] << 16) |
    (rs4851data[tension1DataAddress + 2] << 8) | (rs4851data[tension1DataAddress + 3] << 0);
}

int32_t RS485_2_GetTension(void){
    uint8_t rs4852data[11];
#if RS485_INFO
    printf("send: ");
    for (int i = 0; i < 8; ++i)
        printf("%x,", rs4852GetTensionBuffer[i]);
    printf("\n");
#endif
    RS485SendState(2);
    HAL_UART_Transmit(HRS485_2_USART, rs4852GetTensionBuffer, 8, 50);
    RS485ReceiveState(2);
    HAL_UART_Receive(HRS485_2_USART, rs4852data, 9, 20);
    if(rs4852data[0] == 0x03 && rs4852data[1] == 0x04) tension2DataAddress = 2;
    else if(rs4852data[0] == 0x01 && rs4852data[1] == 0x03 && rs4852data[2] == 0x04) tension2DataAddress = 3;
    else if(rs4852data[1] == 0x01 && rs4852data[2] == 0x03 && rs4852data[3] == 0x04) tension2DataAddress = 4;
    else if(rs4852data[2] == 0x01 && rs4852data[3] == 0x03 && rs4852data[4] == 0x04) tension2DataAddress = 5;
    else{
        printf("rs485 2 error!!!");
        for (int i = 0; i < 11; ++i)
            printf("%x,", rs4852data[i]);
//        printf("\n");
    }
#if RS485_INFO
    printf("2receive: ");
    for (int i = 0; i < 11; ++i)
        printf("%x,", rs4852data[i]);
    printf("\n");
//    printf("%d\n", (rs4852data[tension2DataAddress] << 24) | (rs4852data[tension2DataAddress + 1] << 16) |
//                 (rs4852data[tension2DataAddress + 2] << 8) | (rs4852data[tension2DataAddress + 3] << 0));
#endif
    if(tensionControlFlag){
        printf("2receive: ");
        for (int i = 0; i < 11; ++i)
            printf("%x,", rs4852data[i]);
//        printf("\n");
    }
    return (rs4852data[tension2DataAddress] << 24) | (rs4852data[tension2DataAddress + 1] << 16) |
           (rs4852data[tension2DataAddress + 2] << 8) | (rs4852data[tension2DataAddress + 3] << 0);
}



