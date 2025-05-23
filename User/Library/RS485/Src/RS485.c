//
// Created by root on 24-4-17.
//

#include "../Inc/RS485.h"
#include "usart.h"
#include "main.h"
#include "usart.h"

uint8_t rs4851GetTensionBuffer[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
uint8_t rs4852GetTensionBuffer[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
uint8_t rs4851SetTensionZero[14] = {0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0xF3, 0xAF};
uint8_t rs4852SetTensionZero[14] = {0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0xF3, 0xAF};
//uint8_t rs4852GetTensionBuffer[8] = {0x04, 0x03, 0x00, 0x00, 0x00, 0x02};

uint8_t tension1DataAddress = 0, tension2DataAddress = 0;
extern uint8_t ex_rs4851data[11], ex_rs4852data[11], ex_tension1DataAddress, ex_tension2DataAddress;

extern int tensionControlFlag;
extern double targetTen[2];

void CRC16Calc(int8_t dataBuff[], int dataLen) {
    int16_t CRCResult = 0xFFFF;
    if (dataLen < 2) {
        return;
    }
    for (int i = 0; i < (dataLen - 2); i++) {
        CRCResult = CRCResult ^ dataBuff[i];
        for (int j = 0; j < 8; j++) {
            if ((CRCResult & 1) == 1)
                CRCResult = (CRCResult >> 1) ^ 0xA001;
            else CRCResult >>= 1;
        }
    }
    dataBuff[dataLen - 1] = CRCResult >> 8;
    dataBuff[dataLen - 2] = CRCResult & 0xff;
}

void RS485Init(void) {
    HAL_UART_Init(HRS485_1_USART);
    HAL_UART_Init(HRS485_2_USART);
}

void RS485ReceiveState(int channel) {
#if RS485_NORMAL
    if (channel == 1)
        HAL_GPIO_WritePin(RS485_1_RX_TX_CONTROL_GPIO_Port, RS485_1_RX_TX_CONTROL_Pin, GPIO_PIN_RESET);
    else if (channel == 2)
        HAL_GPIO_WritePin(RS485_2_RX_TX_CONTROL_GPIO_Port, RS485_2_RX_TX_CONTROL_Pin, GPIO_PIN_RESET);
#else
    if (channel == 1)
        HAL_GPIO_WritePin(RS485_2_RX_TX_CONTROL_GPIO_Port, RS485_2_RX_TX_CONTROL_Pin, GPIO_PIN_RESET);
    else if (channel == 2)
        HAL_GPIO_WritePin(RS485_1_RX_TX_CONTROL_GPIO_Port, RS485_1_RX_TX_CONTROL_Pin, GPIO_PIN_RESET);
#endif
}

void RS485SendState(int channel) {
#if RS485_NORMAL
    if (channel == 1)
        HAL_GPIO_WritePin(RS485_1_RX_TX_CONTROL_GPIO_Port, RS485_1_RX_TX_CONTROL_Pin, GPIO_PIN_SET);
    else if (channel == 2)
        HAL_GPIO_WritePin(RS485_2_RX_TX_CONTROL_GPIO_Port, RS485_2_RX_TX_CONTROL_Pin, GPIO_PIN_SET);
#else
    if (channel == 1)
        HAL_GPIO_WritePin(RS485_2_RX_TX_CONTROL_GPIO_Port, RS485_2_RX_TX_CONTROL_Pin, GPIO_PIN_SET);
    else if (channel == 2)
        HAL_GPIO_WritePin(RS485_1_RX_TX_CONTROL_GPIO_Port, RS485_1_RX_TX_CONTROL_Pin, GPIO_PIN_SET);
#endif
}

void RS485_1_SetTensionZero(void) {
    uint8_t rs4851data[11];
    RS485SendState(1);
    HAL_Delay(1);
    HAL_UART_Transmit(HRS485_1_USART, rs4851SetTensionZero, 13, 50);
    HAL_Delay(1);
    RS485ReceiveState(1);
    HAL_UART_Receive(HRS485_1_USART, rs4851data, 9, 30);
    printf("1receive: ");
    for (int i = 0; i < 11; ++i)
        printf("%x,", rs4851data[i]);
    printf("\n");
}

void RS485_2_SetTensionZero(void) {
    uint8_t rs4852data[11];
    RS485SendState(2);
    HAL_Delay(10);
    printf("test\n");
    HAL_UART_Transmit(HRS485_2_USART, rs4852SetTensionZero, 13, 50);
    HAL_Delay(1);
    RS485ReceiveState(2);
    HAL_UART_Receive(HRS485_2_USART, rs4852data, 9, 30);
    printf("2receive: ");
    for (int i = 0; i < 11; ++i)
        printf("%x,", rs4852data[i]);
    printf("\n");
}

int32_t RS485_1_GetTension(void) {
    uint8_t rs4851data[11];
#if RS485_INFO
    static int cont = 0;
    cont++;
    if (cont == 1000) {
        printf("send: ");
        for (int i = 0; i < 8; ++i)
            printf("%x,", rs4851GetTensionBuffer[i]);
        printf("\n");
    }
#endif
    RS485SendState(1);
    HAL_Delay(1);
    HAL_UART_Transmit(HRS485_1_USART, rs4851GetTensionBuffer, 8, 10);
    RS485ReceiveState(1);
    HAL_UART_Receive(HRS485_1_USART, rs4851data, 11, 30);
    if (rs4851data[0] == 0x01 && rs4851data[1] == 0x03 && rs4851data[2] == 0x04) tension1DataAddress = 3;
    else if (rs4851data[1] == 0x01 && rs4851data[2] == 0x03 && rs4851data[3] == 0x04) tension1DataAddress = 4;
    else if (rs4851data[2] == 0x01 && rs4851data[3] == 0x03 && rs4851data[4] == 0x04) tension1DataAddress = 5;
    else if (rs4851data[0] == 0x04) tension1DataAddress = 1;
    else if (rs4851data[0] == 0x03 && rs4851data[1] == 0x04) tension1DataAddress = 2;
#if RS485_LIGHT_INFO
    for (int i = 0; i < 11; ++i){
        ex_rs4851data[i] = rs4851data[i];
    }
    ex_tension1DataAddress = tension1DataAddress;
#endif
#if RS485_INFO
    if (cont == 1000) {
        printf("1receive: ");
        for (int i = 0; i < 11; ++i)
            printf("%x,", rs4851data[i]);
        printf("\n");
        cont = 0;
    }
#endif
    /*
    if(tensionControlFlag){
        printf("1receive: ");
        for (int i = 0; i < 11; ++i)
            printf("%x,", rs4851data[i]);
//        printf("\n");
    }
     */
    HAL_Delay(10);
    static int32_t tension, mutateTension;
    int32_t curTension = (rs4851data[tension1DataAddress] << 24) | (rs4851data[tension1DataAddress + 1] << 16) |
                         (rs4851data[tension1DataAddress + 2] << 8) | (rs4851data[tension1DataAddress + 3] << 0);
    if (curTension <= 1200 && curTension >= -1200 &&
    ((curTension <= tension + RS485_MUTATION_THRESHOLD && curTension >= tension - RS485_MUTATION_THRESHOLD) || tension == 0 ||
            (mutateTension <= curTension + RS485_MUTATION_THRESHOLD && mutateTension >= curTension - RS485_MUTATION_THRESHOLD))) {
//    if (curTension <= 1200 && curTension >= -1200) {
        tension = (rs4851data[tension1DataAddress] << 24) | (rs4851data[tension1DataAddress + 1] << 16) |
                  (rs4851data[tension1DataAddress + 2] << 8) | (rs4851data[tension1DataAddress + 3] << 0);
    }
    else if(!((curTension <= tension + RS485_MUTATION_THRESHOLD && curTension >= tension - RS485_MUTATION_THRESHOLD) || tension == 0)){
        mutateTension = (rs4851data[tension1DataAddress] << 24) | (rs4851data[tension1DataAddress + 1] << 16) |
                  (rs4851data[tension1DataAddress + 2] << 8) | (rs4851data[tension1DataAddress + 3] << 0);
    }
#if NO_STEPPER_TEST
    tension = targetTen[0];
#endif
    return tension;
}

int32_t RS485_2_GetTension(void) {
    uint8_t rs4852data[11];
#if RS485_INFO
    static int cont = 0;
    cont++;
    if (cont == 1000) {
        printf("send: ");
        for (int i = 0; i < 8; ++i)
            printf("%x,", rs4852GetTensionBuffer[i]);
        printf("\n");
    }
#endif
    RS485SendState(2);
    HAL_Delay(1);
    HAL_UART_Transmit(HRS485_2_USART, rs4852GetTensionBuffer, 8, 10);
    RS485ReceiveState(2);
    HAL_UART_Receive(HRS485_2_USART, rs4852data, 9, 30);
    if (rs4852data[0] == 0x01 && rs4852data[1] == 0x03 && rs4852data[2] == 0x04) tension2DataAddress = 3;
    else if (rs4852data[1] == 0x01 && rs4852data[2] == 0x03 && rs4852data[3] == 0x04) tension2DataAddress = 4;
    else if (rs4852data[2] == 0x01 && rs4852data[3] == 0x03 && rs4852data[4] == 0x04) tension2DataAddress = 5;
    else if (rs4852data[0] == 0x04) tension2DataAddress = 1;
    else if (rs4852data[0] == 0x03 && rs4852data[1] == 0x04) tension2DataAddress = 2;
#if RS485_LIGHT_INFO
    for (int i = 0; i < 11; ++i){
        ex_rs4852data[i] = rs4852data[i];
    }
    ex_tension2DataAddress = tension2DataAddress;
#endif
#if RS485_INFO
    if (cont == 1000) {
        printf("2receive: ");
        for (int i = 0; i < 11; ++i)
            printf("%x,", rs4852data[i]);
        printf("\n");
        cont = 0;
//    printf("%d\n", (rs4852data[tension2DataAddress] << 24) | (rs4852data[tension2DataAddress + 1] << 16) |
//                 (rs4852data[tension2DataAddress + 2] << 8) | (rs4852data[tension2DataAddress + 3] << 0));
    }
#endif
/*
    if(tensionControlFlag){
        printf("2receive: ");
        for (int i = 0; i < 11; ++i)
            printf("%x,", rs4852data[i]);
//        printf("\n");
    }
    */
    HAL_Delay(10);
    static int32_t tension, mutateTension;
    int32_t curTension = (rs4852data[tension2DataAddress] << 24) | (rs4852data[tension2DataAddress + 1] << 16) |
                         (rs4852data[tension2DataAddress + 2] << 8) | (rs4852data[tension2DataAddress + 3] << 0);
    if (curTension <= 1200 && curTension >= -1200 &&
        ((curTension <= tension + RS485_MUTATION_THRESHOLD && curTension >= tension - RS485_MUTATION_THRESHOLD) || tension == 0 ||
         (mutateTension <= curTension + RS485_MUTATION_THRESHOLD && mutateTension >= curTension - RS485_MUTATION_THRESHOLD))) {
//    if (curTension <= 1200 && curTension >= -1200) {
        tension = (rs4852data[tension2DataAddress] << 24) | (rs4852data[tension2DataAddress + 1] << 16) |
                  (rs4852data[tension2DataAddress + 2] << 8) | (rs4852data[tension2DataAddress + 3] << 0);
    }
    else if(!((curTension <= tension + RS485_MUTATION_THRESHOLD && curTension >= tension - RS485_MUTATION_THRESHOLD) || tension == 0)){
        mutateTension = (rs4852data[tension2DataAddress] << 24) | (rs4852data[tension2DataAddress + 1] << 16) |
                        (rs4852data[tension2DataAddress + 2] << 8) | (rs4852data[tension2DataAddress + 3] << 0);
    }
#if NO_STEPPER_TEST
    tension = targetTen[1];
#endif
    return tension;
}



