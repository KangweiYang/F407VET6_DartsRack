//
// Created by root on 24-3-12.
//

#include "../Inc/Can.h"
#include "can.h"
#include "usart.h"
#include "main.h"

//CAN_RxHeaderTypeDef M3508_H_Rx;
//uint8_t RXmessage[8];

//void Can3508Init(void) {
//    M3508_H_Rx.StdId = 0x201;
//    M3508_H_Rx.ExtId = 0x0;
//    M3508_H_Rx.IDE = CAN_ID_STD;
//    M3508_H_Rx.RTR = CAN_RTR_DATA;
//    M3508_H_Rx.FilterMatchIndex = 0;
//    M3508_H_Rx.Timestamp = 0;
//    M3508_H_Rx.DLC = 8;
//}

/**
    * @breif    Get pos
    * @note     None
    * @param    None
    * @retval   None
    */
//void GetCur(void) {

//    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &M3508_H_Rx, RXmessage);
//
//    uint16_t pos = RXmessage[0] << 8 | RXmessage[1];
//    uint16_t vel = RXmessage[2] << 8 | RXmessage[3];
//    uint16_t current = RXmessage[4] << 8 || RXmessage[5];
//    printf("RX the CAN data: %02X%02X%02X%02X%02X%02X%02X%02X, pos = %ld, vel = %ld, cur = %ld\r\n", RXmessage[0], RXmessage[1], RXmessage[2],
//           RXmessage[3], RXmessage[4], RXmessage[5], RXmessage[6], RXmessage[7], pos, vel, current);

//}
