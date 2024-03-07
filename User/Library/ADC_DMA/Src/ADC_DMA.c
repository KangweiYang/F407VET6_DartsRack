//
// Created by root on 23-8-23.
//

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "../../MOTOR/Inc/MotorUnit.h"


static uint16_t MotorCount_Data[MOTORCOUNT_NUM * 10];                                          //MotorCount Data
static uint16_t MotorCount_Value[MOTORCOUNT_NUM];                                          //MotorCount ture value
static uint16_t MotorCount_LastValue[MOTORCOUNT_NUM];                                          //MotorCount last value
static uint16_t MotorCount_LastLastValue[MOTORCOUNT_NUM];                                          //MotorCount last last value
static uint16_t MotorCount_Sum[MOTORCOUNT_NUM];                                          //MotorCount sum value
extern int64_t pos[MOTOR_NUM];
static uint32_t lastTrik = 0;
static uint32_t trik = 0;

/**
    * @breif    MotorCount DMA Init
    * @note     None
    * @param    MotorCount channel
    * @retval   MotorCount ture value
    *           or error code: -1
    */
void IT_Init(void) {
    HAL_TIM_Base_Start_IT(&htim2);
}

uint16_t IT_GetMotorCountof(int channel) {
//    if(channel < 0 || channel > MOTORCOUNT_NUM)                                         //channel exceed the normal region
//        Error_Handler();
//    uint16_t MotorCount_Value[MOTORCOUNT_NUM];
//    MotorCount_Value[channel] = 0;
//    for (int i = 0; i < MOTORCOUNT_NUM * 10; ++i){
//        if(i % MOTORCOUNT_NUM == channel)
//            MotorCount_Value[channel] += MotorCount_Data[i];
//    }
//    MotorCount_Value[channel] = (MotorCount_Value[channel] + 5 ) / 10;                       //round off
    if(channel < 0 || channel >= MOTORCOUNT_NUM) Error_Handler();
    return MotorCount_Value[channel];
}

/**
    * @breif    Get velocity
    * @note     None
    * @param    channel
    * @retval   The velocity of motor[channel].
    */
int16_t Get_VelOf(int channel){
    if(channel < 0 || channel >= MOTOR_NUM) Error_Handler();
    static int64_t lastPos[2];
    int64_t temp = lastPos[channel];
    lastPos[channel] = pos[channel];
//    lastTrik = trik;
//    trik = HAL_GetTick();
//    printf("\n \n %lld, %lld, %lld\n \n", trik - lastTrik, trik >> 20, lastTrik >> 20);
    return pos[channel] - temp;
}

uint16_t IT_GetMotorCountofRawData(int num) {
    return MotorCount_Data[num];
}