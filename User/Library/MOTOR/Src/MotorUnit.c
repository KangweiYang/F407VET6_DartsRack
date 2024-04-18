//
// Created by root on 23-8-18.
//

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "can.h"
#include "tim.h"
#include "../../INC_PI/Inc/Inc_PI.h"
#include "../../STEPPER/Inc/Stepper.h"

#define FORGET  1
#define ADC_BUFF_SIZE    10                      //ADC buff size

#define MOTOR_POSI  TIM_CHANNEL_1
#define MOTOR_NEG   TIM_CHANNEL_2

#define MAX_PWM 100
#define MOTOR_MAX_VEL 69

#define DEBUG
#define TAG "MotorUnit"

#ifdef DEBUG
#define printf_t(format, ...)   printf( format "\r\n", ##__VA_ARGS__)
#define info(format, ...) printf("[\t"TAG"]info:" format "\r\n", ##__VA_ARGS__)
#define debug(format, ...) printf("[\t"TAG"]debug:" format "\r\n", ##__VA_ARGS__)
#define error(format, ...) printf("[\t"TAG"]error:" format "\r\n",##__VA_ARGS__)
#else
#define printf(format, ...)
#define info(format, ...)
#define debug(format, ...)
#define error(format, ...)
#endif

extern double posKp, posKi, posKd;

uint32_t    pTxMailbox = 0;
static CAN_TxHeaderTypeDef M3508_H_Tx;

static int Dir[MOTOR_NUM] = {-1};
static int16_t targetPos[MOTOR_NUM];
static int motorCount[MOTOR_NUM];
static int PWM[MOTOR_NUM];
static double biasThreshold = 50;
//int endPoint = 3000;                               //End point AD value;
//uint8_t UART_data[2];
static int biasOfPosBias_And_LastPosBias[MOTOR_NUM];
static int lastposBias[MOTOR_NUM];
static int posBiasIntegration[MOTOR_NUM];
static int lastRPos = 0, lastLPos = 0;
static int posBias[MOTOR_NUM];                    //compute posBias
extern int32_t stepper0Speed;
extern int32_t stepper1Speed;

extern int motor0Flag, motor1Flag, motor2Flag, motor3Flag, stepper0Flag, stepper1Flag;
/**
    * @breif    Compute the absolute value of x.
    * @note     None
    * @param    x
    * @retval   Abstraction of x
    */
int AbsOf(int x) {
    if (x < 0) x = -x;

    return x;
}
/**
    * @breif    Print the infos.
    * @note     None
    * @param    AD_Value
    * @retval   None
    */
void PrintPIDinfo(int AD_Value, int endPoint) {
    printf("%d, %d, %d, %.2lf, %.2lf, %.2lf, %d\n", AD_Value, targetPos[1],
           PWM[0], posKp * posBias[0], posKi * posBiasIntegration[0], posKd * (posBias[0] - lastposBias[0]), endPoint);
}

/**
    * @breif    Change target Rpos, and clear the R integration.
    * @note     None
    * @param    channel, newtargetPos
    * @retval   None
    */
void ChangetargetPosOf(int channel, int newtargetPos) {
    posBiasIntegration[channel] = 0;
    targetPos[channel] = newtargetPos;
}

/**
    * @breif    Motor Init
    * @note     None
    * @param    None
    * @retval   None
    */
void MotorInit(void){
#if USE_CAN1
    HAL_CAN_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
#endif
#if USE_CAN2
    HAL_CAN_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
#endif
    M3508_H_Tx.StdId = 0x200;
    M3508_H_Tx.ExtId = 0x0;
    M3508_H_Tx.IDE = CAN_ID_STD;
    M3508_H_Tx.RTR = CAN_RTR_DATA;
    M3508_H_Tx.TransmitGlobalTime = DISABLE;
    M3508_H_Tx.DLC = 8;
}

/**
    * @breif    Renew the MOTOR's PWM
    * @note     None
    * @param    channel, PWM
    * @retval   None
    */
void PWM_Renew(int channel, int16_t PWM) {
    static uint8_t data[8];
    switch (channel) {
        case 1:
            if (motor0Flag) {
                data[0] = (uint8_t) (PWM >> 8);
                data[1] = (uint8_t) (PWM & 0xFF);
                break;
            } else {
                data[0] = 0;
                data[1] = 1;
                break;
            }
        case 2:
            if (motor1Flag) {
                data[2] = (uint8_t) (PWM >> 8);
                data[3] = (uint8_t) (PWM & 0xFF);
                break;
            } else {
                data[2] = 0;
                data[3] = 1;
                break;
            }
        case 3:
            if (motor2Flag) {
                data[4] = (uint8_t) (PWM >> 8);
                data[5] = (uint8_t) (PWM & 0xFF);
                break;
            } else {
                data[4] = 0;
                data[5] = 1;
                break;
            }
        case 4:
            if (motor3Flag) {
                data[6] = (uint8_t) (PWM >> 8);
                data[7] = (uint8_t) (PWM & 0xFF);
                break;
            } else {
                data[6] = 0;
                data[7] = 1;
                break;
            }
        case 5:
            if (stepper0Flag) {
                StepperSetSpeed(STEPPER1, -PWM);
                stepper0Speed = -PWM;
                break;
            } else {
                stepper0Speed = -PWM;
                break;
            }
        case 6:
            if (stepper1Flag) {
                StepperSetSpeed(STEPPER2, -PWM);
                stepper1Speed = -PWM;
                break;
            } else {
                stepper0Speed = -PWM;
                break;
            }
    }
//    uint8_t data[8] = {0x26, 0xE8, 0x06, 0xE8, 0x06, 0xE8, 0x06, 0xE8};
#if USE_CAN1
    HAL_CAN_AddTxMessage(&hcan1, &M3508_H_Tx, data, &pTxMailbox);
#endif
#if USE_CAN2
    HAL_CAN_AddTxMessage(&hcan2, &M3508_H_Tx, data, &pTxMailbox);
#endif
}


/**
    * @breif    Use position PID algorithm to renew PWM signals.
    * @note     None
    * @param    None
    * @retval   None
    */
void Position_PID(void) {
//    static int lastposBias[0], lastLPosBias, LPosBiasIntegration, posBias[0]Integration;

    for (int i = 0; i < MOTOR_NUM; ++i) {
        posBias[i] = targetPos[i] - motorCount[i];                    //compute posBias
        biasOfPosBias_And_LastPosBias[i] = AbsOf(posBias[0] - lastposBias[0]);
        posBias[i] *= FORGET;
        if (biasOfPosBias_And_LastPosBias[i] <= biasThreshold)                                      //New algorithm designed by me
            if (AbsOf(posBias[0]) <= biasThreshold)                                      //New algorithm designed by me
                posBiasIntegration[i] = posBias[i];
        //compute intergral of posBias
        if (AbsOf(posKi * posBiasIntegration[i]) >= 60)
            posBiasIntegration[i] *= 0.9;                                               //anti-saturation
        PWM[i] = (posKp * posBias[i] + posKi * posBiasIntegration[i] + posKd * (posBias[i] - lastposBias[i]))
                 * MOTOR_MAX_VEL / MAX_PWM + 0.5;
        lastposBias[i] = posBias[i];                            //save posBiass
//        PWM_Renew(PWM[i]);

    }
//    if (LPWM < 0) {
//        LTurnDir();
//        LPWM = -LPWM;
//    }
//    LPWM_Renew(LPWM);
//    Dirction_Renew();
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, RPWM);
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LPWM);                                 //Renew PWN
//    IncrementalPI();
}

/**
    * @breif    Find END point
    * @note     None
    * @param    None
    * @retval   None
    */
int FindEndPoint() {
    int i = 0, sum = 0;
    for (int i = 0; i < MOTOR_NUM; ++i)
        PWM[i] = 45;
    PWM[4] = 70;
    int16_t adcValue[ADC_BUFF_SIZE] = {10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000, 55000};
    for (int i = 0; i < MOTOR_NUM; ++i)
//        PWM_Renew(PWM[i]);

    HAL_Delay(1000);

//    do {
//        HAL_ADC_PollForConversion(&hadc1, 50);//等待转换完成，第二个参数表示超时时间，
//        if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) {
//            adcValue[i] = HAL_ADC_GetValue(&hadc1);//读取ADC转换数据，数据为12位
////            if (AbsOf(lastRPos - RMotorCount) > 1000) {           //error
////                RTurnDir();
////                RPWM_Renew(0);
////                return -1;
////            }
//            printf("%d, %d, %d, %d, %.2lf, %.2lf, %.2lf, 0\n", adcValue, targetPos[1],
//                   __HAL_TIM_GET_COMPARE(&htim3, MOTOR_POSI), __HAL_TIM_GET_COMPARE(&htim3, MOTOR_NEG),
//                   posKp * posBias[0], posKi * posBiasIntegration[0], posKd * (posBias[0] - lastposBias[0]));
//        }
//        i++;
//        i %= ADC_BUFF_SIZE + 1;
//    } while (AbsOf(adcValue[i] - adcValue[(i + 1) % (ADC_BUFF_SIZE + 1)]) >= 1);
    int endPoint = adcValue[i];

    for (int i = 0; i < MOTOR_NUM; ++i)
        PWM[i] = 0;
    for (int i = 0; i < MOTOR_NUM; ++i)
//        PWM_Renew(PWM[i]);
    return endPoint;
}
