//
// Created by root on 23-8-18.
//

#include "main.h"
//#include "adc.h"
#include "tim.h"
#include "usart.h"
//#include "gpio.h"
//#include "dma.h"
#include "../../ADC_DMA/Inc/ADC_DMA.h"
#include "../../INC_PI/Inc/Inc_PI.h"

#define ALL_DIR 1

#define ADC_BUFF_SIZE    10                      //ADC buff size

#define MOTOR_POSI  TIM_CHANNEL_1
#define MOTOR_NEG   TIM_CHANNEL_2

#define FORGET  1
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

extern int RDir, LDir;

extern double torKp, torKi;

uint32_t timChannel[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

const int initDir[MOTOR_NUM] = {1 * ALL_DIR, 1 * ALL_DIR,};

static int Dir[MOTOR_NUM] = {1 * ALL_DIR, 1 * ALL_DIR};
//static int Dir[MOTOR_NUM] = {1, 1, 0, 1, -1, 0, 0, 0};
static int16_t targetPos[MOTOR_NUM];
static int motorCount[MOTOR_NUM];
static int PWM[MOTOR_NUM];
static double biasThreshold = 80;
//int endPoint = 3000;                               //End point AD value;
//uint8_t UART_data[2];
static int biasOfPosBias_And_LastPosBias[MOTOR_NUM];
static int lastposBias[MOTOR_NUM];
static int posBiasIntegration[MOTOR_NUM];
static int lastRPos = 0, lastLPos = 0;
static int posBias[MOTOR_NUM];                    //compute posBias

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
    * @breif    Get Dir of channel
    * @note     None
    * @param    channel
    * @retval   Dir[channel]
    */
int GetDirOf(int channel) {
    return Dir[channel];
}

/**
    * @breif    Change the channel to &htim
    * @note     None
    * @param    channel
    * @retval   TIM_HandleTypeDef htim
    */
TIM_HandleTypeDef *getHtimOf(int motor) {
    switch (motor) {
        case 0:
            return &htim1;
        case 1:
            return &htim1;
    }
}

/**
    * @breif    Get the TIM_CHANNEL of Motor POSITIVE
    * @note     None
    * @param    MOTOR channel
    * @retval   TIM_CHANNEL_x
    */
uint32_t getPosiTimChannelOf(int channel) {
    switch (channel) {
        case 0:
            return TIM_CHANNEL_1;
        case 1:
            return TIM_CHANNEL_4;
    }
}

/**
    * @breif    Get the TIM_CHANNEL of Motor POSITIVE
    * @note     None
    * @param    MOTOR channel
    * @retval   TIM_CHANNEL_x
    */
uint32_t getNegTimChannelOf(int channel) {
    switch (channel) {
        case 0:
            return TIM_CHANNEL_2;
        case 1:
            return TIM_CHANNEL_3;
    }
}


/**
    * @breif    Print the infos.
    * @note     None
    * @param    AD_Value
    * @retval   None
    */
void PrintPIDinfo(int channel) {
//    printf("%d, %d, %d, %.2lf, %.2lf, %.2lf\n", targetPos[channel],
//           __HAL_TIM_GET_COMPARE(getHtimOf(channel), getPosiTimChannelOf(channel)), __HAL_TIM_GET_COMPARE(getHtimOf(channel),getNegTimChannelOf(channel)),
//           posKp * posBias[channel], posKi * posBiasIntegration[channel], posKd * (posBias[channel] - lastposBias[channel]));
}

/**
    * @breif    Motor Init
    * @note     None
    * @param    None
    * @retval   None
    */
void MotorInit(void) {
//    for (int i = 0; i < 4; ++i){
//        HAL_TIM_PWM_Start(getHtimOf(i * 2), TIM_CHANNEL_ALL);
//    }
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_ALL);
//    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_ALL);
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_ALL);
//    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_ALL);
    for (int i = 0; i < 2; ++i) {
        HAL_TIM_PWM_Start(getHtimOf(i), getPosiTimChannelOf(i));
        HAL_TIM_PWM_Start(getHtimOf(i), getNegTimChannelOf(i));
    }
}

/**
    * @breif    Init Dir
    * @note     None
    * @param    None
    * @retval   None
    */
void DirInit(int channel) {
    Dir[channel] = initDir[channel];
}

/**
    * @breif    Turn the right motor's direction
    * @note     None
    * @param    channel
    * @retval   None
    */
void turnDirOf(int channel) {
    if (Dir[channel] == -1) {                                                             //Current direction is reverse
        __HAL_TIM_SET_COMPARE(getHtimOf(channel), getNegTimChannelOf(channel), 0);
    } else if (Dir[channel] == 1) {                             //Current direction is normal
        __HAL_TIM_SET_COMPARE(getHtimOf(channel), getPosiTimChannelOf(channel), 0);
    }
    Dir[channel] = -Dir[channel];
}

/**
    * @breif    Renew the MOTOR's PWM
    * @note     None
    * @param    channel, PWM
    * @retval   None
    */
void PWM_Renew(int channel, int PWM) {
    if (Dir[channel] == -1) {                                                             //Current direction is reverse
        __HAL_TIM_SET_COMPARE(getHtimOf(channel), getNegTimChannelOf(channel), PWM);
    } else if (Dir[channel] == 1) {                                                       //Current direction is normal
        __HAL_TIM_SET_COMPARE(getHtimOf(channel), getPosiTimChannelOf(channel), PWM);
    }
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
        motorCount[i] = ADC_DMA_GetADCof(i);
        posBias[i] = targetPos[i] - motorCount[i];                    //compute posBias
        biasOfPosBias_And_LastPosBias[i] = AbsOf(posBias[0] - lastposBias[0]);
        posBias[i] *= FORGET;
        if (biasOfPosBias_And_LastPosBias[i] <=
            biasThreshold)                                      //New algorithm designed by me
            if (AbsOf(posBias[0]) <= biasThreshold)                                      //New algorithm designed by me
                posBiasIntegration[i] = posBias[i];
        //compute intergral of posBias
        if (AbsOf(posKi * posBiasIntegration[i]) >= 60)
            posBiasIntegration[i] *= 0.9;                                               //anti-saturation
        PWM[i] = (posKp * posBias[i] + posKi * posBiasIntegration[i] + posKd * (posBias[i] - lastposBias[i]))
                 * MOTOR_MAX_VEL / MAX_PWM * Dir[i] + 0.5;
        lastposBias[i] = posBias[i];                            //save posBiass
        if (PWM[i] < 0) {
            turnDirOf(i);
            PWM[i] = -PWM[i];
        }
        if (PWM[i] > MAX_PWM) PWM[i] = MAX_PWM;
        PWM_Renew(i, PWM[i]);                                                             //Pure position PID
//        SetTargetVel(i, (double) PWM[i]);
//        IncrementalPI(i);                                                                    //DIDN'T TEST!!!!!
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
        PWM[i] = 65;
    PWM[4] = 70;
    int16_t adcValue[ADC_BUFF_SIZE] = {10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000, 55000};
    for (int i = 0; i < MOTOR_NUM; ++i) {
        PWM_Renew(i, PWM[i]);
    }

    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
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
        PWM_Renew(i, PWM[i]);
    return endPoint;
}

/**
    * @breif    Open circle motor driver
    * @note     None
    * @param    lMSpd, rMSpd
    * @retval   None
    */
void OpenCir(int64_t lMSpd, int64_t rMSpd){
//    PWM_Renew(0, lMSpd);
//    PWM_Renew(1, rMSpd);
    SetTargetVel(0, lMSpd);
    SetTargetVel(1, rMSpd);
}