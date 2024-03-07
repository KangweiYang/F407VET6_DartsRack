//
// Created by root on 23-8-26.
//

#include "../../ADC_DMA/Inc/ADC_DMA.h"
#include "../../MOTOR/Inc/MotorUnit.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#define MIN_PWM -5000
#define MAX_PWM 5000

#define M

extern double velKp, velKi;
static int PWM[MOTOR_NUM];
static double velBias[MOTOR_NUM], LastVelBias[MOTOR_NUM];
static double TargetVel[MOTOR_NUM];

/**
    * @breif    Inc PI init.
    * @note     None
    * @param    None
    * @retval   None
    */
void IncPI_Init(void) {
}

/**
    * @breif    Print the infos.
    * @note     None
    * @param    AD_Value
    * @retval   None
    */
void PrintPIinfo(void) {
//    printf("%lf, %lf, %d, %d, %d, %.2lf, %.2lf\n", Get_VelOf(0), TargetVel[0],
//           __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1), __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2),
//           PWM[0], velKp * (velBias[0] - LastVelBias[0]), velKi * velBias[0]);
}

/**
    * @breif    Input the new Target velocity.
    * @note     None
    * @param    channel, target velocity
    * @retval   None
    */
void SetTargetVel(int channel, int64_t newTargetVel) {
    if (channel < 0 || channel >= MOTOR_NUM) Error_Handler();
//    if(newTargetVel > 0) {
////        PWM_Renew(channel, 80);
//        PWM[channel] = newTargetVel;
//    }
    if(newTargetVel < 0) {
////        PWM_Renew(channel, -80);
        turnDirOf(channel);
//        PWM[channel] = -newTargetVel;
    }
    TargetVel[channel] = newTargetVel;
}

/**
    * @breif    Use incremental PI algorithm to renew the PWM signals.
    * @note     None
    * @param    None
    * @retval   None
    */
void IncrementalPI(int channel) {
//    for (int channel = 0; channel < MOTOR_NUM; ++channel) {
    double Velocity = Get_VelOf(channel);
    velBias[channel] = Velocity - TargetVel[channel];                                     //compute current bias
    PWM[channel] -= (velKp * (velBias[channel] - LastVelBias[channel]) +
                     velKi * velBias[channel]) * GetDirOf(channel);           //incremental PI controller
    if (PWM[channel] < MIN_PWM) { PWM[channel] = MIN_PWM; }
    if (PWM[channel] > MAX_PWM) { PWM[channel] = MAX_PWM; }
    if (PWM[channel] < 0) {
        turnDirOf(channel);
        PWM[channel] = -PWM[channel];
    }
    LastVelBias[channel] = velBias[channel];                                              //save last velocity bias
    PWM_Renew(channel, PWM[channel]);
}