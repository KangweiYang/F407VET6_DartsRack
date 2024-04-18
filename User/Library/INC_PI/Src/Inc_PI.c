//
// Created by root on 23-8-26.
//

#include "can.h"
#include "main.h"
#include "usart.h"
#include "MotorUnit.h"

#define MIN_PWM -16384
#define MAX_PWM 16384
#define MIN_STEPPER_PWM -1200
#define MAX_STEPPER_PWM 1200


extern int16_t vel;
static int16_t PWM[MOTOR_NUM];
/**
    * @breif    Inc PI init.
    * @note     None
    * @param    None
    * @retval   None
    */
void IncPI_Init(void) {
    MotorInit();
}

/**
    * @breif    Use incremental PI algorithm to renew the PWM signals.
    * @note     None
    * @param    None
    * @retval   None
    */
void IncrementalPI(int channel, double velKp, double velKi, double Velocity, double TargetVel) {
    static double VelBias[MOTOR_NUM], LastVelBias[MOTOR_NUM];
    VelBias[channel] = Velocity - TargetVel;                //compute current bias
    PWM[channel] -= velKp * (VelBias[channel] - LastVelBias[channel]) / 100 + velKi * VelBias[channel] / 1000;        //incremental PI controller
    if(channel <= 3) {
        if (PWM[channel] < MIN_PWM) { PWM[channel] = MIN_PWM; }
        if (PWM[channel] > MAX_PWM) { PWM[channel] = MAX_PWM; }
    } else{
        if(PWM[channel] < MIN_STEPPER_PWM)   {PWM[channel] = MIN_STEPPER_PWM;}
        if(PWM[channel] > MAX_STEPPER_PWM)   {PWM[channel] = MAX_STEPPER_PWM;}
    }
    LastVelBias[channel] = VelBias[channel];                    //save last velocity bias
//    printf("%d, %ld, %d, %lf, %lf\r\n", (int)TargetVel, vel, PWM, velKp * (VelBias - LastVelBias) / 100, velKi * VelBias / 1000);
    PWM_Renew(channel, PWM[channel]);
}