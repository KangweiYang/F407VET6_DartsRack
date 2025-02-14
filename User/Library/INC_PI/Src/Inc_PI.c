//
// Created by root on 23-8-26.
//

#include "can.h"
#include "main.h"
#include "usart.h"
#include "MotorUnit.h"

#define MIN_PWM -10000
#define MAX_PWM 10000
#define MIN_STEPPER_PWM -3900
#define MAX_STEPPER_PWM 3900


extern int16_t vel;
//static int16_t PWM;
static int16_t PWM[MOTOR_NUM];

static int16_t stepper0Speed, stepper1Speed;

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

void IncrementalPI(int channel, double velKp, double velKi, int16_t Velocity, double TargetVel) {
    static double VelBias[MOTOR_NUM], LastVelBias[MOTOR_NUM];
//    printf("channel: %d, tar: %d, vel: %d, PWM: %d, P: %lf, I: %lf\r\n", channel, (int)TargetVel, Velocity, PWM[channel], velKp * (VelBias[channel] - LastVelBias[channel]) / 100, velKi * VelBias[channel] / 1000);

    VelBias[channel] = Velocity - TargetVel;                //compute current bias
    if(channel <= 3) {
        PWM[channel] -= (int16_t)( velKp * (VelBias[channel] - LastVelBias[channel]) / 100 + velKi * VelBias[channel] / 1000);        //incremental PI controller
        if (PWM[channel] < MIN_PWM) { PWM[channel] = MIN_PWM; }
        if (PWM[channel] > MAX_PWM) { PWM[channel] = MAX_PWM; }
    } else {
        if(channel == 4)    PWM[channel] = - velKi * VelBias[channel] / 1000;        //incremental PI controller
        if(channel == 5){
            printf("Tension = %d, PWM[5] = %d\n", Velocity, PWM[5]);
            PWM[channel] =  velKi * VelBias[channel] / 1000;        //incremental PI controller
        }
        if(PWM[channel] < MIN_STEPPER_PWM)   {PWM[channel] = MIN_STEPPER_PWM;}
        if(PWM[channel] > MAX_STEPPER_PWM)   {PWM[channel] = MAX_STEPPER_PWM;}
//        stepper0Speed = PWM[4];
//        stepper1Speed = PWM[5];
    }
    LastVelBias[channel] = VelBias[channel];                    //save last velocity bias
    PWM_Renew(channel, PWM[channel]);
}


/**
    * @breif    Use incremental PI algorithm to renew the PWM signals.
    * @note     None
    * @param    None
    * @retval   None
    */
/*
void IncrementalPI(int channel, double velKp, double velKi, double Velocity, double TargetVel) {
    static double VelBias, LastVelBias;
    VelBias = Velocity - TargetVel;                //compute current bias
    PWM -= velKp * (VelBias - LastVelBias) / 100 +
                    velKi * VelBias / 1000;        //incremental PI controller
    if (channel <= 4) {
        if (PWM< MIN_PWM) { PWM= MIN_PWM; }
        if (PWM> MAX_PWM) { PWM= MAX_PWM; }
    } else {
        if (PWM< MIN_STEPPER_PWM) { PWM= MIN_STEPPER_PWM; }
        if (PWM> MAX_STEPPER_PWM) { PWM= MAX_STEPPER_PWM; }
    }
    LastVelBias = VelBias;                    //save last velocity bias
//    printf("%d, %ld, %d, %lf, %lf\r\n", (int)TargetVel, vel, PWM, velKp * (VelBias - LastVelBias) / 100, velKi * VelBias / 1000);
    PWM_Renew(channel, PWM);
}
*/