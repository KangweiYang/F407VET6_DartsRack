//
// Created by root on 23-9-26.
//

#include "main.h"
#include "usart.h"
#include "../Inc/Double_PID.h"
#include "../../INC_PI/Inc/Inc_PI.h"
#include "../../MOTOR/Inc/MotorUnit.h"

#define MIN_PWM -16384
#define MAX_PWM 16384
#define FORGET  1

extern double velKp, velKi;
extern int targetVel;
extern int16_t vel;

static int Dir[MOTOR_NUM] = {-1};
static int16_t targetPos[MOTOR_NUM];
static int motorCount[MOTOR_NUM];
static int PWM[MOTOR_NUM];
static double biasThreshold = 50;
static int biasOfPosBias_And_LastPosBias[MOTOR_NUM];
static int lastposBias[MOTOR_NUM];
static int posBiasIntegration[MOTOR_NUM];
static int posBias[MOTOR_NUM];                    //compute posBias

void Double_PID_Init(void){
    IncPI_Init();
}

void Double_PID(double posKp, double posKi, double posKd, int16_t pos, int16_t tarPos){
    targetPos[0] = tarPos;
    motorCount[0] = pos;
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
        PWM[i] = (posKp * posBias[i] + posKi * posBiasIntegration[i] + posKd * (posBias[i] - lastposBias[i])) + 0.5;
        printf("%d, %ld, %d, %lf, %lf, %lf\r\n", (int)tarPos, pos, PWM[i], posKp * posBias[i], posKi * posBiasIntegration[i], posKd * (posBias[i] - lastposBias[i]));
        lastposBias[i] = posBias[i];                                                    //save posBiass
//        PWM_Renew(PWM[i]);
        IncrementalPI(i + 1, velKp, velKi, vel, PWM[i]);
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
