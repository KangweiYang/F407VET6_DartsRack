//
// Created by root on 23-8-18.
//

#ifndef MOTORUNIT_H
#define MOTORUNIT_H

int AbsOf(int x);

void PrintPIDinfo(int AD_Value);

void ChangetargetPosOf(int channel, int newtargetPos);

void MotorInit(void);

void PWM_Renew(int channel, int16_t PWM);

void Position_PID(void);

int FindEndPoint();

#endif //MOTORUNIT_H
