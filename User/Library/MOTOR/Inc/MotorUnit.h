//
// Created by root on 23-8-18.
//

#ifndef MOTORUNIT_H
#define MOTORUNIT_H

void PrintPIDinfo(int channel);

void ChangetargetPosOf(int channel, int newtargetPos);

int GetDirOf(int channel);

void MotorInit(void);

void DirInit(int channel);

void turnDirOf(int channel);

int GetCurTor(int motor);

void PWM_Renew(int channel, int PWM);

void Position_PID(void);

int FindEndPoint();

void OpenCir(int64_t lMSpd, int64_t rMSpd);

#endif //MOTORUNIT_H
