//
// Created by root on 23-9-26.
//

#ifndef DOUBLE_PID_H
#define DOUBLE_PID_H

#include "main.h"

void Double_PID_Init(void);

void Double_PID(double posKp, double posKi, double posKd, int16_t pos, int16_t tarPos);

#endif //DOUBLE_PID_H
