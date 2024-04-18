//
// Created by root on 23-8-26.
//

#ifndef INC_PI_H
#define INC_PI_H

#include "can.h"

void IncPI_Init(void);

void IncrementalPI(int channel, double velKp, double velKi, double Velocity, double TargetVel);

#endif //INC_PI_H
