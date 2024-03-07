//
// Created by root on 23-8-26.
//

#ifndef INC_PI_H
#define INC_PI_H

#include "main.h"

void IncPI_Init(void);

void PrintPIinfo(void);

void SetTargetVel(int channel, int64_t newTargetVel);

void IncrementalPI(int channel);

#endif //INC_PI_H
