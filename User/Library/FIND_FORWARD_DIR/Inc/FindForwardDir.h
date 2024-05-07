//
// Created by root on 23-12-5.
//

#ifndef FINDFORWARDDIR_H
#define FINDFORWARDDIR_H

#include "main.h"

//void ResetDelaySensors(uint8_t resetSign);

uint32_t BinToDec(uint8_t bin);

uint8_t FindForwardDir(uint16_t sensors);

void MotorControl(int sensors);

void FullDelayMinors(void);

#endif //FINDFORWARDDIR_H
