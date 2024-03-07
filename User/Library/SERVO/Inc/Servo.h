//
// Created by root on 24-2-29.
//

#ifndef SERVO_H
#define SERVO_H

#include "main.h"

void ServoInit(void);

void ServoSet(int channel, int angle, int delay);

void ServoGraspDart(void);

#endif //SERVO_H
