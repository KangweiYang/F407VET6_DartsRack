//
// Created by root on 24-2-29.
//

#ifndef SERVO_H
#define SERVO_H

#include "main.h"

void ServoInit(void);

void ServoSet(int channel, int angle, int delay);

void ServoGraspDart(void);

void ServoLR_ToNextDart(int dartSerial);

void ServoUpDown_DownToGrasp(int dartSerial);

void ServoGrasp_GraspNextDart(int dartSerial);

void ServoUD_UpToMove(int dartSerial);

void ServoLR_ToMiddle(int dartSerial);

void ServoUD_DownToRelease(int dartSerial);

void ServoGrasp_Realease(int dartSerial);

void ServoUD_UpToAvoidCrash(int dartSerial);

void ServoLR_ToNotEdge(int dartSerial);


#endif //SERVO_H
