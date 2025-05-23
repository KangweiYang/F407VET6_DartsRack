//
// Created by 17200 on 2024/5/1.
//

#ifndef F407VET6_DARTSRACK_SHOOTPROCESS_H
#define F407VET6_DARTSRACK_SHOOTPROCESS_H

#include "main.h"

int FeedFSMState(void);

void DartFeedStartDown(void);

void DartFeedResetUntilHallDetected(void);

void DartFeedStartUp(void);

void DartFeedUpUntilSWDetected(void);

void DartFeedLoading(void);

void DartFeedLoadingEnd(void);

int IsDartReadyToLoad(void);

void DartFeedStopDown(void);

void DartReset(void);

void DartLoad(int loadSpeed, int dartSerial);

void DartRelease(int dartSerial);

void DartShoot(int dartSerial);

#endif //F407VET6_DARTSRACK_SHOOTPROCESS_H
