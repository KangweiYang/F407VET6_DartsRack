//
// Created by root on 23-8-23.
//

#ifndef ADC_DMA_H
#define ADC_DMA_H

#include "main.h"

void IT_Init(void);

uint16_t ADC_DMA_GetADCof(int Channel);

int16_t Get_VelOf(int channel);

uint16_t IT_GetADCofRawData(int num);

#endif //ADC_DMA_H
