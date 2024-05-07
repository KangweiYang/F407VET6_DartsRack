//
// Created by root on 23-8-23.
//

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "../../MOTOR/Inc/MotorUnit.h"


extern uint16_t ADC1Value[100];
extern uint32_t adc1in4, adc1in5, adc1in14, adc1in15;

void ADC_DMA_Renew() {
        int i, tempAdc1in4 = 0, tempAdc1in5 = 0, tempAdc1in14 = 0, tempAdc1in15 = 0;
        for (i = 0, tempAdc1in4 = 0, tempAdc1in5 = 0, tempAdc1in14 = 0, tempAdc1in15 = 0; i < 100;) {
            tempAdc1in4 += ADC1Value[i++];
            tempAdc1in5 += ADC1Value[i++];
            tempAdc1in14 += ADC1Value[i++];
            tempAdc1in15 += ADC1Value[i++];
        }
    adc1in4 = tempAdc1in4 / 25;
    adc1in5 = tempAdc1in5 / 25;
    adc1in14 = tempAdc1in14 / 25;
    adc1in15 = tempAdc1in15 / 25;
#if ADC_DMA_INFO
    printf("\r\n********ADC-DMA-Example********\r\n");
    printf("[\tmain]info:AD4_value=%1.3fV\r\n", adc1in4 * 3.3f / 4096);
    printf("[\tmain]info:AD5_value=%1.3fV\r\n", adc1in5 * 3.3f / 4096);
    printf("[\tmain]info:AD14_value=%1.3fV\r\n", adc1in14 * 3.3f / 4096);
    printf("[\tmain]info:AD15_value=%1.3fV\r\n", adc1in15 * 3.3f / 4096);
    for (int i = 0; i < 100; i += 4) {
        printf("%d, %d, %d, %d\r\n", ADC1Value[i], ADC1Value[i + 1], ADC1Value[i + 2], ADC1Value[i + 3]);
    }
#endif
}