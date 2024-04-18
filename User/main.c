#include "stdlib.h"
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "adc.h"
#include "can.h"
#include "usb_otg.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "Library/SERVO/Inc/Servo.h"
#include "Library/STEPPER/Inc/Stepper.h"
#include "Library/MOTOR/Inc/MotorUnit.h"
#include "Library/ADC_DMA/Inc/ADC_DMA.h"
#include "Library/INC_PI/Inc/Inc_PI.h"
#include "Library/DOU_PID/Inc/Double_PID.h"
#include "Library/LED/Inc/LED.h"
#include "Library/SW/Inc/SW.h"
#include "Library/OLED/Inc/OLED.h"
#include "Library/OLED/Inc/UserOLED.h"
#include "Library/CAN/Inc/Can.h"
#include "Library/RS485/Inc/RS485.h"


int PWMtest = 75;
double posKp = 0.15, posKi = 0.00, posKd = 1;
double velKp = 150, velKi = 10;
double posKpStepper = 0.15, posKiStepper = 0.00, posKdStepper = 1;
double velKpStepper = 0, velKiStepper = 20;

int32_t tension1 = 0;
int32_t tension2 = 0;
int32_t stepper0Speed = 0;
int32_t stepper1Speed = 0;


int16_t current[4], pos[4], vel[4];
double targetVel[4];
int targetPos[4];
double targetTen[2] = {-200, 0};

CAN_RxHeaderTypeDef M3508_H_Rx_1;
CAN_RxHeaderTypeDef M3508_H_Rx_2;
CAN_RxHeaderTypeDef M3508_H_Rx_4;
uint8_t RXmessage1[8];
uint8_t RXmessage2[8];
uint8_t RXmessage4[8];

uint16_t ADC1Value[100];
uint32_t adc1in4, adc1in5, adc1in14, adc1in15;

int motor0Flag = 0, motor1Flag = 0, motor2Flag = 0, motor3Flag = 0, stepper0Flag = 0, stepper1Flag = 0;

void DartLoad(uint16_t delayTime) {
    motor0Flag = 0;
    motor1Flag = 1;
    motor2Flag = 0;
    motor3Flag = 1;
    targetVel[1] = 2000;
    targetVel[3] = 2000;
    targetVel[0] = 0;
//    HAL_Delay(delayTime); //3700
    while (HAL_GPIO_ReadPin(HALL_BACK_SW_GPIO_Port, HALL_BACK_SW_Pin) != HALL_DETECTED)
        targetVel[0] = 0;
    HAL_Delay(100);
    motor0Flag = 0;
    motor1Flag = 0;
    motor2Flag = 0;
    motor3Flag = 0;
    targetVel[1] = 0;
    targetVel[3] = 0;
}

void DartRelease(uint16_t delayTime) {
    motor0Flag = 0;
    motor1Flag = 1;
    motor2Flag = 0;
    motor3Flag = 1;
    targetVel[1] = -3000;
    targetVel[3] = -3000;
    targetVel[0] = 0;
//    HAL_Delay(delayTime); //2100
    while (((HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin) != HALL_DETECTED) ||
            HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin) != HALL_DETECTED) &&
           (targetVel[1] != 0 || targetVel[3] != 0)) {

        targetVel[0] = 0;
        if (HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin) == HALL_DETECTED) targetVel[3] = 0;

        if (HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin) == HALL_DETECTED) targetVel[1] = 0;
    }
    motor0Flag = 0;
    motor1Flag = 0;
    motor2Flag = 0;
    motor3Flag = 0;
    targetVel[1] = 0;
    targetVel[3] = 0;
}

void DartShoot(uint16_t delayTime) {
    motor0Flag = 1;
    motor1Flag = 0;
    motor2Flag = 0;
    motor3Flag = 0;
    targetVel[0] = -2000;
//    HAL_Delay(delayTime);
    while (HAL_GPIO_ReadPin(HALL_BACK_SW_GPIO_Port, HALL_BACK_SW_Pin) == HALL_DETECTED);
    HAL_Delay(300);
    targetVel[0] = 0;
    motor0Flag = 0;
}

void UserInit(void) {

    //adc dma
    HAL_ADC_Start_DMA(&hadc1, ADC1Value, 100);

    HAL_TIM_PWM_Init(&htim1);
    HAL_TIM_PWM_Init(&htim3);
    MotorInit();
    IncPI_Init();

    M3508_H_Rx_1.StdId = 0x201;
    M3508_H_Rx_1.ExtId = 0x0;
    M3508_H_Rx_1.IDE = CAN_ID_STD;
    M3508_H_Rx_1.RTR = CAN_RTR_DATA;
    M3508_H_Rx_1.FilterMatchIndex = 0;
    M3508_H_Rx_1.Timestamp = 0;
    M3508_H_Rx_1.DLC = 8;

    M3508_H_Rx_2.StdId = 0x202;
    M3508_H_Rx_2.ExtId = 0x0;
    M3508_H_Rx_2.IDE = CAN_ID_STD;
    M3508_H_Rx_2.RTR = CAN_RTR_DATA;
    M3508_H_Rx_2.FilterMatchIndex = 0;
    M3508_H_Rx_2.Timestamp = 0;

    M3508_H_Rx_4.StdId = 0x204;
    M3508_H_Rx_4.ExtId = 0x0;
    M3508_H_Rx_4.IDE = CAN_ID_STD;
    M3508_H_Rx_4.RTR = CAN_RTR_DATA;
    M3508_H_Rx_4.FilterMatchIndex = 0;
    M3508_H_Rx_4.Timestamp = 0;
    ServoInit();

    StepperInit(STEPPER1, 1680 - 1);
    StepperInit(STEPPER2, 1680 - 1);
    StepperInit(STEPPER3, 1680 - 1);
    StepperInit(STEPPER4, 1680 - 1);
//    HAL_Delay(1000);
//    ServoSet(1,30, 10);
    ServoSet(2, 109, 10);
    ServoSet(3, 500, 10);
    ServoSet(4, 500, 10);
    Double_PID_Init();

    StepperStart(STEPPER1);
    StepperStart(STEPPER2);

    StepperSetSpeed(STEPPER1, 0);
    HAL_Delay(100);
    for (int i = 0; i < 1200; i += 10){
        HAL_Delay(4);
#if STEPPER1_2_DIR == 1
//        StepperSetSpeed(STEPPER1, i);
//        StepperSetSpeed(STEPPER2, -i);
#endif
#if STEPPER1_2_DIR == -1
        StepperSetSpeed(STEPPER1, -i);
        StepperSetSpeed(STEPPER2, i);
#endif
        StepperSetSpeed(STEPPER3, i);
        StepperSetSpeed(STEPPER4, i);
    }

#if STEPPER1_2_DIR == 0
    StepperStop(STEPPER1);
    StepperStop(STEPPER2);
#endif
    StepperStop(STEPPER2);


    HAL_TIM_Base_Start_IT(&htim6);
    for (int i = 0; i < 10; ++i) {
        tension1 = RS485_1_GetTension();
        printf("%ld, %ld\n", tension1, stepper0Speed);
    }

    if(tension1 > targetTen[0] + 2)    stepper0Speed = 1200;
    else if(tension1 < targetTen[0] - 2)    stepper0Speed = -1200;
    if(stepper0Speed < 0){
        for (int i = 0; i > stepper0Speed; i -= 10){
            HAL_Delay(1);
            StepperSetSpeed(STEPPER1, i);
        }
    }
    else {
        for (int i = 0; i < stepper0Speed; i += 10) {
            HAL_Delay(1);
            StepperSetSpeed(STEPPER1, i);
        }
    }
//    for (int i = 0; i < 10; ++i) {
//        tension1 = RS485_1_GetTension();
//        printf("%ld, %ld\n", tension1, stepper0Speed);
//    }
//    if(stepper0Speed == 0){
//        if(STEPPER1_2_Kp > 0){
//            for (int i = 0; i < STEPPER1_2_Kp; ++i) {
//                StepperSetSpeed(STEPPER1, i);
//                stepper0Speed = i;
//                printf("%ld, %ld\n", tension1, stepper0Speed);
//                HAL_Delay(100);
//            }
//        }
//        if(STEPPER1_2_Kp < 0){
//            for (int i = 0; i > STEPPER1_2_Kp; --i){
//                StepperSetSpeed(STEPPER1, i);
//                stepper0Speed = i;
//                printf("%ld, %ld\n", tension1, stepper0Speed);
//                HAL_Delay(100);
//            }
//        }
//    }

    while(tension1 <= targetTen[0] - 5 || tension1 >= targetTen[0] + 5){
        tension1 = RS485_1_GetTension();
        printf("%ld, %ld\n", tension1, stepper0Speed);
        if(stepper0Speed != 1200 && stepper0Speed != -1200){
            StepperSetSpeed(STEPPER1, STEPPER1_2_Kp);
            stepper0Speed = STEPPER1_2_Kp;

        }
    }
    StepperSetSpeed(STEPPER1, 0);
    stepper0Speed = 0;
    HAL_Delay(200);
//    stepper0Flag = 1;

//    for (int i = 0; i > stepper0Speed; i -= 10){
//        StepperSetSpeed(STEPPER1, i);
//        HAL_Delay(10);

//    }

}

void ShootOneDart(void) {
    HAL_Delay(1000);
    ServoGraspDart();
    HAL_Delay(1000);
    DartLoad(3700);
    DartRelease(2100);
    HAL_Delay(1000);
    DartShoot(605);
}

/**
    * @breif    Get pos
    * @note     None
    * @param    None
    * @retval   None
    */
void GetCur(void) {
//
#if USE_CAN1
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &M3508_H_Rx_1, RXmessage1);
#endif
#if USE_CAN2
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &M3508_H_Rx_1, RXmessage1);
#endif
    if (M3508_H_Rx_1.StdId == 0x201) {

        pos[0] = RXmessage1[0] << 8 | RXmessage1[1];
        vel[0] = RXmessage1[2] << 8 | RXmessage1[3];
        current[0] = RXmessage1[4] << 8 || RXmessage1[5];
//    printf("1RX the CAN data: %02X%02X%02X%02X%02X%02X%02X%02X, pos = %ld, vel = %ld, cur = %ld  ", RXmessage1[0], RXmessage1[1], RXmessage1[2],
//           RXmessage1[3], RXmessage1[4], RXmessage1[5], RXmessage1[6], RXmessage1[7], pos[0], vel[0], current[0]);
    }

#if USE_CAN1
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &M3508_H_Rx_2, RXmessage2);
#endif
#if USE_CAN2
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &M3508_H_Rx_2, RXmessage2);
#endif
    if (M3508_H_Rx_2.StdId == 0x202) {
        pos[1] = RXmessage2[0] << 8 | RXmessage2[1];
        vel[1] = RXmessage2[2] << 8 | RXmessage2[3];
        current[1] = RXmessage2[4] << 8 || RXmessage2[5];
//        printf("2RX the CAN data: %02X%02X%02X%02X%02X%02X%02X%02X, pos = %ld, vel = %ld, cur = %ld  \n", RXmessage2[0], RXmessage2[1], RXmessage2[2],
//               RXmessage2[3], RXmessage2[4], RXmessage2[5], RXmessage2[6], RXmessage2[7], pos[1], vel[1], current[1]);
    }

#if USE_CAN1
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &M3508_H_Rx_4, RXmessage4);
#endif
#if USE_CAN2
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &M3508_H_Rx_4, RXmessage4);
#endif
    if (M3508_H_Rx_4.StdId == 0x204) {
        pos[3] = RXmessage4[0] << 8 | RXmessage4[1];
        vel[3] = RXmessage4[2] << 8 | RXmessage4[3];
        current[3] = RXmessage4[4] << 8 || RXmessage4[5];
//        printf("4RX the CAN data: %02X%02X%02X%02X%02X%02X%02X%02X, pos = %ld, vel = %ld, cur = %ld\r\n", RXmessage4[0],
//               RXmessage4[1], RXmessage4[2],
//               RXmessage4[3], RXmessage4[4], RXmessage4[5], RXmessage4[6], RXmessage4[7], pos[3], vel[3], current[3]);
    }
}

void CubeMXInit(void){
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM9_Init();
    MX_I2C1_Init();
    MX_TIM6_Init();
    MX_CAN2_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_TIM12_Init();
    MX_USB_OTG_FS_PCD_Init();
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    CubeMXInit();
    printf("hello\n");
    RS485Init();
    UserInit();


//    stepper0Flag = 0;
//    ShootOneDart();

    while(1){
//        HAL_Delay(1000);

        tension1 = RS485_1_GetTension();
        printf("%ld, %ld\n", tension1, stepper0Speed);
        StepperSetSpeed(STEPPER1, STEPPER1_2_Kp);
        stepper0Speed = STEPPER1_2_Kp;
//        if(tension1 == targetTen[0]){
//            StepperSetSpeed(STEPPER1, 0);
//            stepper0Speed = 0;
//        }
//        else if(tension1 < targetTen[0] && stepper0Speed >= 0){
//            for (int i = 0; i > -(targetTen[0] - tension1) * 21; i -= 10) {
//                StepperSetSpeed(STEPPER1, i);
//                stepper0Speed = i;
//                HAL_Delay(4);
//            }
//        }
//        else if(tension1 > targetTen[0] && stepper0Speed <= 0) {
//            for (int i = 0; i < (tension1 - targetTen[0]) * 21; i += 10) {
//                StepperSetSpeed(STEPPER1, i);
//                stepper0Speed = i;
//                HAL_Delay(4);
//            }
//        }

    }
/*
    while (1) {
        ADC_DMA_Renew();
        for (int i = 0; i < 50; ++i) {
            StepperSetSpeed(STEPPER1, 10 * i);
            StepperSetSpeed(STEPPER2, 10 * i);
            StepperSetSpeed(STEPPER3, 10 * i);
            StepperSetSpeed(STEPPER4, 10 * i);
            HAL_Delay(1);
        }
        HAL_Delay(1000);

        for (int i = 0; i < 50; ++i) {
            StepperSetSpeed(STEPPER1, -10 * i);
            StepperSetSpeed(STEPPER2, -10 * i);
            StepperSetSpeed(STEPPER3, -10 * i);
            StepperSetSpeed(STEPPER4, -10 * i);
            HAL_Delay(1);
        }
        HAL_Delay(1000);
    }

 */

//        printf("PA0: %d  ", __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1));
//        printf("PA1: %d  \n", __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));
//        printf("PA2: %d  ", __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3));
//        printf("PA3: %d\n", __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4));
    HAL_Delay(60);
//        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//        HAL_Delay(333);
//        HAL_Delay(333);

    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim6.Instance) {
        static int cont = 0;
        cont++;
        GetCur();
//        ADC_DMA_Renew();
        IncrementalPI(1, velKp, velKi, vel[0], targetVel[0]);
        IncrementalPI(2, velKp, velKi, vel[1], targetVel[1]);
        IncrementalPI(4, velKp, velKi, vel[3], targetVel[3]);
//        IncrementalPI(5, velKpStepper, velKiStepper, tension1, targetTen[0]);
//        IncrementalPI(2, velKp, velKi, vel[1], targetVel[3]);
//        IncrementalPI(4, velKp, velKi, vel[3], -800);
//        Double_PID(posKp, posKi, posKd, pos, targetPos);
//        PWM_Renew(1, -1000);
//        PWM_Renew(2, 1300);
//        PWM_Renew(4, 2500);
        if (cont == 100) {
//            printf("targetVel = [%lf], [%lf], [%lf], [%lf]\n", targetVel[0], targetVel[1], targetVel[2], targetVel[3]);
            cont = 0;
        }
    }
}