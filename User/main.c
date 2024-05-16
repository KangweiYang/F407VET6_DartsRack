#include "stdlib.h"
#include "main.h"
#include "dma.h"
#include "tim.h"
//#include "adc.h"
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
#include "Library/SHOOT_PROCESS/Inc/ShootProcess.h"

const uint8_t SetYaw[] = {7, 'S', 'e', 't', 'Y', 'a', 'w'};
const uint8_t SetTen[] = {7, 'S', 'e', 't', 'T', 'e', 'n'};
const uint8_t TestShoot[] = {10, 'T', 'e', 's', 't', 'S', 'h', 'o', 'o', 't'};
const uint8_t AbortShoot[] = {11, 'A', 'b', 'o', 'r', 't', 'S', 'h', 'o', 'o', 't'};
const uint8_t SetCurYawToZero[] = {16, 'S', 'e', 't', 'C', 'u', 'r', 'Y', 'a', 'w', 'T', 'o', 'Z', 'e', 'r', 'o'};
const uint8_t ResetFeed[] = {10, 'R', 'e', 's', 'e', 't', 'F', 'e', 'e', 'd'};
const uint8_t SonicRangeTestSetParas[] = {23, 'S', 'o', 'n', 'i', 'c', 'R', 'a', 'n', 'g', 'e', 'T', 'e', 's', 't', 'S', 'e', 't', 'P', 'a', 'r', 'a', 's'};
const uint8_t SonicRangeTest[] = {15, 'S', 'o', 'n', 'i', 'c', 'R', 'a', 'n', 'g', 'e', 'T', 'e', 's', 't'};
const uint8_t ShootTwoDarts[] = {14, 'S', 'h', 'o', 'o', 't', 'T', 'w', 'o', 'D', 'a', 'r', 't', 's'};

int resetFeedCont = -1;

int left3508StopCont = -1, right3508StopCont = -1, releaseFlag = 0;

uint64_t contFromLastUart = 0;
int sonicRangeUpCloseFlag = 1, sonicRangeDownOpenFlag = 0;

int PWMtest = 75;
double posKp = 0.15, posKi = 0.00, posKd = 1;
double velKp = 150, velKi = 10;
double posKpStepper0 = STEPPER1BIGKP, posKiStepper0 = 0.00, posKdStepper0 = STEPPER1BIGKD;
double posKpStepper1 = STEPPER2BIGKP, posKiStepper1 = 0.00, posKdStepper1 = STEPPER2BIGKD;
double velKpStepper = 0, velKiStepper = 10000;

int32_t tension1 = 0;
int32_t tensionL = 0;
int16_t stepper0Speed = 0;
int16_t stepper1Speed = 0;


int16_t current[4], pos[4], vel[6];
double targetVel[4];
int targetPos[4];
double targetTen[2] = {200, 200};
int targetYawPul = 0;

int furTarTen[4];
int furTarYaw[4];

int shootFlag = 0;

CAN_RxHeaderTypeDef M3508_H_Rx_1;
CAN_RxHeaderTypeDef M3508_H_Rx_2;
CAN_RxHeaderTypeDef M3508_H_Rx_4;
uint8_t RXmessage1[8];
uint8_t RXmessage2[8];
uint8_t RXmessage4[8];

uint16_t ADC1Value[100];
uint32_t adc1in4, adc1in5, adc1in14, adc1in15;

int motor0Flag = 0, motor1Flag = 0, motor2Flag = 0, motor3Flag = 0, stepper0Flag = 0, stepper1Flag = 0;
int stepper0WaitingToStopFlag = 0, stepper1WaitingToStopFlag = 0;
uint8_t USART1RxBuf[RX_BUFF_LENGTH];

uint32_t sonicRangeUp = 0, sonicRangeDown = 0;     //ms * 34 (cm/ms)

int tensionControlFlag = 0;

int16_t RxPointer = 0;
int ContainsAndCopy(uint8_t *buf, int16_t *startPointer, uint8_t byteToFind, uint16_t length, uint8_t *copyBuf){
    if(*startPointer > RX_BUFF_LENGTH)    *startPointer = 0;
    for (int i = *startPointer; i < length; ++i){
        if(buf[i] == byteToFind || buf[i] == '\\'){
            copyBuf[0] = i - *startPointer;
            *startPointer = i + 1;
            return 1;
        }
        copyBuf[i - *startPointer + 1] = buf[i];
    }
    return 0;
}

int ContainsSubString(uint8_t *buf, uint8_t *stringToFind){
    for (int i = 0; i < RX_BUFF_LENGTH; ++i){
        if(buf[i] == stringToFind[1]){
            for (int j = 0; j < stringToFind[0] - 1; ++j){
                if(buf[i + j] != stringToFind[j + 1]){
                    return 0;
                }
            }
            return 1;
        }
    }
    return 0;
}
int IntArrayComp(int *array, int equal, int len){
    int wholeEqual = 0;
    for (int i = 0; i < len; ++i){
        if(array[i] == equal)   wholeEqual++;
    }
    return wholeEqual;
}

void UserInit(void) {

    //adc dma
//    HAL_ADC_Start_DMA(&hadc1, ADC1Value, 100);

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
//    StepperInit(STEPPER3, 1680 - 1);
    StepperInit(STEPPER4, 1680 - 1);
    ServoSet(2, 109, 10);
    ServoSet(3, 500, 10);
    ServoSet(4, 500, 10);
    Double_PID_Init();

    StepperSetSpeed(STEPPER1, 0);
    StepperSetSpeed(STEPPER2, 0);

    StepperStart(STEPPER1);
    StepperStart(STEPPER2);
//    StepperStart(STEPPER3);
    StepperStart(STEPPER4);

//    StepperSetSpeed(STEPPER3, 0);
    StepperSetSpeed(STEPPER4, 0);

    StepperSetSpeed(STEPPER1, 0);
    HAL_Delay(100);

    HAL_TIM_Base_Start_IT(&htim6);
//    StepperTensionControlStart(1);
//    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim10);
    stepper0Flag = 1;
    stepper1Flag = 1;
    for (int i = 0; i < 20; ++i){
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
    }
//    HAL_DMA_Init(&hdma2);
//    HAL_UART_Receive_IT(&huart1, USART1RxBuf, RX_BUFF_LENGTH);
    HAL_UART_Receive_DMA(&huart1, USART1RxBuf, RX_BUFF_LENGTH);
}

void ShootOneDart(int dartSerial) {
    stepper0Flag = 0;
    stepper1Flag = 0;
    targetTen[0] = furTarTen[dartSerial - 1];
    targetTen[1] = furTarTen[dartSerial - 1];
    if(dartSerial == 1) targetYawPul = furTarYaw[0];
    else    targetYawPul = -furTarYaw[dartSerial - 1] + furTarYaw[dartSerial];
    ServoGraspDart();
    DartLoad();
    DartRelease();
    stepper0Flag = 1;
    stepper1Flag = 1;
    tensionControlFlag = 1;
    {
        int prevTen1[WAIT_TIMES], prevTenL[WAIT_TIMES], i = 0;
        while (((tension1 != targetTen[0]) || (tensionL != targetTen[1])
            || (IntArrayComp(prevTen1, targetTen[0], WAIT_TIMES) != WAIT_TIMES)
            || (IntArrayComp(prevTenL, targetTen[1], WAIT_TIMES) != WAIT_TIMES)) || sonicRangeUpCloseFlag) {
            stepper0Flag = 1;
            stepper1Flag = 1;
            prevTen1[i] = tension1;
            tension1 = RS485_1_GetTension();
            prevTenL[i] = tensionL;
            tensionL = RS485_2_GetTension();
            i++;
            if(i >= WAIT_TIMES) i = 0;
            printf("Ten1: %d, Ten2: %d, prevTen1   ", tension1, tensionL);
            for (int j = 0; j < WAIT_TIMES; ++j){
                printf("%d, ", prevTen1[j]);
            }
            printf("///");
            for (int j = 0; j < WAIT_TIMES; ++j){
                printf("%d, ", prevTen1[j]);
            }
            printf("\n");
        }
    }
    tensionControlFlag = 0;
    stepper0Flag = 0;
    stepper1Flag = 0;
    DartShoot();
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
    MX_TIM10_Init();
    MX_UART4_Init();
    MX_UART5_Init();
    MX_USART6_UART_Init();
    MX_TIM7_Init();
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    CubeMXInit();
    RS485Init();
    printf("hello\n");
    UserInit();

    while(1) {
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
        if(shootFlag == 5){
            ShootOneDart(1);
            shootFlag = 0;
        }
        if (sonicRangeDownOpenFlag && contFromLastUart > CONT_TO_READY_TO_SHOOT && furTarTen[1] != 0 && shootFlag < 4){
            shootFlag++;
            printf("shootFlag: %d\n", shootFlag);
            ShootOneDart(shootFlag);
        }
    }
    /* USER CODE BEGIN 3 */
}

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart-> Instance == huart1.Instance ) {
        uint8_t rxHandleBuf[RX_BUFF_LENGTH];
        for (int i = 0; i < 50; ++i){
            printf("%c", rxHandleBuf[i]);
        }
        printf("\n");
        if(ContainsAndCopy(USART1RxBuf, &RxPointer, '\n', RX_BUFF_LENGTH, rxHandleBuf)){
            if(ContainsSubString(rxHandleBuf, SetYaw)){
                printf("SetYaw\n");
            }
        }
        HAL_UART_AbortReceive_IT(&huart1);
        HAL_UART_Receive_IT(&huart1, USART1RxBuf, RX_BUFF_LENGTH);
    }
}
 */


/* USER CODE END 3 */

int StrToInt(uint8_t *buf, int16_t startPointer, uint8_t endChar){
    uint8_t intBuf[50];
    int result = 0;
    if(ContainsAndCopy(buf, &startPointer, endChar, RX_BUFF_LENGTH, intBuf)){
//        printf("\n");
//        for (int i = 0; i < 2; ++i) printf("%c", intBuf[i]);
//        printf("\n");
        int sign = 0;
        if(intBuf[1] < '9' && intBuf[1] > '0')  sign = 1;
        else if(intBuf[1] == '-'){
            intBuf[1] = '0';
            sign = -1;
        }
        for (int i = 1; i <= intBuf[0]; ++i){
            if(intBuf[i] > '9' || intBuf[i] < '0')   return result;
            result *= 10;
            result += sign * (intBuf[i] - '0') % 10;
        }
        return result;
    }
    return 0;
}

//int IndexOf(uint8_t *buf, uint8_t )


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /*
    if (htim->Instance == htim7.Instance && targetYawPul != 0) {     //100us timer
        static int lastYawPul = 1;
        if (targetYawPul * lastYawPul < 0)
            HAL_GPIO_TogglePin(STEPPER3_DIR_GPIO_Port, STEPPER3_DIR_Pin);
        if (targetYawPul > 0) {
            HAL_GPIO_TogglePin(YAW_STEPPER_PUL_GPIO_Port, YAW_STEPPER_PUL_Pin);
//            printf("+\n");
            lastYawPul = targetYawPul;
            targetYawPul--;
        } else if (targetYawPul < 0) {
            HAL_GPIO_TogglePin(YAW_STEPPER_PUL_GPIO_Port, YAW_STEPPER_PUL_Pin);
//            printf("-\n");
            lastYawPul = targetYawPul;
            targetYawPul++;
        }
    }
     */
    if (htim->Instance == htim6.Instance) {     //1ms timer
        static int cont = 0;
        cont++;
        GetCur();
        IncrementalPI(0, velKp, velKi, vel[0], targetVel[0]);
        IncrementalPI(1, velKp, velKi, vel[1], targetVel[1]);
        IncrementalPI(3, velKp, velKi, vel[3], targetVel[3]);
//        IncrementalPI(4, velKpStepper, velKiStepper, tension1, targetTen[0]);
//        IncrementalPI(5, velKpStepper, velKiStepper, tensionL, targetTen[1]);
        static int lastYawPul = 1;
        if (targetYawPul * lastYawPul < 0)
            HAL_GPIO_TogglePin(STEPPER3_DIR_GPIO_Port, STEPPER3_DIR_Pin);
        if (targetYawPul > 0) {
            HAL_GPIO_TogglePin(YAW_STEPPER_PUL_GPIO_Port, YAW_STEPPER_PUL_Pin);
//            printf("+\n");
            lastYawPul = targetYawPul;
            targetYawPul--;
        } else if (targetYawPul < 0) {
            HAL_GPIO_TogglePin(YAW_STEPPER_PUL_GPIO_Port, YAW_STEPPER_PUL_Pin);
//            printf("-\n");
            lastYawPul = targetYawPul;
            targetYawPul++;
        }
        if (cont == 10) {
            if(stepper0Flag == 0) StepperSetSpeed(STEPPER1, 0);
            else if (stepper0Flag && tension1 <= 600 && tension1 >= 5) {
//            IncrementalPI(4, velKpStepper, velKiStepper, tension1, targetTen[0]);
                StepperStart(STEPPER1);
                static double lastBias;
                if(((double) tension1 - targetTen[0]) <= STEPPER_CHANGE_TO_SMALL_K && ((double) tension1 - targetTen[0]) >= -STEPPER_CHANGE_TO_SMALL_K){
                    if(targetTen[0] < 420) {
                        posKpStepper0 = STEPPER1SMALLKP;
                        posKdStepper0 = STEPPER1SMALLKD;
                    }
                    else{
                        posKpStepper0 = STEPPER1SMALLSMALLKP;
                        posKdStepper0 = STEPPER1SMALLSMALLKD;
                    }
                }
                else if(((double) tension1 - targetTen[0]) > 2 * STEPPER_CHANGE_TO_SMALL_K || ((double) tension1 - targetTen[0]) < -2 * STEPPER_CHANGE_TO_SMALL_K) {
                    posKpStepper0 = STEPPER1BIGKP;
                    posKdStepper0 = STEPPER1BIGKD;
                }
                if(STEPPER1_Kp < -STEPPER1_2_MAX_PUL) {
                    StepperSetSpeed(STEPPER1, -STEPPER1_2_MAX_PUL);
                    stepper0Speed = -STEPPER1_2_MAX_PUL;
                }
                else if(STEPPER1_Kp > STEPPER1_2_MAX_PUL) {
                    StepperSetSpeed(STEPPER1, STEPPER1_2_MAX_PUL);
                    stepper0Speed = STEPPER1_2_MAX_PUL;
                }
                else {
                    StepperSetSpeed(STEPPER1, STEPPER1_Kp);
                    stepper0Speed = STEPPER1_Kp;
                }
                lastBias = (double) tension1 - targetTen[0];
//            printf("%lf\n", STEPPER1_Kp);
//                printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d\n", targetYawPul, tension1, tensionL, stepper0Speed, stepper1Speed);
            }
            else if (tension1 > 600)    StepperSetSpeed(STEPPER1, 0);
            if(stepper1Flag == 0) StepperSetSpeed(STEPPER2, 0);
            else if (stepper1Flag && tensionL <= 600 && tensionL >= 5) {
                int32_t tensionLL = tensionL;
                static double lastBias;
                if((targetTen[1] - (double) tensionLL) <= STEPPER_CHANGE_TO_SMALL_K && (targetTen[1] - (double) tensionLL) >= -STEPPER_CHANGE_TO_SMALL_K){
                    if(targetTen[1] < 420) {
                        posKpStepper1 = STEPPER2SMALLKP;
                        posKdStepper1 = STEPPER2SMALLKD;
                    } else{
                        posKpStepper1 = STEPPER2SMALLSMALLKP;
                        posKdStepper1 = STEPPER2SMALLSMALLKD;
                    }
                } else if((targetTen[1] - (double) tensionLL) > 2 * STEPPER_CHANGE_TO_SMALL_K || (targetTen[1] - (double) tensionLL) < -2 * STEPPER_CHANGE_TO_SMALL_K){
                    posKpStepper1 = STEPPER2BIGKP;
                    posKdStepper1 = STEPPER2BIGKD;
                }
//            IncrementalPI(5, velKpStepper, velKiStepper, tensionL, targetTen[1]);
                StepperStart(STEPPER2);
                if(STEPPER2_Kp < -STEPPER1_2_MAX_PUL) {
                    StepperSetSpeed(STEPPER2, -STEPPER1_2_MAX_PUL);
                    stepper1Speed = -STEPPER1_2_MAX_PUL;
                }
                else if(STEPPER2_Kp > STEPPER1_2_MAX_PUL) {
                    StepperSetSpeed(STEPPER2, STEPPER1_2_MAX_PUL);
                    stepper1Speed = STEPPER1_2_MAX_PUL;
                }
                else    {
                    StepperSetSpeed(STEPPER2, STEPPER2_Kp);
                    stepper1Speed = STEPPER2_Kp;
                }
//                lastBias = (double) tensionL - targetTen[1];
                lastBias = targetTen[1] - (double) tensionLL;
//            printf("%ld, %lf = -20 * (%lf - %ld)\n", tensionL, STEPPER2_Kp, targetTen[1], tensionLL);
//                printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d\n", targetYawPul, tension1, tensionLL, stepper0Speed, stepper1Speed);
            }
            else if (tensionL > 600)    StepperSetSpeed(STEPPER2, 0);
            cont = 0;
        }
    }
    if (htim->Instance == htim10.Instance) {     //100ms timer
        static uint16_t pointer, couut;
        couut++;
        static int32_t lastTension1, lastTensionL;
        if(releaseFlag == 1 && lastTension1 - tension1 >= -1 && lastTension1 - tension1 <= 1 && lastTensionL - tensionL >= -1 && lastTensionL - tensionL <= 1){
            stepper0Flag = 1;
            stepper1Flag = 1;
        }
        if(contFromLastUart > 0)
            contFromLastUart++;
        if(left3508StopCont > 0){
            left3508StopCont--;
        }
        else if(left3508StopCont == 0){
            left3508StopCont--;
            targetVel[1] = 0;
        }
        if(right3508StopCont > 0){
            right3508StopCont--;
        } else if(right3508StopCont == 0){
            right3508StopCont--;
            targetVel[3] = 0;
        }
        //sonic range
        if(couut % 2 == 0) {
            HAL_GPIO_TogglePin(SONIC_RANGE_TRIG1_GPIO_Port, SONIC_RANGE_TRIG1_Pin);
        }
        else{
            HAL_GPIO_TogglePin(SONIC_RANGE_TRIG2_GPIO_Port, SONIC_RANGE_TRIG2_Pin);
        }
        //sonic range
        if(couut >= 10){
#if HALL_INFO
        printf("BACK: %d, LEFT: %d, RIGHT: %d\n", HAL_GPIO_ReadPin(HALL_BACK_SW_GPIO_Port, HALL_BACK_SW_Pin),
               HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin),
               HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin));
#endif
#if TEN_INFO
//            printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d, tarYawPul: %d, furYaw[0]=: %d, sonicRangeUp: %ld, sonicRangeDown: %ld, Kp: %.1lf, %.1lf, Kd: %.1lf, %.1lf, UpClose: %d, DownOpen: %d\n", targetYawPul, tension1, tensionL, stepper0Speed, stepper1Speed, targetYawPul, furTarYaw[0], sonicRangeUp, sonicRangeDown, posKpStepper0, posKpStepper1, posKdStepper0, posKdStepper1, sonicRangeUpCloseFlag, sonicRangeDownOpenFlag);
            printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d, sonicRangeUp: %ld, sonicRangeDown: %ld,UpClose: %d, DownOpen: %d\n", targetYawPul, tension1, tensionL, stepper0Speed, stepper1Speed, sonicRangeUp, sonicRangeDown, sonicRangeUpCloseFlag, sonicRangeDownOpenFlag);
/*
            for (int i = 0; i < 500; ++i){
                printf("%c", USART1RxBuf[i]);
            }
            printf("\n");
*/
            couut = 0;
#endif
        }
        if(resetFeedCont > 0)   resetFeedCont--;
        else if (resetFeedCont == 0) {
            resetFeedCont = -1;
            StepperSetSpeed(STEPPER4, 0);
        }
        uint8_t rxHandleBuf[RX_BUFF_LENGTH];
        if (ContainsAndCopy(USART1RxBuf, &RxPointer, '\n', RX_BUFF_LENGTH, rxHandleBuf)) {
            if (ContainsSubString(rxHandleBuf + 1, SetYaw)) {
                contFromLastUart = 1;
                switch (rxHandleBuf[8]) {
                    case '1':
                        furTarYaw[0] = StrToInt(rxHandleBuf, 10, ')');
                        printf("yaw: %d\n", furTarYaw[0]);
                        break;
                    case '2':
                        furTarYaw[1] = StrToInt(rxHandleBuf, 10, ')');
                        printf("yaw: %d\n", furTarYaw[1]);
                        break;
                    case '3':
                        furTarYaw[2] = StrToInt(rxHandleBuf, 10, ')');
                        printf("yaw: %d\n", furTarYaw[2]);
                        break;
                    case '4':
                        furTarYaw[3] = StrToInt(rxHandleBuf, 10, ')');
                        printf("yaw: %d\n", furTarYaw[3]);
                        break;
                }
            } else if (ContainsSubString(rxHandleBuf + 1, SetTen)) {
                contFromLastUart = 1;
                shootFlag = 0;
                switch (rxHandleBuf[8]) {
                    case '1':
                        furTarTen[0] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten: %d\n", furTarTen[0]);
                        break;
                    case '2':
                        furTarTen[1] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten: %d\n", furTarTen[1]);
                        break;
                    case '3':
                        furTarTen[2] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten: %d\n", furTarTen[2]);
                        break;
                    case '4':
                        furTarTen[3] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten: %d\n", furTarTen[3]);
                }
            } else if (ContainsSubString(rxHandleBuf, TestShoot)) {
                static int cout1;
                cout1++;
                printf("TestShoot %d\n", cout1);
                shootFlag = 5;
            } else if (ContainsSubString(rxHandleBuf, AbortShoot)) {
                static int cout2;
                cout2++;
                printf("AbortShoot %d\n", cout2);
                shootFlag = 0;
            } else if (ContainsSubString(rxHandleBuf, SetCurYawToZero)) {
                static int cout3;
                cout3++;
                contFromLastUart = 1;
                shootFlag = 0;
                printf("SetCurYawToZero %d\n", cout3);
                targetYawPul = furTarYaw[0];
            } else if (ContainsSubString(rxHandleBuf, ResetFeed)) {
                static int cout4;
                cout4++;
                contFromLastUart = 1;
                shootFlag = 0;
                printf("ResetFeed %d\n", cout4);
                StepperSetSpeed(STEPPER4, -500);
                StepperStart(STEPPER4);
                resetFeedCont += 28;
            } else if (ContainsSubString(rxHandleBuf, SonicRangeTestSetParas)) {
                static int cout6;
                cout6++;
                printf("SonicRangeTestSetParas %d\n", cout6);
            } else if (ContainsSubString(rxHandleBuf, SonicRangeTest)) {
                static int cout5;
                cout5++;
                printf("SonicRangeTest %d\n", cout5);
            } else if (ContainsSubString(rxHandleBuf, ShootTwoDarts)) {
                static int cout7;
                cout7++;
                printf("ShootTwoDarts %d\n", cout7);
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == SONIC_RANGE_ECHO1_Pin) {
        static uint32_t risingTime;
        if(HAL_GPIO_ReadPin(SONIC_RANGE_ECHO1_GPIO_Port, SONIC_RANGE_ECHO1_Pin) == GPIO_PIN_SET){
            risingTime = HAL_GetTick();
        }
        else if(risingTime != 0){
            uint32_t deltaTime = HAL_GetTick() - risingTime;
            risingTime = 0;
            if(deltaTime < 12) {
                sonicRangeUp = deltaTime * 34;
            }
        }
        static uint32_t lastSonicRangeUp;
        if(sonicRangeUp < 50 && lastSonicRangeUp > 100){
            sonicRangeUpCloseFlag = 1;
        }
        if(sonicRangeUp > 100){
            sonicRangeUpCloseFlag = 0;
        }
        lastSonicRangeUp = sonicRangeUp;
    }
    if(GPIO_Pin == SONIC_RANGE_ECHO2_Pin) {
        static uint32_t risingTime;
        if (HAL_GPIO_ReadPin(SONIC_RANGE_ECHO2_GPIO_Port, SONIC_RANGE_ECHO2_Pin) == GPIO_PIN_SET) {
            risingTime = HAL_GetTick();
        } else if (risingTime != 0) {
            uint32_t deltaTime = HAL_GetTick() - risingTime;
//            printf("%ld, %ld\n", HAL_GetTick(), risingTime);
            risingTime = 0;
            if(deltaTime < 12) {
                sonicRangeDown = deltaTime * 34;
            }
//        }
        }
        static uint32_t lastSonicRangeDown;
        if(lastSonicRangeDown < 50 && sonicRangeDown > 100){
            sonicRangeDownOpenFlag = 1;
        }
        if(sonicRangeDown < 50){
            sonicRangeDownOpenFlag = 0;
        }
        lastSonicRangeDown = sonicRangeDown;
    }
    if(GPIO_Pin == SONIC_RANGE_ECHO2_Pin) {
    }
//    if(GPIO_Pin == HALL_BACK_SW_Pin){
//
//    }
    if(GPIO_Pin == HALL_LEFT_SW_Pin){
        printf("LEFT_HALL_EXTI\n");
        if(targetVel[1] == RESET_SPEED){
            targetVel[1] = 0;
//        } else if(targetVel[1] == -RESET_SPEED){
//            targetVel[1] = 0;
        }
        if(releaseFlag) {
            left3508StopCont = 1;
        }
    }
    if(GPIO_Pin == HALL_RIGHT_SW_Pin){
        printf("RIGHT_HALL_EXTI\n");
        if(targetVel[3] == RESET_SPEED){
            targetVel[3] = 0;
//        } else if (targetVel[3] == -RESET_SPEED){
//            targetVel[3] = 0;
        }
        if(releaseFlag) {
            right3508StopCont = 1;
        }
    }
}