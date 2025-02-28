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
#include "Library/REMOTE/Inc/remote.h"

const uint8_t SetYaw[] = {7, 'S', 'e', 't', 'Y', 'a', 'w'};
const uint8_t SetTen[] = {7, 'S', 'e', 't', 'T', 'e', 'n'};
const uint8_t TestShoot[] = {10, 'T', 'e', 's', 't', 'S', 'h', 'o', 'o', 't'};
const uint8_t AbortShoot[] = {11, 'A', 'b', 'o', 'r', 't', 'S', 'h', 'o', 'o', 't'};
const uint8_t SetCurYawToZero[] = {16, 'S', 'e', 't', 'C', 'u', 'r', 'Y', 'a', 'w', 'T', 'o', 'Z', 'e', 'r', 'o'};
const uint8_t ResetFeed[] = {10, 'R', 'e', 's', 'e', 't', 'F', 'e', 'e', 'd'};
const uint8_t SonicRangeTestSetParas[] = {23, 'S', 'o', 'n', 'i', 'c', 'R', 'a', 'n', 'g', 'e', 'T', 'e', 's', 't', 'S', 'e', 't', 'P', 'a', 'r', 'a', 's'};
const uint8_t SonicRangeTest[] = {15, 'S', 'o', 'n', 'i', 'c', 'R', 'a', 'n', 'g', 'e', 'T', 'e', 's', 't'};
const uint8_t ShootTwoDarts[] = {14, 'S', 'h', 'o', 'o', 't', 'T', 'w', 'o', 'D', 'a', 'r', 't', 's'};
const uint8_t SetZeroTension_1[] = {18, 'S', 'e', 't', 'Z', 'e', 'r', 'o', 'T', 'e', 'n', 's', 'i', 'o', 'n', '(', '1', ')'};
const uint8_t SetZeroTension_2[] = {18, 'S', 'e', 't', 'Z', 'e', 'r', 'o', 'T', 'e', 'n', 's', 'i', 'o', 'n', '(', '2', ')'};

const uint8_t JudgeUart020A[] = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x02};
const uint8_t JudgeUart0105[] = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01};
const uint8_t JudgeUart0001[] = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};

int resetFeedCont = -1;

int left3508StopCont = -1, right3508StopCont = -1, releaseFlag = 0;

uint64_t contFromLastUart = 0;
int sonicRangeUpCloseFlag = 1, sonicRangeDownOpenFlag = 0, canShootFlag = 0;

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
double targetTen[2] = {START_TENSION, START_TENSION};
int targetYawPul = 0;

int furTarTen[4];
int furTarYaw[4];

int shootFlag = 0;

CAN_RxHeaderTypeDef M3508_H_Rx_1;
CAN_RxHeaderTypeDef M3508_H_Rx_2;
CAN_RxHeaderTypeDef M3508_H_Rx_3;
CAN_RxHeaderTypeDef M3508_H_Rx_4;
uint8_t RXmessage1[8];
uint8_t RXmessage2[8];
uint8_t RXmessage3[8];
uint8_t RXmessage4[8];

uint16_t ADC1Value[100];
uint32_t adc1in4, adc1in5, adc1in14, adc1in15;

int motor0Flag = 0, motor1Flag = 0, motor2Flag = 0, motor3Flag = 0, stepper0Flag = 0, stepper1Flag = 0;
int stepper0WaitingToStopFlag = 0, stepper1WaitingToStopFlag = 0;
uint8_t USART1RxBuf[RX_BUFF_LENGTH], USART6RxBuf[RX6_BUFF_LENGTH];

uint32_t sonicRangeUp = 200, sonicRangeDown = 180;     //ms * 34 (cm/ms)

int tensionControlFlag = 0;

int tension1SetZeroFlag = 0, tension2SetZeroFlag = 0;

int16_t RxPointer = 0, Rx6Pointer = 0;
extern int backCont;
extern uint8_t _rx_buf[18];

enum {
    UNKNOWN = 0, RMUC, RMSINGLE, RMAI, RM3V3, RM1V1
}game_type;
enum {
    OUT_OF_GAME = 0, PREPARE, SELF_TEST_15S, COUNTDOWN_5S, IN_GAME, GAME_SETTLE
}game_progress;
uint16_t stage_remain_time = 0;

enum SILO_GATE
{
    OPEN = 0, CLOSE, MOVING
}dart_launch_opening_status;
uint16_t latest_launch_cmd_time = 0;
uint16_t dart_remaining_time = 0;

/**
 * @brief    在缓冲区中查找指定字节并复制数据
 * @note     该函数用于在指定的缓冲区范围内查找特定字节，如果找到，则将从起始指针到找到位置之间的数据复制到目标缓冲区，并更新起始指针的位置。
 *           如果未找到指定字节，则将整个范围内的数据复制到目标缓冲区。
 * @param    buf            指向源缓冲区的指针
 * @param    startPointer   指向起始指针的指针，用于记录查找的起始位置
 * @param    byteToFind      需要查找的字节
 * @param    length          查找的长度范围
 * @param    copyBuf         指向目标缓冲区的指针，用于存放复制的数据
 * @retval   1               如果找到指定字节，返回1
 * @retval   0               如果未找到指定字节，返回0
 */
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

/**
 * @brief    在缓冲区中查找指定字节序列并复制数据
 * @note     该函数用于在指定的缓冲区范围内查找特定字节序列，如果找到，则将从字节序列后的第7个字节开始，长度为字节序列中第2个字节所表示的长度的数据复制到目标缓冲区，并更新起始指针的位置。
 *           如果未找到指定字节序列，则不进行复制操作。
 * @param    buf            指向源缓冲区的指针
 * @param    startPointer   指向起始指针的指针，用于记录查找的起始位置
 * @param    bytesToFind     指向需要查找的字节序列的指针，至少包含7个字节
 * @param    length          查找的长度范围
 * @param    copyBuf         指向目标缓冲区的指针，用于存放复制的数据
 * @retval   1               如果找到指定字节序列，返回1
 * @retval   0               如果未找到指定字节序列，返回0
 */
int ContainsBytesAndCopy(uint8_t *buf, int16_t *startPointer, uint8_t *bytesToFind, uint16_t length, uint8_t *copyBuf){
//    if(*startPointer > RX6_BUFF_LENGTH)
        *startPointer = 0;
    for (int i = *startPointer; i < length && *startPointer <= RX6_BUFF_LENGTH; ++i){
        if(buf[i] == bytesToFind[0] && buf[i + 5] == bytesToFind[5] && buf[i + 6] == bytesToFind[6]) {
            for (int j = 1; j < buf[i + 1] + 1; ++j) {
                copyBuf[j] = buf[i + j + 6];
            }
            copyBuf[0] = buf[i + 1];            //length
            buf[i] = '\0';                      //avoid receive again
            *startPointer = i + 1;
            return 1;
        }
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

    M3508_H_Rx_3.StdId = 0x203;
    M3508_H_Rx_3.ExtId = 0x0;
    M3508_H_Rx_3.IDE = CAN_ID_STD;
    M3508_H_Rx_3.RTR = CAN_RTR_DATA;
    M3508_H_Rx_3.FilterMatchIndex = 0;
    M3508_H_Rx_3.Timestamp = 0;

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
    ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, 10);
    ServoSet(SERVO_UP_DOWN, SERVO_UP_DOWN_UP, 1);                      //Start up
    ServoSet(4, 500, 10);
    Double_PID_Init();

    StepperSetSpeed(STEPPER1, 0);
    StepperSetSpeed(STEPPER2, 0);


//    StepperSetSpeed(STEPPER3, 0);
    StepperSetSpeed(STEPPER4, 0);
    HAL_Delay(100);

//    StepperTensionControlStart(1);
//    HAL_TIM_Base_Start_IT(&htim7);
    stepper0Flag = 1;
    stepper1Flag = 1;
    for (int i = 0; i < 20; ++i){
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
    }
//    HAL_DMA_Init(&hdma2);
//    HAL_UART_Receive_IT(&huart1, USART1RxBuf, RX_BUFF_LENGTH);
    HAL_UART_Receive_DMA(&huart1, USART1RxBuf, RX_BUFF_LENGTH);
    HAL_UART_Receive_DMA(&huart6, USART6RxBuf, RX6_BUFF_LENGTH); //judge system uart
//    RemoteInit();

    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);

}

void ShootFirstDart(){
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
    stepper0Flag = 0;
    stepper1Flag = 0;
    targetTen[0] = furTarTen[0];
    targetTen[1] = furTarTen[0];
    targetYawPul = furTarYaw[0];
    DartReset();
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
    DartLoad();
    DartRelease();
    tensionControlFlag = 1;
    {
        int prevTen1[WAIT_TIMES], prevTenL[WAIT_TIMES], i = 0;
        while ((tension1 != targetTen[0]) || (tensionL != targetTen[1])
               || (IntArrayComp(prevTen1, targetTen[0], WAIT_TIMES) != WAIT_TIMES)
               || (IntArrayComp(prevTenL, targetTen[1], WAIT_TIMES) != WAIT_TIMES)) {
            stepper0Flag = 1;
            stepper1Flag = 1;
            prevTen1[i] = tension1;
            tension1 = RS485_1_GetTension();
            prevTenL[i] = tensionL;
            tensionL = RS485_2_GetTension();
            i++;
            if(i >= WAIT_TIMES) i = 0;
            /*
            printf("Ten1: %d, Ten2: %d, prevTen1   ", tension1, tensionL);
            for (int j = 0; j < WAIT_TIMES; ++j){
                printf("%d, ", prevTen1[j]);
            }
            printf("///");
            for (int j = 0; j < WAIT_TIMES; ++j){
                printf("%d, ", prevTen1[j]);
            }
            printf("\n");
             */
        }
    }
    tensionControlFlag = 0;
    stepper0Flag = 0;
    stepper1Flag = 0;
    DartShoot();
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
}
void ShootOneDart(int dartSerial) {
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
    stepper0Flag = 0;
    stepper1Flag = 0;
    targetTen[0] = furTarTen[dartSerial - 1];
    targetTen[1] = furTarTen[dartSerial - 1];
    if(shootFlag == 5) targetYawPul = 0;
    else if(dartSerial == 1) targetYawPul = furTarYaw[0];
    else    targetYawPul = furTarYaw[dartSerial - 1];
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
    ServoGraspDart();
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
    DartLoad();
    DartRelease();
    tensionControlFlag = 1;
    {
        int prevTen1[WAIT_TIMES], prevTenL[WAIT_TIMES], i = 0;
        while ((tension1 != targetTen[0]) || (tensionL != targetTen[1])
            || (IntArrayComp(prevTen1, targetTen[0], WAIT_TIMES) != WAIT_TIMES)
            || (IntArrayComp(prevTenL, targetTen[1], WAIT_TIMES) != WAIT_TIMES)) {
            stepper0Flag = 1;
            stepper1Flag = 1;
            prevTen1[i] = tension1;
            tension1 = RS485_1_GetTension();
            prevTenL[i] = tensionL;
            tensionL = RS485_2_GetTension();
            i++;
            if(i >= WAIT_TIMES) i = 0;
            /*
            printf("Ten1: %d, Ten2: %d, prevTen1   ", tension1, tensionL);
            for (int j = 0; j < WAIT_TIMES; ++j){
                printf("%d, ", prevTen1[j]);
            }
            printf("///");
            for (int j = 0; j < WAIT_TIMES; ++j){
                printf("%d, ", prevTen1[j]);
            }
            printf("\n");
             */
        }
    }
    tensionControlFlag = 0;
    stepper0Flag = 0;
    stepper1Flag = 0;
    DartShoot();
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
    HAL_Delay(300);
    targetYawPul = -furTarYaw[dartSerial - 1];
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
#if CAN_INFO
    printf("\n1RX the CAN data: %02X%02X%02X%02X%02X%02X%02X%02X, pos = %ld, vel = %ld, cur = %ld  \n", RXmessage1[0], RXmessage1[1], RXmessage1[2],
           RXmessage1[3], RXmessage1[4], RXmessage1[5], RXmessage1[6], RXmessage1[7], pos[0], vel[0], current[0]);
#endif
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
#if CAN_INFO
        printf("2RX the CAN data: %02X%02X%02X%02X%02X%02X%02X%02X, pos = %ld, vel = %ld, cur = %ld  \n", RXmessage2[0], RXmessage2[1], RXmessage2[2],
               RXmessage2[3], RXmessage2[4], RXmessage2[5], RXmessage2[6], RXmessage2[7], pos[1], vel[1], current[1]);
#endif
    }

#if USE_CAN1
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &M3508_H_Rx_3, RXmessage3);
#endif
#if USE_CAN2
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &M3508_H_Rx_3, RXmessage3);
#endif
    if (M3508_H_Rx_3.StdId == 0x203) {
        pos[2] = RXmessage3[0] << 8 | RXmessage3[1];
        vel[2] = RXmessage3[2] << 8 | RXmessage3[3];
        current[2] = RXmessage3[4] << 8 || RXmessage3[5];
#if CAN_INFO
        printf("3RX the CAN data: %02X%02X%02X%02X%02X%02X%02X%02X, pos = %ld, vel = %ld, cur = %ld\r\n", RXmessage3[0],
               RXmessage3[1], RXmessage3[2],
               RXmessage3[3], RXmessage3[3], RXmessage3[5], RXmessage3[6], RXmessage3[7], pos[3], vel[3], current[3]);
#endif
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
#if CAN_INFO
        printf("4RX the CAN data: %02X%02X%02X%02X%02X%02X%02X%02X, pos = %ld, vel = %ld, cur = %ld\r\n", RXmessage4[0],
               RXmessage4[1], RXmessage4[2],
               RXmessage4[3], RXmessage4[4], RXmessage4[5], RXmessage4[6], RXmessage4[7], pos[3], vel[3], current[3]);
#endif
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
    MX_UART5_Init();
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

    HAL_Delay(100);
    //self test
    motor0Flag = 1;
    motor1Flag = 1;
    motor2Flag = 1;
    motor3Flag = 1;
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim10);
    for (int i = 0; i < 4; ++i) {
        tension1 = targetTen[0];
        tensionL = targetTen[1];
        targetVel[i] = -500;
        HAL_Delay(300);
        targetVel[i] = 500;
        HAL_Delay(300);
        targetVel[i] = 0;
        HAL_Delay(500); //need time to stope to stop
    }
    HAL_Delay(1000); //need time to stop
    StepperStart(STEPPER1);
    StepperStart(STEPPER2);
//    StepperStart(STEPPER3);
    StepperStart(STEPPER4);
//    motor0Flag = 0;
//    motor1Flag = 0;
//    motor2Flag = 0;
//    motor3Flag = 0;
    while(1) {
        if(tension1SetZeroFlag){
            RS485_1_SetTensionZero();
            tension1SetZeroFlag = 0;
        }
        if(tension2SetZeroFlag){
            RS485_2_SetTensionZero();
            tension2SetZeroFlag = 0;
        }
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
        if(contFromLastUart >= CONT_TO_READY_TO_SHOOT - SHOOT_BREAK && contFromLastUart <= CONT_TO_READY_TO_SHOOT - SHOOT_BREAK + 10){
            stepper0Flag = 1;
            stepper1Flag = 1;
            targetTen[0] = START_TENSION;
            targetTen[1] = START_TENSION;
            HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
        }
        if(shootFlag == 5){
            ShootOneDart(1);
            HAL_Delay(1000);
            stepper0Flag = 2;
            stepper1Flag = 2;
            StepperSetSpeed(STEPPER1, BALANCE_OFFSET_SPEED);
            StepperSetSpeed(STEPPER2, BALANCE_OFFSET_SPEED);
            HAL_Delay(BALANCE_OFFSET_MS);
            stepper0Flag = 1;
            stepper1Flag = 1;
            targetTen[0] = START_TENSION;
            targetTen[1] = START_TENSION;
            HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
            shootFlag = 0;
        }
        if (canShootFlag && furTarTen[1] != 0 && shootFlag == 1) {
            printf("shootFlag: %d\n", 1);
            ShootFirstDart();
            HAL_Delay(1000);
            stepper0Flag = 2;
            stepper1Flag = 2;
            StepperSetSpeed(STEPPER1, BALANCE_OFFSET_SPEED);
            StepperSetSpeed(STEPPER2, BALANCE_OFFSET_SPEED);
            HAL_Delay(BALANCE_OFFSET_MS);
            stepper0Flag = 1;
            stepper1Flag = 1;
            targetTen[0] = START_TENSION;
            targetTen[1] = START_TENSION;
            HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
            while (tension1 != START_TENSION || tensionL != START_TENSION){
                tension1 = RS485_1_GetTension();
                tensionL = RS485_2_GetTension();
            }
            shootFlag++;
        }
        if (canShootFlag && furTarTen[1] != 0 && shootFlag <= 4 && shootFlag > 1){
            printf("shootFlag: %d\n", shootFlag);
            ShootOneDart(shootFlag);
            HAL_Delay(1000);
            stepper0Flag = 2;
            stepper1Flag = 2;
            StepperSetSpeed(STEPPER1, BALANCE_OFFSET_SPEED);
            StepperSetSpeed(STEPPER2, BALANCE_OFFSET_SPEED);
            HAL_Delay(BALANCE_OFFSET_MS);
            stepper0Flag = 1;
            stepper1Flag = 1;
            targetTen[0] = START_TENSION;
            targetTen[1] = START_TENSION;
            HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
            while (tension1 != START_TENSION || tensionL != START_TENSION){
                tension1 = RS485_1_GetTension();
                tensionL = RS485_2_GetTension();
            }
            if(shootFlag == 4)  {
                shootFlag = 0;
                canShootFlag = 0;
            } else{
                shootFlag++;
            }
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
        IncrementalPI(2, velKp, velKi, vel[2], targetVel[2]);
        IncrementalPI(3, velKp, velKi, vel[3], targetVel[3]);
//        IncrementalPI(4, velKpStepper, velKiStepper, tension1, targetTen[0]);
//        IncrementalPI(5, velKpStepper, velKiStepper, tensionL, targetTen[1]);
        static int lastYawPul = 1;
        DartFeedUpUntilSWDetected();
        DartFeedResetUntilHallDetected();
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
            else if (stepper0Flag == 1 && tension1 <= 900 && tension1 >= -40) {
//            IncrementalPI(4, velKpStepper, velKiStepper, tension1, targetTen[0]);
                StepperStart(STEPPER1);
                static double lastBias;
                if(((double) tension1 - targetTen[0]) <= STEPPER_CHANGE_TO_SMALL_K && ((double) tension1 - targetTen[0]) >= -STEPPER_CHANGE_TO_SMALL_K){
                    if(targetTen[0] < 420) {
                        posKpStepper0 = STEPPER1SMALLKP;
                        posKdStepper0 = STEPPER1SMALLKD;
                    } else if(targetTen[0] < 520) {
                        posKpStepper0 = STEPPER1SMALLSMALLKP;
                        posKdStepper0 = STEPPER1SMALLSMALLKD;
                    } else if(targetTen[0] < 580){
                            posKpStepper0 = STEPPER1SMALLSMALLSMALLKP;
                            posKdStepper0 = STEPPER1SMALLSMALLSMALLKD;
                    } else {
                        posKpStepper0 = STEPPER1SMALLSMALLSMALLSMALLKP;
                        posKdStepper0 = STEPPER1SMALLSMALLSMALLSMALLKD;
                        if((tension1 - targetTen[0]) == 1 || (tension1 - targetTen[0]) == -1){
                            posKpStepper0 = STILL_RATE * posKpStepper0;
                            posKdStepper0 = STILL_RATE * posKdStepper0;
                        }
                    }
                }
                else if(((double) tension1 - targetTen[0]) > 2 * STEPPER_CHANGE_TO_SMALL_K || ((double) tension1 - targetTen[0]) < -2 * STEPPER_CHANGE_TO_SMALL_K) {
                    posKpStepper0 = STEPPER1BIGKP;
                    posKdStepper0 = STEPPER1BIGKD;
                }
                if(STEPPER1_Kp < -STEPPER1_MAX_PUL) {
                    StepperSetSpeed(STEPPER1, -STEPPER1_MAX_PUL);
                    stepper0Speed = -STEPPER1_MAX_PUL;
                }
                else if(STEPPER1_Kp > STEPPER1_MAX_PUL) {
                    StepperSetSpeed(STEPPER1, STEPPER1_MAX_PUL);
                    stepper0Speed = STEPPER1_MAX_PUL;
                }
                else {
                    StepperSetSpeed(STEPPER1, STEPPER1_Kp);
                    stepper0Speed = STEPPER1_Kp;
                }
                lastBias = (double) tension1 - targetTen[0];
//            printf("%lf\n", STEPPER1_Kp);
//                printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d\n", targetYawPul, tension1, tensionL, stepper0Speed, stepper1Speed);
            }
//            else if (tension1 > 900)    StepperSetSpeed(STEPPER1, 0);
            if(stepper1Flag == 0) StepperSetSpeed(STEPPER2, 0);
            else if (stepper1Flag == 1 && tensionL <= 900 && tensionL >= -40) {
                int32_t tensionLL = tensionL;
                static double lastBias;
                if((targetTen[1] - (double) tensionLL) <= STEPPER_CHANGE_TO_SMALL_K && (targetTen[1] - (double) tensionLL) >= -STEPPER_CHANGE_TO_SMALL_K){
                    if(targetTen[1] < 420) {
                        posKpStepper1 = STEPPER2SMALLKP;
                        posKdStepper1 = STEPPER2SMALLKD;
                    } else if(targetTen[1] < 520){
                        posKpStepper1 = STEPPER2SMALLSMALLKP;
                        posKdStepper1 = STEPPER2SMALLSMALLKD;
                    } else if(targetTen[1] < 580){
                        posKpStepper1 = STEPPER2SMALLSMALLSMALLKP;
                        posKdStepper1 = STEPPER2SMALLSMALLSMALLKD;
                    } else{
                        posKpStepper1 = STEPPER2SMALLSMALLSMALLSMALLKP;
                        posKdStepper1 = STEPPER2SMALLSMALLSMALLSMALLKD;
                        if((targetTen[1] - tensionLL) == 1 || (targetTen[1] - tensionLL) == -1){
                            posKpStepper1 = STILL_RATE * posKpStepper0;
                            posKdStepper1 = STILL_RATE * posKdStepper0;
                        }
                    }
                } else if((targetTen[1] - (double) tensionLL) > 2 * STEPPER_CHANGE_TO_SMALL_K || (targetTen[1] - (double) tensionLL) < -2 * STEPPER_CHANGE_TO_SMALL_K){
                    posKpStepper1 = STEPPER2BIGKP;
                    posKdStepper1 = STEPPER2BIGKD;
                }
//            IncrementalPI(5, velKpStepper, velKiStepper, tensionL, targetTen[1]);
                StepperStart(STEPPER2);
                if(STEPPER2_Kp < -STEPPER2_MAX_PUL) {
                    StepperSetSpeed(STEPPER2, -STEPPER2_MAX_PUL);
                    stepper1Speed = -STEPPER2_MAX_PUL;
                }
                else if(STEPPER2_Kp > STEPPER2_MAX_PUL) {
                    StepperSetSpeed(STEPPER2, STEPPER2_MAX_PUL);
                    stepper1Speed = STEPPER2_MAX_PUL;
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
//            else if (tensionL > 900)    StepperSetSpeed(STEPPER2, 0);
            cont = 0;
        }
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
            targetVel[2] = 0;
        }
    }
    if (htim->Instance == htim10.Instance) {     //100ms timer
        static uint16_t pointer, couut;
        couut++;
        static int32_t lastTension1, lastTensionL;
        if(backCont){
            if(backCont <= 1){
                DartFeedStopDown();
                printf("STOP FEED STEPPER DOWN\n");
            }
            backCont--;
        }
        /*
        printf("0: %d, 1: %d\n", abs(targetTen[0] - lastTension1) - abs(targetTen[0] - tension1), abs(targetTen[1] - lastTensionL) - abs(targetTen[1] - tensionL));
        if(abs(targetTen[0] - lastTension1) - abs(targetTen[0] - tension1) < -3 &&
                abs(targetTen[0] - lastTension1) - abs(targetTen[0] - tension1) > -8) {
            HAL_GPIO_TogglePin(STEPPER1_DIR_GPIO_Port, STEPPER1_DIR_Pin);
        }
        if(abs(targetTen[1] - lastTensionL) - abs(targetTen[1] - tensionL) < -3 &&
                abs(targetTen[1] - lastTensionL) - abs(targetTen[1] - tensionL) > -8) {
            HAL_GPIO_TogglePin(STEPPER2_DIR_GPIO_Port, STEPPER2_DIR_Pin);
        }
        if(releaseFlag == 1 && lastTension1 - tension1 >= -1 && lastTension1 - tension1 <= 1 && lastTensionL - tensionL >= -1 && lastTensionL - tensionL <= 1){
            stepper0Flag = 1;
            stepper1Flag = 1;
        }
         */
        lastTension1 = tension1;
        lastTensionL = tensionL;
        if(contFromLastUart > 0)
            contFromLastUart++;
#if SONIC_ENABLE
        //sonic range
        if(couut % 2 == 0) {
            HAL_GPIO_TogglePin(SONIC_RANGE_TRIG1_GPIO_Port, SONIC_RANGE_TRIG1_Pin);
        }
        else{
            HAL_GPIO_TogglePin(SONIC_RANGE_TRIG2_GPIO_Port, SONIC_RANGE_TRIG2_Pin);
        }
        //sonic range
#endif
        if(couut % 10 == 0){
#if MOTOR_INFO
            for (int i = 0; i < 4; ++i){
                printf("pos[%d] = %d, vel[%d] = %d, current[%d] = %d\n", i, pos[i], i, vel[i], i, current[i]);
            }
#endif
#if HALL_INFO
        printf("BACK: %d, LEFT: %d, RIGHT: %d\n", HAL_GPIO_ReadPin(HALL_BACK_SW_GPIO_Port, HALL_BACK_SW_Pin),
               HAL_GPIO_ReadPin(HALL_LEFT_SW_GPIO_Port, HALL_LEFT_SW_Pin),
               HAL_GPIO_ReadPin(HALL_RIGHT_SW_GPIO_Port, HALL_RIGHT_SW_Pin));
#endif
#if TEN_INFO
//            printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d, tarYawPul: %d, furYaw[0]=: %d, sonicRangeUp: %ld, sonicRangeDown: %ld, Kp: %.1lf, %.1lf, Kd: %.1lf, %.1lf, UpClose: %d, DownOpen: %d\n", targetYawPul, tension1, tensionL, stepper0Speed, stepper1Speed, targetYawPul, furTarYaw[0], sonicRangeUp, sonicRangeDown, posKpStepper0, posKpStepper1, posKdStepper0, posKdStepper1, sonicRangeUpCloseFlag, sonicRangeDownOpenFlag);
            printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d, Relay GPIO: %d\n", targetYawPul, tension1, tensionL, stepper0Speed, stepper1Speed,
                   HAL_GPIO_ReadPin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin));
            HAL_UART_Receive_DMA(&huart5, _rx_buf, 18);
            printf("FSM: %d, contFromLastUart: %lld\n", FeedFSMState(), contFromLastUart);
#if UART5_INFO
            for (int i = 0; i < 18; ++i){
                printf("%x, ",_rx_buf[i]);
            }
            printf("\n");
#endif
//            HAL_UART_Receive(&huart5, _rx_buf, 18, 2);
            RemoteControl();
            /*
            int x = 16, y = 96;
            if(couut == 100) {
                ServoSet(SERVO_UP_DOWN, x, 0);
                printf("servo: %d\n", x);
            }
            else if(couut == 200) {
                ServoSet(SERVO_UP_DOWN, y, 0);
                printf("servo: %d\n", y);
                couut = 0;
            }
             */
            if(contFromLastUart > CONT_TO_READY_TO_SHOOT && furTarTen[0] != 0  && shootFlag < 4)
                DartFeedStartUp();
            couut = 0;
/*
            for (int i = 0; i < 500; ++i){
                printf("%c", USART1RxBuf[i]);
            }
            printf("\n");
*/
#endif
#if JUDGE_INFO
            printf("Judge uart: \n");
//            for (int i = 0; USART6RxBuf[i] != '\0'; ++i){
            for (int i = 0; i < RX6_BUFF_LENGTH; ++i){
                printf("%x ", USART6RxBuf[i]);
            }
//            HAL_UART_Receive(&huart5, USART6RxBuf, 50, 1);
//            printf("\nreceive:\n");
//            for (int i = 0; i < RX6_BUFF_LENGTH; ++i){
//                printf("%x", USART6RxBuf[i]);
//            }
#endif
        }
        uint8_t rx6HandleBuf[RX6_BUFF_LENGTH];
        if (ContainsBytesAndCopy(USART6RxBuf, &Rx6Pointer, JudgeUart0001, RX6_BUFF_LENGTH, rx6HandleBuf)) {
            //没开赛时:rx6HandleBuf[] = b 51 0 0 0 0 0 0 0 0 0 0 , while the first 'b' = 11 is the length of rx6HandleBuf
            //rx6HandleBuf[1]:
            // bit 0-3:比赛类型
            //• 1:RoboMaster 机甲大师超级对抗赛
            //• 2:RoboMaster 机甲大师高校单项赛
            //• 3:ICRA RoboMaster 高校人工智能挑战赛
            //• 4:RoboMaster 机甲大师高校联盟赛 3V3 对抗 • 5:RoboMaster 机甲大师高校联盟赛步兵对抗 bit 4-7:当前比赛阶段
            //• 0:未开始比赛
            //• 1:准备阶段
            //• 2:十五秒裁判系统自检阶段
            //• 3:五秒倒计时
            //• 4:比赛中
            //• 5:比赛结算中
            //rx6HandleBuf[2], [3]:当前阶段剩余时间，单位:秒
            game_type = rx6HandleBuf[1] & 0xF;
            game_progress = rx6HandleBuf[1] >> 4;
            stage_remain_time = rx6HandleBuf[2] | (rx6HandleBuf[3] << 8);
#if JUDGE0001_INFO
            printf("0001:\n");
            for (int i = 0; i < rx6HandleBuf[0] + 1; ++i){
                printf("%x ", rx6HandleBuf[i]);
            }
            printf("\ngame_type: %d\n", game_type);
            printf("game_progress: %d\n", game_progress);
            printf("stage_remain_time: %d\n", stage_remain_time);
            printf("\n");
#endif
        }
        if (ContainsBytesAndCopy(USART6RxBuf, &Rx6Pointer, JudgeUart0105, RX6_BUFF_LENGTH, rx6HandleBuf)) {
            //没开赛时:rx6HandleBuf[] = 3 0 0 0, while the first '3' is the length of rx6HandleBuf
            //rx6HandleBuf[1]:  己方飞镖发射剩余时间，单位:秒
            dart_remaining_time = rx6HandleBuf[1];
#if JUDGE0105_INFO
            printf("0105:\n");
            for (int i = 0; i < rx6HandleBuf[0] + 1; ++i){
                printf("%x ", rx6HandleBuf[i]);
            }
            printf("\ndart_remaining_time: %d\n", dart_remaining_time);
            printf("\n");
#endif
        }
        if (ContainsBytesAndCopy(USART6RxBuf, &Rx6Pointer, JudgeUart020A, RX6_BUFF_LENGTH, rx6HandleBuf)) {
            //没开赛时:rx6HandleBuf[] = 6 0 0 0 0 0 0, while the first '6' is the length of rx6HandleBuf
            //rx6HandleBuf[1]:  1:关闭 2:正在开启或者关闭中 0:已经开启
            dart_launch_opening_status = rx6HandleBuf[1];
            //x6HandleBuf[3]: 切换击打目标时的比赛剩余时间，单位:秒，无/未切换动作，默认为 0。
            latest_launch_cmd_time = rx6HandleBuf[3];
#if JUDGE020A_INFO
            printf("020A:\n");
            for (int i = 0; i < rx6HandleBuf[0] + 1; ++i) {
                printf("%x ", rx6HandleBuf[i]);
            }
            printf("\ndart_launch_opening_status: %d\n", dart_launch_opening_status);
            printf("latest_launch_cmd_time: %d\n", latest_launch_cmd_time);
            printf("\n");
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
                HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
                contFromLastUart = 1;
                shootFlag = 0;
                switch (rxHandleBuf[8]) {
                    case '1':
                        furTarTen[0] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten[0]: %d\n", furTarTen[0]);
                        break;
                    case '2':
                        furTarTen[1] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten[1]: %d\n", furTarTen[1]);
                        break;
                    case '3':
                        furTarTen[2] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten[2]: %d\n", furTarTen[2]);
                        break;
                    case '4':
                        furTarTen[3] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten[3]: %d\n", furTarTen[3]);
                        canShootFlag = 1;       //仅测试四发连发时使用
                        shootFlag = 1;         //仅测试四发连发时使用
                }
            } else if (ContainsSubString(rxHandleBuf, TestShoot)) {
                static int cout1;
                cout1++;
                printf("TestShoot %d\n", cout1);
                shootFlag = 5;
                DartFeedStartUp();
                HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
            } else if (ContainsSubString(rxHandleBuf, AbortShoot)) {
                static int cout2;
                cout2++;
                printf("AbortShoot %d\n", cout2);
                canShootFlag = 0;
                shootFlag = 0;
            } else if (ContainsSubString(rxHandleBuf, SetCurYawToZero)) {
                static int cout3;
                cout3++;
                contFromLastUart = 1;
                shootFlag = 0;
                printf("SetCurYawToZero %d\n", cout3);
                targetYawPul = furTarYaw[0];
                furTarYaw[0] = 0;
            } else if (ContainsSubString(rxHandleBuf, ResetFeed)) {
                static int cout4;
                cout4++;
                contFromLastUart = 1;
                shootFlag = 0;
                printf("ResetFeed %d\n", cout4);
//                DartFeedStartDown();
                StepperSetSpeed(STEPPER4, -500);
                StepperStart(STEPPER4);
                resetFeedCont += 40;
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
            } else if (ContainsSubString(rxHandleBuf, SetZeroTension_1)) {
                static int cout8;
                cout8++;
                printf("SetZeroTension(1) %d\n", cout8);
//                RS485_1_SetTensionZero();     //因为被抢占导致无效，使用补丁
                tension1SetZeroFlag = 1;
            } else if (ContainsSubString(rxHandleBuf, SetZeroTension_2)) {
                static int cout9;
                cout9++;
                printf("SetZeroTension(2) %d\n", cout9);
//                RS485_2_SetTensionZero();     //因为被抢占导致无效，使用补丁
                tension2SetZeroFlag = 1;

            }

        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
#if SONIC_ENABLE
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
        if(sonicRangeUp < 120 && lastSonicRangeUp > 140){
            sonicRangeUpCloseFlag = 1;
        }
        if(sonicRangeUp > 140 && contFromLastUart > CONT_TO_READY_TO_SHOOT){
            sonicRangeUpCloseFlag = 0;
        }
        if(shootFlag == 5){
            sonicRangeUpCloseFlag = 0;
            sonicRangeDownOpenFlag = 1;
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
        static uint32_t lastSonicRangeDown = 170;
        if(lastSonicRangeDown < 150 && sonicRangeDown > 150 && contFromLastUart > CONT_TO_READY_TO_SHOOT){
            sonicRangeDownOpenFlag = 1;
        }
        if(sonicRangeDown < 150){
            sonicRangeDownOpenFlag = 0;
        }
        /*
        if(shootFlag == 5){
            sonicRangeUpCloseFlag = 0;
            sonicRangeDownOpenFlag = 1;
        }
         */
        lastSonicRangeDown = sonicRangeDown;
    }
    if(GPIO_Pin == SONIC_RANGE_ECHO2_Pin) {
    }
#endif
//    if(GPIO_Pin == HALL_BACK_SW_Pin){
//
//    }
    if(GPIO_Pin == HALL_LEFT_SW_Pin){
        printf("LEFT_HALL_EXTI\n");
        if(targetVel[1] == RESET_SPEED){
            targetVel[1] = 0;
        } else if(targetVel[1] == -RESET_SPEED){
            targetVel[1] = 0;
        }
        if(releaseFlag) {
            left3508StopCont = 30;
        }
    }
    if(GPIO_Pin == HALL_RIGHT_SW_Pin){
        printf("RIGHT_HALL_EXTI\n");
        if(targetVel[2] == RESET_SPEED){
            targetVel[2] = 0;
//        } else if (targetVel[2] == -RESET_SPEED){
//            targetVel[2] = 0;
        }
        if(releaseFlag) {
            right3508StopCont = 30;
        }
    }
}