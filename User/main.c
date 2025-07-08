#include "stdlib.h"
#include "string.h"
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
double velKp = 100, velKi = 9.4;
double posKpStepper0 = STEPPER1BIGKP, posKiStepper0 = 0.00, posKdStepper0 = STEPPER1BIGKD;
double posKpStepper1 = STEPPER2BIGKP, posKiStepper1 = 0.00, posKdStepper1 = STEPPER2BIGKD;
double velKpStepper = 0, velKiStepper = 10000;

int32_t tension1 = 0;
int32_t tensionL = 0;
int16_t stepper0Speed = 0;
int16_t stepper1Speed = 0;

uint8_t ex_rs4851data[11], ex_rs4852data[11], ex_tension1DataAddress = 0, ex_tension2DataAddress = 0;

int16_t current[4], pos[4], vel[6];
double targetVel[4];
int targetPos[4];
double targetTen[2] = {START_TENSION_R, START_TENSION_L};
int targetYawPul = 0, UART_TargetYawPul = 0, UART_TargetYawSpeed = 0;
float yaw_error;
uint8_t target_status;
float lastYawError, integralYawError;
int aimbot_mode = 1; // 0: 不开自瞄, 不录像 1: 开自瞄且录像 2: 录像
int tensionControlled = 0;

int furTarTen[5] = {TARGET_TEN};
int furTarYaw[5] = {TARGET_YAW};

int shootFlag = 0;
int shooting = 0;

CAN_RxHeaderTypeDef M3508_H_Rx_1;
CAN_RxHeaderTypeDef M3508_H_Rx_2;
CAN_RxHeaderTypeDef M3508_H_Rx_3;
CAN_RxHeaderTypeDef M3508_H_Rx_4;
CAN_RxHeaderTypeDef Joint_Motor1234;
CAN_RxHeaderTypeDef Joint_Motor5678;
uint8_t RXmessage1[8];
uint8_t RXmessage2[8];
uint8_t RXmessage3[8];
uint8_t RXmessage4[8];

uint16_t ADC1Value[100];
uint32_t adc1in4, adc1in5, adc1in14, adc1in15;

int motor0Flag = 0, motor1Flag = 0, motor2Flag = 0, motor3Flag = 0, stepper0Flag = 0, stepper1Flag = 0, triggerResetFlag = 0;
int stepper0WaitingToStopFlag = 0, stepper1WaitingToStopFlag = 0;
uint8_t USART1RxBuf[RX_BUFF_LENGTH], USART6RxBuf[RX6_BUFF_LENGTH];

uint32_t sonicRangeUp = 200, sonicRangeDown = 180;     //ms * 34 (cm/ms)

int tensionControlFlag = 0;

int tension1SetZeroFlag = 0, tension2SetZeroFlag = 0;

int16_t RxPointer = 0, Rx6Pointer = 0;
extern int backCont;
extern uint8_t _rx_buf[AIMBOT_RX_BUF_LEN];

int servoTriggerCont = 0;

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
enum SILO_GATE lastDart_launch_opening_status;
uint16_t target_change_time = 0;
uint16_t latest_launch_cmd_time = 0;
uint16_t dart_remaining_time = 0;
uint16_t dart_target = 0;

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
int ContainsBytesAndCopy(uint8_t *buf, int16_t *startPointer, uint8_t *bytesToFind, uint16_t length, uint8_t *copyBuf, int bagLen){
    if(*startPointer >= RX6_BUFF_LENGTH)
        *startPointer = 0;
    for (int i = *startPointer; i < length && *startPointer <= RX6_BUFF_LENGTH; ++i){
        if(buf[i] == bytesToFind[0] && buf[i + 5] == bytesToFind[5] && buf[i + 6] == bytesToFind[6] && buf[i + 1] == bagLen) {
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
    HAL_TIM_PWM_Init(&htim8);
    HAL_TIM_PWM_Init(&htim12);
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

    Joint_Motor1234.StdId = 0x100;
    Joint_Motor1234.ExtId = 0x0;
    Joint_Motor1234.IDE = CAN_ID_STD;
    Joint_Motor1234.RTR = CAN_RTR_DATA;
    Joint_Motor1234.FilterMatchIndex = 0;
    Joint_Motor1234.Timestamp = 0;

    Joint_Motor5678.StdId = 0x200;
    Joint_Motor5678.ExtId = 0x0;
    Joint_Motor5678.IDE = CAN_ID_STD;
    Joint_Motor5678.RTR = CAN_RTR_DATA;
    Joint_Motor5678.FilterMatchIndex = 0;
    Joint_Motor5678.Timestamp = 0;
    ServoInit();


    StepperInit(STEPPER1, 1680 - 1);
    StepperInit(STEPPER2, 1680 - 1);
    StepperInit(STEPPER3, 1680 - 1);
#if OLD_FEED
    StepperInit(STEPPER4, 1680 - 1);
#endif
    ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_INIT, 10);
    ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_INIT, 10);
    ServoSet(SERVO_GRASP, SERVO_GRASP_INIT, 10);
    ServoSet(SERVO_TRIGGER, SERVO_TRIGGER_INIT, 10);
    ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_INIT, 10);
#if OLD_FEED
    ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, 10);
    ServoSet(SERVO_UP_DOWN, SERVO_UP_DOWN_UP, 1);                      //Start up
#endif
    ServoSet(4, 500, 10);
    Double_PID_Init();

    StepperSetSpeed(STEPPER1, 0);
    StepperSetSpeed(STEPPER2, 0);


    StepperSetSpeed(STEPPER3, 0);
#if OLD_FEED
    StepperSetSpeed(STEPPER4, 0);
#endif
    HAL_Delay(100);

//    StepperTensionControlStart(1);
//    HAL_TIM_Base_Start_IT(&htim7);
    for (int i = 0; i < 20; ++i){
        tension1 = RS485_1_GetTension();
        tension1 = RS485_1_GetTension();
        tensionL = RS485_2_GetTension();
    }
//    HAL_DMA_Init(&hdma2);
//    HAL_UART_Receive_IT(&huart1, USART1RxBuf, RX_BUFF_LENGTH);
    HAL_UART_Receive_DMA(&huart1, USART1RxBuf, RX_BUFF_LENGTH);
    HAL_UART_Receive_DMA(&huart6, USART6RxBuf, RX6_BUFF_LENGTH); //judge system uart
//    RemoteInit();

#if USE_RELAY_CONTROL
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
#endif

}

void ShootFirstDart(){
#if USE_RELAY_CONTROL
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
#endif
    stepper0Flag = 0;
    stepper1Flag = 0;
    targetTen[0] = furTarTen[0];
    targetTen[1] = furTarTen[0];
    targetYawPul = furTarYaw[0];
    DartReset();
#if USE_RELAY_CONTROL
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
#endif
    DartLoad(LOAD_SPEED, 0);
    DartRelease(0);
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
            tension1 = RS485_1_GetTension();
            prevTenL[i] = tensionL;
            tensionL = RS485_2_GetTension();
            if(tension1 == 0){
//                HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
//                HAL_Delay(500);
//                printf("Tension error:RELAY_REOPEN\n");
//                HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
//                HAL_Delay(1000);
            }
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
    DartShoot(0);
#if USE_RELAY_CONTROL
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
#endif
    HAL_Delay(500);
}
void ShootOneDart(int dartSerial) {
    START_SPEAK_1;
#if USE_RELAY_CONTROL
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
#endif
    stepper0Flag = 0;
    stepper1Flag = 0;
    targetTen[0] = furTarTen[dartSerial - 1];
    targetTen[1] = furTarTen[dartSerial - 1];
    if(dartSerial == 1) targetYawPul = furTarYaw[0];
    else    targetYawPul = furTarYaw[dartSerial - 1];
#if USE_RELAY_CONTROL
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
#endif
    DartLoad(LOAD_SPEED, dartSerial);
    DartRelease(dartSerial);
    tensionControlFlag = 1;
    {
        int prevTen1[WAIT_TIMES], prevTenL[WAIT_TIMES], i = 0;
#if USE_game_progress_AND_stage_remain_time
        while ((tension1 != targetTen[0]) || (tensionL != targetTen[1])
               || (IntArrayComp(prevTen1, targetTen[0], WAIT_TIMES) != WAIT_TIMES)
               || (IntArrayComp(prevTenL, targetTen[1], WAIT_TIMES) != WAIT_TIMES)
               || stage_remain_time >= START_SHOOT_TIME1
               || ((stage_remain_time <= STOP_SHOOT_TIME1) && (stage_remain_time >= START_SHOOT_TIME2))
               || stage_remain_time <= STOP_SHOOT_TIME2) {
#elif USE_dart_remaining_time

#if USE_dart_launch_opening_status
            while ((tension1 != targetTen[0]) || (tensionL != targetTen[1])
               || (IntArrayComp(prevTen1, targetTen[0], WAIT_TIMES) != WAIT_TIMES)
               || (IntArrayComp(prevTenL, targetTen[1], WAIT_TIMES) != WAIT_TIMES)
               || ((dart_remaining_time == 0
               || dart_launch_opening_status != OPEN
               || game_progress != IN_GAME
               || dart_target == 0) && shootFlag != 5)){
#else
        while ((tension1 != targetTen[0]) || (tensionL != targetTen[1])
               || (IntArrayComp(prevTen1, targetTen[0], WAIT_TIMES) != WAIT_TIMES)
               || (IntArrayComp(prevTenL, targetTen[1], WAIT_TIMES) != WAIT_TIMES)
               || ((dart_remaining_time == 0
                    || game_progress != IN_GAME
                    || dart_target == 0) && shootFlag != 5)){
#endif
//               || (target_status && (yaw_error - (float)targetYawPul) >= SHOOT_YAW_THRESOLD || (yaw_error - (float)targetYawPul) <= -SHOOT_YAW_THRESOLD)) {

#if SHOOT_INFO
//                printf("循环继续原因：");
//                if (tension1 != targetTen[0])           printf(" tension1未达目标");
//                if (tensionL != targetTen[1])           printf(" tensionL未达目标");
//                if (IntArrayComp(prevTen1, targetTen[0], WAIT_TIMES) != WAIT_TIMES)
//                    printf(" prevTen1历史不一致");
//                if (IntArrayComp(prevTenL, targetTen[1], WAIT_TIMES) != WAIT_TIMES)
//                    printf(" prevTenL历史不一致");
//                if ((dart_remaining_time == 0
//                     || dart_launch_opening_status != OPEN
//                     || game_progress != IN_GAME
//                     || dart_target == 0) && shootFlag != 5)
//                    printf(" 发射条件未就绪");
//                if ((yaw_error - (float)targetYawPul) >= SHOOT_YAW_THRESOLD ||
//                    (yaw_error - (float)targetYawPul) <= -SHOOT_YAW_THRESOLD)
//                    printf(" 偏航误差超阈值");
//                printf("\n");
#endif
#else
        while ((tension1 != targetTen[0]) || (tensionL != targetTen[1])
            || (IntArrayComp(prevTen1, targetTen[0], WAIT_TIMES) != WAIT_TIMES)
            || (IntArrayComp(prevTenL, targetTen[1], WAIT_TIMES) != WAIT_TIMES)) {
#endif
//                printf("yaw_error = %f, targetYawPul = %d, shootFlag = %d\n", yaw_error, targetYawPul, shootFlag);
            shooting = 0;
            if(tension1 <= TENSION_LOW_ERROR_THRESOLD && tensionL <= TENSION_LOW_ERROR_THRESOLD){
                printf("TENISON TOO LOW!!!!!!!!!!!!!!! R:%d, L:%d\n\n\n", tension1, tensionL);
                stepper0Flag = 0;
                stepper1Flag = 0;
                prevTen1[i] = tension1;
                tension1 = RS485_1_GetTension();
                tension1 = RS485_1_GetTension();
                prevTenL[i] = tensionL;
                tensionL = RS485_2_GetTension();
                continue;
            }
            stepper0Flag = 1;
            stepper1Flag = 1;
            prevTen1[i] = tension1;
            tension1 = RS485_1_GetTension();
            prevTenL[i] = tensionL;
            tensionL = RS485_2_GetTension();
#if TEN_ERROR_INFO
            static int32_t printTick = 0, tenErrorCont = 0;
                if((tension1 == 0 || tensionL == 0) && printTick <= HAL_GetTick() - TEN_ERROR_INFO_DELAY_MS){
                    if(tension1 == 0)   stepper0Flag = 0;
                    else stepper0Flag = 1;
                    if(tensionL == 0)   stepper1Flag = 0;
                    else stepper1Flag = 1;
                    printf("tension ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! R:%d, L:%d\n\n\n", tension1, tensionL);
                    printTick = HAL_GetTick();
                    tenErrorCont ++;
                    if(tenErrorCont > TEN_ERROR_SHOOT_CONT_THRESOLD && (dart_remaining_time > 0
                                                                     && dart_launch_opening_status == OPEN
                                                                     && game_progress == IN_GAME
                                                                     && dart_target != 0)){
                        tenErrorCont = 0;
                        break;
                    }
                }
#endif
                tenErrorCont == 0;
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
    STOP_SPEAK_1;
    shooting = 1;
    tensionControlFlag = 0;
    stepper0Flag = 0;
    stepper1Flag = 0;
    StepperSetSpeed(STEPPER1, 0);
    StepperSetSpeed(STEPPER2, 0);
    START_SPEAK_2;
    DartShoot(dartSerial);
#if USE_RELAY_CONTROL
    HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
#endif
    HAL_Delay(500);
    STOP_SPEAK_2;
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
    MX_TIM9_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_I2C1_Init();
    MX_TIM6_Init();
    MX_TIM8_Init();
    MX_CAN2_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_UART5_Init();
    MX_TIM12_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_TIM10_Init();
    MX_UART4_Init();
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
#if !STEPPER_VOFA
    printf("hello\n");
#endif
    UserInit();

#if !STEPPER_PARAS_TEST
    HAL_Delay(100);
    //self test
    motor0Flag = 1;
    motor1Flag = 1;
    motor2Flag = 1;
    motor3Flag = 1;
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim10);
    START_SPEAK_2;
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
    STOP_SPEAK_2;
    HAL_Delay(1000); //need time to stop
#endif
    StepperStart(STEPPER1);
    StepperStart(STEPPER2);
    StepperStart(STEPPER3);
#if OLD_FEED
    StepperStart(STEPPER4);
#endif
//    motor0Flag = 0;
//    motor1Flag = 0;
//    motor2Flag = 0;
//    motor3Flag = 0;
    while(1) {

#if TENSION_CONTROL_WHEN_CONNECT_TO_SERVER
    if(game_progress == 1 && stepper0Flag == 0 && stepper1Flag == 0 && tensionControlled == 0) {
        canShootFlag = 0;
        shootFlag = 0;
        targetTen[0] = SERVER_TENSION_R;
        targetTen[1] = SERVER_TENSION_L;
#if USE_RELAY_CONTROL
        HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
#endif
        stepper1Flag = 1;
        stepper0Flag = 1;
#endif
    }

#if RESET_BY_REMAIN_TIME
        if(stage_remain_time == RESET_BY_REMAIN_TIME) {HAL_NVIC_SystemReset();}
#endif

#if USE_game_progress_AND_stage_remain_time
      if(game_progress == 4 && stage_remain_time <= START_LOAD_TIME1 && stage_remain_time >= STOP_SHOOT_TIME1 && shootFlag <= 2){
          canShootFlag = 1;
          if(shootFlag == 0)    shootFlag = 1;
      }
      else if(game_progress == 4 && stage_remain_time <= START_LOAD_TIME2 && stage_remain_time >= STOP_SHOOT_TIME2){
          canShootFlag = 1;
      }
      else{
          canShootFlag = 0;
      }
#elif USE_dart_remaining_time

#if USE_dart_launch_opening_status
      if(canShootFlag == 0 && game_progress == IN_GAME && dart_target != 0 && (dart_remaining_time >= LEAST_SHOOT_TIME || (lastDart_launch_opening_status == 1 && dart_launch_opening_status == 2))){
#else
      if(canShootFlag == 0 && game_progress == IN_GAME && dart_target != 0 && (dart_remaining_time >= LEAST_SHOOT_TIME || (lastDart_launch_opening_status == 1 && dart_launch_opening_status == 2))){
#endif
          canShootFlag = 1;
          aimbot_mode = 1;
          printf("set canShootFlag = %d!!!!!!!!\n\n", canShootFlag);
          if(shootFlag == 0) shootFlag = 1;
      }
#if USE_dart_launch_opening_status
      if(canShootFlag == 1 && ((dart_remaining_time < LEAST_SHOOT_TIME && dart_launch_opening_status == 0) || (lastDart_launch_opening_status == 0 && dart_launch_opening_status == 2))){
#else
        if(canShootFlag == 1 && dart_remaining_time < LEAST_SHOOT_TIME){
#endif
          canShootFlag = 0;
#if AUTO_OPEN_AIMBOT
          aimbot_mode = 0;              //关闭自瞄
#endif
          printf("set canShootFlag = %d!!!!!!!!\n\n", canShootFlag);
      }
      if(lastDart_launch_opening_status != dart_launch_opening_status)  lastDart_launch_opening_status = dart_launch_opening_status;
#endif
      static int startFlag;
      if(HAL_GetTick() >= FIRST_TENSION_START_TIME_MS && startFlag == 0){
          printf("Tick = %ld\n", HAL_GetTick());
#if USE_RELAY_CONTROL
          HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
#endif
#if AUTO_OPEN_AIMBOT
          aimbot_mode = 0; //关自瞄
#endif
          tension1 = 0;
          tensionL = 0;
          startFlag = 1;
      }
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
        static int32_t lastTension1, lastTensionL;
        if(stepper0Flag == 1 && tensionL == targetTen[1]){
#if USE_RELAY_CONTROL
            HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
#endif
            stepper0Flag = 0;
            stepper1Flag = 0;
            tensionControlled = 1;
        }
        if(contFromLastUart >= CONT_TO_READY_TO_SHOOT - SHOOT_BREAK && contFromLastUart <= CONT_TO_READY_TO_SHOOT - SHOOT_BREAK + 10){
//            stepper0Flag = 1;
//            stepper1Flag = 1;
//            targetTen[0] = START_TENSION_R;
//            targetTen[1] = START_TENSION_L;
//            HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
        }
        if(shootFlag == 5){
            ShootOneDart(5);
            HAL_Delay(1000);
            shootFlag = 0;
        }
        if (canShootFlag && shootFlag <= 4 && shootFlag >= 1 && furTarTen[shootFlag - 1] != 0){
            printf("shootFlag: %d\n", shootFlag);
            ShootOneDart(shootFlag);
//            HAL_Delay(1000);
            if(shootFlag == TOTAL_DART_NUM)  {
                if(game_progress != IN_GAME && game_type != 1)  shootFlag = 0;
                else    shootFlag = -1;                     //防止最后再发一发
                canShootFlag = 0;
                DartFeedStartDown();
#if OLD_FEED
                StepperStart(STEPPER4);
#endif
                resetFeedCont += 128;
                HAL_Delay(13000);
#if !USE_game_progress_AND_stage_remain_time
//                HAL_NVIC_SystemReset();          	/* 重启 */
#endif
            } else{
                shootFlag++;
            }
        }
        if (canShootFlag && furTarTen[0] != 0 && shootFlag == 1) {
            printf("shootFlag: %d\n", 1);
//            ShootFirstDart();               //这里会让测力离线
            ShootOneDart(shootFlag);
            shootFlag++;
//            HAL_Delay(1000);
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


double stepper_Kp0[90] = {
        // 0-160线性保持（0-160实际为430）
        430,430,430,430,430,430,430,430,430,430,  // 5-95
        430,430,330,230,180,140,130,              // 105-155

        // 160-170突变区
        150,                                       // 165

        // 170-420线性下降（170→120）
        150,110,140,158,166,150,146,142,138,134,  // 175-265
        130,126,122,118,114,110,106,102,54,50,    // 275-365
        48,45,42,39,                              // 375-405

        // 420-520二次曲线下降（80→40）
        60,56,46,45,44,39,37, 36,32, 30,            // 425-465
        30,27,26,25,24,23,27,27,26,25,            // 475-515

        // 520-580指数衰减（40→24）
        29,28,27,29,28,27,26,25,24,23,            // 525-575

        // 580-700对数保持（24）
        19.5,19,18.5,18,17.5,17,16.5,16,15.5,15,            // 585-675
        14.6,14.2,                                     // 685-695

        // 700-900反比例衰减（12→12）
        13.9,13.6,13.4,13.2,13,12.7,12,12,12,12,            // 705-795
        12,12,12,12,12,12,12,12,12,12             // 805-895
};
double stepper_Kp1[90] = {
        // 0-160线性保持（0-160实际为430）
        430,430,430,430,430,430,430,430,430,430,  // 5-95
        430,430,330,230,180,140,130,              // 105-155

        // 160-170突变区
        150,                                       // 165

        // 170-420线性下降（170→120）
        150,110,140,158,166,150,146,142,138,134,  // 175-265
        130,126,122,118,114,110,106,102, 65,54,    // 275-365
        51,48,46,43,                              // 375-405

        // 420-520二次曲线下降（80→40）
        60,56,46,45,44,39,37, 36,35, 34,            // 425-465
        33,27,26,25,24,23,27,27,26,25,            // 475-515

        // 520-580指数衰减（40→24）
        29,28,27,29,28,27,26,25,24,23,            // 525-575

        // 580-700对数保持（24）
        19.5,19,18.5,18,17.5,17,16.5,16,15.5,15,            // 585-675
        14.6,14.2,                                     // 685-695

        // 700-900反比例衰减（12→12）
        13.9,13.6,13.4,13.2,13,12.7,12,12,12,12,            // 705-795
        12,12,12,12,12,12,12,12,12,12             // 805-895
};

double stepper_Ki[90] = {
        // 0-160线性保持（0-160实际为430）
        430,430,430,430,430,430,430,430,430,430,  // 5-95
        430,430,330,230,200,180,170,              // 105-155

        // 160-170突变区
        170,                                       // 165

        // 170-420线性下降（170→120）
        150,150,140,158,154,140,136,132,128,124,  // 175-265
        120,116,112,108,104,101,98,95,92,84,    // 275-365
        70,66,62,58,                              // 375-405

        // 420-520二次曲线下降（80→40）
        54,50,46,43,42,41,40,39,38, 35,            // 425-465
        33,32,32,30,29,28,28,27,31,30,            // 475-515

        // 520-580指数衰减（40→24）
        29,28,27,29,28,27,26,25,24,23,            // 525-575

        // 580-700对数保持（24）
        19.5,19,18.5,18,17.5,17,16.5,16,15.5,15,            // 585-675
        14.6,14.2,                                     // 685-695

        // 700-900反比例衰减（12→12）
        13.9,13.6,13.4,13.2,13,12.7,12,12,12,12,            // 705-795
        12,12,12,12,12,12,12,12,12,12             // 805-895
};

double stepper_Kd[90] = {
        // 0-160恒定高阻尼
        150,150,150,150,150,150,150,150,150,150,
        150,150,150,150,150,150,150,

        // 160-170快速降阻尼
        90,

        // 170-420线性缓降（90→80）
        90,89,88,87,86,85,84,83,82,81,
        80,79,78,77,76,75,74,73,72,71,
        70,69,68,67,

        // 420-520阻尼回升（80→90）
        66,67,90,90,90,90,90,90,90,90,
        89,88,87,86,85,84,83,82,81,84,

        // 520-580过阻尼抑制（90→70）
        84,82,80,78,76,74,72,70,70,70,

        // 580-700适度阻尼
        70,70,70,70,70,70,70,70,70,70,
        70,70,

        // 700-900低阻尼
        70,65,60,55,50,50,50,50,50,50,
        50,50,50,50,50,50,50,50,50,50
};
#include <stdint.h>
#include <stddef.h>

typedef struct CarData {
    uint8_t header;
    uint8_t mode; // 0: 不开自瞄, 不录像 1: 开自瞄且录像 2: 录像
    uint8_t CRC8;

    uint8_t status;
    uint8_t number; // 第number号飞镖
    uint8_t dune;   // 0：完全展开 1：完全关闭 2：正在进行时
    uint16_t CRC16;
} CarData;


// CRC8计算函数（多项式0x07）
uint8_t computeCRC8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00; // 初始值
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07; // 多项式0x07
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// CRC16计算函数（CRC-CCITT，多项式0x1021）
uint16_t computeCRC16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF; // 初始值
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8; // 处理高位在前
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; // 多项式0x1021
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void PrintCarData(CarData *data) {
    uint8_t *p = (uint8_t*)data;
    for (int i = 0; i < sizeof(CarData); i++) {
        printf("%02X ", p[i]);
    }
    printf("\n");
}

void CarDataHandle(CarData *cardata, uint8_t number, uint8_t dune) {
    cardata->header = 0xA5;
    cardata->mode = aimbot_mode;

    // 计算CRC8（覆盖header和mode）
    uint8_t crc8_data[2] = {cardata->header, cardata->mode};
    cardata->CRC8 = computeCRC8(crc8_data, 2);

    cardata->status = 0x00;
    cardata->number = number;
    cardata->dune = dune;

    // 计算CRC16（覆盖status、number、dune）
    uint8_t crc16_data[3] = {cardata->status, cardata->number, cardata->dune};
    cardata->CRC16 = computeCRC16(crc16_data, 3);
}

void AimbotSendData(CarData *carData,uint8_t len)
{
    static uint8_t* dataAddr=0,i=0;
    for(i=0;i<len;i++) //使用sizeof计算结构体
    {
        dataAddr = (((uint8_t *)&carData->header)+i); //从帧头开始 然后依次向下指向
        HAL_UART_Transmit(&huart5,dataAddr,1,20);	//发送接收到的数据
        while(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_TC)!=SET);		//等待发送结束
    }
}


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
        if(!triggerResetFlag)  IncrementalPI(0, velKp, velKi, vel[0], targetVel[0]);
        IncrementalPI(1, velKp, velKi, vel[1], targetVel[1]);
        IncrementalPI(2, velKp, velKi, vel[2], targetVel[2]);
        IncrementalPI(3, velKp, velKi, vel[3], targetVel[3]);
//        IncrementalPI(4, velKpStepper, velKiStepper, tension1, targetTen[0]);
//        IncrementalPI(5, velKpStepper, velKiStepper, tensionL, targetTen[1]);
        static int lastYawPul = 1;
        DartFeedUpUntilSWDetected();
        DartFeedResetUntilHallDetected();

        //软件舵机PWM
//        static int16_t softServoPWMCont;
//        if(softServoPWMCont < SOFTWARE_SERVO_PERIOD)  softServoPWMCont++;
//        else    softServoPWMCont = 0;
//        if(softServoPWMCont == 0){
//            HAL_GPIO_WritePin(SERVO5_GPIO_Port, SERVO5_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(SERVO6_GPIO_Port, SERVO6_Pin, GPIO_PIN_RESET);
//        } else if(softServoPWMCont == servo5comp){
//            HAL_GPIO_WritePin(SERVO5_GPIO_Port, SERVO5_Pin, GPIO_PIN_SET);
//            printf("SERVO5 SET : %d\n", servo5comp);
//        }

/*
        if (UART_TargetYawPul * lastYawPul < 0)
            HAL_GPIO_TogglePin(STEPPER3_DIR_GPIO_Port, STEPPER3_DIR_Pin);
        if (UART_TargetYawPul > 0) {
            HAL_GPIO_TogglePin(YAW_STEPPER_PUL_GPIO_Port, YAW_STEPPER_PUL_Pin);
//            printf("+\n");
            lastYawPul = UART_TargetYawPul;
            UART_TargetYawPul--;
        } else if (UART_TargetYawPul < 0) {
            HAL_GPIO_TogglePin(YAW_STEPPER_PUL_GPIO_Port, YAW_STEPPER_PUL_Pin);
//            printf("-\n");
            lastYawPul = UART_TargetYawPul;
            UART_TargetYawPul++;
        }
        */
//
//        static int lastYawSpeed, yawSpeedCont;
//        if(UART_TargetYawSpeed > -1000 && UART_TargetYawSpeed < 1000)   yawSpeedCont ++;
//        if(UART_TargetYawSpeed * lastYawSpeed < 0){
//            if(UART_TargetYawSpeed > 0) HAL_GPIO_WritePin(STEPPER3_DIR_GPIO_Port, STEPPER3_DIR_Pin, GPIO_PIN_SET);
//            else    if(UART_TargetYawSpeed < 0) HAL_GPIO_WritePin(STEPPER3_DIR_GPIO_Port, STEPPER3_DIR_Pin, GPIO_PIN_SET);
//        }
//        if()
//        if(yawSpeedCont >= 100){
//            HAL_GPIO_TogglePin(YAW_STEPPER_PUL_GPIO_Port, YAW_STEPPER_PUL_Pin);
//        }
//        lastYawSpeed = UART_TargetYawSpeed;
if(aimbot_mode) {
//        static int16_t aimbotDelayCont = 0;
    static int32_t aim_lost_cont;
#if AIMBOT_DEBUG
    if(target_status && shooting == 0){                  //如果检测到绿灯且不在发射中
#else
    if (target_status && shooting == 0 && shootFlag != 0) {                  //如果检测到绿灯且不在发射中
#endif
        if ((yaw_error - targetYawPul) <= INTEGRAL_YAW_START_BIAS &&
            (yaw_error - targetYawPul) >= -INTEGRAL_YAW_START_BIAS)
            integralYawError += yaw_error - targetYawPul;
        if (integralYawError > INTEGRAL_YAW_BIAS_SUB) integralYawError -= INTEGRAL_YAW_BIAS_SUB;
        else if (integralYawError < -INTEGRAL_YAW_BIAS_SUB) integralYawError += INTEGRAL_YAW_BIAS_SUB;
        if (integralYawError <= -INTEGRAL_YAW_MAX) integralYawError = -INTEGRAL_YAW_MAX;
        if (integralYawError >= INTEGRAL_YAW_MAX) integralYawError = INTEGRAL_YAW_MAX;
        if ((yaw_error - targetYawPul) >= -INTEGRAL_YAW_SET_ZERO &&
            (yaw_error - targetYawPul) <= INTEGRAL_YAW_SET_ZERO)
            integralYawError = 0;
        if ((yaw_error - targetYawPul) <= AIMBOT_SET_ZERO && (yaw_error - targetYawPul) >= -AIMBOT_SET_ZERO)
            StepperStop(STEPPER3);
        else {
            if (AIMBOT_PID >= AIMBOT_MAX_SPEED) UART_TargetYawSpeed = AIMBOT_MAX_SPEED;
            else if (AIMBOT_PID <= -AIMBOT_MAX_SPEED) UART_TargetYawSpeed = -AIMBOT_MAX_SPEED;
            else UART_TargetYawSpeed = AIMBOT_PID;
            StepperStart(STEPPER3);
        }
//            StepperStart(STEPPER3);
//            if(aimbotDelayCont < AIMBOT_CONTROL_DELAY)  aimbotDelayCont ++;
//            else{
//                aimbotDelayCont = 0;
//                UART_TargetYawPul = yaw_error / 2;
//            UART_TargetYawSpeed = yaw_error / 5;
//                printf("UART_TargetYawSpeed = %d\n", UART_TargetYawSpeed);
//            }
        StepperSetSpeed(STEPPER3, (int16_t) UART_TargetYawSpeed);
        aim_lost_cont = 0;
    } else if (target_status == 0) {
        aim_lost_cont++;
//            printf("aim lost cont = %d\n", aim_lost_cont);
        if (aim_lost_cont >= AIMBOT_SET_STEPPER3_ZERO_THRESOLD) {
            aim_lost_cont = 0;
            StepperSetSpeed(STEPPER3, 0);
        }
    }
#if AIMBOT_DEBUG
    //        if(canShootFlag == 0 && shootFlag != 0) StepperStop(STEPPER3);
#endif
    lastYawError = yaw_error - targetYawPul;
}
        //手动调yaw
#if MANUAL_YAW
        if(shootFlag == 0 && targetYawPul != 0){
            if(targetYawPul > 0)    {
                StepperSetSpeed(STEPPER3, 100);
                StepperStart(STEPPER3);
                targetYawPul--;
            }
            if(targetYawPul < 0)    {
                StepperSetSpeed(STEPPER3, -100);
                StepperStart(STEPPER3);
                targetYawPul++;
            }
            if (targetYawPul == 0) {
                StepperStop(STEPPER3);
            }
        }
#endif
            if (cont == 10) {                           //10ms
            static double integralBias[2];
                //AIMBOT UART
                HAL_UART_Receive_DMA(&huart5, _rx_buf, AIMBOT_RX_BUF_LEN);
                if(aimbot_mode) {
                    CarData *car_data;
                    CarDataHandle(&car_data, 1, 1);
                    AimbotSendData(&car_data, 8);
                }
            if(stepper0Flag == 0) StepperSetSpeed(STEPPER1, 0);
            else if (stepper0Flag == 1 && tension1 <= TENSION_PROTECT_HIGH && tension1 >= TENSION_PROTECT_LOW) {
//            IncrementalPI(4, velKpStepper, velKiStepper, tension1, targetTen[0]);
                StepperStart(STEPPER1);
                static double lastBias;

                if(tension1 - targetTen[0]<= INTEGRAL_START_BIAS && tension1 - targetTen[0] >= -INTEGRAL_START_BIAS)
                    integralBias[0] += tension1 - targetTen[0] - INTEGRAL_BIAS_SUB;
                else    integralBias[0] -= INTEGRAL_BIAS_SUB;
                if(integralBias[0] <= -INTEGRAL_MAX)    integralBias[0] = -INTEGRAL_MAX;
                if(integralBias[0] >= INTEGRAL_MAX) integralBias[0] = INTEGRAL_MAX;
                if(integralBias[0] >= -INTEGRAL_SET_ZERO && integralBias[0] <= INTEGRAL_SET_ZERO) integralBias[0] = 0;

                if(((double) tension1 - targetTen[0]) <= STEPPER_CHANGE_TO_SMALL_K && ((double) tension1 - targetTen[0]) >= -STEPPER_CHANGE_TO_SMALL_K){
                    posKpStepper0 = stepper_Kp0[(int)targetTen[0] / 10];
                    posKiStepper0 = stepper_Kp0[(int)targetTen[0] / 10] / KI_DIVIDE;
                    posKdStepper0 = stepper_Kd[(int)targetTen[0] / 10];
                    if(targetTen[0] >= STEPPER_NOR_SQ_TEN_THRESOLD){
                        if((tension1 - targetTen[0]) == 1 || (tension1 - targetTen[0]) == -1){
                            posKpStepper0 = STILL_RATE * posKpStepper0;
                            posKdStepper0 = STILL_RATE * posKdStepper0;
                        }
                    }
                }
                else if(((double) tension1 - targetTen[0]) > 3 * STEPPER_CHANGE_TO_SMALL_K || ((double) tension1 - targetTen[0]) < -3 * STEPPER_CHANGE_TO_SMALL_K) {
                    posKpStepper0 = STEPPER1BIGKP;
                    posKdStepper0 = STEPPER1BIGKD;
                }
                if(targetTen[0] < STEPPER_NOR_SQ_TEN_THRESOLD) {
                    if (STEPPER1_Kp < -STEPPER1_MAX_PUL) {
                        StepperSetSpeed(STEPPER1, -STEPPER1_MAX_PUL);
                        stepper0Speed = -STEPPER1_MAX_PUL;
                    } else if (STEPPER1_Kp > STEPPER1_MAX_PUL) {
                        StepperSetSpeed(STEPPER1, STEPPER1_MAX_PUL);
                        stepper0Speed = STEPPER1_MAX_PUL;
                    } else {
                        StepperSetSpeed(STEPPER1, STEPPER1_Kp);
                        stepper0Speed = STEPPER1_Kp;
                    }
                } else {
                    if (STEPPER1_Kp_SQ < -STEPPER1_MAX_PUL) {
                        StepperSetSpeed(STEPPER1, -STEPPER1_MAX_PUL);
                        stepper0Speed = -STEPPER1_MAX_PUL;
                    } else if (STEPPER1_Kp_SQ > STEPPER1_MAX_PUL) {
                        StepperSetSpeed(STEPPER1, STEPPER1_MAX_PUL);
                        stepper0Speed = STEPPER1_MAX_PUL;
                    } else {
                        StepperSetSpeed(STEPPER1, STEPPER1_Kp_SQ);
                        stepper0Speed = STEPPER1_Kp_SQ;
                    }
                }
                lastBias = (double) tension1 - targetTen[0];
//            printf("%lf\n", STEPPER1_Kp);
//                printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d\n", targetYawPul, tension1, tensionL, stepper0Speed, stepper1Speed);
#if STEPPER_VOFA
                printf("S:%lf,%ld,%lf,%lf,%lf,%lf,", targetTen[0], tension1, STEPPER1_Kp, STEPPER1_P, STEPPER1_I, STEPPER1_D);
#endif
            }
//            else if (tension1 > 900)    StepperSetSpeed(STEPPER1, 0);

            if(stepper1Flag == 0) StepperSetSpeed(STEPPER2, 0);
            else if (stepper1Flag == 1 && tensionL <= TENSION_PROTECT_HIGH && tensionL >= TENSION_PROTECT_LOW) {
                int32_t tensionLL = tensionL;
                static double lastBias;

                if(targetTen[1] - (double) tensionLL <= INTEGRAL_START_BIAS && targetTen[1] - (double) tensionLL >= -INTEGRAL_START_BIAS)
                    integralBias[1] += targetTen[1] - (double) tensionLL - INTEGRAL_BIAS_SUB;
                else    integralBias[1] -= INTEGRAL_BIAS_SUB;
                if(integralBias[1] <= -INTEGRAL_MAX)    integralBias[1] = -INTEGRAL_MAX;
                if(integralBias[1] >= INTEGRAL_MAX) integralBias[1] = INTEGRAL_MAX;
                if(integralBias[1] >= -INTEGRAL_SET_ZERO && integralBias[1] <= INTEGRAL_SET_ZERO) integralBias[1] = 0;

                if((targetTen[1] - (double) tensionLL) <= STEPPER_CHANGE_TO_SMALL_K && (targetTen[1] - (double) tensionLL) >= -STEPPER_CHANGE_TO_SMALL_K){
                    posKpStepper1 = stepper_Kp1[(int)targetTen[1] / 10];
                    posKiStepper1 = stepper_Kp1[(int)targetTen[1] / 10] / KI_DIVIDE;
                    posKdStepper1 = stepper_Kd[(int)targetTen[1] / 10];
                    if(targetTen[1] >= STEPPER_NOR_SQ_TEN_THRESOLD){
                        if((targetTen[1] - tensionLL) == 1 || (targetTen[1] - tensionLL) == -1){
                            posKpStepper1 = STILL_RATE * posKpStepper1;
                            posKdStepper1 = STILL_RATE * posKdStepper1;
                        }
                    }
                    if(targetTen[1] > 160 && targetTen[1] <= 170){
                        posKpStepper1 = STEPPER2_161_170KP;
                        posKdStepper1 = STEPPER2_161_170KD;
                    }
                } else if((targetTen[1] - (double) tensionLL) > 2 * STEPPER_CHANGE_TO_SMALL_K || (targetTen[1] - (double) tensionLL) < -2 * STEPPER_CHANGE_TO_SMALL_K){
                    posKpStepper1 = STEPPER2BIGKP;
                    posKdStepper1 = STEPPER2BIGKD;
                }
//            IncrementalPI(5, velKpStepper, velKiStepper, tensionL, targetTen[1]);
                StepperStart(STEPPER2);
                if(targetTen[1] < STEPPER_NOR_SQ_TEN_THRESOLD) {
                    if (STEPPER2_Kp < -STEPPER2_MAX_PUL) {
                        StepperSetSpeed(STEPPER2, -STEPPER2_MAX_PUL);
                        stepper1Speed = -STEPPER2_MAX_PUL;
                    } else if (STEPPER2_Kp > STEPPER2_MAX_PUL) {
                        StepperSetSpeed(STEPPER2, STEPPER2_MAX_PUL);
                        stepper1Speed = STEPPER2_MAX_PUL;
                    } else {
                        StepperSetSpeed(STEPPER2, STEPPER2_Kp);
                        stepper1Speed = STEPPER2_Kp;
                    }
                } else {
                    if (STEPPER2_Kp_SQ < -STEPPER2_MAX_PUL) {
                        StepperSetSpeed(STEPPER2, -STEPPER2_MAX_PUL);
                        stepper1Speed = -STEPPER2_MAX_PUL;
                    } else if (STEPPER2_Kp_SQ > STEPPER2_MAX_PUL) {
                        StepperSetSpeed(STEPPER2, STEPPER2_MAX_PUL);
                        stepper1Speed = STEPPER2_MAX_PUL;
                    } else {
                        StepperSetSpeed(STEPPER2, STEPPER2_Kp_SQ);
                        stepper1Speed = STEPPER2_Kp_SQ;
                    }
                }
//                lastBias = (double) tensionL - targetTen[1];
                lastBias = targetTen[1] - (double) tensionLL;
//            printf("%ld, %lf = -20 * (%lf - %ld)\n", tensionL, STEPPER2_Kp, targetTen[1], tensionLL);
//                printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d\n", targetYawPul, tension1, tensionLL, stepper0Speed, stepper1Speed);
#if STEPPER_VOFA
                printf("%lf,%ld,%lf,%lf,%lf,%lf\n", targetTen[1], tensionL, STEPPER2_Kp, STEPPER2_P, STEPPER2_I, STEPPER2_D);
#endif
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

if(aimbot_mode) {
    // 在数据接收后的处理逻辑中：
    int not_detect_cont = 0;
    for (int j = 0; j < AIMBOT_RX_BUF_LEN - 9; ++j) {
        if (_rx_buf[j + 0] == 0xA5) {
            // 提取 yaw_error（大端转小端）
            uint8_t *yaw_ptr = &_rx_buf[j + 2];  // 原数据：0x90,0xA0,0x2A,0xC4（大端）

            // 手动反转字节序（大端 -> 小端）
            union {
                uint32_t u;
                float f;
            } yaw_convert;

            // 将反转后的字节按大端序组合成uint32
            yaw_convert.u = (yaw_ptr[j + 3] << 24) |  // 原第4字节（0xC4）移到最高位
                            (yaw_ptr[j + 2] << 16) |  // 原第3字节（0x2A）
                            (yaw_ptr[j + 1] << 8) |  // 原第2字节（0xA0）
                            yaw_ptr[j + 0];          // 原第1字节（0x90）在最低位

            yaw_error = yaw_convert.f;     // 通过联合体转为float

            // 打印调试信息
//                printf("yaw_bytes: %02X %02X %02X %02X\n",
//                       yaw_ptr[3], yaw_ptr[2], yaw_ptr[1], yaw_ptr[0]);
//                printf("yaw_uint: %08lX\n", (uint32_t)yaw_convert.u);

            // 提取 target_status
            target_status = _rx_buf[j + 6];

//            printf("yaw_error = %f\n", yaw_error);
//            printf("target_status = %d\n", target_status);

            for (int i = 0; i < 18; ++i) {
                _rx_buf[i] = '\0';
            }
        } else {
            not_detect_cont++;

//            printf("yaw_error = %f\n", yaw_error);
//            printf("target_status = %d\n", target_status);
        }
    }
    if (not_detect_cont >= AIMBOT_RX_BUF_LEN - 9) target_status = 0;
//            PrintCarData(&car_data);
}
    }
    if (htim->Instance == htim10.Instance) {     //100ms timer
        static uint16_t pointer, couut;
        couut++;
        static int32_t lastTension1, lastTensionL;
        if(HAL_GPIO_ReadPin(DART_STOP_SW_GPIO_Port, DART_STOP_SW_Pin) == GPIO_PIN_RESET && shootFlag == 0) {
            ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, 0);
        } else if(shootFlag == 0) {
            ServoSet(SERVO_GRASP, SERVO_GRASP_GRASP, 0);
        }
//        printf("dart_launch_opening_status = %d\n", dart_launch_opening_status);
//        printf("shootFlag = %d\n", shootFlag);
//        printf("canShootFlag = %d\n", canShootFlag);
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
        if(couut % 10 == 0) {
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
            printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d, tarYawPul: %d, furYaw[0]=: %d, Kp: %.1lf, %.1lf, Ki: %.1lf, %.1lf, Kd: %.1lf, %.1lf\n", targetYawPul, tension1, tensionL, stepper0Speed, stepper1Speed, targetYawPul, furTarYaw[0], posKpStepper0, posKpStepper1, posKiStepper0, posKiStepper1, posKdStepper0, posKdStepper1);
//            printf("curYaw: %d/ curTen: R: %ld, L: %ld; stepper1speed: %d, stepper2speed: %d, Relay GPIO: %d\n", targetYawPul, tension1, tensionL, stepper0Speed, stepper1Speed,
//                   HAL_GPIO_ReadPin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin));
            printf("FSM: %d, contFromLastUart: %lld\n", FeedFSMState(), contFromLastUart);
#endif
#if CAN_SHOOT_INFO
            printf("can_shoot_flag = %d, shootFlag = %d, furTarTen[%d - 1] = %d, game_progress = %d, dart_target = %d, dart_remaining_time = %d, lastDart_launch_opening_status = %d, dart_launch_opening_status = %d, stage_time = %d\n", canShootFlag, shootFlag, shootFlag, furTarTen[shootFlag - 1], game_progress, dart_target, dart_remaining_time, lastDart_launch_opening_status, dart_launch_opening_status, stage_remain_time);

#endif
#if TEN_LIGHT_INFO
            printf("curTen: R: %ld, L: %ld\n", tension1, tensionL);
#endif
#if AIMBOT_INFO
            printf("AIMBOT_PID: %f\n", AIMBOT_PID);
            printf("integralYawError: %f\n", integralYawError);
#endif
#if RS485_LIGHT_INFO
            printf("tension1DataAddress: %d, rs485_1:", ex_tension1DataAddress);
            for (int i = 0; i < READ_TENSION_RX_BUF_LENGTH; ++i)
                printf("%x,", ex_rs4851data[i]);
            printf("\n");
            printf("tension2DataAddress: %d, rs485_2:", ex_tension2DataAddress);
            for (int i = 0; i < READ_TENSION_RX_BUF_LENGTH; ++i)
                printf("%x,", ex_rs4852data[i]);
            printf("\n");
#endif
#if UART5_INFO
            for (int i = 0; i < 18; ++i){
                printf("%x, ",_rx_buf[i]);
            }
            printf("\n");
#endif

#if UART5_HADDLE_INFO
            printf("yaw_error = %f, tarYawPul = %d\n", yaw_error, targetYawPul);
            printf("target_status = %d\n", target_status);
#endif

//            HAL_UART_Transmit(&huart5, (uint8_t *)&(car_data->header), 1, 10);
//            uint8_t A5 = 0xA5;
//            HAL_UART_Transmit(&huart5, &A5, 1, 10);
#if USE_REMOTE
            RemoteControl();
#endif
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
            if (contFromLastUart > CONT_TO_READY_TO_SHOOT && furTarTen[0] != 0 && shootFlag < 4)
                DartFeedStartUp();
            couut = 0;
/*
            for (int i = 0; i < 500; ++i){
                printf("%c", USART1RxBuf[i]);
            }
            printf("\n");
*/
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
            uint8_t rx6HandleBuf[RX6_BUFF_LENGTH];
            if (ContainsBytesAndCopy(USART6RxBuf, &Rx6Pointer, JudgeUart0001, RX6_BUFF_LENGTH, rx6HandleBuf, 0x0b)) {
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

#if JUDGE0001_RAW_INFO
                    printf("0001:\n");
                    for (int i = 0; i < rx6HandleBuf[0] + 1; ++i) {
                        printf("%x ", rx6HandleBuf[i]);
                    }
#endif
#if JUDGE0001_HANDLED_INFO
                    printf("\ngame_type: %d\n", game_type);
                    printf("game_progress: %d\n", game_progress);
                    printf("stage_remain_time: %d\n", stage_remain_time);
                    printf("\n");
#endif
            }
            if (ContainsBytesAndCopy(USART6RxBuf, &Rx6Pointer, JudgeUart0105, RX6_BUFF_LENGTH, rx6HandleBuf, 0x03)) {
                    //没开赛时:rx6HandleBuf[] = 3 0 0 0, while the first '3' is the length of rx6HandleBuf
                    //rx6HandleBuf[1]:  己方飞镖发射剩余时间，单位:秒
                    dart_remaining_time = rx6HandleBuf[1];
                    dart_target = rx6HandleBuf[2] >> 6;

#if JUDGE0105_RAW_INFO
                    printf("0105:\n");
                    for (int i = 0; i < rx6HandleBuf[0] + 1; ++i) {
                        printf("%x ", rx6HandleBuf[i]);
                    }
#endif
#if JUDGE0105_HANDLED_INFO
                    printf("\ndart_remaining_time: %d\n", dart_remaining_time);
                    printf("\ndart_target: %d\n", dart_target);
                    printf("\n");
#endif
            }
            if (ContainsBytesAndCopy(USART6RxBuf, &Rx6Pointer, JudgeUart020A, RX6_BUFF_LENGTH, rx6HandleBuf, 0x06)) {
                    //没开赛时:rx6HandleBuf[] = 6 0 0 0 0 0 0, while the first '6' is the length of rx6HandleBuf
                    //rx6HandleBuf[1]:  1:关闭 2:正在开启或者关闭中 0:已经开启
                    dart_launch_opening_status = rx6HandleBuf[1];
                    //rx6HandleBuf[3-4]: 切换击打目标时的比赛剩余时间，单位:秒，无/未切换动作，默认为 0。
                    target_change_time = rx6HandleBuf[3] | (rx6HandleBuf[4] << 8);
                    //rx6HandleBuf[5-6]: 最后一次操作手确定发射指令时的比赛剩余时间，单位:秒，默认为 0。
                    latest_launch_cmd_time = rx6HandleBuf[5] | (rx6HandleBuf[6] << 8);


#if JUDGE020A_RAW_INFO
                    printf("020A:\n");
                    for (int i = 0; i < rx6HandleBuf[0] + 1; ++i) {
                        printf("%x ", rx6HandleBuf[i]);
                    }
#endif
#if JUDGE020A_HANDLED_INFO
                    printf("\ntarget_change_time: %d\n", target_change_time);
                    printf("\ndart_launch_opening_status: %d\n", dart_launch_opening_status);
                    printf("latest_launch_cmd_time: %d\n", latest_launch_cmd_time);
                    printf("\n");
#endif
            }
        }
        if(resetFeedCont > 0)   resetFeedCont--;
        else if (resetFeedCont == 0) {
            resetFeedCont = -1;
            DartFeedStopDown();
        }
        uint8_t rxHandleBuf[RX_BUFF_LENGTH];
        //扳机舵机回中
        if(servoTriggerCont < SERVO_TRIGGER_MIDDLE_DELAY)        servoTriggerCont++;
        else if(HAL_GetTick() >= 8000){
            ServoSet(SERVO_TRIGGER, SERVO_TRIGGER_MIDDLE, 0);
        }

        if (ContainsAndCopy(USART1RxBuf, &RxPointer, '\n', RX_BUFF_LENGTH, rxHandleBuf)) {
            if (ContainsSubString(rxHandleBuf + 1, SetYaw)) {
                contFromLastUart = 1;
                switch (rxHandleBuf[8]) {
                    case '1':
                        furTarYaw[0] = StrToInt(rxHandleBuf, 10, ')');
                        printf("yaw[0]: %d\n", furTarYaw[0]);
                        StepperStop(STEPPER1);
                        StepperStop(STEPPER2);
                        break;
                    case '2':
                        furTarYaw[1] = StrToInt(rxHandleBuf, 10, ')');
                        printf("yaw[1]: %d\n", furTarYaw[1]);
                        StepperStop(STEPPER1);
                        StepperStop(STEPPER2);
                        break;
                    case '3':
                        furTarYaw[2] = StrToInt(rxHandleBuf, 10, ')');
                        printf("yaw[2]: %d\n", furTarYaw[2]);
                        StepperStop(STEPPER1);
                        StepperStop(STEPPER2);
                        break;
                    case '4':
                        furTarYaw[3] = StrToInt(rxHandleBuf, 10, ')');
                        printf("yaw[3]: %d\n", furTarYaw[3]);
                        StepperStop(STEPPER1);
                        StepperStop(STEPPER2);
                        break;
                    case '5':
                        furTarYaw[4] = StrToInt(rxHandleBuf, 10, ')');
                        printf("yaw[4]: %d\n", furTarYaw[4]);
                        targetYawPul = furTarYaw[4];
                        break;
                }
            } else if (ContainsSubString(rxHandleBuf + 1, SetTen)) {
#if USE_RELAY_CONTROL
                HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
#endif
                contFromLastUart = 1;
                shootFlag = 0;
                switch (rxHandleBuf[8]) {
                    case '1':
                        furTarTen[0] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten[0]: %d\n", furTarTen[0]);
                        StepperStop(STEPPER1);
                        StepperStop(STEPPER2);
                        break;
                    case '2':
                        furTarTen[1] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten[1]: %d\n", furTarTen[1]);
                        StepperStop(STEPPER1);
                        StepperStop(STEPPER2);
                        break;
                    case '3':
                        furTarTen[2] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten[2]: %d\n", furTarTen[2]);
                        StepperStop(STEPPER1);
                        StepperStop(STEPPER2);
                        break;
                    case '4':
                        furTarTen[3] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten[3]: %d\n", furTarTen[3]);
                        StepperStop(STEPPER1);
                        StepperStop(STEPPER2);
#if !USE_game_progress_AND_stage_remain_time && !USE_dart_remaining_time
                        canShootFlag = 1;       //仅测试四发连发时使用
                        shootFlag = 1;         //仅测试四发连发时使用
#endif
                    case '5':
                        furTarTen[4] = StrToInt(rxHandleBuf, 10, ')');
                        printf("ten[4]: %d\n", furTarTen[4]);
                        break;
                }
            } else if (ContainsSubString(rxHandleBuf, TestShoot)) {
                static int cout1;
                cout1++;
                printf("TestShoot %d\n", cout1);
                shootFlag = 5;
                DartFeedStartUp();
#if USE_RELAY_CONTROL
                HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_RESET);
#endif
            } else if (ContainsSubString(rxHandleBuf, AbortShoot)) {
                static int cout2;
                cout2++;
                printf("AbortShoot %d\n", cout2);
                canShootFlag = 0;
                shootFlag = 0;
                targetTen[0] = furTarTen[4];
                targetTen[1] = furTarTen[4];
#if USE_RELAY_CONTROL
                HAL_GPIO_WritePin(RELAY_CONTROL_GPIO_Port, RELAY_CONTROL_Pin, GPIO_PIN_SET);
#endif
                stepper1Flag = 1;
                stepper0Flag = 1;
            } else if (ContainsSubString(rxHandleBuf, SetCurYawToZero)) {
                static int cout3;
                cout3++;
                contFromLastUart = 1;
                printf("SetCurYawToZero %d\n", cout3);
                UART_TargetYawPul = furTarYaw[5];
                furTarYaw[5] = 0;
            } else if (ContainsSubString(rxHandleBuf, ResetFeed)) {
                static int cout4;
                cout4++;
                contFromLastUart = 1;
                shootFlag = 0;
                printf("ResetFeed %d\n", cout4);
//                DartFeedStartDown();
                DartFeedStartDown();
#if OLD_FEED
                StepperStart(STEPPER4);
#endif
                resetFeedCont += 40;
            } else if (ContainsSubString(rxHandleBuf, SonicRangeTestSetParas)) {
                static int cout6;
                cout6++;
                printf("SonicRangeTestSetParas %d\n", cout6);
            } else if (ContainsSubString(rxHandleBuf, SonicRangeTest)) {
                static int cout5;
                cout5++;
                printf("SonicRangeTest %d\n", cout5);
#if TEST_SERVO_TRIGGER
                static int trig;
                if(trig == 0){
                    trig = 1;
//                    ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_UP, 0);
//                    ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_UP, 0);
                    ServoSet(SERVO_TRIGGER, SERVO_TRIGGER_RESET, 0);
//                    ServoSet(SERVO_GRASP, SERVO_GRASP_RELEASE, 0);
//                    ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_RIGHT, 0);
                    printf("RESET TRIGGE, init grasp, SERVO_LEFT_RIGHT_RIGHT, SERVO_UP_DOWN_LEFT_UP, SERVO_UP_DOWN_RIGHT_UP\n");
                    servoTriggerCont = 0;
                } else if(trig == 1){
                    trig = 0;
//                    ServoSet(SERVO_UP_DOWN_LEFT, SERVO_UP_DOWN_LEFT_DOWN, 0);
//                    ServoSet(SERVO_UP_DOWN_RIGHT, SERVO_UP_DOWN_RIGHT_DOWN, 0);
                    ServoSet(SERVO_TRIGGER, SERVO_TRIGGER_SHOOT, 0);
//                    ServoSet(SERVO_GRASP, SERVO_GRASP_GRASP, 0);
//                    ServoSet(SERVO_LEFT_RIGHT, SERVO_LEFT_RIGHT_LEFT, 0);
                    printf("SHOT TRIGGE, grasp grasp, SERVO_LEFT_RIGHT_LEFT, SERVO_UP_DOWN_LEFT_DOWN, SERVO_UP_DOWN_RIGHT_DOWN\n");
                    servoTriggerCont = 0;
                }

#endif
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
            left3508StopCont = MOTOR_RELEASE_DELAY;
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
            right3508StopCont = MOTOR_RELEASE_DELAY;
        }
    }
}