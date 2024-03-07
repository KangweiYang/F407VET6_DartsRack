//
// Created by root on 23-12-5.
//

#include "../Inc/FindForwardDir.h"
#include "../../MOTOR/Inc/MotorUnit.h"
#include "../../OLED/Inc/OLED.h"
#include "../../SW/Inc/SW.h"
#include "main.h"


extern uint8_t uartTemp[2];
extern uint16_t delaySensors;
extern int64_t restTimeToResetDelaySensors[7];
extern uint64_t fullSensorsTimes;
extern int32_t fullDelay;
extern int64_t pos[2];
extern uint8_t exitYDelay;

uint8_t finishSign = 0;

int FullDelay = 0;

uint64_t lastFullSensorsTimes = 0;

uint16_t lastSensors = 0;
uint64_t diffSensorCount = 0;
uint64_t sensorChangeSerial[8] = {0};

/**
    * @breif    Find the lowest bit
    * @note     None
    * @param    binary data. Ex. 0b1011
    * @retval   binary data. Ex. 0b1000
    */
uint8_t FindLowestBit(uint8_t input) {
//    uint8_t n = 0;
//    while (!(0b1110 & (0b1 << n))) {
//        n++;
//    }
//    return 0b1 << n;
    return input & (-input);
}

/**
    * @breif    Find the highest bit
    * @note     None
    * @param    binary data. Ex. 0b1011
    * @retval   binary data. Ex. 0b1000
    */
uint8_t FindHighestBit(uint8_t input) {
    uint8_t n = 7;
    while (!(0b1010 & (0b1 << n))) {
        n--;
    }
    return 0b1 << n;
}

/**
    * @breif    Reset delaySensors with delay time.
    * @note     None
    * @param    resetSign: ex. 7 means to reset 7th value.
    * @retval   None
    */
//void ResetDelaySensors(uint8_t resetSign) {
////    delaySensors -= (0b1 << (resetSign - 1));
//}

/**
    * @breif    find which bit is 1.
    * @note     None
    * @param    bits: ex. 0b100 -> 2
    * @retval   which bit is 1. ex. 2
    */
uint8_t whichBitIs1(uint8_t bits) {
    for (uint8_t i = 0; i < 7; ++i) {
        if ((bits >> i) & 0b1)
            return i;
    }
    return 8;
}

/**
    * @breif    how many 1s
    * @note     None
    * @param    bits. ex. 0b10110
    * @retval   how many 1s: ex. 3
    */
uint8_t howMany1s(uint8_t bits) {
    uint8_t result = 0;
    while (bits) {
        result++;
        bits &= bits - 1;
    }
    return result;
}

/**
    * @breif    Power product
    * @note     None
    * @param    x^p
    * @retval   x^p
    */
uint32_t Power(uint32_t x, uint32_t p) {
    uint64_t result = 1;
    for (int i = 0; i < p; ++i) {
        result *= x;
    }
    return result;
}

/**
    * @breif    change 0b1010 to 1010
    * @note     None
    * @param    0b1010
    * @retval   1010
    */
uint32_t BinToDec(uint8_t bin) {
    uint32_t dec = 0;
    if (bin == 0) return 0b0;
    for (int i = 0; i < 7; ++i) {
        if ((bin >> i) & 0b1) dec += Power(10, i);
    }
    return dec;
}

/**
    * @breif    resetTime deal: 0b10101 -> resetTimeToResetDelaySensors[0b1 and 0b10 and 0b10101] = RESET_DELAY_TIME
    * @note     None
    * @param    0b10101
    * @retval   None
    */
void ResetTimeToResetDelaySensors(uint8_t bin) {
    while (bin) {
        restTimeToResetDelaySensors[whichBitIs1(FindLowestBit(bin))] = RESET_DELAY_TIME;//-1-1-1-1-1-1-1-1-1!!!!!!!!!!
        bin &= (bin - 1);
    }
}

/**
    * @breif    Find forward direction of the ideal direction, from greyscale sensors.
    * @note     None
    * @param    greyscale sensors' value, 0b00100 means the straight direction.
    * @retval   None
    */
uint8_t FindForwardDir(uint16_t sensors) {
    //full:1.start  2.enter circle 3.exit circle 4.enter Y 5.exit Y 6.the 2nd round 7.enter circle 8.exit circle 9.enter Y 10.exit Y 11.finish

//    if (sensors ^ lastSensors) {
    //different sensors value
//        if(SW_Get() == 0b1) {
//            OLED_ShowNum(0, 2, BinToDec(delaySensors), 7, 12, 0);
//            OLED_ShowNum(0, 3, BinToDec(sensors), 7, 12, 0);
//            OLED_ShowNum(0, 4, BinToDec(lastSensors), 7, 12, 0);
//        }
//        delaySensors |= (sensors & (sensors ^ lastSensors));                                   //instant set black line
//        diffSensorCount++;
//        sensorChangeSerial[whichBitIs1(sensors & (sensors ^ lastSensors))] = diffSensorCount;
////        restTimeToResetDelaySensors[lastSensors & (sensors ^ lastSensors)] = RESET_DELAY_TIME;      //delay reset value
//        ResetTimeToResetDelaySensors(lastSensors & (sensors ^ lastSensors));                      //delay reset value
//    }
    if (sensors == 0) {
//        if (fullSensorsTimes & (STATE_ENTER_Y | STATE_EXIT_Y)) {
//            return 0b0000001;
//        } else if (fullSensorsTimes & (STATE_ENTER_Y_2 | STATE_EXIT_Y_2)) {
//            return 0b1000000;
//        }
        return lastSensors;

    } else lastSensors = sensors;                                      //store sensor values


//    if ((sensors & 0b0010000) && (sensors & 0b0001000) && (sensors & 0b0000100) && (sensors & 0b0000010) && (sensors & 0b0000001) ) {
//        lastFullSensorsTimes = fullSensorsTimes;
////        delaySensors = 0;
//        if (fullSensorsTimes == 0) {
//            fullSensorsTimes = 0b1;
//            return sensors;                                    //start
//        } else if (fullSensorsTimes & STATE_FINISH) return 0b0;
//////        if(fullDelay == 0)  fullSensorsTimes = fullSensorsTimes << 1;
//////        fullDelay = FULL_DELAY;                       //md, use fullDelay will be reset to 0 wrongly, what fack.
//        if (FullDelay == 0) fullSensorsTimes = fullSensorsTimes << 1;
//        FullDelay = FULL_DELAY;
////        static uint64_t lastFullSensorsTimes;
//        if (lastFullSensorsTimes ^ fullSensorsTimes) {           //new full
//            lastFullSensorsTimes = fullSensorsTimes;

//        if (fullSensorsTimes & (STATE_ENTER_LEFT_S_TURN | STATE_ENTER_LEFT_S_TURN_2)) {
//            return 0b1000000;
//        } else if (fullSensorsTimes & (STATE_ENTER_Y | STATE_EXIT_Y)) {
//            return 0b0000001;
//        } else if (fullSensorsTimes & (STATE_ENTER_Y_2 | STATE_EXIT_Y_2)) {
//            return 0b1000000;
//        } else if (fullSensorsTimes &
//                   (STATE_ENTER_CIRCLE | STATE_ENTER_CIRCLE_2 | STATE_EXIT_CIRCLE | STATE_EXIT_CIRCLE_2)) {
//            return 0b0001000;
//        } else if (fullSensorsTimes & STATE_FINISH) {
//            return 0b0;                                         //stop
//        }
//        }

    if ((sensors & 0b0000001) && (sensors & 0b0000010) && (sensors & 0b0000100) && (sensors & 0b0001000)) {
        if (pos[0] > POS_ALMOST_FINISH) {
            finishSign = 1;
            return 0b0;
        } else if ((pos[0] > POS_EXIT_CIRCLE_2) &&
                   (pos[0] < POS_ALMOST_FINISH)) {
            exitYDelay = EXIT_Y_DELAY_TIME;
            lastSensors = 0b0000001;
            return lastSensors;
        } else {
            return 0b0001000;
        }
    } else if ((sensors & 0b0000010) && (sensors & 0b0000100) && (pos[0] > POS_EXIT_CIRCLE_2) &&
               (pos[0] < POS_ALMOST_FINISH)) {
        exitYDelay = EXIT_Y_DELAY_TIME;
        lastSensors = 0b0000001;
        return lastSensors;
    }
    if (exitYDelay > 0) {
        lastSensors = 0b0000001;
        return lastSensors;
    }

    return sensors;
}

/**
    * @breif    Motor control (while not suitable putting here)
    * @note     None
    * @param    direction from FindForwardDir(sensors)
    * @retval   None
    */
void MotorControl(int sensors) {
    static uint8_t justFinishRightTurnSign, justFinishLeftTurnSign;
    int dir = FindForwardDir(sensors);
//    if ((delaySensors & 0b1111110) & 0b1110000 && howMany1s(sensors) == 1 &&
//        sensorChangeSerial[6] < sensorChangeSerial[5] && sensorChangeSerial[5] < sensorChangeSerial[4] &&
//        sensors & 0b0001110)    dir >> 1;
//    else if ((delaySensors & 0b111111) & 0b111 && howMany1s(sensors) == 1 &&
//        sensorChangeSerial[0] < sensorChangeSerial[1] && sensorChangeSerial[1] < sensorChangeSerial[2] &&
//        sensors & 0b0111000)    dir << 1;

//add the below code, the car will not start?

//    if(dir = 0b0001000 && (justFinishLeftTurnSign == 1 || justFinishRightTurnSign == 1)){
//        justFinishRightTurnSign = 0;
//        justFinishLeftTurnSign = 0;
//        dir = 0b0001000;
//    }



//    if(justFinishRightTurnSign == 1){
//        if(sensorChangeSerial[3] > sensorChangeSerial[4] && sensorChangeSerial[4] > sensorChangeSerial[2])   justFinishRightTurnSign = 0;
//        if(dir & 0b0110000)  dir = dir << 2;
//    } else if(justFinishLeftTurnSign == 1){
//        if(sensorChangeSerial[3] > sensorChangeSerial[2] && sensorChangeSerial[2] > sensorChangeSerial[4])   justFinishLeftTurnSign = 0;
//        if(dir & 0b0000100) dir = dir >> 2;
//        else if(dir & 0b0000010)    dir = 0b1;
//    } else if(sensorChangeSerial[0] < sensorChangeSerial[1] && sensorChangeSerial[1] < sensorChangeSerial[2] && sensorChangeSerial[2] < sensorChangeSerial[3] && sensorChangeSerial[4] < sensorChangeSerial[2] && justFinishRightTurnSign == 0){
//        justFinishRightTurnSign = 1;
//        dir = dir << 2;
//    }
//    else if (sensorChangeSerial[6] < sensorChangeSerial[4] && sensorChangeSerial[4] < sensorChangeSerial[3] && sensorChangeSerial[2] < sensorChangeSerial[4] && justFinishLeftTurnSign == 0){
//        justFinishLeftTurnSign = 1;
//        dir = dir >> 2;
//    }
    if (SW_Get() == 0b10) {
        OLED_ShowNum(20, 0, (unsigned int) justFinishRightTurnSign, 7, 12, 0);
        OLED_ShowNum(20, 1, (unsigned int) justFinishLeftTurnSign, 7, 12, 0);
        OLED_ShowNum(20, 7, (unsigned int) BinToDec(dir), 7, 12, 0);
    } else if (SW_Get() == 0b11) {
        OLED_ShowNum(20, 0, (unsigned int) BinToDec(fullSensorsTimes), 12, 12, 0);
        OLED_ShowNum(0, 3, fullDelay, 12, 12, 0);
        OLED_ShowNum(0, 4, FullDelay, 12, 12, 0);
        OLED_ShowNum(20, 7, (unsigned int) BinToDec(dir), 7, 12, 0);
    } else if (SW_Get() & 0b110) {
        OLED_ShowNum(20, 0, (unsigned int) BinToDec(sensors), 12, 12, 0);
        OLED_ShowNum(0, 3, (unsigned int) BinToDec(lastSensors), 12, 12, 0);
        OLED_ShowNum(20, 7, (unsigned int) BinToDec(dir), 7, 12, 0);
    }
//    if (fullSensorsTimes < STATE_ROUND_2) {
//        if (dir & 0b0000001) OpenCir(GENERAL_SPEED * (2 - XXX_SPEED), GENERAL_SPEED * XXX_SPEED);
//        else if (dir & 0b0000010) OpenCir(GENERAL_SPEED * (2 - XX_SPEED), GENERAL_SPEED * XX_SPEED);
//        else if (dir & 0b0000100) OpenCir(GENERAL_SPEED * (2 - X_SPEED), GENERAL_SPEED * X_SPEED);
//        else if (dir & 0b11000000) OpenCir(GENERAL_SPEED * XXX_SPEED, GENERAL_SPEED * (2 - XXX_SPEED));
//        else if (dir & 0b0100000) OpenCir(GENERAL_SPEED * XX_SPEED, GENERAL_SPEED * (2 - XX_SPEED));
//        else if (dir & 0b0010000) OpenCir(GENERAL_SPEED * X_SPEED, GENERAL_SPEED * (2 - X_SPEED));
//        else if (dir & 0b0001000) OpenCir(GENERAL_SPEED, GENERAL_SPEED);
//        else OpenCir(0, 0);
//    } else if (fullSensorsTimes < STATE_FINISH) {
//        if (dir & 0b11000000) OpenCir(GENERAL_SPEED * XXX_SPEED, GENERAL_SPEED * (2 - XXX_SPEED));
//        else if (dir & 0b0100000) OpenCir(GENERAL_SPEED * XX_SPEED, GENERAL_SPEED * (2 - XX_SPEED));
//        else if (dir & 0b0010000) OpenCir(GENERAL_SPEED * X_SPEED, GENERAL_SPEED * (2 - X_SPEED));
//        else if (dir & 0b0000001) OpenCir(GENERAL_SPEED * (2 - XXX_SPEED), GENERAL_SPEED * XXX_SPEED);
//        else if (dir & 0b0000010) OpenCir(GENERAL_SPEED * (2 - XX_SPEED), GENERAL_SPEED * XX_SPEED);
//        else if (dir & 0b0000100) OpenCir(GENERAL_SPEED * (2 - X_SPEED), GENERAL_SPEED * X_SPEED);
//        else if (dir & 0b0001000) OpenCir(GENERAL_SPEED, GENERAL_SPEED);
//        else OpenCir(0, 0);
//    } else {
//        OpenCir(0, 0);
//    }
    if (finishSign == 1) {
        OpenCir(0, 0);
        return;
    }
    if (pos[0] < POS_ROUND_2) {
        if (dir & 0b11000000) OpenCir(GENERAL_SPEED * XXX_SPEED, GENERAL_SPEED * (2 - XXX_SPEED));
        else if (dir & 0b0100000) OpenCir(GENERAL_SPEED * XX_SPEED, GENERAL_SPEED * (2 - XX_SPEED));
        else if (dir & 0b0010000) OpenCir(GENERAL_SPEED * X_SPEED, GENERAL_SPEED * (2 - X_SPEED));
        else if (dir & 0b0000001) OpenCir(GENERAL_SPEED * (2 - XXX_SPEED), GENERAL_SPEED * XXX_SPEED);
        else if (dir & 0b0000010) OpenCir(GENERAL_SPEED * (2 - XX_SPEED), GENERAL_SPEED * XX_SPEED);
        else if (dir & 0b0000100) OpenCir(GENERAL_SPEED * (2 - X_SPEED), GENERAL_SPEED * X_SPEED);
        else if (dir & 0b0001000) OpenCir(GENERAL_SPEED, GENERAL_SPEED);
        else OpenCir(0, 0);
    } else if (pos[0] > POS_ROUND_2) {
        if (dir & 0b0000001) OpenCir(GENERAL_SPEED * (2 - XXX_SPEED), GENERAL_SPEED * XXX_SPEED);
        else if (dir & 0b0000010) OpenCir(GENERAL_SPEED * (2 - XX_SPEED), GENERAL_SPEED * XX_SPEED);
        else if (dir & 0b0000100) OpenCir(GENERAL_SPEED * (2 - X_SPEED), GENERAL_SPEED * X_SPEED);
        else if (dir & 0b11000000) OpenCir(GENERAL_SPEED * XXX_SPEED, GENERAL_SPEED * (2 - XXX_SPEED));
        else if (dir & 0b0100000) OpenCir(GENERAL_SPEED * XX_SPEED, GENERAL_SPEED * (2 - XX_SPEED));
        else if (dir & 0b0010000) OpenCir(GENERAL_SPEED * X_SPEED, GENERAL_SPEED * (2 - X_SPEED));
        else if (dir & 0b0001000) OpenCir(GENERAL_SPEED, GENERAL_SPEED);
        else OpenCir(0, 0);
    } else {
        OpenCir(0, 0);
    }
}

/**
    * @breif    FullDelay --
    * @note     None
    * @param    None
    * @retval   None
    */
void FullDelayMinors(void) {
    if (FullDelay > 0) {
        FullDelay--;
    }
    if (exitYDelay > 0) {
        exitYDelay--;
    }
}