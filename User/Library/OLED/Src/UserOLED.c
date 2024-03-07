//
// Created by root on 23-12-7.
//

#include "../Inc/UserOLED.h"
#include "../Inc/OLED.h"
#include "../../FIND_FORWARD_DIR/Inc/FindForwardDir.h"
#include "main.h"

#define UartTemp    0
#define DelaySensors    1
#define RestTimeToResetDelaySensors 2
#define FullSensorsTimes    3
#define LastSensors     4
#define DiffSensorCount 5
#define SensorChangeSerial  6
#define Dir 7

extern uint8_t uartTemp[2];
extern uint16_t delaySensors;
extern int64_t restTimeToResetDelaySensors[7];
extern uint8_t fullSensorsTimes;
extern uint16_t lastSensors;
extern uint64_t diffSensorCount;
extern uint64_t sensorChangeSerial[8];

uint16_t printSign[10] = {0};

/**
    * @breif    print value to oled.
    * @note     None
    * @param    Serial num of value.
    * @retval   None
    */
void Print(uint16_t valueSerial) {
    if (valueSerial == UartTemp) {
        if (printSign[UartTemp] == 0) {
            OLED_ShowString(0, UartTemp, "sr            ", 12, 0);
            printSign[UartTemp] = 1;
        }
        OLED_ShowNum(20, UartTemp, (unsigned int) BinToDec(forwardDir), 7, 12, 0);
    } else if (valueSerial == DelaySensors) {
        if (printSign[DelaySensors] == 0) {
            OLED_ShowString(0, DelaySensors, "dS          ", 12, 0);
            printSign[DelaySensors] = 1;
        }
        OLED_ShowNum(20, DelaySensors, (unsigned int) BinToDec(delaySensors), 7, 12, 0);
    } else if (valueSerial == RestTimeToResetDelaySensors) {
        if (printSign[RestTimeToResetDelaySensors] == 0) {
            OLED_ShowString(0, RestTimeToResetDelaySensors, "rT          ", 12, 0);
            printSign[RestTimeToResetDelaySensors] = 1;
        }
        OLED_ShowNum(20, RestTimeToResetDelaySensors, (unsigned int) restTimeToResetDelaySensors[6], 1, 12, 0);
        OLED_ShowNum(35, RestTimeToResetDelaySensors, (unsigned int) restTimeToResetDelaySensors[5], 1, 12, 0);
        OLED_ShowNum(50, RestTimeToResetDelaySensors, (unsigned int) restTimeToResetDelaySensors[4], 1, 12, 0);
        OLED_ShowNum(65, RestTimeToResetDelaySensors, (unsigned int) restTimeToResetDelaySensors[3], 1, 12, 0);
        OLED_ShowNum(80, RestTimeToResetDelaySensors, (unsigned int) restTimeToResetDelaySensors[2], 1, 12, 0);
        OLED_ShowNum(95, RestTimeToResetDelaySensors, (unsigned int) restTimeToResetDelaySensors[1], 1, 12, 0);
        OLED_ShowNum(110, RestTimeToResetDelaySensors, (unsigned int) restTimeToResetDelaySensors[0], 1, 12, 0);
    } else if (valueSerial == FullSensorsTimes) {
        if (printSign[FullSensorsTimes] == 0) {
            OLED_ShowString(0, FullSensorsTimes, "fT          ", 15, 0);
            printSign[FullSensorsTimes] = 1;
        }
        OLED_ShowNum(20, FullSensorsTimes, (unsigned int) BinToDec(fullSensorsTimes), 14, 12, 0);
    } else if (valueSerial == LastSensors) {
        if (printSign[LastSensors] == 0) {
            OLED_ShowString(0, LastSensors, "lS          ", 12, 0);
            printSign[LastSensors] = 1;
        }
        OLED_ShowNum(20, LastSensors, (unsigned int) BinToDec(lastSensors), 7, 12, 0);
    } else if (valueSerial == DiffSensorCount) {
        if (printSign[DiffSensorCount] == 0) {
            OLED_ShowString(0, DiffSensorCount, "dC          ", 12, 0);
            printSign[DiffSensorCount] = 1;
        }
                OLED_ShowNum(20, DiffSensorCount, (unsigned int) diffSensorCount, 7, 12, 0);
    } else if(valueSerial == SensorChangeSerial){

        if (printSign[SensorChangeSerial] == 0) {
            OLED_ShowString(0, SensorChangeSerial, "CS          ", 12, 0);
            printSign[SensorChangeSerial] = 1;
        }
        OLED_ShowNum(20, SensorChangeSerial, (unsigned int) sensorChangeSerial[5], 2, 12, 0);
        OLED_ShowNum(35, SensorChangeSerial, (unsigned int) sensorChangeSerial[4], 2, 12, 0);
        OLED_ShowNum(50, SensorChangeSerial, (unsigned int) sensorChangeSerial[3], 2, 12, 0);
        OLED_ShowNum(65, SensorChangeSerial, (unsigned int) sensorChangeSerial[2], 2, 12, 0);
        OLED_ShowNum(80, SensorChangeSerial, (unsigned int) sensorChangeSerial[1], 2, 12, 0);
        OLED_ShowNum(95, SensorChangeSerial, (unsigned int) sensorChangeSerial[0], 2, 12, 0);

    }else if (valueSerial == Dir) {
        if (printSign[Dir] == 0) {
            OLED_ShowString(0, Dir, "dir          ", 12, 0);
            printSign[Dir] = 1;
        }
        OLED_ShowNum(20, Dir, (unsigned int) BinToDec(FindForwardDir(UartTemp)), 7, 12, 0);
    }
}
