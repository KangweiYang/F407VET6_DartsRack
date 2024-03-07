#include "stdlib.h"
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "can.h"
#include "usb_host.h"
#include "usart.h"
#include "gpio.h"
#include "Library/SERVO/Inc/Servo.h"
#include "Library/STEPPER/Inc/Stepper.h"
#include "Library/MOTOR/Inc/MotorUnit.h"
#include "Library/ADC_DMA/Inc/ADC_DMA.h"
#include "Library/INC_PI/Inc/Inc_PI.h"
#include "Library/LED/Inc/LED.h"
#include "Library/GREYSCALE/Inc/Greyscale.h"
#include "Library/SW/Inc/SW.h"
#include "Library/FIND_FORWARD_DIR/Inc/FindForwardDir.h"
#include "Library/OLED/Inc/OLED.h"
#include "Library/OLED/Inc/UserOLED.h"

int PWMtest = 75;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_USB_HOST_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */

    HAL_Delay(100);
    ServoInit();
//    HAL_Delay(1000);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
    StepperInit(STEPPER1, 1680 - 1);
    ServoSet(1,75, 10);
    ServoSet(2,109, 10);
    ServoSet(3,500, 10);
    ServoSet(4,500, 10);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        MX_USB_HOST_Process();
        if(HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == GPIO_PIN_SET){
            HAL_Delay(50);
            while(HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == GPIO_PIN_SET);
            HAL_Delay(50);
            PWMtest -= 10;
            ServoGraspDart();
//            PWMtest = 71;
//            ServoSet(1, PWMtest, 10);
        }
        if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_SET){
            HAL_Delay(50);
            while(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_SET);
            HAL_Delay(50);
            PWMtest -= 1;
            ServoGraspDart();
//            ServoSet(1, PWMtest, 10);
        }
        if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_SET){
            HAL_Delay(50);
            while(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_SET);
            HAL_Delay(50);
            PWMtest += 1;
            ServoGraspDart();
//            ServoSet(1, PWMtest, 10);
        }
        if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_SET){
            HAL_Delay(50);
            while(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_SET);
            HAL_Delay(50);
            PWMtest += 10;
            ServoGraspDart();
//            PWMtest = 77;
//            ServoSet(1, PWMtest, 10);
        }


//        printf("PA0: %d  ", __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1));
//        printf("PA1: %d  \n", __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));
//        printf("PA2: %d  ", __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3));
//        printf("PA3: %d\n", __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4));
        HAL_Delay(60);
//        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//        HAL_Delay(333);
//        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
//        HAL_Delay(333);

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}