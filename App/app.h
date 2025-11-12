#ifndef __APP_MAIN
#define __APP_MAIN

#include "serial.h"
#include "stm32u5xx_hal.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim3;

#define CAN_MSG__IN_LED_CONTROL 100
#define CAN_MSG__IN_SIGNAL_FILL 110
#define CAN_MSG__OUT_CAN_DIAG	0x6942

#define MAPPING_LED_FACTOR 3.53
#define MAPPING_LED_OFFSET 100

#define MAPPING_PWM_TO_TIMER_FACTOR 31.372549
#define MAPPING_PWM_TO_TIMER_OFFSET 0

#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA

void app_main(void);
void decode_obd2_frame(uint32_t id, uint8_t *data, uint8_t dlc);

#endif
