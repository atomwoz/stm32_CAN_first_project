#include "app.h"
#include <string.h>
#include <stdio.h>
#include "stm32u5xx_hal.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim3;
int serial_printf(const char *format, ...);
void serial_println(const char *str);
void serial_print(const char *str);
static HAL_StatusTypeDef send_can_frame(uint32_t id, uint8_t *data, uint8_t dlc);
void rx_router();
int32_t map_u8_to_i32(uint8_t input, double factor, double offset);

// Helper function to convert byte count to FDCAN DLC code
static uint32_t dlc_to_code(uint8_t dlc) {
	if (dlc == 0)
		return FDCAN_DLC_BYTES_0;
	if (dlc == 1)
		return FDCAN_DLC_BYTES_1;
	if (dlc == 2)
		return FDCAN_DLC_BYTES_2;
	if (dlc == 3)
		return FDCAN_DLC_BYTES_3;
	if (dlc == 4)
		return FDCAN_DLC_BYTES_4;
	if (dlc == 5)
		return FDCAN_DLC_BYTES_5;
	if (dlc == 6)
		return FDCAN_DLC_BYTES_6;
	if (dlc == 7)
		return FDCAN_DLC_BYTES_7;
	return FDCAN_DLC_BYTES_8;
}

// Helper function to convert FDCAN DLC code to byte count
static uint8_t dlc_to_bytes(uint32_t dlc_code) {
	if (dlc_code == FDCAN_DLC_BYTES_0)
		return 0;
	if (dlc_code == FDCAN_DLC_BYTES_1)
		return 1;
	if (dlc_code == FDCAN_DLC_BYTES_2)
		return 2;
	if (dlc_code == FDCAN_DLC_BYTES_3)
		return 3;
	if (dlc_code == FDCAN_DLC_BYTES_4)
		return 4;
	if (dlc_code == FDCAN_DLC_BYTES_5)
		return 5;
	if (dlc_code == FDCAN_DLC_BYTES_6)
		return 6;
	if (dlc_code == FDCAN_DLC_BYTES_7)
		return 7;
	return 8;
}

static uint8_t RxData[8];
static FDCAN_RxHeaderTypeDef RxHeader;
static uint32_t TicksFromLastBlink;
static uint32_t BlinkingWaitTime;

// Pozwole sobie komentarz po polsku wrzuić ;)
// Dla tego projektu overkillem było by używanie RTOS'a dlatetgo zrobię
// taką prostą funkcję zachowowującą się jak by
// była schedulowaną przez async schedulera
void led_pseudo_async_task() {
	uint32_t tick = HAL_GetTick();
	if (tick >= TicksFromLastBlink + BlinkingWaitTime) {
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		TicksFromLastBlink = tick;
	}
}

//Main app loop code
void app_main(void) {

	uint32_t last_tx_time = HAL_GetTick();
	TicksFromLastBlink = last_tx_time;
	BlinkingWaitTime = 500;

	// Start CAN
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		serial_println("ERROR: Failed to start FDCAN");
		return;
	}
	if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
	{
		serial_println("ERROR: Failed to start TIMER for PWM");
		return;
	}
	serial_println("CAN driver Started");

	while (1) {
		uint32_t current_tick = HAL_GetTick();
		memset(RxData, 0, sizeof(RxData));
		memset(&RxHeader, 0, sizeof(FDCAN_RxHeaderTypeDef));

		if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
			if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData)
					!= HAL_OK) {
				serial_printf("ERROR: CAN RX %u\r\n", hfdcan1.ErrorCode);
				continue;
			}
			rx_router();
		}

		led_pseudo_async_task();
	}
}

void rx_router() {
	switch (RxHeader.Identifier) {

	case CAN_MSG__IN_LED_CONTROL:
		BlinkingWaitTime = (uint32_t) map_u8_to_i32(RxData[0],
		MAPPING_LED_FACTOR, MAPPING_LED_OFFSET);
		serial_printf("INFO: Green LED blinking time change request to %u ms\r\n", BlinkingWaitTime);
		break;

	case CAN_MSG__IN_SIGNAL_FILL:
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)  map_u8_to_i32(RxData[0], MAPPING_PWM_TO_TIMER_FACTOR , MAPPING_PWM_TO_TIMER_OFFSET));
		serial_printf("INFO: PWM duty change request %u \r\n", RxData[0]);
		break;

	}
}

// Helper function to send_can_frame
HAL_StatusTypeDef send_can_frame(uint32_t id, uint8_t *data, uint8_t dlc) {
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = dlc_to_code(dlc);
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
			&TxHeader, data);
	return status;
}

//CAn values mappping utils:
int32_t map_u8_to_i32(uint8_t input, double factor, double offset) {
	return (int32_t) ((double) input * factor) + offset;
}
float map_u8_to_float(uint8_t input, float factor, float offset) {
	// Explicitly cast input to float for clarity in the calculation
	return ((float) input * factor) + offset;
}
