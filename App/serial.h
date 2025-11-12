#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32u5xx_hal.h"
#include <stdarg.h>

extern UART_HandleTypeDef huart1;

void serial_print(const char *str);
void serial_println(const char *str);
int serial_printf(const char *format, ...);

#endif
