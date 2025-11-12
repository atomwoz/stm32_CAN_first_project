#include "serial.h"
#include <string.h>
#include <stdio.h>

void serial_print(const char *str)
{
  if (str != NULL)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
  }
}

void serial_println(const char *str)
{
  if (str != NULL)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
  }
  serial_print("\r\n");
}

int serial_printf(const char *format, ...)
{
  char buffer[256];
  va_list args;

  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (len > 0 && len < (int)sizeof(buffer))
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, HAL_MAX_DELAY);
  }

  return len;
}
