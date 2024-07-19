#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"

/* external variable */

static UART_HandleTypeDef* puart = NULL;

void printf_init(UART_HandleTypeDef* puart_)
{
  puart = puart_;
}

#if 0
int __io_putchar(int ch_)
{
  uint8_t ch = ch_;
  return HAL_UART_Transmit(puart, &ch, 1U, 0xFFFFU);
}
#endif

int _write(int file_, char* ptr_, int len_)
{
  (void)file_;
  return HAL_UART_Transmit(puart, (uint8_t*)ptr_, len_, 0xFFFFU);
}
