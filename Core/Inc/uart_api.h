#ifndef __UART_API_H
#define __UART_API_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"
#include <sys/_intsup.h>
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int  printf_init(UART_HandleTypeDef* puart_);
extern int  uart_send(char* ptr_, int len_);
extern int  register_rxqueue(QueueHandle_t const que_);
extern int  uart_rx_it_enable(void);
extern void PrintString(const char* format, ...);

#ifdef __cplusplus
}
#endif

#endif
