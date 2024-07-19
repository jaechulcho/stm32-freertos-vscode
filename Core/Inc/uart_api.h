#ifndef __UART_API_H
#define __UART_API_H

#include "stm32l4xx_hal_uart.h"
#ifdef __cplusplus
extern "C" {
#endif

extern void printf_init(UART_HandleTypeDef* puart_);

#ifdef __cplusplus
}
#endif

#endif
