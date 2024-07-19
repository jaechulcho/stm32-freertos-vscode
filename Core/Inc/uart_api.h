#ifndef __UART_API_H
#define __UART_API_H

#include "stm32l4xx_hal_uart.h"
#include <sys/_intsup.h>
#ifdef __cplusplus
extern "C" {
#endif

extern int printf_init(UART_HandleTypeDef* puart_);

#ifdef __cplusplus
}
#endif

#endif
