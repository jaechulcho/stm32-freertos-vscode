#include "FreeRTOS.h"
#include "portmacro.h"
#include "semphr.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_uart.h"

/* external variable */

static UART_HandleTypeDef* puart = NULL;
static StaticSemaphore_t   xUartSemaphorBuffer;
static SemaphoreHandle_t   huartsema = NULL;
static StaticSemaphore_t   xUartMutexBuffer;
static SemaphoreHandle_t   huartmutex = NULL;

int printf_init(UART_HandleTypeDef* puart_)
{
  int retval = 0;
  puart      = puart_;
  huartsema  = xSemaphoreCreateBinaryStatic(&xUartSemaphorBuffer);
  if (NULL == huartsema) {
    retval = -1;
  }
  if (retval == 0) {
    huartmutex = xSemaphoreCreateMutexStatic(&xUartMutexBuffer);
    if (NULL == huartmutex) {
      retval = -2;
    }
  }

  return retval;
}

int _write(int file_, char* ptr_, int len_)
{
  (void)file_;
  BaseType_t        state;
  HAL_StatusTypeDef retval;

  state = xTaskGetSchedulerState();

  if (taskSCHEDULER_NOT_STARTED == state) {
    retval = HAL_UART_Transmit(puart, (uint8_t*)ptr_, (uint16_t)len_, 0xFFFFU);
    if (retval != HAL_OK) {
      len_ = 0;
    }
  } else {

    xSemaphoreTake(huartmutex, portMAX_DELAY);

    retval = HAL_UART_Transmit_DMA(puart, (uint8_t*)ptr_, (uint16_t)len_);
    if (retval != HAL_OK) {
      len_ = 0;
    } else {
      xSemaphoreTake(huartsema, portMAX_DELAY);
    }

    xSemaphoreGive(huartmutex);
  }
  return len_;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
  (void)huart;
  BaseType_t xHigherPriorityWoken = pdFALSE;

  xSemaphoreGiveFromISR(huartsema, &xHigherPriorityWoken);
  portYIELD_FROM_ISR(xHigherPriorityWoken);
}
