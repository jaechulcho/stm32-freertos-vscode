#include "FreeRTOS.h"
#include "portmacro.h"
#include "semphr.h"
#include "main.h"
#include "stm32l4r5xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_uart.h"
#include "uart_api.h"
#include "myerror.h"

#define MAX_RXQUE_SIZE (1)
/* external variable */

/* interal variables */

// for tasks using uart rx data
static uint8_t       rxData;
static int           curr_rxque_index = 0;
static QueueHandle_t rxquelist[MAX_RXQUE_SIZE];

static UART_HandleTypeDef* puart = NULL;
static StaticSemaphore_t   xUartSemaphorBuffer;
static SemaphoreHandle_t   huarttxsema = NULL;
static StaticSemaphore_t   xUartMutexBuffer;
static SemaphoreHandle_t   huartmutex = NULL;

int printf_init(UART_HandleTypeDef* puart_)
{
  int retval = 0;
  puart      = puart_;

  huarttxsema = xSemaphoreCreateBinaryStatic(&xUartSemaphorBuffer);
  if (NULL == huarttxsema) {
    retval = ERR_CREATESEMA_FAIL;
  }
  if (retval == 0) {
    huartmutex = xSemaphoreCreateMutexStatic(&xUartMutexBuffer);
    if (NULL == huartmutex) {
      retval = ERR_CREATEMUTEX_FAIL;
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

    (void)xSemaphoreTake(huartmutex, portMAX_DELAY);

    retval = HAL_UART_Transmit_DMA(puart, (uint8_t*)ptr_, (uint16_t)len_);
    if (retval != HAL_OK) {
      len_ = 0;
    } else {
      (void)xSemaphoreTake(huarttxsema, portMAX_DELAY);
    }

    (void)xSemaphoreGive(huartmutex);
  }
  return len_;
}

/*
should check the option that LPUART1 global interrupt in NVIC Settings
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
  BaseType_t xHigherPriorityWoken = pdFALSE;

  if (huart->Instance == LPUART1) {
    (void)xSemaphoreGiveFromISR(huarttxsema, &xHigherPriorityWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityWoken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  BaseType_t xHigherPriorityWoken = pdFALSE;

  if (huart->Instance == LPUART1) {
    for (int i = 0; i < MAX_RXQUE_SIZE; i++) {
      xQueueSendToBackFromISR(rxquelist[i], &rxData, &xHigherPriorityWoken);
    }
  }
  (void)uart_rx_it_enable();
  portYIELD_FROM_ISR(xHigherPriorityWoken);
}

int register_rxqueue(QueueHandle_t que_)
{
  int retval = ERR_NONE;

  if (curr_rxque_index < MAX_RXQUE_SIZE) {
    rxquelist[curr_rxque_index] = que_;
    curr_rxque_index++;
  } else {
    retval = ERR_UARTRXQUEUE_FULL;
  }

  return retval;
}

int uart_rx_it_enable(void)
{
  int retval = ERR_NONE;

  if (puart == NULL) {
    retval = ERR_INVALID_UARTPORT;
  }
  if (retval == ERR_NONE) {
    /* enable rx interrupt */
    (void)HAL_UART_Receive_IT(puart, &rxData, 1U);
  }

  return retval;
}
