#include "FreeRTOSConfig.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "main.h"
#include "portmacro.h"
#include "projdefs.h"
#include "stm32l4xx_hal_spi.h"
#include "task.h"

#include "spi_slave.h"

#define BUFFERSIZE (sizeof(aTxBuffer) - 1)

#define SPITASK_STACK_SIZE (256)

static TaskHandle_t SpiTaskHandle;
static StackType_t  SPITaskBuffer[SPITASK_STACK_SIZE];
static StaticTask_t xSPITaskBuffer;

static SPI_HandleTypeDef* hspi = NULL;

/* Buffer used for transmission */
static uint8_t aTxBuffer[] = "****SPI - Two Boards communication based on DMA **** SPI Message ******** SPI Message ******** SPI Message ****";

/* Buffer used for reception */
static uint8_t aRxBuffer[BUFFERSIZE];

/* transfer state */
static volatile uint32_t wTransferState = TRANSFER_WAIT;

/* static function declaration */
static void SPISlaveTask(void* pvParameters);

/* function declaration */

int StartSPITask(SPI_HandleTypeDef* phspi_)
{
  int retval = 0;

  hspi = phspi_;

  SpiTaskHandle = xTaskCreateStatic(
      SPISlaveTask,
      "SPISlaveTask",
      SPITASK_STACK_SIZE,
      NULL,
      configMAX_PRIORITIES,
      SPITaskBuffer,
      &xSPITaskBuffer);

  if (NULL == SpiTaskHandle) {
    retval = -1;
  }

  return retval;
}

static void SPISlaveTask(void* pvParameters)
{
  (void)pvParameters;
  while (pdTRUE) {
    if (HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, BUFFERSIZE) != HAL_OK) {
      Error_Handler();
    }
    ulTaskNotifyTake(0, portMAX_DELAY);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
  (void)hspi;

  BaseType_t HigherPriorityTaskWoken = pdFALSE;

  vTaskNotifyGiveFromISR(SpiTaskHandle, &HigherPriorityTaskWoken);

  portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}
