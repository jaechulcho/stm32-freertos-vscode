#include <stdio.h>
#include "FreeRTOSConfig.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_dma.h"
#include "stm32l4xx_hal_spi.h"
#include "task.h"
#include "led.h"
#include "uart_api.h"

#include "spi_slave.h"

#define BUFFERSIZE 16

#define SPITASK_STACK_SIZE (256)

// extern objects
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel5;
extern void              Error_Handler(void);

// private variables
static TaskHandle_t SpiTaskHandle;
static StackType_t  SPITaskBuffer[SPITASK_STACK_SIZE];
static StaticTask_t xSPITaskBuffer;

static SPI_HandleTypeDef* hspi = NULL;

static volatile uint32_t spi_error_cnt;
static volatile uint32_t spi_trxcmplt_cnt;

/* Buffer used for transmission */
static uint8_t aTxBuffer[BUFFERSIZE];

/* Buffer used for reception */
static uint8_t aRxBuffer[BUFFERSIZE];

/* transfer state */
// static volatile uint32_t wTransferState = TRANSFER_WAIT;

/* static function declaration */
static void SPISlaveTask(void* pvParameters);
static void XferCpltCallback(DMA_HandleTypeDef* hdma);

/* function declaration */
uint32_t GetSPITXBuf(void)
{
  uint32_t addr;

  addr = (uint32_t)(&aTxBuffer[0]);

  return addr;
}

int StartSPITask(SPI_HandleTypeDef* phspi_)
{
  int retval = 0;

  hspi = phspi_;

  hdma_memtomem_dma1_channel5.XferCpltCallback = XferCpltCallback;

  SpiTaskHandle = xTaskCreateStatic(
      SPISlaveTask,
      "SPISlaveTask",
      SPITASK_STACK_SIZE,
      NULL,
      configMAX_PRIORITIES - 5,
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
  uint32_t uloopcnt     = 0UL;
  uint32_t uloopcntprev = 0UL;

  while (pdTRUE) {
    // memory to memory transfer complete waiting
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if ((uloopcnt & 0x400UL) && ((uloopcntprev & 0x400UL) == 0UL)) {
      LEDToggle(LED3);
    }
    uloopcntprev = uloopcnt;
    uloopcnt += 1UL;
  }
}

// spi dma transfer complete callback
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi_)
{
  BaseType_t HigherPriorityTaskWoken = pdFALSE;

  if (hspi_ == hspi) {
    vTaskNotifyGiveFromISR(SpiTaskHandle, &HigherPriorityTaskWoken);
    spi_trxcmplt_cnt += 1UL;
  }

  portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

// memory dma transfer complet callback
static void XferCpltCallback(DMA_HandleTypeDef* hdma)
{
  BaseType_t           HigherPriorityTaskWoken = pdFALSE;
  HAL_StatusTypeDef    retapi;
  HAL_SPI_StateTypeDef retspi;

  if (hdma == &hdma_memtomem_dma1_channel5) {
    retspi = HAL_SPI_GetState(hspi);
    if (HAL_SPI_STATE_READY != retspi) {
      (void)HAL_SPI_DMAStop(hspi);
      spi_error_cnt += 1UL;
    }
    retapi = HAL_SPI_TransmitReceive_DMA(
        hspi,
        aTxBuffer,
        aRxBuffer,
        BUFFERSIZE);
    if (HAL_OK == retapi) {
      GPOOn(ADCRDY);
      GPOOff(ADCRDY);
    }
  }

  portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}
