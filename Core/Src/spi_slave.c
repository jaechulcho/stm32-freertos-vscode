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
  HAL_StatusTypeDef retapi;
  // HAL_SPI_StateTypeDef retspi;

  while (pdTRUE) {
    // memory to memory transfer complete waiting
    ulTaskNotifyTake(0, portMAX_DELAY);
    PrintString("SPITask[%06u]: ", spi_error_cnt);
    for (int i = 0; i < BUFFERSIZE / 2; i++) {
      PrintString("%04x ", (int)((uint16_t*)aTxBuffer)[i]);
    }
    PrintString("\r\n");

    // retspi = HAL_SPI_GetState(hspi);
    // if (HAL_SPI_STATE_READY != retspi) {
    //   (void)HAL_SPI_DMAStop(hspi);
    // }
    retapi = HAL_SPI_TransmitReceive_DMA(
        hspi,
        aTxBuffer,
        aRxBuffer,
        BUFFERSIZE);
    if (HAL_OK == retapi) {
      LEDToggle(LED3);
      LEDOn(ADCRDY);
      LEDOff(ADCRDY);
      // spi transfer complete waiting
      ulTaskNotifyTake(0, portMAX_DELAY);
    } else {
      PrintString("HAL_SPI_TransmitReceive_DMA error\r\n");
    }
  }
}

// spi dma transfer complete callback
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi_)
{
  BaseType_t HigherPriorityTaskWoken = pdFALSE;

  if (hspi_ == hspi) {
    vTaskNotifyGiveFromISR(SpiTaskHandle, &HigherPriorityTaskWoken);
    spi_error_cnt++;
  }

  portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

// memory dma transfer complet callback
static void XferCpltCallback(DMA_HandleTypeDef* hdma)
{
  BaseType_t HigherPriorityTaskWoken = pdFALSE;

  if (hdma == &hdma_memtomem_dma1_channel5) {
    vTaskNotifyGiveFromISR(SpiTaskHandle, &HigherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}
