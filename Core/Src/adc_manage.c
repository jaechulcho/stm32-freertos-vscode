#include <stdio.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "led.h"
#include "portmacro.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_dma.h"
#include "stm32l4xx_hal_tim.h"
#include "task.h"
#include "adc_manage.h"
#include "myerror.h"
#include "spi_slave.h"
#include "uart_api.h"

// defines
#define ADC_RESULT_SIZE    (8)
#define ADCTASK_STACK_SIZE (256)

// external
extern void              Error_Handler(void);
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel5;

// private variables
static volatile uint32_t  adc_error_cnt;
static uint16_t           ADCResult[ADC_RESULT_SIZE];
static ADC_HandleTypeDef* hmyadc;
static TaskHandle_t       ADCTaskHandle;
static StackType_t        ADCTaskBuffer[ADCTASK_STACK_SIZE];
static StaticTask_t       xADCTaskBuffer;

// private functions
static void ADCTask(void* pvParameters);

// definition
uint32_t get_adc_error_cnt(void)
{
  return adc_error_cnt;
}

int MyADCInit(ADC_HandleTypeDef* hadc)
{
  hmyadc = hadc;

  return ERR_NONE;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  HAL_StatusTypeDef retapi;
  BaseType_t        HigherPriorityTaskWoken = pdFALSE;

  if ((NULL != hadc) && (hadc == hmyadc)) {
    // HAL_ADC_Start_DMA(hmyadc, (uint32_t*)ADCResult, 1U);
    LEDToggle(LED2);
    vTaskNotifyGiveFromISR(ADCTaskHandle, &HigherPriorityTaskWoken);
    // MEMTOMEM DMA
    retapi = HAL_DMA_Start_IT(
        &hdma_memtomem_dma1_channel5,
        (uint32_t)(ADCResult),
        GetSPITXBuf(),
        sizeof(ADCResult) / sizeof(uint32_t));
    if (HAL_ERROR == retapi) {
      adc_error_cnt++;
    }
  }
  portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

int ADCTaskStart(ADC_HandleTypeDef* hadc)
{
  int retval = ERR_NONE;

  hmyadc = hadc;

  ADCTaskHandle = xTaskCreateStatic(
      ADCTask,
      "ADCTask",
      ADCTASK_STACK_SIZE,
      NULL,
      configMAX_PRIORITIES - 4,
      ADCTaskBuffer,
      &xADCTaskBuffer);

  if (NULL == ADCTaskHandle) {
    retval = -1;
  }

  return retval;
}

static void ADCTask(void* pvParameters)
{
  (void)pvParameters;

  if (hmyadc == NULL) {
    Error_Handler();
  }

  HAL_ADC_Start_DMA(hmyadc, (uint32_t*)ADCResult, ADC_RESULT_SIZE);

  HAL_TIM_Base_Start(&htim3);

  while (pdTRUE) {
    ulTaskNotifyTake(0, portMAX_DELAY);
    // PrintString("ADCTask[%06u]: ", adc_error_cnt);
    // for (int i = 0; i < ADC_RESULT_SIZE; i++) {
    //   PrintString("%04x ", (int)ADCResult[i]);
    // }
    // PrintString("\r\n");
  }
}
