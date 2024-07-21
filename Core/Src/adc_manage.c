#include <stdio.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "led.h"
#include "portmacro.h"
#include "stm32l4xx_hal_tim.h"
#include "task.h"
#include "adc_manage.h"
#include "myerror.h"

// defines
#define ADC_RESULT_SIZE    (4)
#define ADCTASK_STACK_SIZE (256)

// external
extern void              Error_Handler(void);
extern TIM_HandleTypeDef htim3;

// private variables
static uint16_t           ADCResult[ADC_RESULT_SIZE];
static ADC_HandleTypeDef* hmyadc;
static TaskHandle_t       ADCTaskHandle;
static StackType_t        ADCTaskBuffer[ADCTASK_STACK_SIZE];
static StaticTask_t       xADCTaskBuffer;

// private functions
static void ADCTask(void* pvParameters);

int MyADCInit(ADC_HandleTypeDef* hadc)
{
  hmyadc = hadc;

  return ERR_NONE;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  BaseType_t HigherPriorityTaskWoken = pdFALSE;

  if ((NULL != hmyadc) && (hadc == hmyadc)) {
    // HAL_ADC_Start_DMA(hmyadc, (uint32_t*)ADCResult, 1U);
    LEDToggle(LED2);
    vTaskNotifyGiveFromISR(ADCTaskHandle, &HigherPriorityTaskWoken);
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
      configMAX_PRIORITIES - 5,
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

  HAL_ADC_Start_DMA(hmyadc, (uint32_t*)ADCResult, 1U);

  HAL_TIM_Base_Start(&htim3);

  while (pdTRUE) {
    ulTaskNotifyTake(0, portMAX_DELAY);
    printf("%u\r\n", (int)ADCResult[0]);
  }
}
