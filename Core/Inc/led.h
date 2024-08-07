#ifndef __LED_H
#define __LED_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED1   GPIOC, GPIO_PIN_7
#define LED2   GPIOB, GPIO_PIN_7
#define LED3   GPIOB, GPIO_PIN_14
#define ADCRDY GPIOB, GPIO_PIN_2

__STATIC_INLINE void LEDOn(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

__STATIC_INLINE void LEDOff(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

__STATIC_INLINE void LEDToggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}

__STATIC_INLINE void GPOOn(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

__STATIC_INLINE void GPOOff(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

__STATIC_INLINE void GPOToggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}

__STATIC_INLINE GPIO_PinState GetGPOState(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

#ifdef __cplusplus
}
#endif

#endif
