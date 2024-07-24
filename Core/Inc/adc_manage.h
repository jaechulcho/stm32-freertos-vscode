#ifndef __ADC_MANAGE_H
#define __ADC_MANAGE_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_adc.h"
#include <sys/_intsup.h>
#include "myerror.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t get_adc_error_cnt(void);
extern int      MyADCInit(ADC_HandleTypeDef* hadc);
extern int      ADCTaskStart(ADC_HandleTypeDef* hadc);

#ifdef __cplusplus
}
#endif

#endif
