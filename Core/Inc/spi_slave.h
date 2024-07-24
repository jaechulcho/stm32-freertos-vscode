#ifndef __SPI_SLAVE_H
#define __SPI_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"

enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR,
};

extern int      StartSPITask(SPI_HandleTypeDef* phspi_);
extern uint32_t GetSPITXBuf(void);

#ifdef __cplusplus
}
#endif

#endif
