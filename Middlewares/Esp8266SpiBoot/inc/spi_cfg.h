
#ifndef __SPI_CFG_H__
#define __SPI_CFG_H__

#include "stm32l4xx_hal.h"

void SpiHwIni(void);
uint8_t SpiProtocolIni(void);

HAL_StatusTypeDef STSpiRead(char *Buf, int size);
HAL_StatusTypeDef STSpiWrite(char *Buf, int size);

#endif
