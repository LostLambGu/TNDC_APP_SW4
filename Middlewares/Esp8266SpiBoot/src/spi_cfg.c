#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "spi.h"
#include "gpio.h"
#include "spi_cfg.h"
#include "uart_api.h"

#define SPI_CFG_LOG(format, ...) DebugPrintf(1, "\r\n" format, ##__VA_ARGS__)

#define ESP_SPI_TRANSFER_TIMEOUT_MS (5000)

void SpiHwIni(void)
{
        GPIO_InitTypeDef GPIO_InitStruct;

        HAL_SPI_Abort(&hspi3);
        HAL_SPI_DeInit(&hspi3);
        HAL_Delay(100);

        MX_SPI3_Init();

        HAL_GPIO_WritePin(GPIOA, PA4_SPI3_NSS_Pin, GPIO_PIN_RESET);

        GPIO_InitStruct.Pin = PA4_SPI3_NSS_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

extern volatile uint8_t txEspSpiCpl;
extern volatile uint8_t txEspSpiErr;

HAL_StatusTypeDef STSpiRead(char *Buf, int size)
{
        uint32_t sysTickRec = 0;

        txEspSpiCpl = 0;
        HAL_StatusTypeDef Status = HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)Buf, (uint8_t *)Buf, size);
        if (HAL_OK != Status)
        {
                SPI_CFG_LOG("STSpiRead ERROR(%d)\r\n", Status);
        }

        sysTickRec = HAL_GetTick();

        while (txEspSpiCpl == 0 && txEspSpiErr == 0)
        {
                if ((HAL_GetTick() - sysTickRec) > ESP_SPI_TRANSFER_TIMEOUT_MS)
                {
                        SPI_CFG_LOG("STSpiRead timeout\r\n");
                        return HAL_TIMEOUT;
                }
        }

        while (1)
        {
                if (((hspi3.Instance->SR & SPI_FLAG_FTLVL) == SPI_FTLVL_EMPTY) 
                        && ((hspi3.Instance->SR & SPI_FLAG_FRLVL) == SPI_FRLVL_EMPTY) 
                        && (!__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY)))
                {
                        break;
                }

                if ((HAL_GetTick() - sysTickRec) > ESP_SPI_TRANSFER_TIMEOUT_MS)
                {
                        SPI_CFG_LOG("STSpiRead timeout1\r\n");
                        return HAL_TIMEOUT;
                }
        }

        if (txEspSpiErr == 1)
        {
                txEspSpiErr = 0;
                HAL_SPI_Abort(&hspi3);
                HAL_SPI_DeInit(&hspi3);
                HAL_Delay(100);
                MX_SPI3_Init();
                SPI_CFG_LOG("STSpiRead ERR\r\n", Status);
        }

        Status = HAL_SPI_DMAStop(&hspi3);

        return Status;
}

HAL_StatusTypeDef STSpiWrite(char *Buf, int size)
{
        uint32_t sysTickRec = 0;
  
        txEspSpiCpl = 0;
        HAL_StatusTypeDef Status = HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)Buf, (uint8_t *)Buf, size);
        if (HAL_OK != Status)
        {
                SPI_CFG_LOG("STSpiWrite ERROR(%d)\r\n", Status);
        }

        sysTickRec = HAL_GetTick();

        while (txEspSpiCpl == 0 && txEspSpiErr == 0)
        {
                if ((HAL_GetTick() - sysTickRec) > ESP_SPI_TRANSFER_TIMEOUT_MS)
                {
                        SPI_CFG_LOG("STSpiWrite timeout\r\n");
                        return HAL_TIMEOUT;
                }
        }

        while (1)
        {
                if (((hspi3.Instance->SR & SPI_FLAG_FTLVL) == SPI_FTLVL_EMPTY) 
                        && ((hspi3.Instance->SR & SPI_FLAG_FRLVL) == SPI_FRLVL_EMPTY) 
                        && (!__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY)))
                {
                        break;
                }

                if ((HAL_GetTick() - sysTickRec) > ESP_SPI_TRANSFER_TIMEOUT_MS)
                {
                        SPI_CFG_LOG("STSpiWrite timeout1\r\n");
                        return HAL_TIMEOUT;
                }
        }

        if (txEspSpiErr == 1)
        {
                txEspSpiErr = 0;
                HAL_SPI_Abort(&hspi3);
                HAL_SPI_DeInit(&hspi3);
                HAL_Delay(100);
                MX_SPI3_Init();
                SPI_CFG_LOG("STSpiWrite ERR\r\n", Status);
        }

        Status = HAL_SPI_DMAStop(&hspi3);

        return Status;
}
