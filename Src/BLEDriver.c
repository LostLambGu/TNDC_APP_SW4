
/*******************************************************************************
* File Name          : BLEDriver.c
* Author             : Yangjie Gu
* Description        : This file provides all the BLEDriver functions.

* History:
*  07/20/2018 : BLEDriver V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "BLEDriver.h"
#include "spi.h"
#include "fifo.h"
#include "uart_api.h"

/* Private define ------------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define BLE_DRIVER_LOG(format, ...) DebugPrintf(DbgCtl.BLEDriverDbgEn, "\r\n" format, ##__VA_ARGS__)

#define BLE_SPI_HANDLE hspi2

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SPI_HEADER_LEN  (uint8_t)(7)
#define SPI_CTRL_WRITE  (uint8_t)(0x0A)
#define SPI_CTRL_READ   (uint8_t)(0x0B)

#define FIFO_ALIGNMENT 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t bleCmdBuf[BLE_CMD_BUFFER_SIZE + BLE_SPI_ITEM_MAX_SIZE];
uint8_t bleEvtBuf[BLE_EVT_BUFFER_SIZE];
uint8_t bleRxTmpBuf[BLE_TMP_BUFFER_SIZE] = {0};
uint8_t bleTxTmpBuf[BLE_TMP_BUFFER_SIZE] = {0};
uint16_t bleTxTmpSize = 0;
uint16_t bleRxTmpSize = 0;
uint8_t bleHeaderBuf[SPI_HEADER_LEN + 1] = {0};
static uint8_t bleHeader[SPI_HEADER_LEN] = {0};
circular_fifo_t bleCmdFifo, bleEvtFifo;
uint8_t spiSendFlag = 0;
uint16_t bleEvtFifoLeftLen;
volatile SpiProtoType spi_proto_state = SPI_PROT_INIT_STATE;
uint8_t BleSpiBusError = 0;
/* Public variables ----------------------------------------------------------*/
extern volatile bool spi2txCpl;
extern volatile bool spi2txErr;

extern uint16_t Cal_CRC16(const uint8_t *data, uint32_t size);
HAL_StatusTypeDef BLESpiCheckCRC(uint8_t *data, uint16_t size, uint16_t crc16);
HAL_StatusTypeDef BLETransportReceiveHeader(uint8_t *header);
HAL_StatusTypeDef BLETransportSendHeader(uint16_t data_length, uint8_t *header);
HAL_StatusTypeDef BLETransportSendReceiveData(uint8_t *txData, uint8_t *rxData,uint16_t data_length);

void BLEDriverInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_GPIO_WritePin(GPIOB, PB0_BLE_IRQ_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOA, PA7_WAKE_BLE_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = PA7_WAKE_BLE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PB0_BLE_IRQ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PBPin PBPin PBPin */
    GPIO_InitStruct.Pin = PB12_BLE_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    MX_SPI2_Init();

    // HAL_GPIO_WritePin(GPIOA, PA7_WAKE_BLE_Pin, GPIO_PIN_SET);

    /* Queue index init */
    fifo_init(&bleCmdFifo, BLE_CMD_BUFFER_SIZE, bleCmdBuf, FIFO_ALIGNMENT);
    fifo_init(&bleEvtFifo, BLE_EVT_BUFFER_SIZE, bleEvtBuf, FIFO_ALIGNMENT);

    SPI_STATE_TRANSACTION(SPI_PROT_CONFIGURED_STATE);
}

void BLETransportTick(void)
{
    #define BLE_WAIT_HEADER_TIMEOUT_MS (200)
    #define BLE_WAIT_EVENT_PEND_TIMEOUT_MS (1000)
    #define BLE_WAIT_DATA_TIMEOUT_MS (5000)
    #define BLE_WAIT_COMPLETE_TIMEOUT_MS (100)
    static volatile uint32_t sysTickRec = 0;

    if (SPI_STATE_CHECK(SPI_PROT_CONFIGURED_HOST_REQ_STATE))
    {
        BLE_DRIVER_LOG("\r\nBLETransportTick PARSE_HOST_REQ");

        SPI_STATE_TRANSACTION(SPI_PROT_WAITING_HEADER_STATE);
        sysTickRec = HAL_GetTick() - 1;

        memset(bleHeaderBuf, 0, sizeof(bleHeaderBuf));

        if (BLETransportReceiveHeader(bleHeaderBuf))
        {
            BLE_DRIVER_LOG("BLETransportTick received header err");
            SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
            goto BLE_SPI_ERROR;
        }

        BLE_SPI_IRQ_PIN_SET(); /* Issue the SPI communication request */
    }
    else if ((fifo_size(&bleCmdFifo) > 0) && 
        (SPI_STATE_CHECK(SPI_PROT_CONFIGURED_STATE)
        || SPI_STATE_CHECK(SPI_PROT_SLEEP_STATE)))
    {
        bleTxTmpSize = 0;
        memset(bleTxTmpBuf, 0, sizeof(bleTxTmpBuf));

        if (fifo_get_var_len_item(&bleCmdFifo, &bleTxTmpSize, bleTxTmpBuf) == 0)
        {
            BLE_DRIVER_LOG("BLETransportTick PARSE_EVENT_PEND");

            SPI_STATE_TRANSACTION(SPI_PROT_CONFIGURED_EVENT_PEND_STATE);

            if (SPI_STATE_CHECK(SPI_PROT_CONFIGURED_EVENT_PEND_STATE) && 
                BLE_SPI_CS_READ() == GPIO_PIN_RESET)
            {
                SPI_STATE_TRANSACTION(SPI_PROT_WAITING_HEADER_STATE);
            }

            sysTickRec = HAL_GetTick() - 1;

            BLE_DRIVER_LOG("BLETransportSendHeader Size: (%d)", bleTxTmpSize);

            memset(bleHeaderBuf, 0, sizeof(bleHeaderBuf));
            
            if (BLETransportSendHeader(bleTxTmpSize, bleHeaderBuf))
            {
                BLE_DRIVER_LOG("BLETransportTick Send Header Err");
                SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                goto BLE_SPI_ERROR;
            }

            BLE_SPI_IRQ_PIN_SET(); /* Issue the SPI communication request */
        }
        else
        {
            BLE_DRIVER_LOG("BLETransportTick bleCmdFifo get item Err");
        }
    }

    while (SPI_STATE_CHECK(SPI_PROT_CONFIGURED_EVENT_PEND_STATE))
    {
        if (((HAL_GetTick() - sysTickRec) > BLE_WAIT_EVENT_PEND_TIMEOUT_MS))
        {
            BLE_DRIVER_LOG("BLETransportTick CONFIGURED_EVENT_PEND_STATE Timeout");
            SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
            goto BLE_SPI_ERROR;
        }
    }

    while (SPI_STATE_CHECK(SPI_PROT_WAITING_HEADER_STATE))
    {
        // BLE_DRIVER_LOG("BLETransportTick SPI_PROT_WAITING_HEADER_STATE");

        if (spi2txCpl == true)
        {
            spi2txCpl = false;

            sysTickRec = HAL_GetTick() - 1;

            if (BLEWaitSpiTransferCpl(BLE_WAIT_HEADER_TIMEOUT_MS, sysTickRec))
            {
                BLE_DRIVER_LOG("BLETransportTick WAITING_HEADER Complete TIMEOUT");
                SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                goto BLE_SPI_ERROR;
            }

            if (BLESpiCheckCRC(bleHeaderBuf, 5, (((uint16_t)bleHeaderBuf[6])<<8 | (uint16_t)bleHeaderBuf[5])))
            {
                BLE_DRIVER_LOG("BLETransportTick header crc err");
                SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                goto BLE_SPI_ERROR;
            }

            BLE_DRIVER_LOG("BLETransportTick HEADER_RECEIVED");

            SPI_STATE_TRANSACTION(SPI_PROT_WAITING_DATA_STATE);

            if (SPI_CTRL_READ == bleHeaderBuf[0])
            {
                spiSendFlag = SPI_CTRL_READ;
                if (bleTxTmpSize > 0)
                {
                    uint16_t bleLeftLen = (((uint16_t)bleHeaderBuf[2])<<8 | (uint16_t)bleHeaderBuf[1]);
                    if (bleLeftLen < bleTxTmpSize)
                    {
                        BLE_DRIVER_LOG("BLETransportTick no enough rom bleTxTmpSize: (%d) > bleLeftLen: (%d)", bleTxTmpSize, bleLeftLen);
                        SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                        goto BLE_SPI_ERROR;
                    }
                    else
                    {
                        memset(bleRxTmpBuf, 0, sizeof(bleRxTmpBuf));
                        BLE_DRIVER_LOG("BLETransportTick send data size: (%d)", bleTxTmpSize);
                        BLETransportSendReceiveData(bleTxTmpBuf, bleRxTmpBuf, bleTxTmpSize);
                    }
                }
                else
                {
                    BLE_DRIVER_LOG("BLETransportTick send conflict data size: (%d)", bleTxTmpSize);
                    SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                    goto BLE_SPI_ERROR;
                }
            }
            else if (SPI_CTRL_WRITE == bleHeaderBuf[0])
            {
                spiSendFlag = SPI_CTRL_WRITE;
                bleRxTmpSize = (((uint16_t)bleHeaderBuf[4])<<8 | (uint16_t)bleHeaderBuf[3]);
                if (bleRxTmpSize == 0)
                {
                    BLE_DRIVER_LOG("BLETransportTick size: (%d) data receive err", bleRxTmpSize);
                    SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                    goto BLE_SPI_ERROR;
                }
                else
                {
                    uint16_t bleEvtFifoLeftLen = fifo_left_size(&bleEvtFifo);
                    if (bleRxTmpSize > bleEvtFifoLeftLen)
                    {
                        BLE_DRIVER_LOG("BLETransportTick bleRxTmpSize: (%d) > bleEvtFifo left: (%d)", bleRxTmpSize
                            , bleEvtFifoLeftLen);
                        SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                        goto BLE_SPI_ERROR;
                    }
                    else
                    {
                        uint16_t crc16Tmp = 0;
                        memset(bleRxTmpBuf, 0, sizeof(bleRxTmpBuf));
                        memset(bleTxTmpBuf, 0x6a, sizeof(bleTxTmpBuf));
                        crc16Tmp = Cal_CRC16(bleTxTmpBuf, bleRxTmpSize - 2);
                        bleTxTmpBuf[bleRxTmpSize - 2] = (uint8_t)crc16Tmp;
                        bleTxTmpBuf[bleRxTmpSize - 1] = (uint8_t)(crc16Tmp >> 8);
                        BLE_DRIVER_LOG("BLETransportTick receive data size: (%d)", bleRxTmpSize);
                        BLETransportSendReceiveData(bleTxTmpBuf, bleRxTmpBuf, bleRxTmpSize);
                    }
                }
            }
            else
            {
                BLE_DRIVER_LOG("BLETransportTick header first data(%x) err", bleHeaderBuf[0]);
                SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                goto BLE_SPI_ERROR;
            }

            sysTickRec = HAL_GetTick() - 1;

            BLE_DRIVER_LOG("BLETransportTick SPI_PROT_WAITING_DATA");

            BLE_SPI_IRQ_PIN_RESET(); /* Issue the SPI communication request */

            break;
        }

        /* wait 2 second */
        if (((HAL_GetTick() - sysTickRec) > BLE_WAIT_HEADER_TIMEOUT_MS)
        || (spi2txErr == true))
        {
            if (spi2txErr == true)
            {
                BLE_DRIVER_LOG("BLETransportTick HEADER spi2txErr");
            }

            spi2txErr = false;

            BLE_DRIVER_LOG("BLETransportTick HEADER_NOT_RECEIVED");

            SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);

            goto BLE_SPI_ERROR;
        }
    }
    
    while (SPI_STATE_CHECK(SPI_PROT_WAITING_DATA_STATE))
    {
        if (((HAL_GetTick() - sysTickRec) > BLE_WAIT_DATA_TIMEOUT_MS))
        {
            BLE_DRIVER_LOG("BLETransportTick SPI_PROT_WAITING_DATA_STATE Timeout");
            SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
            goto BLE_SPI_ERROR;
        }
    }

    while (SPI_STATE_CHECK(SPI_PROT_TRANS_COMPLETE_STATE))
    {
        if (spi2txCpl == true)
        {
            spi2txCpl = false;

            if (BLEWaitSpiTransferCpl(BLE_WAIT_HEADER_TIMEOUT_MS, sysTickRec))
            {
                BLE_DRIVER_LOG("BLETransportTick WAITING_COMPLETE TIMEOUT");
                SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                goto BLE_SPI_ERROR;
            }

            if (spiSendFlag == SPI_CTRL_READ)
            {
                uint16_t crc16Tmp = (((uint16_t)bleRxTmpBuf[bleTxTmpSize - 1]) << 8) | (uint16_t)bleRxTmpBuf[bleTxTmpSize - 2];
                
                if (BLESpiCheckCRC(bleRxTmpBuf, bleTxTmpSize - 2, crc16Tmp))
                {
                    BLE_DRIVER_LOG("BLETransportTick send data check crc fail size(%d)", bleTxTmpSize);
                    SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                    goto BLE_SPI_ERROR;
                }
                else
                {
                    BLE_DRIVER_LOG("BLETransportTick send data success size(%d)", bleTxTmpSize);
                }

                // if ((((uint16_t)bleRxTmpBuf[bleTxTmpSize - 1])<<8 | (uint16_t)bleRxTmpBuf[bleTxTmpSize - 2]) == bleTxTmpSize)
                // {
                //     BLE_DRIVER_LOG("BLETransportTick send data success size(%d)", bleTxTmpSize);
                // }
                // else
                // {
                //     BLE_DRIVER_LOG("BLETransportTick send data fail size(%d)", bleTxTmpSize);
                //     SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                //     goto BLE_SPI_ERROR;
                // }

                BLE_DRIVER_LOG("BLETransportTick bleCmdFifo left size(%d)", fifo_left_size(&bleCmdFifo));

                if (fifo_discard_var_len_item(&bleCmdFifo))
                {
                    BLE_DRIVER_LOG("BLETransportTick discard bleCmdFifo fail");
                    SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                    goto BLE_SPI_ERROR;
                }

                BLE_DRIVER_LOG("BLETransportTick bleCmdFifo after discard left size(%d)", fifo_left_size(&bleCmdFifo));
            }
            else if (spiSendFlag == SPI_CTRL_WRITE)
            {
                if (BLESpiCheckCRC(bleRxTmpBuf, bleRxTmpSize - 2, (((uint16_t)bleRxTmpBuf[bleRxTmpSize - 1]) << 8 | (uint16_t)bleRxTmpBuf[bleRxTmpSize - 2])))
                {
                    BLE_DRIVER_LOG("BLETransportTick receive data crc err");
                    SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);
                    goto BLE_SPI_ERROR;
                }

                BLE_DRIVER_LOG("BLETransportTick receive data success size(%d)", bleRxTmpSize);

                if (fifo_put_var_len_item(&bleEvtFifo, bleRxTmpSize - 2, bleRxTmpBuf))
                {
                    BLE_DRIVER_LOG("BLETransportTick receive data put bleEvtFifo err");
                }

                BLE_DRIVER_LOG("BLETransportTick bleEvtFifo left size(%d)", fifo_left_size(&bleEvtFifo));

                // BLE_DRIVER_LOG("%s\r\n", bleRxTmpBuf);
            }

            spiSendFlag = 0;
            bleTxTmpSize = 0;
            bleRxTmpSize = 0;

            SPI_STATE_TRANSACTION(SPI_PROT_CONFIGURED_STATE);

            BleSpiBusError = 0;
            break;
        }

        if (((HAL_GetTick() - sysTickRec) > BLE_WAIT_COMPLETE_TIMEOUT_MS)
        || (spi2txErr == true))
        {
            if (spi2txErr == true)
            {
                BLE_DRIVER_LOG("BLETransportTick TRANS_COMPLETE_STATE Err");
            }
            else
            {
                BLE_DRIVER_LOG("BLETransportTick TRANS_COMPLETE_STATE timeout");
            }
            spi2txErr = false;

            SPI_STATE_TRANSACTION(SPI_PROT_TRANS_ERROR_STATE);

            goto BLE_SPI_ERROR;
        }
    }

    if (SPI_STATE_CHECK(SPI_PROT_TRANS_ERROR_STATE))
    {
BLE_SPI_ERROR:
        BLE_DRIVER_LOG("BLETransportTick TRANS_ERROR_STATE");

        // BLE_DRIVER_LOG("1->FTLVL: %x", BLE_SPI_HANDLE.Instance->SR & SPI_FLAG_FTLVL);
        // BLE_DRIVER_LOG("1->FRLVL: %x", (BLE_SPI_HANDLE.Instance->SR & SPI_FLAG_FRLVL));
        // BLE_DRIVER_LOG("1->BSY: %x", __HAL_SPI_GET_FLAG(&BLE_SPI_HANDLE, SPI_FLAG_BSY));

        spi2txErr = false;
        spi2txCpl = false;
        spiSendFlag = 0;
        bleTxTmpSize = 0;
        bleRxTmpSize = 0;

        SPI2_Recover();

        HAL_Delay(150);

        BLE_SPI_IRQ_PIN_RESET(); /* Issue the SPI communication request */

        SPI_STATE_TRANSACTION(SPI_PROT_CONFIGURED_STATE);

        if (BleSpiBusError == 0)
        {
            BleSpiBusError = 1;
            if (fifo_size(&bleCmdFifo) > 0)
            {
                BLE_DRIVER_LOG("BLETransportTick Error State bleCmdFifo > 0");
            }
            else
            {
                BLE_DRIVER_LOG("BLETransportTick BLEBusErrorQuery");
                extern uint8_t BLEBusErrorQuery(void);
                BLEBusErrorQuery();
            }
        }
        // else
        // {
        //     BLE_DRIVER_LOG("BLETransportTick HAL_Delay");
        //     HAL_Delay(500);
        // }
    }
}

HAL_StatusTypeDef BLETransportReceiveHeader(uint8_t *header)
{
  uint16_t crc16 = 0;

  memset(bleHeader, 0, sizeof(bleHeader));
   
//   bleEvtFifoLeftLen = (bleEvtFifo.max_size - fifo_size(&bleEvtFifo));
  bleEvtFifoLeftLen = fifo_left_size(&bleEvtFifo);
  
  bleHeader[0] = 0xff;
  bleHeader[1] = (uint8_t)bleEvtFifoLeftLen;
  bleHeader[2] = (uint8_t)(bleEvtFifoLeftLen>>8);
  bleHeader[3] = 0;
  bleHeader[4] = 0;

  crc16 = Cal_CRC16(bleHeader, 5);
  bleHeader[5] = (uint8_t)crc16;
  bleHeader[6] = (uint8_t)(crc16>>8);

  return HAL_SPI_TransmitReceive_DMA(&BLE_SPI_HANDLE, bleHeader, header, SPI_HEADER_LEN);
}

HAL_StatusTypeDef BLETransportSendHeader(uint16_t data_length, uint8_t *header)
{
  uint16_t crc16 = 0;

  memset(bleHeader, 0, sizeof(bleHeader));
  
//   bleEvtFifoLeftLen = (bleEvtFifo.max_size - fifo_size(&bleEvtFifo));
  bleEvtFifoLeftLen = fifo_left_size(&bleEvtFifo);
  
  bleHeader[0] = 0xff;
  bleHeader[1] = (uint8_t)bleEvtFifoLeftLen;
  bleHeader[2] = (uint8_t)(bleEvtFifoLeftLen>>8);
  bleHeader[3] = (uint8_t)data_length;
  bleHeader[4] = (uint8_t)(data_length>>8);

  crc16 = Cal_CRC16(bleHeader, 5);
  bleHeader[5] = (uint8_t)crc16;
  bleHeader[6] = (uint8_t)(crc16>>8);

  return HAL_SPI_TransmitReceive_DMA(&BLE_SPI_HANDLE, bleHeader, header, SPI_HEADER_LEN);
}

HAL_StatusTypeDef BLETransportSendReceiveData(uint8_t *txData, uint8_t *rxData,uint16_t data_length)
{
    return HAL_SPI_TransmitReceive_DMA(&BLE_SPI_HANDLE, txData, rxData, data_length);
}

HAL_StatusTypeDef BLEWaitSpiTransferCpl(uint32_t timeout, uint32_t tickStart)
{
    while (1)
    {
        if (((BLE_SPI_HANDLE.Instance->SR & SPI_FLAG_FTLVL) == SPI_FTLVL_EMPTY)
         && ((BLE_SPI_HANDLE.Instance->SR & SPI_FLAG_FRLVL) == SPI_FRLVL_EMPTY)
         && (!__HAL_SPI_GET_FLAG(&BLE_SPI_HANDLE, SPI_FLAG_BSY)))
        {
            break;
        }

        if (((BLE_SPI_HANDLE.Instance->SR & SPI_FLAG_FTLVL) == SPI_FTLVL_EMPTY)
         && (!__HAL_SPI_GET_FLAG(&BLE_SPI_HANDLE, SPI_FLAG_BSY)))
        {
            if ((BLE_SPI_HANDLE.Instance->SR & SPI_FLAG_FRLVL) != SPI_FRLVL_EMPTY)
            {
                __IO uint8_t tmpreg;
                tmpreg = *((__IO uint8_t *)BLE_SPI_HANDLE.Instance->DR);
                /* To avoid GCC warning */
                UNUSED(tmpreg);
            }
        }

        if ((timeout == 0U) || ((HAL_GetTick() - tickStart) >= timeout))
        {
            return HAL_TIMEOUT;
        }
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef BLESpiCheckCRC(uint8_t *data, uint16_t size, uint16_t crc16)
{
    if (Cal_CRC16(data, size) == crc16)
    {
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
