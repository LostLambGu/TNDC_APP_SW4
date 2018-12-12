/*******************************************************************************
* File Name          : WifiDriver.c
* Author             : Yangjie Gu
* Description        : This file provides all the WifiDriver functions.

* History:
*  07/18/2018 : WifiDriver V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "WifiDriver.h"
#include "spi.h"
#include "fifo.h"
#include "uart_api.h"
#include "WifiProcess.h"

/* Private define ------------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define WIFI_STARTUP_MODE_NORMAL (0)
#define WIFI_STARTUP_MODE_SPI (1)
#define WIFI_STARTUP_MODE WIFI_STARTUP_MODE_NORMAL

#define WIFI_DRIVER_LOG(format, ...) DebugPrintf(DbgCtl.WifiDriverDbgEn, "\r\n" format, ##__VA_ARGS__)

#define WIFI_SPI_HANDLE hspi3

volatile WifiSpiProtoType wifi_spi_proto_state;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WIFI_SPI_HEADER_LEN  (uint8_t)(6)
#define WSHC_LOW_B (4) // WIFI SPI HEADER CRC16 LOW BYTE
#define WSHC_HIGH_B (5) // WIFI SPI HEADER CRC16 HIGH BYTE
#define WIFI_SPI_CTRL_WRITE  (uint8_t)(0x02)
#define WIFI_SPI_CTRL_READ   (uint8_t)(0x03)

#define WIFI_PACK_ELEMENT_LEN (32)
#define WIFI_DATA_LEN_ALIGNMENT (4)
#define WIFI_INSERT_VALUE (0xFF)
#define WIFI_INSERT_VALUE_LEN (2)

#define WIFI_TMP_BUFFER_SIZE (WIFI_MAX_ITEM_SIZE + WIFI_INSERT_VALUE_LEN * (WIFI_MAX_ITEM_SIZE / WIFI_PACK_ELEMENT_LEN + 1))

uint8_t wifiHeaderBuf[WIFI_SPI_HEADER_LEN + 1] = {0};
static uint8_t wifiHeader[WIFI_SPI_HEADER_LEN] = {0};
uint8_t wifiRxTmpBuf[WIFI_TMP_BUFFER_SIZE] = {0};
uint8_t wifiTxTmpBuf[WIFI_TMP_BUFFER_SIZE] = {0};
uint16_t wifiTxTmpSize = 0;
uint16_t wifiTxTmpSizeAfterInsert = 0;
uint16_t wifiTxTmpSizeInsertFlag = 0;
uint16_t wifiRxTmpSize = 0;
uint16_t wifiRxTmpSizeAfterInsert = 0;
uint8_t wifiSpiSendFlag = 0;
uint8_t wifiSpiBusError = 0;
uint16_t wifiEvtFifoLeftLen;

uint16_t wifiSpiCsLowCount = 0;
uint16_t wifiSpiCsLowCountOld = 0;
uint16_t wifiSpiCsHighCount = 0;
uint16_t wifiSpiCsHighCountOld = 0;

extern circular_fifo_t wifiCmdFifo, wifiHiPrioCmdFifo, wifiEvtFifo;
extern volatile uint8_t txEspSpiCpl;
extern volatile uint8_t txEspSpiErr;
extern uint16_t Cal_CRC16(const uint8_t *data, uint32_t size);
HAL_StatusTypeDef WifiSpiCheckCRC(uint8_t *data, uint16_t size, uint16_t crc16);
HAL_StatusTypeDef WifiTransportReceiveHeader(uint8_t *header);
HAL_StatusTypeDef WifiTransportSendHeader(uint16_t data_length, uint8_t *header);
HAL_StatusTypeDef WifiTransportSendReceiveData(uint8_t *txData, uint8_t *rxData,uint16_t data_length);
HAL_StatusTypeDef WifiWaitSpiTransferCpl(uint32_t timeout, uint32_t tickStart);

uint16_t WifiGetLenAfterInsert(uint16_t len);
uint16_t WifiGetLenAlignment(uint16_t len);
uint16_t WifiGetLenAfterDelInsert(uint16_t len);
void WifiInsertValue(uint8_t *data, uint16_t len);
void WifiInsertAlignmentValue(uint8_t *data, uint16_t len, uint16_t lenAlign);
void WifiDelInsertValue(uint8_t *data, uint16_t len);

volatile uint8_t wifiRestart = 0;

/* Function definition -------------------------------------------------------*/
void WifiInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_GPIO_WritePin(WIFI_RESET_PORT, WIFI_RESET_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WIFI_CHIP_EN_PORT, WIFI_CHIP_EN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WIFI_FLASH_MODE_PORT, WIFI_FLASH_MODE_PIN, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(WIFI_MTDO_GPIO15_PORT, WIFI_MTDO_GPIO15_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = WIFI_RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WIFI_RESET_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = WIFI_CHIP_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WIFI_CHIP_EN_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = WIFI_FLASH_MODE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WIFI_FLASH_MODE_PORT, &GPIO_InitStruct);

    // GPIO_InitStruct.Pin = WIFI_MTDO_GPIO15_PIN;
    // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    // HAL_GPIO_Init(WIFI_MTDO_GPIO15_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = WIFI_MTDO_GPIO15_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(WIFI_MTDO_GPIO15_PORT, &GPIO_InitStruct);
}

void WifiEnterFlashDownLoadMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    WifiInit();

    HAL_GPIO_WritePin(WIFI_MTDO_GPIO15_PORT, WIFI_MTDO_GPIO15_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = WIFI_MTDO_GPIO15_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WIFI_MTDO_GPIO15_PORT, &GPIO_InitStruct);

    WIFI_FLASH_MODE_DOWNLOAD();
    WIFI_MTDO_GPIO15_LOW();
    WIFI_CHIP_DISABLE();
    WIFI_RESET();
    WIFI_CHIP_ENABLE();
    HAL_Delay(100);

    GPIO_InitStruct.Pin = WIFI_MTDO_GPIO15_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(WIFI_MTDO_GPIO15_PORT, &GPIO_InitStruct);
}

void WifiNormalStartUp(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    // GPIO_InitStruct.Pin = WIFI_MTDO_GPIO15_PIN;
    // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // HAL_GPIO_Init(WIFI_MTDO_GPIO15_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = WIFI_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WIFI_IRQ_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = WIFI_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(WIFI_CS_PORT, &GPIO_InitStruct);

    extern void WIFIProcessInit(void);
    WIFIProcessInit();

    HAL_SPI_Abort(&hspi3);
    HAL_SPI_DeInit(&hspi3);
    HAL_Delay(100);

    MX_SPI3_Init();

    WifiInit();
    // WIFI_CHIP_DISABLE();
    WIFI_FLASH_MODE_RUN();
    // WIFI_MTDO_GPIO15_HIGH();
    HAL_Delay(1000);
    WIFI_CHIP_ENABLE();
    WIFI_RESET();

    WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_CONFIGURED_STATE);
}

void WifiSdioStartUp(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    WifiInit();
    
    HAL_GPIO_WritePin(WIFI_MTDO_GPIO15_PORT, WIFI_MTDO_GPIO15_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = WIFI_MTDO_GPIO15_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WIFI_MTDO_GPIO15_PORT, &GPIO_InitStruct);
    
    // WIFI_CHIP_DISABLE();
    WIFI_FLASH_MODE_DOWNLOAD();
    WIFI_MTDO_GPIO15_HIGH();
    WIFI_CHIP_DISABLE();
    WIFI_RESET();
    WIFI_CHIP_ENABLE();
    HAL_Delay(100);
    GPIO_InitStruct.Pin = WIFI_MTDO_GPIO15_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(WIFI_MTDO_GPIO15_PORT, &GPIO_InitStruct);
    
    extern uint8_t Esp8266SpiBoot(void);
    extern void WIFIProcessInit(void);
    if (0 == Esp8266SpiBoot())
    {
        WIFIProcessInit();
        HAL_Delay(3000);
        extern void SdioRW(void *pvParameters);
        SdioRW(NULL);
    }
}

void WifiStartUp(void)
{
    #if (WIFI_STARTUP_MODE == WIFI_STARTUP_MODE_SPI)
    WifiSdioStartUp();
    #elif (WIFI_STARTUP_MODE == WIFI_STARTUP_MODE_NORMAL)
    WifiNormalStartUp();
    #endif /* WIFI_STARTUP_MODE */
}

void WifiStateTrans(void)
{
    /* CS pin rising edge - close SPI communication */
    if (wifiSpiCsHighCountOld != wifiSpiCsHighCount)
    {
        wifiSpiCsHighCountOld = wifiSpiCsHighCount;
        // DEBUG_NOTES(GPIO_CS_RISING);
        if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_WAITING_DATA_STATE))
        {
            WIFI_IRQ_SET_LOW();

            WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_COMPLETE_STATE);
        }
    }
    /* CS pin falling edge - start SPI communication */
    else if (wifiSpiCsLowCountOld != wifiSpiCsLowCount)
    {
        wifiSpiCsLowCountOld = wifiSpiCsLowCount;
        // DEBUG_NOTES(GPIO_CS_FALLING);
        if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_CONFIGURED_EVENT_PEND_STATE))
        {
            WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_WAITING_HEADER_STATE);
        }
        else if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_SLEEP_STATE) || WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_CONFIGURED_STATE))
        {
            // if (WIFI_SPI_WAKEUP_PIN_READ() != GPIO_PIN_RESET) // To prevent MCU Spi state go wrong
            {
                WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_CONFIGURED_HOST_REQ_STATE);
            }
        }
    }
}

void WifiTransportTick(void)
{
    #define WIFI_WAIT_HEADER_TIMEOUT_MS (200)
    #define WIFI_WAIT_EVENT_PEND_TIMEOUT_MS (1000)
    #define WIFI_WAIT_DATA_TIMEOUT_MS (5000)
    #define WIFI_WAIT_COMPLETE_TIMEOUT_MS (100)
    static volatile uint32_t sysTickRec = 0;

    if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_CONFIGURED_HOST_REQ_STATE))
    {
        WIFI_DRIVER_LOG("\r\nWifiTransportTick PARSE_HOST_REQ");

        WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_WAITING_HEADER_STATE);
        sysTickRec = HAL_GetTick() - 1;

        memset(wifiHeaderBuf, 0, sizeof(wifiHeaderBuf));

        if (WifiTransportReceiveHeader(wifiHeaderBuf))
        {
            WIFI_DRIVER_LOG("WifiTransportTick received header err");
            WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
            goto WIFI_SPI_ERROR;
        }

        WIFI_IRQ_SET_HIGH(); /* Issue the SPI communication request */
    }
    else if ((WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_CONFIGURED_STATE)
        || WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_SLEEP_STATE)))
    {
        if (wifiTxTmpSize == 0)
        {
            memset(wifiTxTmpBuf, 0, sizeof(wifiTxTmpBuf));

            if (0 == WIFIGetCmdAndSize(wifiTxTmpBuf, &wifiTxTmpSize))
            {
            }
            else
            {
                // WIFI_DRIVER_LOG("WifiTransportTick wifiCmdFifo did not get item");
            }
        }
        
        if (wifiTxTmpSize != 0)
        {
            WIFI_DRIVER_LOG("WifiTransportTick PARSE_EVENT_PEND");

            WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_CONFIGURED_EVENT_PEND_STATE);

            if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_CONFIGURED_EVENT_PEND_STATE) &&
                WIFI_CS_PIN_READ() == GPIO_PIN_RESET)
            {
                WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_WAITING_HEADER_STATE);
            }

            sysTickRec = HAL_GetTick() - 1;

            WIFI_DRIVER_LOG("WifiTransportSendHeader Size: (%d)", wifiTxTmpSize);

            memset(wifiHeaderBuf, 0, sizeof(wifiHeaderBuf));

            if (WifiTransportSendHeader(wifiTxTmpSize, wifiHeaderBuf))
            {
                WIFI_DRIVER_LOG("WifiTransportTick Send Header Err");
                WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
                goto WIFI_SPI_ERROR;
            }

            WIFI_IRQ_SET_HIGH(); /* Issue the SPI communication request */
        }
    }

    if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_CONFIGURED_EVENT_PEND_STATE))
    {
        if (((HAL_GetTick() - sysTickRec) > WIFI_WAIT_EVENT_PEND_TIMEOUT_MS))
        {
            WIFI_DRIVER_LOG("WifiTransportTick CONFIGURED_EVENT_PEND_STATE Timeout");
            WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
            goto WIFI_SPI_ERROR;
        }
    }

    if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_WAITING_HEADER_STATE))
    {
        // WIFI_DRIVER_LOG("WifiTransportTick SPI_PROT_WAITING_HEADER_STATE");

        static uint8_t waitWifiHeaderComplete = 0;

        if (txEspSpiCpl == TRUE)
        {
            txEspSpiCpl = FALSE;

            waitWifiHeaderComplete = 1;

            sysTickRec = HAL_GetTick() - 1;
        }
        else if (waitWifiHeaderComplete == 1)
        {
            if (((WIFI_SPI_HANDLE.Instance->SR & SPI_FLAG_FTLVL) == SPI_FTLVL_EMPTY) 
            && ((WIFI_SPI_HANDLE.Instance->SR & SPI_FLAG_FRLVL) == SPI_FRLVL_EMPTY) 
            && (!__HAL_SPI_GET_FLAG(&WIFI_SPI_HANDLE, SPI_FLAG_BSY)))
            {
                waitWifiHeaderComplete = 0;
                WIFI_DRIVER_LOG("WifiTransportTick HEADER_RECEIVED");
                WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_HEADER_RECEIVED_STATE);
            }
        }

        /* wait 2 second */
        if (((HAL_GetTick() - sysTickRec) > WIFI_WAIT_HEADER_TIMEOUT_MS)
        || (txEspSpiErr == TRUE))
        {
            if (txEspSpiErr == TRUE)
            {
                WIFI_DRIVER_LOG("WifiTransportTick HEADER txEspSpiErr");
            }

            txEspSpiErr = FALSE;

            waitWifiHeaderComplete = 0;

            WIFI_DRIVER_LOG("WifiTransportTick HEADER_NOT_RECEIVED");

            WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);

            goto WIFI_SPI_ERROR;
        }
    }

    if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_HEADER_RECEIVED_STATE))
    {
        WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_WAITING_DATA_STATE);

        if (WIFI_SPI_CTRL_READ == wifiHeaderBuf[0])
        {
            wifiSpiSendFlag = WIFI_SPI_CTRL_READ;
            if (wifiTxTmpSize > 0)
            {
                if (wifiTxTmpSizeInsertFlag == 0)
                {
                    uint16_t tempSize = 0;
                    wifiTxTmpSizeInsertFlag = 1;
                    WifiInsertValue(wifiTxTmpBuf, wifiTxTmpSize);
                    wifiTxTmpSizeAfterInsert = WifiGetLenAfterInsert(wifiTxTmpSize);
                    tempSize = wifiTxTmpSizeAfterInsert;
                    wifiTxTmpSizeAfterInsert += WifiGetLenAlignment(wifiTxTmpSize);
                    WifiInsertAlignmentValue(wifiTxTmpBuf, tempSize, wifiTxTmpSizeAfterInsert);
                    // WIFI_DRIVER_LOG("WifiTransportTick send data inserted size: (%d)", wifiTxTmpSizeAfterInsert);
                }
                WIFI_DRIVER_LOG("WifiTransportTick send data size: (%d) inserted: (%d)", wifiTxTmpSize, wifiTxTmpSizeAfterInsert);
                WifiTransportSendReceiveData(wifiTxTmpBuf, wifiRxTmpBuf, wifiTxTmpSizeAfterInsert);

                WIFI_DRIVER_LOG("WifiTransportTick send data:\r\n");
                for (uint16_t i = 0; i < wifiTxTmpSizeAfterInsert; i++)
                {
                    DebugPrintf(DbgCtl.WifiDriverDbgEn, "0x%x ", wifiTxTmpBuf[i]);
                }
                WIFI_DRIVER_LOG("\r\n");
            }
            else
            {
                WIFI_DRIVER_LOG("WifiTransportTick send conflict data size: (%d)", wifiTxTmpSize);
                WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
                goto WIFI_SPI_ERROR;
            }
        }
        else if (WIFI_SPI_CTRL_WRITE == wifiHeaderBuf[0])
        {
            if (WifiSpiCheckCRC(wifiHeaderBuf + 2, 2, (((uint16_t)wifiHeaderBuf[5]) << 8 | (uint16_t)wifiHeaderBuf[4])))
            {
                WIFI_DRIVER_LOG("WifiTransportTick header crc err");
                WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
                goto WIFI_SPI_ERROR;
            }

            wifiSpiSendFlag = WIFI_SPI_CTRL_WRITE;
            wifiRxTmpSize = (((uint16_t)wifiHeaderBuf[3]) << 8 | (uint16_t)wifiHeaderBuf[2]);
            if (wifiRxTmpSize == 0)
            {
                WIFI_DRIVER_LOG("WifiTransportTick size: (%d) data receive err", wifiRxTmpSize);
                WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
                goto WIFI_SPI_ERROR;
            }
            else
            {
                uint16_t wifiEvtFifoLeftLen = fifo_left_size(&wifiEvtFifo);
                if (wifiRxTmpSize > wifiEvtFifoLeftLen)
                {
                    WIFI_DRIVER_LOG("WifiTransportTick wifiRxTmpSize: (%d) > wifiEvtFifo left: (%d)", wifiRxTmpSize, wifiEvtFifoLeftLen);
                    WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
                    goto WIFI_SPI_ERROR;
                }
                else
                {
                    memset(wifiRxTmpBuf, 0xff, sizeof(wifiRxTmpBuf));
                    wifiRxTmpSizeAfterInsert = WifiGetLenAfterInsert(wifiRxTmpSize);
                    wifiRxTmpSizeAfterInsert += WifiGetLenAlignment(wifiRxTmpSize);
                    WIFI_DRIVER_LOG("WifiTransportTick receive data size: (%d) after insert: (%d)", wifiRxTmpSize, wifiRxTmpSizeAfterInsert);
                    WifiTransportSendReceiveData(wifiRxTmpBuf, wifiRxTmpBuf, wifiRxTmpSizeAfterInsert);
                }
            }
        }
        else
        {
            WIFI_DRIVER_LOG("WifiTransportTick header first data(%x) err", wifiHeaderBuf[0]);
            WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
            goto WIFI_SPI_ERROR;
        }

        sysTickRec = HAL_GetTick() - 1;

        WIFI_DRIVER_LOG("WifiTransportTick SPI_PROT_WAITING_DATA tick(%d)", sysTickRec);

        WIFI_IRQ_SET_LOW(); /* Issue the SPI communication request */
    }
    
    if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_WAITING_DATA_STATE))
    {
        if (((HAL_GetTick() - sysTickRec) > WIFI_WAIT_DATA_TIMEOUT_MS))
        {
            WIFI_DRIVER_LOG("WifiTransportTick SPI_PROT_WAITING_DATA_STATE Timeout");
            WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
            goto WIFI_SPI_ERROR;
        }
    }

    if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_TRANS_COMPLETE_STATE))
    {
        sysTickRec = HAL_GetTick() - 1;
        WIFI_DRIVER_LOG("WifiTransportTick SPI_PROT_TRANS_COMPLETE_STATE tick(%d)", sysTickRec);
        WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_WAIT_COMPLETE_STATE);
    }

    if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_TRANS_WAIT_COMPLETE_STATE))
    {
        static uint8_t waitWifiDataComplete = 0;

        if (txEspSpiCpl == TRUE)
        {
            txEspSpiCpl = FALSE;

            waitWifiDataComplete = 1;
        }
        else if (waitWifiDataComplete == 1)
        {
            if (((WIFI_SPI_HANDLE.Instance->SR & SPI_FLAG_FTLVL) == SPI_FTLVL_EMPTY) 
            && ((WIFI_SPI_HANDLE.Instance->SR & SPI_FLAG_FRLVL) == SPI_FRLVL_EMPTY) 
            && (!__HAL_SPI_GET_FLAG(&WIFI_SPI_HANDLE, SPI_FLAG_BSY)))
            {
                waitWifiDataComplete = 0;

                if (wifiSpiSendFlag == WIFI_SPI_CTRL_READ)
                {
                    WIFI_DRIVER_LOG("WifiTransportTick wifiCmdFifo left size(%d) tick(%d)", fifo_left_size(&wifiCmdFifo), HAL_GetTick());

                    // if (fifo_discard_var_len_item(&wifiCmdFifo))
                    // {
                    //     WIFI_DRIVER_LOG("WifiTransportTick discard wifiCmdFifo fail");
                    //     WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
                    //     goto WIFI_SPI_ERROR;
                    // }

                    // WIFI_DRIVER_LOG("WifiTransportTick wifiCmdFifo after discard left size(%d)", fifo_left_size(&wifiCmdFifo));

                    wifiTxTmpSize = 0;
                    wifiTxTmpSizeInsertFlag = 0;
                }
                else if (wifiSpiSendFlag == WIFI_SPI_CTRL_WRITE)
                {
                    WifiDelInsertValue(wifiRxTmpBuf, wifiRxTmpSizeAfterInsert);

                    if (WifiSpiCheckCRC(wifiRxTmpBuf, wifiRxTmpSize - 2, (((uint16_t)wifiRxTmpBuf[wifiRxTmpSize - 1]) << 8 | (uint16_t)wifiRxTmpBuf[wifiRxTmpSize - 2])))
                    {
                        WIFI_DRIVER_LOG("WifiTransportTick receive data crc err");
                        WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);
                        goto WIFI_SPI_ERROR;
                    }

                    WIFI_DRIVER_LOG("WifiTransportTick receive data success size(%d)", wifiRxTmpSize);

                    if (fifo_put_var_len_item(&wifiEvtFifo, wifiRxTmpSize - 2, wifiRxTmpBuf))
                    {
                        WIFI_DRIVER_LOG("WifiTransportTick receive data put wifiEvtFifo err");
                    }

                    WIFI_DRIVER_LOG("WifiTransportTick wifiEvtFifo left size(%d)", fifo_left_size(&wifiEvtFifo));

                    // WIFI_DRIVER_LOG("%s\r\n", wifiRxTmpBuf);

                    wifiRxTmpSize = 0;
                }

                wifiSpiSendFlag = 0;

                WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_CONFIGURED_STATE);

                wifiSpiBusError = 0;
            }
        }

        if (((HAL_GetTick() - sysTickRec) > WIFI_WAIT_COMPLETE_TIMEOUT_MS)
        || (txEspSpiErr == TRUE))
        {
            if (txEspSpiErr == TRUE)
            {
                WIFI_DRIVER_LOG("WifiTransportTick TRANS_COMPLETE_STATE Err");
            }
            else
            {
                WIFI_DRIVER_LOG("WifiTransportTick TRANS_COMPLETE_STATE timeout tick(%d)", HAL_GetTick());
            }
            txEspSpiErr = FALSE;

            waitWifiDataComplete = 0;

            WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_TRANS_ERROR_STATE);

            goto WIFI_SPI_ERROR;
        }
    }

    if (WIFI_SPI_STATE_CHECK(WIFI_SPI_PROT_TRANS_ERROR_STATE))
    {
WIFI_SPI_ERROR:
        WIFI_DRIVER_LOG("WifiTransportTick TRANS_ERROR_STATE");

        // WIFI_DRIVER_LOG("1->FTLVL: %x", WIFI_SPI_HANDLE.Instance->SR & SPI_FLAG_FTLVL);
        // WIFI_DRIVER_LOG("1->FRLVL: %x", (WIFI_SPI_HANDLE.Instance->SR & SPI_FLAG_FRLVL));
        // WIFI_DRIVER_LOG("1->BSY: %x", __HAL_SPI_GET_FLAG(&WIFI_SPI_HANDLE, SPI_FLAG_BSY));

        txEspSpiErr = FALSE;
        txEspSpiCpl = FALSE;
        wifiSpiSendFlag = 0;

        SPI3_Recover();

        HAL_Delay(150);

        WIFI_IRQ_SET_LOW(); /* Issue the SPI communication request */

        WIFI_SPI_STATE_TRANSACTION(WIFI_SPI_PROT_CONFIGURED_STATE);

        if (wifiSpiBusError == 0)
        {
            wifiSpiBusError = 1;
            if (wifiTxTmpSize > 0)
            {
                WIFI_DRIVER_LOG("WifiTransportTick Error State wifiTxTmpSize(%d) > 0", wifiTxTmpSize);
            }
            else
            {
                WIFI_DRIVER_LOG("WifiTransportTick WifiBusErrorQuery");
                extern uint8_t WifiBusErrorQuery(void);
                WifiBusErrorQuery();
            }
        }
        else if (wifiSpiBusError == 1)
        {
            if (wifiTxTmpSize > 0)
            {
                WIFI_DRIVER_LOG("WifiTransportTick Error State wifiTxTmpSize(%d) > 0", wifiTxTmpSize);
            }
        }
        // else
        // {
        //     WIFI_DRIVER_LOG("WifiTransportTick HAL_Delay");
        //     HAL_Delay(500);
        // }
    }
}

HAL_StatusTypeDef WifiTransportReceiveHeader(uint8_t *header)
{
  memset(wifiHeader, 0xff, sizeof(wifiHeader));

  return HAL_SPI_TransmitReceive_DMA(&WIFI_SPI_HANDLE, wifiHeader, header, WIFI_SPI_HEADER_LEN);
}

HAL_StatusTypeDef WifiTransportSendHeader(uint16_t data_length, uint8_t *header)
{
  uint16_t crc16 = 0;

  memset(wifiHeader, 0xff, sizeof(wifiHeader));
  
  wifiHeader[2] = (uint8_t)data_length;
  wifiHeader[3] = (uint8_t)(data_length>>8);

  crc16 = Cal_CRC16(wifiHeader + 2, 2);
  wifiHeader[4] = (uint8_t)crc16;
  wifiHeader[5] = (uint8_t)(crc16>>8);

  return HAL_SPI_TransmitReceive_DMA(&WIFI_SPI_HANDLE, wifiHeader, header, WIFI_SPI_HEADER_LEN);
}

HAL_StatusTypeDef WifiTransportSendReceiveData(uint8_t *txData, uint8_t *rxData,uint16_t data_length)
{
    return HAL_SPI_TransmitReceive_DMA(&WIFI_SPI_HANDLE, txData, rxData, data_length);
}

#if (!1)
HAL_StatusTypeDef WifiWaitSpiTransferCpl(uint32_t timeout, uint32_t tickStart)
{
    while (1)
    {
        if (((WIFI_SPI_HANDLE.Instance->SR & SPI_FLAG_FTLVL) == SPI_FTLVL_EMPTY)
         && ((WIFI_SPI_HANDLE.Instance->SR & SPI_FLAG_FRLVL) == SPI_FRLVL_EMPTY)
         && (!__HAL_SPI_GET_FLAG(&WIFI_SPI_HANDLE, SPI_FLAG_BSY)))
        {
            break;
        }

        if (((WIFI_SPI_HANDLE.Instance->SR & SPI_FLAG_FTLVL) == SPI_FTLVL_EMPTY)
         && (!__HAL_SPI_GET_FLAG(&WIFI_SPI_HANDLE, SPI_FLAG_BSY)))
        {
            if ((WIFI_SPI_HANDLE.Instance->SR & SPI_FLAG_FRLVL) != SPI_FRLVL_EMPTY)
            {
                __IO uint8_t tmpreg;
                tmpreg = *((__IO uint8_t *)WIFI_SPI_HANDLE.Instance->DR);
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
#endif //#if (!1)

HAL_StatusTypeDef WifiSpiCheckCRC(uint8_t *data, uint16_t size, uint16_t crc16)
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

uint16_t WifiGetLenAfterInsert(uint16_t len)
{
    uint16_t count = len / WIFI_PACK_ELEMENT_LEN;

    if (len % WIFI_PACK_ELEMENT_LEN)
    {
        count++;
    }

    return (len + count * WIFI_INSERT_VALUE_LEN);
}

uint16_t WifiGetLenAlignment(uint16_t len)
{
    uint16_t lenTmp = ((len + WIFI_DATA_LEN_ALIGNMENT - 1) / WIFI_DATA_LEN_ALIGNMENT) * WIFI_DATA_LEN_ALIGNMENT;
    return (lenTmp - len);
}

uint16_t WifiGetLenAfterDelInsert(uint16_t len)
{
    uint16_t count = len / (WIFI_PACK_ELEMENT_LEN + WIFI_INSERT_VALUE_LEN);

    if (len % (WIFI_PACK_ELEMENT_LEN + WIFI_INSERT_VALUE_LEN))
    {
        count++;
    }

    return (len - count * WIFI_INSERT_VALUE_LEN);
}

void WifiInsertValue(uint8_t *data, uint16_t len)
{
    uint16_t count = len / WIFI_PACK_ELEMENT_LEN;
    uint16_t left = len % WIFI_PACK_ELEMENT_LEN;
    uint16_t i = 0;
    uint16_t index = len;

    if (left)
    {
        for (i = 0; i < left; i++)
        {
            index--;
            data[index + WIFI_INSERT_VALUE_LEN * (count + 1)] = data[index];
        }

        for (i = 0; i < WIFI_INSERT_VALUE_LEN; i++)
        {
            data[index + WIFI_INSERT_VALUE_LEN * (count + 1) - i - 1] = WIFI_INSERT_VALUE;
        }
    }

    while (count)
    {
        for (i = 0; i < WIFI_PACK_ELEMENT_LEN ; i++)
        {
            index--;
            data[index + WIFI_INSERT_VALUE_LEN * count] = data[index];
        }

        for (i = 0; i < WIFI_INSERT_VALUE_LEN; i++)
        {
            data[index + WIFI_INSERT_VALUE_LEN * count - i - 1] = WIFI_INSERT_VALUE;
        }

        count--;
    }
}

void WifiInsertAlignmentValue(uint8_t *data, uint16_t len, uint16_t lenAlign)
{
    uint16_t i = 0;

    for (i = len; i < lenAlign; i++)
    {
        data[i] = WIFI_INSERT_VALUE;
    }
}

void WifiDelInsertValue(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    uint16_t count = 1;
    uint16_t index = 0;

    while (len >= (WIFI_PACK_ELEMENT_LEN + WIFI_INSERT_VALUE_LEN))
    {
        for (i = 0; i < WIFI_PACK_ELEMENT_LEN; i++)
        {
            data[index] = data[index + WIFI_INSERT_VALUE_LEN * count];
            index++;
        }
        len -= (WIFI_PACK_ELEMENT_LEN + WIFI_INSERT_VALUE_LEN);
        count++;
    }

    if (len)
    {
        for (i = 0; i < (len - WIFI_INSERT_VALUE_LEN); i++)
        {
            data[index] = data[index + WIFI_INSERT_VALUE_LEN * count];
            index++;
        }
    }
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
