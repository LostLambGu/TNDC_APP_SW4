/*******************************************************************************
* File Name          : WifiProcess.c
* Author             : Yangjie Gu
* Description        : This file provides all the WifiProcess functions.

* History:
*  08/31/2018 : WifiProcess V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "fifo.h"
#include "uart_api.h"

#include "WifiDriver.h"
#include "WifiProcess.h"

/* Private define ------------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define WIFI_PROCESS_LOG(format, ...) DebugPrintf(DbgCtl.WifiProcessDbgEn, "\r\n" format, ##__VA_ARGS__)

#define FIFO_ALIGNMENT 1

#define WIFI_CMD_HEADER_CRC16_LEN (sizeof(WIFICmdPackHeaderTypeDef) + sizeof(uint16_t))
#define WIFI_CMD_CRC16_LEN (sizeof(uint16_t))

#define WIFI_EVT_HEADER_CRC16_LEN (sizeof(WIFIEvtPackHeaderTypeDef) + sizeof(uint16_t))
#define WIFI_EVT_HEADER_LEN (sizeof(WIFIEvtPackHeaderTypeDef))

#define WIFI_CMD_TMP_BUFFER_SIZE (WIFI_MAX_ITEM_SIZE + 32)
#define WIFI_EVT_TMP_BUFFER_SIZE (WIFI_MAX_ITEM_SIZE + 32)
#define WIFI_CMD_BUFFER_SIZE (2 * 1024)
#define WIFI_EVT_BUFFER_SIZE (2 * 1024)

extern uint16_t httpRspRec;
extern uint16_t httpAckRec;

circular_fifo_t wifiCmdFifo, wifiHiPrioCmdFifo, wifiEvtFifo;
uint8_t wifiCmdBuf[WIFI_CMD_BUFFER_SIZE];
uint8_t wifiHiPrioCmdBuf[WIFI_CMD_BUFFER_SIZE];
uint8_t wifiEvtBuf[WIFI_EVT_BUFFER_SIZE];

WIFITypeDef WIFIRecord;

extern uint16_t Cal_CRC16(const uint8_t *data, uint32_t size);
HAL_StatusTypeDef WIFIRecDataCheckCRC(uint8_t *data, uint16_t size, uint16_t crc16);
uint8_t WIFIEvtDataProcess(uint16_t evt, uint8_t *data, uint16_t dataLen);
uint8_t WIFIEvtResponseProces(uint16_t evt, uint8_t *data, uint16_t dataLen);

uint8_t WIFIPutDataInFifo(uint16_t dataLen, uint8_t *data);
uint8_t WIFIPutDataInHiPrioFifo(uint16_t dataLen, uint8_t *data);
uint8_t WIFISendCmd(WIFICmdTypeDef cmd, uint8_t *data, uint16_t dataLen, uint8_t HiPrio);

const char *WIFICmdStrArray[WIFI_CMD_MAX] = 
{
    [WIFI_CMD_RAW_DATA] = "WIFI_CMD_RAW_DATA",
    [WIFI_CMD_SET_SSID_KEY] = "WIFI_CMD_SET_SSID_KEY",
    [WIFI_CMD_GET_IPV4_ADDR] = "WIFI_CMD_GET_IPV4_ADDR",
    [WIFI_CMD_SPI_BUS_ERROR_QUERY] = "WIFI_CMD_SPI_BUS_ERROR_QUERY",
    [WIFI_RSP_STATION_STATE] = "WIFI_RSP_STATION_STATE",
    [WIFI_CMD_HTTP_POST] = "WIFI_CMD_HTTP_POST",
    [WIFI_RSP_HTTP_POST_STATE] = "WIFI_RSP_HTTP_POST_STATE",
    [WIFI_CMD_HTTP_POST_SEND_DATA] = "WIFI_CMD_HTTP_POST_SEND_DATA",
    [WIFI_RSP_HTTP_POST_FINISH] = "WIFI_RSP_HTTP_POST_FINISH",
    [WIFI_RSP_HTTP_POST_ERROR] = "WIFI_RSP_HTTP_POST_ERROR",
    [WIFI_CMD_HTTP_GET] = "WIFI_CMD_HTTP_GET",
};

void WIFIProcessInit(void)
{
    memset(&WIFIRecord, 0, sizeof(WIFIRecord));

    /* Queue index init */
    fifo_init(&wifiCmdFifo, WIFI_CMD_BUFFER_SIZE, wifiCmdBuf, FIFO_ALIGNMENT);
    fifo_init(&wifiHiPrioCmdFifo, WIFI_CMD_BUFFER_SIZE, wifiHiPrioCmdBuf, FIFO_ALIGNMENT);
    fifo_init(&wifiEvtFifo, WIFI_EVT_BUFFER_SIZE, wifiEvtBuf, FIFO_ALIGNMENT);
}

/* Wifi evt process ----------------------------------------------------------*/
uint8_t WIFIEvtFifoAvailable(void)
{
    if (fifo_left_size(&wifiEvtFifo) > (WIFI_MAX_ITEM_SIZE + WIFI_CMD_HEADER_CRC16_LEN))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t WIFIEvtProcess(void)
{
    uint8_t tmpEvtBuf[WIFI_EVT_TMP_BUFFER_SIZE];
    uint16_t size = fifo_size(&wifiEvtFifo);
    uint16_t evt = 0;
    uint16_t len = 0;

    if (size == 0)
    {
        return 0;
    }

    memset(tmpEvtBuf, 0, sizeof(tmpEvtBuf));

    if (fifo_get_var_len_item(&wifiEvtFifo, &len, tmpEvtBuf))
    {
        WIFI_PROCESS_LOG("WIFIPutEvtProcess get evt err");
        return 1;
    }

    evt = *((uint16_t *)(&tmpEvtBuf[2]));

    if (WIFIEvtDataProcess(evt, tmpEvtBuf + WIFI_EVT_HEADER_LEN, len - WIFI_EVT_HEADER_CRC16_LEN))
    {
        WIFI_PROCESS_LOG("WIFIPutEvtProcess evt(0x%x) rsplen(%d) process err", evt, len);
        return 2;
    }

    if (fifo_discard_var_len_item(&wifiEvtFifo))
    {
        WIFI_PROCESS_LOG("WIFIPutEvtProcess evt discard err");
        return 3;
    }

    return 0;
}

uint8_t WIFIEvtDataProcess(uint16_t evt, uint8_t *data, uint16_t dataLen)
{
    if ((evt & WIFI_CMD_RESPONSE_MASK) != 0)
    {
        if (WIFIEvtResponseProces(evt, data, dataLen))
        {
            WIFI_PROCESS_LOG("WIFIEvtDataProcess response err");
            return 1;
        }
        else
        {
            return 0;
        }
    }

    switch (evt)
    {
        case WIFI_RSP_STATION_STATE:
        WIFI_PROCESS_LOG("WIFIEvtDataProcess STATION_STATE");
        break;

        case WIFI_RSP_HTTP_POST_STATE:
        httpRspRec = *((uint16_t *)data);
        WIFI_PROCESS_LOG("WIFIEvtDataProcess HTTP_POST_STATE(%d)", httpRspRec);
        break;

        case WIFI_RSP_HTTP_POST_FINISH:
        httpAckRec = 2;
        WIFI_PROCESS_LOG("WIFIEvtDataProcess HTTP_POST_FINISH");
        break;

        case WIFI_RSP_HTTP_POST_ERROR:
        httpAckRec = 3;
        WIFI_PROCESS_LOG("WIFIEvtDataProcess HTTP_POST_ERROR");
        break;

        default:
        WIFI_PROCESS_LOG("WIFIEvtDataProcess evt err");
        return 2;
        // break;
    }
    return 0;
}

uint8_t WIFIEvtResponseProces(uint16_t evt, uint8_t *data, uint16_t dataLen)
{
    uint16_t len = *((uint16_t *)(data));
    evt = evt & (~WIFI_CMD_RESPONSE_MASK);

    if (evt == WIFI_CMD_HTTP_POST_SEND_DATA)
    {
        httpAckRec = 1;
    }

    if ((evt & WIFI_CMD_HI_PRIO_MASK) != 0)
    {
        if ((WIFIRecord.cmdHiPrioAck.cmd == evt) && (WIFIRecord.cmdHiPrioAck.len == len))
        {
            WIFIRecord.cmdHiPrioWaitAck = 0;
            WIFIRecord.cmdHiPrioAck.cmd = 0;
            WIFIRecord.cmdHiPrioAck.len = 0;
            WIFI_PROCESS_LOG("WIFIEvtResponseProces hi prio evt(%d) len(%d) OK", evt, len);
            if (fifo_discard_var_len_item(&wifiHiPrioCmdFifo))
            {
                WIFI_PROCESS_LOG("WIFIEvtResponseProces hi prio discard err");
                return 1;
            }
            return 0;
        }
        else
        {
            WIFI_PROCESS_LOG("WIFIEvtResponseProces hi prio err");
            return 2;
        }
    }
    else
    {
        if ((WIFIRecord.cmdAck.cmd == evt) && (WIFIRecord.cmdAck.len == len))
        {
            WIFIRecord.cmdWaitAck = 0;
            WIFIRecord.cmdAck.cmd = 0;
            WIFIRecord.cmdAck.len = 0;
            WIFI_PROCESS_LOG("WIFIEvtResponseProces evt(%d) len(%d) OK", evt, len);
            if (fifo_discard_var_len_item(&wifiCmdFifo))
            {
                WIFI_PROCESS_LOG("WIFIEvtResponseProces discard err");
                return 3;
            }
            return 0;
        }
        else
        {
            WIFI_PROCESS_LOG("WIFIEvtResponseProces err");
            return 4;
        }
    }
}

uint8_t WIFIPutEvtInFifo(uint16_t dataLen, uint8_t *data)
{
    uint16_t crc16 = 0;
    if ((dataLen == 0) || (data == NULL))
    {
        WIFI_PROCESS_LOG("WIFIPutEvtInFifo parm err");
        return 2;
    }

    if (dataLen > (WIFI_MAX_ITEM_SIZE + WIFI_EVT_HEADER_CRC16_LEN))
    {
        WIFI_PROCESS_LOG("WIFIPutEvtInFifo data len > WIFI_MAX_ITEM_SIZE(%d)", WIFI_MAX_ITEM_SIZE);
        return 3;
    }

    crc16 = ((uint16_t)(data[dataLen - 1]) << 8) | (uint16_t)(data[dataLen - 2]);

    if (WIFIRecDataCheckCRC(data, dataLen - sizeof(uint16_t), crc16))
    {
        WIFI_PROCESS_LOG("WIFIPutEvtInFifo crc err");
        return 4;
    }

    return fifo_put_var_len_item(&wifiEvtFifo, dataLen, data);
}

HAL_StatusTypeDef WIFIRecDataCheckCRC(uint8_t *data, uint16_t size, uint16_t crc16)
{
    WIFI_PROCESS_LOG("WIFIRecDataCheckCRC size(%d) crc16(0x%x)", size, crc16);
    if (Cal_CRC16(data, size) == crc16)
    {
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/* Wifi cmd process ----------------------------------------------------------*/
uint8_t WIFIGetCmdAndSize(uint8_t *pBuf, uint16_t *size)
{
    uint16_t cmdLen = fifo_size(&wifiCmdFifo);
    uint16_t cmdHiPrioLen = fifo_size(&wifiHiPrioCmdFifo);
    if ((cmdHiPrioLen + cmdLen) == 0)
    {
        return 1;
    }

    if (WIFIRecord.cmdHiPrioWaitAck != 0)
    {
        WIFI_PROCESS_LOG("WIFIGetCmdAndLen hi prio cmd need ack");
        return 2;
    }

    if (cmdHiPrioLen > 0)
    {
        if (fifo_get_var_len_item(&wifiHiPrioCmdFifo, size, pBuf))
        {
            WIFI_PROCESS_LOG("WIFIGetCmdAndLen hi prio cmd get err");
            return 3;
        }
        WIFIRecord.cmdHiPrioWaitAck = 1;
        memcpy(&(WIFIRecord.cmdHiPrioAck), pBuf, sizeof(WIFICmdPackHeaderTypeDef));
    }
    else if ((cmdLen > 0) && (WIFIRecord.cmdWaitAck == 0))
    {
        if (fifo_get_var_len_item(&wifiCmdFifo, size, pBuf))
        {
            WIFI_PROCESS_LOG("WIFIGetCmdAndLen cmd get err");
            return 4;
        }
        WIFIRecord.cmdWaitAck = 1;
        memcpy(&(WIFIRecord.cmdAck), pBuf, sizeof(WIFICmdPackHeaderTypeDef));
        // if (fifo_discard_var_len_item(&wifiCmdFifo))
        // {
        //     WIFI_PROCESS_LOG("WIFIGetCmdAndSize evt discard err");
        //     return 6;
        // }
    }
    else
    {
        WIFI_PROCESS_LOG("WIFIGetCmdAndLen cmdWaitAck(%d)", WIFIRecord.cmdWaitAck);
        return 5;
    }
    
    return 0;
}

uint8_t WIFISendRawData(uint8_t *data, uint16_t dataLen)
{
    if ((data == NULL) || (dataLen == 0))
    {
        WIFI_PROCESS_LOG("WIFISendRawData parm err");
        return 1;
    }

    if (WIFISendCmd(WIFI_CMD_RAW_DATA, data, dataLen, 0))
    {
        WIFI_PROCESS_LOG("WIFISendRawData send cmd err");
        return 2;
    }

    return 0;
}

uint8_t WIFISetSSIDAndKey(uint8_t *pSsid, uint16_t ssidLen, uint8_t *pKey, uint16_t keyLen)
{
    uint8_t buf[WIFI_SSID_LEN_MAX + WIFI_MAX_KEY_LEN + 8];
    uint16_t offset = 0;
    uint16_t totalLen = 0;

    if ((pSsid == NULL) || (pKey == NULL) || (keyLen > WIFI_MAX_KEY_LEN) 
        || (ssidLen > WIFI_SSID_LEN_MAX) || (ssidLen == 0))
    {
        WIFI_PROCESS_LOG("WIFISetSSIDAndKey parm err");
        return 1;
    }

    memset(WIFIRecord.ssid, 0, WIFI_SSID_LEN_MAX);
    memcpy(WIFIRecord.ssid, pSsid, ssidLen);
    memset(WIFIRecord.key, 0, WIFI_MAX_KEY_LEN);
    memcpy(WIFIRecord.key, pKey, keyLen);

    offset = 0;
    memcpy(buf + offset, &ssidLen, sizeof(ssidLen));
    offset += sizeof(ssidLen);
    memcpy(buf + offset, pSsid, ssidLen);
    offset += ssidLen;
    memcpy(buf + offset, &keyLen, sizeof(keyLen));
    offset += sizeof(keyLen);
    memcpy(buf + offset, pKey, keyLen);

    totalLen = ssidLen + keyLen + sizeof(ssidLen) + sizeof(keyLen);

    if (WIFISendCmd(WIFI_CMD_SET_SSID_KEY, buf, totalLen, 0))
    {
        WIFI_PROCESS_LOG("WIFISetSSIDAndKey cmd send err");
        return 2;
    }

    return 0;
}

uint8_t WifiQueryHttpState(char *ip, char *path, uint16_t port, char *fileName, uint32_t fileSize, uint8_t mode)
{
    #define WIFI_HTTP_QUERY_MODE_POST (1)
    #define WIFI_HTTP_QUERY_MODE_GET (2)

    uint8_t buf[WIFI_CMD_TMP_BUFFER_SIZE];
    uint16_t totalLen = 0;
    uint16_t ipLen;
    uint16_t pathLen;
    uint16_t portLen;
    uint16_t filenameLen;
    uint16_t fileSizeLen;

    if ((ip == NULL) || ((mode != WIFI_HTTP_QUERY_MODE_POST) && (mode != WIFI_HTTP_QUERY_MODE_GET)))
    {
        WIFI_PROCESS_LOG("WifiQueryHttpState param err");
        return 1;
    }

    ipLen = strlen(ip);

    if (path != NULL)
    {
        pathLen = strlen(path);
    }
    else
    {
        pathLen = 0;
    }

    portLen = sizeof(port);
    
    if (fileName != NULL)
    {
        filenameLen = strlen(fileName);
    }
    else
    {
        filenameLen = 0;
    }

    fileSizeLen = sizeof(fileSize);

    memset(buf, 0, sizeof(buf));

    memcpy(buf, &ipLen, sizeof(ipLen));
    totalLen += sizeof(ipLen);
    memcpy(buf + totalLen, ip, ipLen);
    totalLen += ipLen;

    memcpy(buf + totalLen, &pathLen, sizeof(pathLen));
    totalLen += sizeof(pathLen);
    if (pathLen)
    {
        memcpy(buf + totalLen, path, pathLen);
        totalLen += pathLen;
    }

    memcpy(buf + totalLen, &portLen, sizeof(portLen));
    totalLen += sizeof(portLen);
    memcpy(buf + totalLen, &port, portLen);
    totalLen += portLen;

    memcpy(buf + totalLen, &filenameLen, sizeof(filenameLen));
    totalLen += sizeof(filenameLen);
    if (filenameLen)
    {
        memcpy(buf + totalLen, fileName, filenameLen);
        totalLen += filenameLen;
    }

    memcpy(buf + totalLen, &fileSizeLen, sizeof(fileSizeLen));
    totalLen += sizeof(fileSizeLen);
    memcpy(buf + totalLen, &fileSize, fileSizeLen);
    totalLen += fileSizeLen;

    if (mode == WIFI_HTTP_QUERY_MODE_POST)
    {
        if (WIFISendCmd(WIFI_CMD_HTTP_POST, buf, totalLen, 0))
        {
            WIFI_PROCESS_LOG("WifiQueryHttpState cmd send err");
            return 2;
        }
    }
    else if (mode == WIFI_HTTP_QUERY_MODE_GET)
    {
        if (WIFISendCmd(WIFI_CMD_HTTP_GET, buf, totalLen, 0))
        {
            WIFI_PROCESS_LOG("WifiQueryHttpState cmd send err");
            return 3;
        }
    }

    return 0;
}

uint8_t WifiBusErrorQuery(void)
{
    if (WIFISendCmd(WIFI_CMD_SPI_BUS_ERROR_QUERY, NULL, 0, 0))
    {
        WIFI_PROCESS_LOG("WifiBusErrorQuery send cmd err");
        return 1;
    }

    return 0;
}

uint8_t WIFISendCmd(WIFICmdTypeDef cmd, uint8_t *data, uint16_t dataLen, uint8_t HiPrio)
{
    uint8_t wifiCmdTmpBuf[WIFI_CMD_TMP_BUFFER_SIZE];
    WIFICmdPackHeaderTypeDef cmdHeader;
    uint16_t inFifoLen = 0;

    if ((cmd >= WIFI_CMD_MAX)) //  || (data == NULL)
    {
        WIFI_PROCESS_LOG("WIFISendOneCmd parm err");
        return 1;
    }

    memset(wifiCmdTmpBuf, 0, sizeof(wifiCmdTmpBuf));

    cmdHeader.cmd = (uint16_t)cmd | (HiPrio ? WIFI_CMD_HI_PRIO_MASK : 0);
    cmdHeader.len = dataLen + WIFI_CMD_HEADER_CRC16_LEN;
    inFifoLen = cmdHeader.len - WIFI_CMD_CRC16_LEN;

    memcpy(wifiCmdTmpBuf, &cmdHeader, sizeof(cmdHeader));

    if (dataLen > 0)
    {
        memcpy(wifiCmdTmpBuf + sizeof(cmdHeader), data, dataLen);
    }

    if (HiPrio == 0)
    {
        if (WIFIPutDataInFifo(inFifoLen, wifiCmdTmpBuf))
        {
            WIFI_PROCESS_LOG("WIFISendOneCmd inFifo err");
            return 2;
        }
    }
    else
    {
        if (WIFIPutDataInHiPrioFifo(inFifoLen, wifiCmdTmpBuf))
        {
            WIFI_PROCESS_LOG("WIFISendOneCmd inFifo err1");
            return 3;
        }
    }

    return 0;
}

uint8_t WIFIPutRawDataInFifo(uint16_t dataLen, uint8_t *data)
{
    if (dataLen > (WIFI_MAX_ITEM_SIZE + WIFI_CMD_HEADER_CRC16_LEN))
    {
        WIFI_PROCESS_LOG("WIFIPutDataInFifo data len > WIFI_MAX_ITEM_SIZE(%d)", WIFI_MAX_ITEM_SIZE);
        return 2;
    }
    return fifo_put_var_len_item(&wifiCmdFifo, dataLen, data);
}

uint8_t WIFIPutDataInFifo(uint16_t dataLen, uint8_t *data)
{
    if (dataLen > (WIFI_MAX_ITEM_SIZE + WIFI_CMD_HEADER_CRC16_LEN))
    {
        WIFI_PROCESS_LOG("WIFIPutDataInFifo data len > WIFI_MAX_ITEM_SIZE(%d)", WIFI_MAX_ITEM_SIZE);
        return 2;
    }
    return fifo_put_var_len_item_crc16(&wifiCmdFifo, dataLen, data);
}

uint8_t WIFIPutDataInHiPrioFifo(uint16_t dataLen, uint8_t *data)
{
    if (dataLen > (WIFI_MAX_ITEM_SIZE + WIFI_CMD_HEADER_CRC16_LEN))
    {
        WIFI_PROCESS_LOG("WIFIPutDataInHiPrioFifo data len > WIFI_MAX_ITEM_SIZE(%d)", WIFI_MAX_ITEM_SIZE);
        return 2;
    }
    return fifo_put_var_len_item_crc16(&wifiHiPrioCmdFifo, dataLen, data);
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
