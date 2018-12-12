/*******************************************************************************
* File Name          : WifiProcess.h
* Author             : Yangjie Gu
* Description        : This file provides all the WifiProcess functions.

* History:
*  10/11/2018 : WifiProcess V1.00
*******************************************************************************/
#ifndef WIFI_PROCESS_H_
#define WIFI_PROCESS_H_

#include "stm32l4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_MAX_ITEM_SIZE (128 * 3)

typedef enum
{
    WIFI_CMD_RAW_DATA = 0,
    WIFI_CMD_SET_SSID_KEY,
    WIFI_CMD_GET_IPV4_ADDR,

    WIFI_CMD_SPI_BUS_ERROR_QUERY,

    WIFI_RSP_STATION_STATE,

    WIFI_CMD_HTTP_POST,
    WIFI_RSP_HTTP_POST_STATE,
    WIFI_CMD_HTTP_POST_SEND_DATA,
    WIFI_RSP_HTTP_POST_FINISH,
    WIFI_RSP_HTTP_POST_ERROR,

    WIFI_CMD_HTTP_GET,

    WIFI_CMD_MAX
} WIFICmdTypeDef;

extern const char *WIFICmdStrArray[WIFI_CMD_MAX];

typedef struct
{
    uint16_t len;
    uint16_t cmd;
} WIFICmdPackHeaderTypeDef;

typedef struct
{
    uint16_t len;
    uint16_t evt;
} WIFIEvtPackHeaderTypeDef;

#define WIFI_SSID_LEN_MAX (32)
#define WIFI_MAX_KEY_LEN (32)
#define WIFI_IPV4_ADDR_LEN (4)
#define WIFI_CMD_RESPONSE_MASK (0x8000)
#define WIFI_CMD_HI_PRIO_MASK (0x4000)
typedef struct
{
    uint8_t status;
    char ssid[WIFI_SSID_LEN_MAX];
    uint8_t key[WIFI_MAX_KEY_LEN];
    uint8_t addrType;
    uint8_t addr[WIFI_IPV4_ADDR_LEN];
    uint8_t staConnected;
    uint8_t cmdWaitAck;
    WIFICmdPackHeaderTypeDef cmdAck;
    uint8_t cmdHiPrioWaitAck;
    WIFICmdPackHeaderTypeDef cmdHiPrioAck;
} WIFITypeDef;

extern void WIFIProcessInit(void);

extern uint8_t WIFIEvtProcess(void);
extern uint8_t WIFIPutEvtInFifo(uint16_t dataLen, uint8_t *data);

extern uint8_t WIFIGetCmdAndSize(uint8_t *pBuf, uint16_t *size);
extern uint8_t WIFISendRawData(uint8_t *data, uint16_t dataLen);
extern uint8_t WIFISetSSIDAndKey(uint8_t *pSsid, uint16_t ssidLen, uint8_t *pKey, uint16_t keyLen);
extern uint8_t WifiQueryHttpState(char *ip, char *path, uint16_t port, char *fileName, uint32_t fileSize, uint8_t mode);
extern uint8_t WifiBusErrorQuery(void);
extern uint8_t WIFISendCmd(WIFICmdTypeDef cmd, uint8_t *data, uint16_t dataLen, uint8_t HiPrio);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_PROCESS_H_ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
