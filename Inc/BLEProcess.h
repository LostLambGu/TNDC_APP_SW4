/*******************************************************************************
* File Name          : BLEProcess.h
* Author             : Yangjie Gu
* Description        : This file provides all the BLEProcess functions.

* History:
*  08/31/2018 : BLEProcess V1.00
*******************************************************************************/

#ifndef BLE_PROCESS_H_
#define BLE_PROCESS_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32l4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RTOS_AND_FS_SUPPORT (1)

typedef enum
{
    BLE_CMD_SET_ADV_NAME = 0,
    BLE_CMD_GET_ADV_ADDR,
    BLE_CMD_START_ADV,

    BLE_CMD_NUS_DATA_SEND,

    BLE_CMD_BIN_INFO_SEND,
    BLE_CMD_BIN_DATA_SEND,
    BLE_CMD_ENTER_BOOT,
    BLE_CMD_RESET,
    BLE_CMD_READY_QUERY,

    BLE_CMD_VERION_QUERY,

    BLE_CMD_BUS_ERROR_QUERY,

    BLE_CMD_BUS_RAW_DATA,

    BLE_CMD_GPIO_SET_CLEAR,
    BLE_CMD_GPIO_READ,
    BLE_CMD_GPIO_SET_INPUT,

    BLE_CMD_MAX
} BLECmdTypeDef;

typedef struct
{
    uint16_t len;
    uint16_t cmd;
} BLECmdPackHeaderTypeDef;

typedef struct _BLECmdPack_t
{
    uint16_t len;
    uint16_t cmd;
    uint8_t *data;
    struct _BLECmdPack_t *next;
} BLECmdPackTypeDef;

#define BLE_NAME_LEN_MAX (32)
#define BLE_GAP_ADDR_LEN (6)
typedef struct
{
    char name[BLE_NAME_LEN_MAX];
    uint8_t addrType;
    uint8_t addr[BLE_GAP_ADDR_LEN];
    BLECmdPackTypeDef *CmdPackHead;
    BLECmdPackTypeDef *CmdPackTail;
    uint16_t cmdTotalLen;
    uint16_t cmdTotalNum;
    bool connected;
    uint8_t binTransExist;
} BLETypeDef;

extern const char *BLECmdStrArray[BLE_CMD_MAX];

#define BLE_FLASH_PAGE_SIZE (1024)
typedef struct
{
    uint32_t size;
    uint32_t crc32;
    uint32_t version;
    uint32_t dstAddr;
    uint32_t packTotal;
    uint32_t packSize;
    uint32_t packNum;
    uint32_t offset;
} BLETransferBinTypeDef;

typedef struct
{
	uint32_t size;
	uint32_t crc32;
} BLEBinSizeCrc32TypeDef;

extern BLETransferBinTypeDef BLETransferBin;

typedef enum
{
    BLE_EVT_ADV_NAME_RSP = 0,
    BLE_EVT_ADV_ADDR_RSP,
    BLE_EVT_START_ADV_RSP,
    BLE_EVT_GAP_CONNECTED,
    BLE_EVT_GAP_DISCONNECTED,

    BLE_EVT_NUS_DATA_REC,

    BLE_EVT_BIN_INFO_ERR,
    BLE_EVT_BIN_DATA_RSP,
    BLE_EVT_IN_BOOT_RSP,
    BLE_EVT_READY_RSP,

    BLE_EVT_VERION_QUERY_RSP,

    BLE_EVT_BUS_ERROR_RSP,

    BLE_EVT_BUS_RAW_DATA_RSP,

    BLE_EVT_GPIO_SET_CLEAR_RSP,
    BLE_EVT_GPIO_READ_RSP,
    BLE_EVT_GPIO_SET_INPUT_RSP,

    BLE_EVT_MAX
} BLEEvtTypeDef;

typedef struct
{
    uint16_t len;
    uint16_t evt;
} BLEEvtPackHeaderTypeDef;

typedef enum
{
    BLE_INFO_FW_VERSION,
    BLE_INFO_READY_STATE,
    BLE_INFO_IN_BOOT_MODE,

    BLE_INFO_BIN_INFO_SEND_NEED_ACK,
    BLE_INFO_BIN_INFO_OK_REQUIRE_PACK_1,
    BLE_INFO_BIN_DATA_PACK_N_OK_REQUIRE_N_PLUS_1,
    BLE_INFO_BIN_DATA_SEND_TOTAL_DONE,

    BLE_INFO_BIN_INFO_ERROR,
    BLE_INFO_BIN_DATA_SEND_ERROR,

    BLE_INFO_GPIO_SET_CLEAR,
    BLE_INFO_GPIO_READ,
    BLE_INFO_GPIO_SET_INPUT,

    BLE_INFO_MAX
} BLEInfoTypeDef;

#define BLE_GPIO3_PIN (30)
#define BLE_GPIO4_PIN (0)
#define BLE_GPIO_3_D_PIN (1)
#define BLE_GPIO_4_D_PIN (2)
#define BLE_GPIO_RES3 (3)
#define BLE_GPIO_RES4 (4)
#define BLE_CHGOK_PIN (5)
#define BLE_DC_PGOOD_PIN (8)

#define NRF_GPIO_PIN_NOPULL (0)
#define NRF_GPIO_PIN_PULLDOWN (1)
#define NRF_GPIO_PIN_PULLUP (2)

extern void BLERecordReset(void);
extern void BLEEvtProcess(void);
extern uint8_t BLEEvtProcessFunc(BLEEvtTypeDef evt, uint8_t *data, uint16_t len);
extern uint8_t BLESetAdvName(char *name, uint8_t nameLen);
extern uint8_t BLEGetAdvAddr(void);
extern uint8_t BLESetAdvStart(void);
extern uint8_t BLESendToNus(uint8_t *data, uint16_t dataLen);

#if RTOS_AND_FS_SUPPORT
extern uint8_t BLESendBinInfo(char *binName, uint32_t version, uint32_t dstAddr, uint32_t packSize);
#else
extern uint8_t BLESendBinInfo(BLETransferBinTypeDef *pBinInfo);
#endif /* RTOS_AND_FS_SUPPORT */

extern uint8_t BLESendBinData(uint8_t *data, uint16_t dataLen);
extern uint8_t BLEEnterBoot(void);
extern uint8_t BLEReset(void);
extern uint8_t BLEReadyQuery(void);
extern uint8_t BLEVersionQuery(void);
extern uint8_t BLEBusErrorQuery(void);
extern uint8_t BLESendRawData(uint8_t len);
extern uint8_t BLEGpioSetOrClear(uint32_t pin, uint32_t set);
extern uint8_t BLEGpioRead(uint32_t pin);
extern uint8_t BLEGpioSetInput(uint32_t pin, uint32_t pull);
extern uint8_t BLESendOneCmd(BLECmdTypeDef cmd, uint8_t *data, uint16_t dataLen);
extern uint8_t BLEPutCmdPackInList(BLECmdPackTypeDef *cmdPack);
extern uint8_t BLEFlushCmdPackList(void);
extern uint8_t BLEPutCmdPackInfifo(BLECmdPackTypeDef *cmdPack, uint16_t cmdNum);
extern uint8_t BLEPutDataInFifo(uint16_t dataLen, uint8_t *data);

extern void BLERegisterHandleCallBack(void (*pCallbackP)(uint32_t InfoId, void *InfoBufferP, uint32_t size));
extern void BLEClearHandleCallBack(void);
extern void BLEHandle(uint32_t InfoId, void *InfoBufferP, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* BLE_PROCESS_H_ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
