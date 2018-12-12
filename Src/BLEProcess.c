/*******************************************************************************
* File Name          : BLEProcess.c
* Author             : Yangjie Gu
* Description        : This file provides all the BLEProcess functions.

* History:
*  08/31/2018 : BLEProcess V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include "fifo.h"
#include "uart_api.h"

#include "BLEDriver.h"
#include "BLEProcess.h"

#if RTOS_AND_FS_SUPPORT
#include "cmsis_os.h"
#include "fatfs.h"
#endif /* RTOS_AND_FS_SUPPORT */

/* Private define ------------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define BLE_PROCESS_LOG(format, ...) DebugPrintf(DbgCtl.BLEProcessDbgEn, "\r\n" format, ##__VA_ARGS__)

#define BLE_CMD_HEADER_LEN (sizeof(BLECmdPackHeaderTypeDef))
#define BLE_EVT_HEADER_LEN (sizeof(BLEEvtPackHeaderTypeDef))

BLETypeDef BLERecord;
BLETransferBinTypeDef BLETransferBin;

#if RTOS_AND_FS_SUPPORT
FIL *pBLEBinFile = NULL;
#endif /* RTOS_AND_FS_SUPPORT */

uint32_t nextPackNum = 0;

const char *BLECmdStrArray[BLE_CMD_MAX] = 
{
    [BLE_CMD_SET_ADV_NAME] = "BLE_CMD_SET_ADV_NAME",
    [BLE_CMD_GET_ADV_ADDR] = "BLE_CMD_GET_ADV_ADDR",
    [BLE_CMD_START_ADV] = "BLE_CMD_START_ADV",
    [BLE_CMD_NUS_DATA_SEND] = "BLE_CMD_NUS_DATA_SEND",
    [BLE_CMD_BIN_INFO_SEND] = "BLE_CMD_BIN_INFO_SEND",
    [BLE_CMD_BIN_DATA_SEND] = "BLE_CMD_BIN_DATA_SEND",
    [BLE_CMD_ENTER_BOOT] = "BLE_CMD_ENTER_BOOT",
    [BLE_CMD_RESET] = "BLE_CMD_RESET",
    [BLE_CMD_READY_QUERY] = "BLE_CMD_READY_QUERY",
    [BLE_CMD_VERION_QUERY] = "BLE_CMD_VERION_QUERY",
    [BLE_CMD_BUS_ERROR_QUERY] = "BLE_CMD_BUS_ERROR_QUERY",
    [BLE_CMD_BUS_RAW_DATA] = "BLE_CMD_BUS_RAW_DATA",
    [BLE_CMD_GPIO_SET_CLEAR] = "BLE_CMD_GPIO_SET_CLEAR",
    [BLE_CMD_GPIO_READ] = "BLE_CMD_GPIO_READ",
    [BLE_CMD_GPIO_SET_INPUT] = "BLE_CMD_GPIO_SET_INPUT",
};

uint8_t BLEEvtAdvAddrRsp(uint8_t *data, uint16_t len);
uint8_t BLEEvtNusDataRec(uint8_t *data, uint16_t len);
uint8_t BLEEvtBinInfoErr(void);
uint8_t BLEEvtBinDataSend(uint8_t *data, uint16_t len);

uint8_t BLESendBinData(uint8_t *data, uint16_t dataLen);

uint32_t crc32_compute(uint8_t const * p_data, uint32_t size, uint32_t const * p_crc);

void BLERecordReset(void)
{
    memset(&BLERecord, 0, sizeof(BLERecord));
}

void BLEEvtProcess(void)
{
    uint16_t evtSize = 0;
    uint8_t buffer[BLE_TMP_BUFFER_SIZE] = {0};
    uint16_t len = 0, evt = 0, i = 0;

    if (fifo_size(&bleEvtFifo) > 0)
    {
        if (fifo_get_var_len_item(&bleEvtFifo, &evtSize, buffer))
        {
            BLE_PROCESS_LOG("BLEEvtProcess item get err");
            return;
        }
    }
    else
    {
        return;
    }

    if (evtSize < BLE_EVT_HEADER_LEN)
    {
        BLE_PROCESS_LOG("BLEEvtProcess item get len err");
        return;
    }

    i = 0;
    len = (uint16_t)(buffer[i]) | (((uint16_t)buffer[i + 1]) << 8);
    evt = (uint16_t)(buffer[i + 2]) | (((uint16_t)buffer[i + 3]) << 8);
    if (BLEEvtProcessFunc((BLEEvtTypeDef)evt, buffer + BLE_EVT_HEADER_LEN, len - BLE_EVT_HEADER_LEN))
    {
        BLE_PROCESS_LOG("BLEEvtProcess err");
    }

    fifo_discard_var_len_item(&bleEvtFifo);
}

uint8_t BLEEvtProcessFunc(BLEEvtTypeDef evt, uint8_t *data, uint16_t len)
{
    uint8_t ret = 0;

    switch (evt)
    {
        case BLE_EVT_ADV_NAME_RSP:
        BLE_PROCESS_LOG("BLEEvtProcessFunc set name ok");
        break;

        case BLE_EVT_ADV_ADDR_RSP:
        if (BLEEvtAdvAddrRsp(data, len))
        {
            return BLE_EVT_ADV_ADDR_RSP + 1;
        }
        break;

        case BLE_EVT_START_ADV_RSP:
        BLE_PROCESS_LOG("BLEEvtProcessFunc start adv ok");
        break;

        case BLE_EVT_NUS_DATA_REC:
        if (BLEEvtNusDataRec(data, len))
        {
            ret = BLE_EVT_NUS_DATA_REC + 1;
        }
        break;

        case BLE_EVT_GAP_CONNECTED:
        BLE_PROCESS_LOG("BLEEvtProcessFunc evt BLE_EVT_GAP_CONNECTED");
        BLERecord.connected = true;
        break;

        case BLE_EVT_GAP_DISCONNECTED:
        BLE_PROCESS_LOG("BLEEvtProcessFunc evt BLE_EVT_GAP_DISCONNECTED");
        BLERecord.connected = false;
        break;

        case BLE_EVT_BIN_INFO_ERR:
        if (BLEEvtBinInfoErr())
        {
            ret = BLE_EVT_BIN_INFO_ERR + 1;
        }
        break;

        case BLE_EVT_BIN_DATA_RSP:
        if (BLEEvtBinDataSend(data, len))
        {
            ret = BLE_EVT_BIN_DATA_RSP + 1;
        }
        break;

        case BLE_EVT_IN_BOOT_RSP:
        BLE_PROCESS_LOG("BLEEvtProcessFunc ble in boot");
        BLEHandle(BLE_INFO_IN_BOOT_MODE, NULL, 0);
        break;

        case BLE_EVT_READY_RSP:
        BLE_PROCESS_LOG("BLEEvtProcessFunc evt: BLE_EVT_READY_RSP");
        BLEHandle(BLE_INFO_READY_STATE, NULL, 0);
        break;

        case BLE_EVT_VERION_QUERY_RSP:
        BLE_PROCESS_LOG("BLEEvtProcessFunc evt: BLE_EVT_VERION_QUERY_RSP");
        BLE_PROCESS_LOG("Version: %s", data);
        BLEHandle(BLE_INFO_FW_VERSION, data, len);
        break;

        case BLE_EVT_BUS_ERROR_RSP:
        BLE_PROCESS_LOG("BLEEvtProcessFunc evt: BLE_EVT_BUS_ERROR_RSP");
        break;

        case BLE_EVT_BUS_RAW_DATA_RSP:
        BLE_PROCESS_LOG("BLEEvtProcessFunc evt: BLE_EVT_BUS_RAW_DATA_RSP len(%d)", len);
        break;

        case BLE_EVT_GPIO_SET_CLEAR_RSP:
        BLE_PROCESS_LOG("BLEEvtProcessFunc evt: BLE_CMD_GPIO_SET_CLEAR");
        BLEHandle(BLE_INFO_GPIO_SET_CLEAR, data, len);
        break;

        case BLE_EVT_GPIO_READ_RSP:
        BLE_PROCESS_LOG("BLEEvtProcessFunc evt: BLE_EVT_GPIO_READ_RSP");
        BLEHandle(BLE_INFO_GPIO_READ, data, len);
        break;

        case BLE_EVT_GPIO_SET_INPUT_RSP:
        BLE_PROCESS_LOG("BLEEvtProcessFunc evt: BLE_EVT_GPIO_SET_INPUT_RSP");
        BLEHandle(BLE_INFO_GPIO_SET_INPUT, data, len);
        break;

        default:
        BLE_PROCESS_LOG("BLEEvtProcessFunc evt err");
        ret = BLE_EVT_MAX + 1;
        break;
    }

    return ret;
}

uint8_t BLEEvtAdvAddrRsp(uint8_t *data, uint16_t len)
{
    DebugPrintf(DbgCtl.FactoryTestInfoEn, "\r\nBLEEvtAdvAddrRsp type(%d) addr(%02x:%02x:%02x:%02x:%02x:%02x)", 
        data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

    BLERecord.addrType = data[0];
    memset(BLERecord.addr, 0, BLE_GAP_ADDR_LEN);
    memcpy(BLERecord.addr, data + 1, BLE_GAP_ADDR_LEN);

    return 0;
}

uint8_t BLEEvtNusDataRec(uint8_t *data, uint16_t len)
{
    BLE_PROCESS_LOG("BLEEvtNusDataRec len(%d):", len);

    UARTPrintMassData(data, len);
    
    return 0;
}

uint8_t BLEEvtBinInfoErr(void)
{
    uint8_t ret = 0;
    #if RTOS_AND_FS_SUPPORT
    FRESULT res;

    if (pBLEBinFile != NULL)
    {
        res = f_close(pBLEBinFile);
        if (FR_OK != res)
        {
            BLE_PROCESS_LOG("BLEEvtBinInfoErr close file err");
            ret = 1;
        }

        vPortFree(pBLEBinFile);
        pBLEBinFile = NULL;
    }
    #else
    BLEHandle(BLE_INFO_BIN_INFO_ERROR, NULL, 0);
    #endif /* RTOS_AND_FS_SUPPORT */

    BLERecord.binTransExist = 0;

    BLE_PROCESS_LOG("BLEEvtBinInfoErr");

    return ret;
}

#if RTOS_AND_FS_SUPPORT
uint8_t BLEEvtBinDataSend(uint8_t *data, uint16_t len)
{
    uint32_t *pPackNum = (uint32_t *)data;
    uint8_t ret = 0;
    FRESULT res;
    uint32_t offset;
    uint32_t rcount = 0;
    uint8_t buf[512] = {0};
    static uint32_t crc32 = 0;

    if (len != sizeof(*pPackNum))
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend parm err");
        return 1;
    }

    BLE_PROCESS_LOG("BLEEvtBinDataSend expect packnum(%d)", *pPackNum);

    if (BLERecord.binTransExist == 0)
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend bin transfer ended!");
        return 2;
    }

    if (*pPackNum == (BLETransferBin.packTotal + 1))
    {
        BLERecord.binTransExist = 0;

        if (pBLEBinFile != NULL)
        {
            res = f_close(pBLEBinFile);
            if (FR_OK != res)
            {
                BLE_PROCESS_LOG("BLEEvtBinDataSend close file err");
                ret = 3;
            }

            vPortFree(pBLEBinFile);
            pBLEBinFile = NULL;
        }

        crc32 = 0;

        BLE_PROCESS_LOG("BLEEvtBinDataSend bin complete ret(%d)", ret);
        return ret;
    }

    if ((*pPackNum > (BLETransferBin.packTotal + 1)) || (*pPackNum == 0))
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend expect packnum err");
        ret = 4;
        goto BLE_EVT_BIN_DATA_SEND_ERROR;
    }

    if (pBLEBinFile == NULL)
    {
        BLERecord.binTransExist = 0;
        crc32 = 0;
        BLE_PROCESS_LOG("BLEEvtBinDataSend file not opened");
        return 5;
    }

    offset = (*pPackNum - 1) * BLETransferBin.packSize;
    if (FR_OK != f_lseek(pBLEBinFile, offset))
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend lseek failed offset(%d)", offset);
        ret = 6;
        goto BLE_EVT_BIN_DATA_SEND_ERROR;
    }
    else
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend lseek ok offset(%d)", offset);
    }

    memset(buf, 0, sizeof(buf));
    memcpy(buf, pPackNum, sizeof(*pPackNum));
    res = f_read(pBLEBinFile, buf + sizeof(*pPackNum), BLETransferBin.packSize, &rcount);
    if (res == FR_OK)
    {
        if (rcount > 0)
        {
            BLE_PROCESS_LOG("BLEEvtBinDataSend rcount(%d)", BLETransferBin.packSize);
            if (BLESendBinData(buf, rcount + sizeof(*pPackNum)))
            {
                BLE_PROCESS_LOG("BLEEvtBinDataSend send bin data err");
                ret = 7;
                goto BLE_EVT_BIN_DATA_SEND_ERROR;
            }

            crc32 = crc32_compute(buf + sizeof(*pPackNum), rcount, &crc32);
            if ((*pPackNum % (BLE_FLASH_PAGE_SIZE / BLETransferBin.packSize)) == 0)
            {
                BLE_PROCESS_LOG("BLEEvtBinDataSend PackNum(%d)====crc32(0x%x)====", *pPackNum, crc32);
            }
            else
            {
                BLE_PROCESS_LOG("BLEEvtBinDataSend PackNum(%d) crc32(0x%x)", *pPackNum, crc32);
            }
        }
        else
        {
            BLE_PROCESS_LOG("BLEEvtBinDataSend read rcount(%d)", rcount);
            ret = 8;
            goto BLE_EVT_BIN_DATA_SEND_ERROR;
        }
    }
    else
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend read failed");
        ret = 9;
        goto BLE_EVT_BIN_DATA_SEND_ERROR;
    }

    return ret;

BLE_EVT_BIN_DATA_SEND_ERROR:

    BLERecord.binTransExist = 0;

    if (pBLEBinFile != NULL)
    {
        res = f_close(pBLEBinFile);
        if (FR_OK != res)
        {
            BLE_PROCESS_LOG("BLEEvtBinDataSend close file err1");
            ret = 10;
        }

        vPortFree(pBLEBinFile);
        pBLEBinFile = NULL;
    }

    crc32 = 0;

    return ret;
}
#else /* RTOS_AND_FS_SUPPORT */
uint8_t BLEEvtBinDataSend(uint8_t *data, uint16_t len)
{
    uint32_t *pPackNum = (uint32_t *)data;
    uint8_t ret = 0;

    if (len != sizeof(*pPackNum))
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend parm err");
        return 1;
    }

    BLE_PROCESS_LOG("BLEEvtBinDataSend expect packnum(%d)", *pPackNum);

    if (BLERecord.binTransExist == 0)
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend bin transfer ended!");
        return 2;
    }

    if (*pPackNum == 0xffffffff)
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend bin transfer failed");
        ret = 6;
        goto BLE_EVT_BIN_DATA_SEND_ERROR;
    }

    if (*pPackNum == (BLETransferBin.packTotal + 1))
    {
        BLERecord.binTransExist = 0;

        BLEHandle(BLE_INFO_BIN_DATA_SEND_TOTAL_DONE, NULL, 0);

        BLE_PROCESS_LOG("BLEEvtBinDataSend bin complete ret(%d)", ret);

        return ret;
    }

    if ((*pPackNum > (BLETransferBin.packTotal + 1)) || (*pPackNum == 0))
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend expect packnum err");
        ret = 4;
        goto BLE_EVT_BIN_DATA_SEND_ERROR;
    }

    nextPackNum = *pPackNum;
    if (*pPackNum == 1)
    {
        BLEHandle(BLE_INFO_BIN_INFO_OK_REQUIRE_PACK_1, pPackNum, sizeof(*pPackNum));
        BLE_PROCESS_LOG("BLEEvtBinDataSend bin info ok");
    }
    else
    {
        BLE_PROCESS_LOG("BLEEvtBinDataSend bin send data done packNum(%d)", *pPackNum - 1);
        BLEHandle(BLE_INFO_BIN_DATA_PACK_N_OK_REQUIRE_N_PLUS_1, pPackNum, sizeof(*pPackNum));
    }

    return ret;

BLE_EVT_BIN_DATA_SEND_ERROR:

    BLERecord.binTransExist = 0;
    
    BLEHandle(BLE_INFO_BIN_DATA_SEND_ERROR, NULL, 0);

    return ret;
}
#endif /* RTOS_AND_FS_SUPPORT */

/* Private define ------------------------------------------------------------*/
/* BLE CMD Related Functions -------------------------------------------------*/
uint8_t BLESetAdvName(char *name, uint8_t nameLen)
{
    if ((name == NULL) || (nameLen == 0))
    {
        BLE_PROCESS_LOG("BLESetAdvName param err");
        return 1;
    }

    memset(BLERecord.name, 0, sizeof(BLERecord.name));
    memcpy(BLERecord.name, name, nameLen);

    if (BLESendOneCmd(BLE_CMD_SET_ADV_NAME, (uint8_t *)BLERecord.name, nameLen))
    {
        BLE_PROCESS_LOG("BLESetAdvName err");
        return 1;
    }

    return 0;
}

uint8_t BLEGetAdvAddr(void)
{
    if (BLESendOneCmd(BLE_CMD_GET_ADV_ADDR, NULL, 0))
    {
        BLE_PROCESS_LOG("BLEGetAdvAddr err");
        return 1;
    }

    return 0;
}

uint8_t BLESetAdvStart(void)
{
    if (BLESendOneCmd(BLE_CMD_START_ADV, NULL, 0))
    {
        BLE_PROCESS_LOG("BLESetAdvStart err");
        return 1;
    }

    return 0;
}

uint8_t BLESendToNus(uint8_t *data, uint16_t dataLen)
{
    if (BLESendOneCmd(BLE_CMD_NUS_DATA_SEND, data, dataLen))
    {
        BLE_PROCESS_LOG("BLESendToNus err");
        return 1;
    }

    return 0;
}

#if RTOS_AND_FS_SUPPORT
extern uint8_t FileSystemMountFlag;
uint8_t BLESendBinInfo(char *binName, uint32_t version, uint32_t dstAddr, uint32_t packSize)
{
    FRESULT res;
    uint8_t ret = 0;
    char namebuf[32] = {0};
    // uint32_t rcount = 0;
    // BLEBinSizeCrc32TypeDef BLEBinSizeCrc32;

    memset(&BLETransferBin, 0, sizeof(BLETransferBin));

    if (pBLEBinFile != NULL)
    {
        vPortFree(pBLEBinFile);
        pBLEBinFile = NULL;
    }

    pBLEBinFile = pvPortMalloc(sizeof(FIL));
    if (pBLEBinFile == NULL)
    {
        BLE_PROCESS_LOG("BLESendBinInfo FIL malloc err");
        return 1;
    }

    memset(pBLEBinFile, 0, sizeof(FIL));

    if (FileSystemMountFlag == 0)
	{
		MX_FATFS_Init();

		res = f_mount(&EMMCFatFS, EMMCPath, 1);
		if (res == FR_OK)
		{
			FileSystemMountFlag = 1;
		}
		else
		{
			FATFS_UnLinkDriver(EMMCPath);
			BLE_PROCESS_LOG("BLESendBinInfo mount failed: %d", res);
			ret = 2;
            goto BLE_BIN_INFO_ERROR;
		}
	}

    namebuf[0] = '0';
	namebuf[1] = ':';
	namebuf[2] = '/';
	memcpy(namebuf + 3, binName, ((strlen((char *)binName) + 4) < sizeof(namebuf)) ? strlen((char *)binName): (sizeof(namebuf) - 4));
	res = f_open(pBLEBinFile, namebuf, FA_READ | FA_OPEN_EXISTING);
    if (res != FR_OK)
	{
		BLE_PROCESS_LOG("BLESendBinInfo Open file(%s) failed: %d", namebuf, res);
        ret = 3;
        goto BLE_BIN_INFO_ERROR;
	}

    if (FR_OK != f_lseek(pBLEBinFile, 0))
    {
        BLE_PROCESS_LOG("BLESendBinInfo lseek failed");
        ret = 4;
        goto BLE_BIN_INFO_ERROR0;
    }

    // rcount = 0;
    // memset(&BLEBinSizeCrc32, 0, sizeof(BLEBinSizeCrc32));
    // res = f_read(pBLEBinFile, &BLEBinSizeCrc32, sizeof(BLEBinSizeCrc32), &rcount);
    // if (res == FR_OK)
    // {
    //     if (rcount == sizeof(BLEBinSizeCrc32))
    //     {
    //         BLETransferBin.size = BLEBinSizeCrc32.size;
    //         BLETransferBin.crc32 = BLEBinSizeCrc32.crc32;
    //         BLE_PROCESS_LOG("BLESendBinInfo bin(%s) size(0x%x) crc32(0x%x)", 
    //         namebuf, BLETransferBin.size, BLETransferBin.crc32);
    //     }
    //     else
    //     {
    //         BLE_PROCESS_LOG("BLESendBinInfo read rcount(%d)", rcount);
    //         ret = 5;
    //         goto BLE_BIN_INFO_ERROR0;
    //     }
    // }
    // else
    // {
    //     BLE_PROCESS_LOG("BLESendBinInfo read failed");
    //     ret = 6;
    //     goto BLE_BIN_INFO_ERROR0;
    // }

    BLETransferBin.size = f_size(pBLEBinFile);
    BLETransferBin.version = version;
    BLETransferBin.dstAddr = dstAddr;
    BLETransferBin.packSize = packSize;
    BLETransferBin.packTotal = (BLETransferBin.size + packSize - 1) / packSize;
    BLE_PROCESS_LOG("BLESendBinInfo version(0x%x) dstAddr(0x%x) packSize(%d) packTotal(%d)", 
    BLETransferBin.version, BLETransferBin.dstAddr, BLETransferBin.packSize, BLETransferBin.packTotal);

    if (BLESendOneCmd(BLE_CMD_BIN_INFO_SEND, (uint8_t *)&BLETransferBin, sizeof(BLETransferBin)))
    {
        BLE_PROCESS_LOG("BLESendBinInfo send cmd err");
        ret = 7;
        goto BLE_BIN_INFO_ERROR0;
    }
    else
    {
        BLE_PROCESS_LOG("BLESendBinInfo send cmd ok");
        BLERecord.binTransExist = 1;
    }

    return ret;

    BLE_BIN_INFO_ERROR0:
    f_close(pBLEBinFile);
    
    BLE_BIN_INFO_ERROR:
    if (pBLEBinFile != NULL)
    {
        vPortFree(pBLEBinFile);
        pBLEBinFile = NULL;
    }

    BLERecord.binTransExist = 0;

    return ret;
}
#else /* RTOS_AND_FS_SUPPORT */
uint8_t BLESendBinInfo(BLETransferBinTypeDef *pBinInfo)
{
    uint8_t ret = 0;

    if (pBinInfo == NULL)
    {
        ret = 1;
        BLE_PROCESS_LOG("BLESendBinInfo parm err");
        goto BLE_BIN_INFO_ERROR;
    }

    memset(&BLETransferBin, 0, sizeof(BLETransferBin));
    memcpy(&BLETransferBin, pBinInfo, sizeof(BLETransferBinTypeDef));

	BLETransferBin.packTotal = (BLETransferBin.size + BLETransferBin.packSize - 1) / BLETransferBin.packSize;

    BLE_PROCESS_LOG("BLESendBinInfo size(0x%x) crc32(0x%x)", BLETransferBin.size, BLETransferBin.crc32);
    BLE_PROCESS_LOG("BLESendBinInfo version(0x%x) dstAddr(0x%x) packSize(%d) packTotal(%d)", 
    BLETransferBin.version, BLETransferBin.dstAddr, BLETransferBin.packSize, BLETransferBin.packTotal);

    if (BLESendOneCmd(BLE_CMD_BIN_INFO_SEND, (uint8_t *)&BLETransferBin, sizeof(BLETransferBin)))
    {
        BLE_PROCESS_LOG("BLESendBinInfo send cmd err");
        ret = 7;
        goto BLE_BIN_INFO_ERROR;
    }
    else
    {
        BLE_PROCESS_LOG("BLESendBinInfo send cmd ok");
        BLERecord.binTransExist = 1;
        BLEHandle(BLE_INFO_BIN_INFO_SEND_NEED_ACK, NULL, 0);
    }

    return ret;
    
BLE_BIN_INFO_ERROR:
    BLERecord.binTransExist = 0;
    BLEHandle(BLE_INFO_BIN_INFO_ERROR, NULL, 0);

    return ret;
}
#endif /* RTOS_AND_FS_SUPPORT */

uint8_t BLESendBinData(uint8_t *data, uint16_t dataLen)
{
    #if RTOS_AND_FS_SUPPORT
    if (BLESendOneCmd(BLE_CMD_BIN_DATA_SEND, data, dataLen))
    {
        BLE_PROCESS_LOG("BLESendBinData err");
        return 1;
    }
    #else
    uint8_t buf[BLE_SPI_ITEM_MAX_SIZE];
    uint32_t packNum = nextPackNum;

    memset(buf, 0, sizeof(buf));
    memcpy(buf, &packNum, sizeof(packNum));
    memcpy(buf + sizeof(packNum), data, dataLen);

    if (BLESendOneCmd(BLE_CMD_BIN_DATA_SEND, buf, dataLen + sizeof(packNum)))
    {
        BLE_PROCESS_LOG("BLESendBinData err");
        return 1;
    }
    
    BLE_PROCESS_LOG("BLESendBinData packNum(%d)", packNum);
    #endif /* RTOS_AND_FS_SUPPORT */

    return 0;
}

uint8_t BLEEnterBoot(void)
{
    if (BLESendOneCmd(BLE_CMD_ENTER_BOOT, NULL, 0))
    {
        BLE_PROCESS_LOG("BLEEnterBoot err");
        return 1;
    }

    return 0;
}

uint8_t BLEReset(void)
{
    if (BLESendOneCmd(BLE_CMD_RESET, NULL, 0))
    {
        BLE_PROCESS_LOG("BLEReset err");
        return 1;
    }

    return 0;
}

uint8_t BLEReadyQuery(void)
{
    if (BLESendOneCmd(BLE_CMD_READY_QUERY, NULL, 0))
    {
        BLE_PROCESS_LOG("BLEReadyQuery err");
        return 1;
    }

    return 0;
}

uint8_t BLEVersionQuery(void)
{
    if (BLESendOneCmd(BLE_CMD_VERION_QUERY, NULL, 0))
    {
        BLE_PROCESS_LOG("BLEReadyQuery err");
        return 1;
    }

    return 0;
}

uint8_t BLEBusErrorQuery(void)
{
    if (BLESendOneCmd(BLE_CMD_BUS_ERROR_QUERY, NULL, 0))
    {
        BLE_PROCESS_LOG("BLEBusErrorQuery err");
        return 1;
    }

    return 0;
}

uint8_t BLESendRawData(uint8_t len)
{
    uint8_t buf[BLE_TMP_BUFFER_SIZE];

    memset(buf, 0xab, BLE_TMP_BUFFER_SIZE);

    if (BLESendOneCmd(BLE_CMD_BUS_RAW_DATA, buf, (len > 240) ? 240 : len))
    {
        BLE_PROCESS_LOG("BLESendRawData err");
        return 1;
    }

    return 0;
}

uint8_t BLEGpioSetOrClear(uint32_t pin, uint32_t set)
{
    uint32_t buf[2];
    buf[0] = pin;
    buf[1] = set;

    if (BLESendOneCmd(BLE_CMD_GPIO_SET_CLEAR, (uint8_t *)buf, sizeof(buf)))
    {
        BLE_PROCESS_LOG("BLEGpioSetOrClear err");
        return 1;
    }

    return 0;
}

uint8_t BLEGpioRead(uint32_t pin)
{
    if (BLESendOneCmd(BLE_CMD_GPIO_READ, (uint8_t *)(&pin), sizeof(pin)))
    {
        BLE_PROCESS_LOG("BLEGpioRead err");
        return 1;
    }

    return 0;
}

uint8_t BLEGpioSetInput(uint32_t pin, uint32_t pull)
{
    uint32_t buf[2];
    buf[0] = pin;
    buf[1] = pull;

    if (BLESendOneCmd(BLE_CMD_GPIO_SET_INPUT, (uint8_t *)buf, sizeof(buf)))
    {
        BLE_PROCESS_LOG("BLEGpioSetInput err");
        return 1;
    }

    return 0;
}

uint8_t BLESendOneCmd(BLECmdTypeDef cmd, uint8_t *data, uint16_t dataLen)
{
    BLECmdPackTypeDef cmdPack;
    memset(&cmdPack, 0, sizeof(cmdPack));

    cmdPack.len = BLE_CMD_HEADER_LEN + dataLen;
    cmdPack.cmd = cmd;
    cmdPack.data = data;

    if (BLEPutCmdPackInList(&cmdPack))
    {
        BLE_PROCESS_LOG("BLESendOneCmd put in list err");
        return 1;
    }

    if (BLEFlushCmdPackList())
    {
        BLE_PROCESS_LOG("BLESendOneCmd flush err");
        return 2;
    }

    return 0;
}

uint8_t BLEPutCmdPackInList(BLECmdPackTypeDef *cmdPack)
{
    if ((cmdPack == NULL) || (cmdPack->len == 0) || (cmdPack->len > BLE_SPI_ITEM_MAX_SIZE)
        || (cmdPack->cmd >= BLE_CMD_MAX))
    {
        BLE_PROCESS_LOG("BLEPutCmdPackInList param err");
        return 1;
    }

    if (BLERecord.CmdPackHead == NULL)
    {
        BLERecord.CmdPackHead = cmdPack;
        BLERecord.CmdPackTail = cmdPack;
        BLERecord.cmdTotalLen = cmdPack->len;
        BLERecord.cmdTotalNum++;
    }
    else
    {
        if ((BLERecord.cmdTotalLen + cmdPack->len) > BLE_SPI_ITEM_MAX_SIZE)
        {
            if (BLEPutCmdPackInfifo(BLERecord.CmdPackHead, BLERecord.cmdTotalNum))
            {
                BLE_PROCESS_LOG("BLEPutCmdPackInList cmdPack in fifo err2");
                return 2;
            }
        }

        if (BLERecord.CmdPackHead == NULL)
        {
            BLERecord.CmdPackHead = cmdPack;
            BLERecord.CmdPackTail = cmdPack;
            BLERecord.cmdTotalLen = cmdPack->len;
        }
        else
        {
            BLERecord.CmdPackTail->next = cmdPack;
            BLERecord.CmdPackTail = cmdPack;
            BLERecord.cmdTotalLen += cmdPack->len;
        }
        BLERecord.cmdTotalNum++;
    }

    if (BLERecord.cmdTotalLen > (BLE_SPI_ITEM_MAX_SIZE / 2))
    {
        if (BLEPutCmdPackInfifo(BLERecord.CmdPackHead, BLERecord.cmdTotalNum))
        {
            BLE_PROCESS_LOG("BLEPutCmdPackInList cmdPack in fifo err3");
            return 3;
        }
    }

    return 0;
}

uint8_t BLEFlushCmdPackList(void)
{
    if ((BLERecord.CmdPackHead == NULL) || (BLERecord.cmdTotalNum == 0))
    {
        BLE_PROCESS_LOG("BLEFlushCmdPackList no cmdPack");
        return 0;
    }

    if (BLEPutCmdPackInfifo(BLERecord.CmdPackHead, BLERecord.cmdTotalNum))
    {
        BLE_PROCESS_LOG("BLEFlushCmdPackList cmdPack in fifo err");
        return 1;
    }

    return 0;
}

uint8_t BLEPutCmdPackInfifo(BLECmdPackTypeDef *cmdPack, uint16_t cmdNum)
{
    uint8_t tmpBuf[BLE_TMP_BUFFER_SIZE] = {0};
    uint16_t i = 0, len = 0, tmpLen = 0;
    BLECmdPackTypeDef *ptemp = cmdPack;

    for (i = 0; i < cmdNum; i++)
    {
        memcpy(tmpBuf + len, &(ptemp->len), sizeof(ptemp->len));
        len += sizeof(ptemp->len);
        memcpy(tmpBuf + len, &(ptemp->cmd), sizeof(ptemp->cmd));
        len += sizeof(ptemp->cmd);
        tmpLen = ptemp->len - sizeof(ptemp->len) - sizeof(ptemp->cmd);
        if (tmpLen > 0)
        {
            memcpy(tmpBuf + len, ptemp->data, tmpLen);
            len += tmpLen;
        }

        ptemp = ptemp->next;
    }

    if (BLEPutDataInFifo(len, tmpBuf))
    {
        BLE_PROCESS_LOG("BLEPutCmdPackInfifo err");
        return 2;
    }

    BLERecord.CmdPackHead = NULL;
    BLERecord.CmdPackTail = NULL;
    BLERecord.cmdTotalLen = 0;
    BLERecord.cmdTotalNum = 0;

    return 0;
}

uint8_t BLEPutDataInFifo(uint16_t dataLen, uint8_t *data)
{
    if (dataLen > BLE_SPI_ITEM_MAX_SIZE)
    {
        BLE_PROCESS_LOG("BLEPutDataInFifo data len > BLE_SPI_ITEM_MAX_SIZE(%d)", BLE_SPI_ITEM_MAX_SIZE);
        return 2;
    }
    return fifo_put_var_len_item_crc16(&bleCmdFifo, dataLen, data);
}

uint32_t crc32_compute(uint8_t const * p_data, uint32_t size, uint32_t const * p_crc)
{
    uint32_t crc;

    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (uint32_t i = 0; i < size; i++)
    {
        crc = crc ^ p_data[i];
        for (uint32_t j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
}

// BLE app update callback
static void BLEHandleCallBackTest(uint32_t InfoId, void *InfoBufferP, uint32_t size);
void (*pBLEHandleCallBack)(uint32_t InfoId, void *InfoBufferP, uint32_t size) = BLEHandleCallBackTest;
void BLERegisterHandleCallBack(void (*pCallbackP)(uint32_t InfoId, void *InfoBufferP, uint32_t size))
{
    pBLEHandleCallBack = pCallbackP;
}

void BLEClearHandleCallBack(void)
{
    pBLEHandleCallBack = NULL;
}

void BLEHandle(uint32_t InfoId, void *InfoBufferP, uint32_t size)
{
    if (pBLEHandleCallBack != NULL)
        (*pBLEHandleCallBack)(InfoId, InfoBufferP, size);
}

static void BLEHandleCallBackTest(uint32_t InfoId, void *InfoBufferP, uint32_t size)
{
    BLE_PROCESS_LOG("In function BLEHandleCallBackTest");
    switch(InfoId)
    {
        case BLE_INFO_FW_VERSION:
        BLE_PROCESS_LOG("BLE_INFO_FW_VERSION");
        break;

        case BLE_INFO_READY_STATE:
        BLE_PROCESS_LOG("BLE_INFO_READY_STATE");
        break;

        case BLE_INFO_IN_BOOT_MODE:
        break;

        case BLE_INFO_BIN_INFO_SEND_NEED_ACK:
        break;

        case BLE_INFO_BIN_INFO_OK_REQUIRE_PACK_1:
        break;

        case BLE_INFO_BIN_DATA_PACK_N_OK_REQUIRE_N_PLUS_1:
        break;

        case BLE_INFO_BIN_DATA_SEND_TOTAL_DONE:
        break;

        case BLE_INFO_BIN_INFO_ERROR:
        break;

        case BLE_INFO_BIN_DATA_SEND_ERROR:
        break;

        case BLE_INFO_GPIO_SET_CLEAR:
        BLE_PROCESS_LOG("BLE_INFO: Set Pin(%d) (%d)", *((uint32_t *)InfoBufferP), *((uint32_t *)((uint8_t *)InfoBufferP + 4)));
        break;

        case BLE_INFO_GPIO_READ:
        BLE_PROCESS_LOG("BLE_INFO: Read Pin(%d) (%d)", *((uint32_t *)InfoBufferP), *((uint32_t *)((uint8_t *)InfoBufferP + 4)));
        break;

        case BLE_INFO_GPIO_SET_INPUT:
        BLE_PROCESS_LOG("BLE_INFO: Set Input Pin(%d) Pull(%d)", *((uint32_t *)InfoBufferP), *((uint32_t *)((uint8_t *)InfoBufferP + 4)));
        break;

        default:
        BLE_PROCESS_LOG("InfoId Error");
        break;
    }
}
/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
