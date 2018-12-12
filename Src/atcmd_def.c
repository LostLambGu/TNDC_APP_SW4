/*******************************************************************************
* File Name          : atcmd_def.c
* Author             : Yangjie Gu
* Description        : This file provides all the atcmd_def functions.

* History:
*  09/28/2017 : atcmd_def V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "iwdg.h"
#include "can.h"
#include "usart.h"
#include "atcmd.h"
#include "common.h"
#include "initialization.h"
#include "fatfs.h"
#include "flash.h"
#include "rtcclock.h"
#include "lis2dh_driver.h"
#include "WifiDriver.h"
#include "io_control.h"
#include "deepsleep.h"
#include "version.h"
#include "WifiDriver.h"
#include "SdioEmmcDrive.h"
#include "sha204_lib_return_codes.h"
#include "atsha204_read_sn.h"
#include "software_timer_utilities.h"
#include "sha204_physical.h"
#include "sha204_comm.h"
#include "BLEDriver.h"
#include "BLEProcess.h"
#include "WifiProcess.h"

/* Private define ------------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define ATCMD_DEF_PRINT(format, ...) DebugPrintf(DbgCtl.FactoryTestInfoEn, "\r\n" format, ##__VA_ARGS__)
/* Variables -----------------------------------------------------------------*/
// extern void ATCmdDefOEMMSG(uint8_t *dataBuf);

extern char *TNDCFwVerStr;

extern const ATCompareTable_t ATCmdTypeTable[AT_CMD_DEF_LAST];

__IO uint8_t factorymodeindicatefalg = FALSE;
__IO uint8_t uartFactoryRec = 0;

extern void PreLoadFileProcessing(void);
extern void PostLoadFileProcessing(void);

extern void OperationTestTask(void const *argument);

/* Static function declarations ----------------------------------------------*/
static void ATCmdDefVERFactoryTest(void);
static void ATCmdEmmcFactoryTest(void);
static void ATCmdGsensorFactoryTest(void);
static void ATCmdSha204FactoryTest(void);
static void ATCmdDefRtcFactoryTest(void);
static void ATCmdDefCanFactoryTest(void);
static void ATCmdDefUart1FactoryTest(void);
static void ATCmdIoFactoryTest(void);
static void ATCmdDefFactoryChmodem(void);
static void ATCmdDefFactoryYmodem(void);
static void ATCmdDefFactoryMakeFatfs(void);
static void ATCmdDefFactoryFatfsInfo(void);
static void ATCmdDefFactoryFirmwareUpdate(void);
static void ATCmdDefFactoryBootChmodem(void);
static void ATCmdDefFactoryBootYmodem(void);
static void ATCmdDefFactoryFileIOTest(void);

static void ATCmdDefPrototype(void);
static void ATCmdDefReset(void);
static void ATCmdDefSleep(uint8_t Len, int32_t Param);
static void ATCmdDefPwrCtl(uint8_t Len, int32_t Param, uint8_t *dataBuf);
static void ATCmdGsensorTest(uint8_t Len, int Param);
static void ATCmdDefGPIOREAD(uint8_t Len, int Param, uint8_t *dataBuf);
static void ATCmdDefGPIOWRITE(uint8_t Len, int32_t Param, uint8_t *dataBuf);
static void ATCmdDefWatchDog(uint8_t Len, int32_t Param, uint8_t *dataBuf);
static void ATCmdDefDbgCtl(uint8_t Len, int32_t Param, uint8_t *dataBuf);
static void ATCmdDefCanBaudTest(uint8_t Len, int32_t Param);
static void ATCmdDefChmodem(uint8_t Len, int32_t Param);
static void ATCmdDefWifiMode(uint8_t Len, int32_t Param);
static void ATCmdDefWifiOperations(uint8_t Len, int32_t Param, uint8_t *dataBuf);
static void ATCmdDefBleOperations(uint8_t Len, int32_t Param, uint8_t *dataBuf);
static void ATCmdDefUpdateFirmwareFromFile(uint8_t Len, int32_t Param, uint8_t *dataBuf);
static void ATCmdDefHttpTest(uint8_t Len, int32_t Param, uint8_t *dataBuf);

static void ATCmdDefDefault(void);

/* Function definitions ------------------------------------------------------*/
void ATCmdProcessing(uint8_t Type, uint8_t FactoryMode, uint8_t Len, int32_t Param, uint8_t *dataBuf)
{
	if (dataBuf == NULL)
		ATDbgPrintf(DbgCtl.ATCmdDbgEn, "[%s] AT Process: Type(%d) Len(%d) Param(%d)",
					FmtTimeShow(), Type, Len, Param);
	else
		ATDbgPrintf(DbgCtl.ATCmdDbgEn, "[%s] AT Process: Type(%d) Len(%d) Param(%d) Buf(%s)",
					FmtTimeShow(), Type, Len, Param, dataBuf);
	if (Type == AT_CMD_DEF_NULL)
	{
		ATCmdPrintf(DbgCtl.ATCmdInfoEn, "ERROR");
		ATDbgPrintf(DbgCtl.ATCmdDbgEn, "[%s] ERROR", FmtTimeShow());
		// return;
	}

	if (FactoryMode == TRUE)
	{
		switch (Type)
		{
		case AT_CMD_DEF_VER:
			ATCmdDefVERFactoryTest();
			break;

		case AT_CMD_DEF_EMMC:
			ATCmdEmmcFactoryTest();
			break;

		case AT_CMD_DEF_GSENSOR:
			ATCmdGsensorFactoryTest();
			break;

		case AT_CMD_DEF_SHA204:
			ATCmdSha204FactoryTest();
			break;

		case AT_CMD_DEF_RTC:
			ATCmdDefRtcFactoryTest();
			break;

		case AT_CMD_DEF_CAN:
			ATCmdDefCanFactoryTest();
			break;

		case AT_CMD_DEF_UART1:
			ATCmdDefUart1FactoryTest();
			break;

		case AT_CMD_DEF_IOINFO:
			ATCmdIoFactoryTest();
			break;

		case AT_CMD_DEF_CHMODEM:
			ATCmdDefFactoryChmodem();
			break;

		case AT_CMD_DEF_YMODEM:
			ATCmdDefFactoryYmodem();
			break;

		case AT_CMD_DEF_MAKE_FATFS:
			ATCmdDefFactoryMakeFatfs();
			break;

		case AT_CMD_DEF_FATFSINFO:
			ATCmdDefFactoryFatfsInfo();
			break;

		case AT_CMD_DEF_FIRMWARE_UPDATE:
			ATCmdDefFactoryFirmwareUpdate();
			break;
			
		case AT_CMD_DEF_BOOT_CHMODEM:
			ATCmdDefFactoryBootChmodem();
			break;

		case AT_CMD_DEF_BOOT_YMODEM:
			ATCmdDefFactoryBootYmodem();
			break;

		case AT_CMD_DEF_FILEIOTEST:
			ATCmdDefFactoryFileIOTest();
			break;

		case AT_CMD_DEF_TASKTEST:
		{
			osThreadDef(OperationTestTask, OperationTestTask, osPriorityNormal, 0, 128 * 6);
			osThreadCreate(osThread(OperationTestTask), NULL);
		}
		break;

		default:
			ATCmdDefDefault();
			break;
		}
	}
	else
	{
		switch (Type)
		{
		case AT_CMD_DEF_INITIAL:
			DebugLog("OK\r\n");
			break;

		case AT_CMD_DEF_PROTOTYPE:
			ATCmdDefPrototype();
			break;

		case AT_CMD_DEF_RESET:
			ATCmdDefReset();
			break;

		case AT_CMD_DEF_SLEEP:
			ATCmdDefSleep(Len, Param);
			break;

		case AT_CMD_DEF_PWRCTL:
			ATCmdDefPwrCtl(Len, Param, dataBuf);
			break;

		case AT_CMD_DEF_GSENSOR:
			ATCmdGsensorTest(Len, Param);
			break;

		case AT_CMD_DEF_GPIOREAD:
			ATCmdDefGPIOREAD(Len, Param, dataBuf);
			break;

		case AT_CMD_DEF_GPIOWRITE:
			ATCmdDefGPIOWRITE(Len, Param, dataBuf);
			break;

		case AT_CMD_DEF_WATCHDOG:
			ATCmdDefWatchDog(Len, Param, dataBuf);
			break;

		case AT_CMD_DEF_DBGCTL:
			ATCmdDefDbgCtl(Len, Param, dataBuf);
			break;

		case AT_CMD_DEF_CAN:
			ATCmdDefCanBaudTest(Len, Param);
			break;

		case AT_CMD_DEF_CHMODEM:
			ATCmdDefChmodem(Len, Param);
			break;
		
		case AT_CMD_DEF_WIFIMODE:
			ATCmdDefWifiMode(Len, Param);
			break;

		case AT_CMD_DEF_WIFIOP:
			ATCmdDefWifiOperations(Len, Param, dataBuf);
			break;

		case AT_CMD_DEF_BLEOP:
			ATCmdDefBleOperations(Len, Param, dataBuf);
			break;

		case AT_CMD_DEF_FILEUPDATE:
			ATCmdDefUpdateFirmwareFromFile(Len, Param, dataBuf);
			break;

		case AT_CMD_DEF_HTTPTEST:
			ATCmdDefHttpTest(Len, Param, dataBuf);
			break;

		default:
			ATCmdDefDefault();
			break;
		}
	}
}

static void ATCmdDefVERFactoryTest(void)
{
	DebugLog("MCU FW VER: %s", TNDCFwVerStr);
}

static void ATCmdEmmcFactoryTest(void)
{
	extern EmmcCardInfo MyEmmcCardInfo;
	extern uint8_t BSP_EMMC_Init(void);
	if (BSP_EMMC_Init())
	{
		ATCMD_DEF_PRINT("ATCmdEmmcFactoryTest emmc init fail");
	}
	ATCMD_DEF_PRINT("Manufacture ID: 0x%02x", MyEmmcCardInfo.EmmcCid.ManufacturerID & 0xff);
	ATCMD_DEF_PRINT("OEM/Application ID: 0x%02x\r\n", MyEmmcCardInfo.EmmcCid.OEM_AppliID & 0xff);
}

static void ATCmdGsensorFactoryTest(void)
{
	uint8_t ChipID;
	/* Check Chip ID */
	ChipID = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_WHO_AM_I);

	if (ChipID == WHOAMI_LIS2DH_ACC)
	{
		ATCMD_DEF_PRINT("GSENSOR: PASS, 0x%x", ChipID);
		ATCMD_DEF_PRINT("OK\r\n");
	}
	else
	{
		ATCMD_DEF_PRINT("GSENSOR: FAIL, 0x%x", ChipID);
		ATCMD_DEF_PRINT("ERROR\r\n");
	}
}

static void ATCmdSha204FactoryTest(void)
{
	extern volatile unsigned int i2c2Lock;

	uint8_t sha204_lib_return = SHA204_SUCCESS;
	uint8_t serial_number[9] = {0};
	uint8_t wakeup_response_buffer[4] = {0};

	GetExclusiveLock(&i2c2Lock);
	sha204p_init();
	sha204_lib_return = sha204c_wakeup(wakeup_response_buffer);
	FreeExclusiveLock(&i2c2Lock);
	if(SHA204_SUCCESS != sha204_lib_return)
	{
		ATCMD_DEF_PRINT("ATSHA204 Wake-up FALIED!");	
		return;
	}
	else
	{
		ATCMD_DEF_PRINT("ATSHA204 Wake-up SUCESS!\r\n");	
	}

	GetExclusiveLock(&i2c2Lock);
	sha204p_sleep();
	FreeExclusiveLock(&i2c2Lock);

	memset(serial_number, 0, sizeof(serial_number));

	sha204_lib_return = atsha204_read_sn(serial_number);
	if (SHA204_SUCCESS != sha204_lib_return)
	{
		ATCMD_DEF_PRINT("ATSHA204 serial number: FAILED!");
		return;
	}

	if (DbgCtl.FactoryTestInfoEn)
	{
		printf_array("SN", serial_number, sizeof(serial_number));
	}
}

static void ATCmdDefRtcFactoryTest(void)
{
	TimeTableT timeTable = GetRTCDatetime();
	ATCMD_DEF_PRINT("RTC:%04d-%02d-%02d %02d:%02d:%02d",
				timeTable.year, timeTable.month, timeTable.day,
				timeTable.hour, timeTable.minute, timeTable.second);
	ATCMD_DEF_PRINT("OK\r\n");
}

static void ATCmdDefCanFactoryTest(void)
{
	CanRxMsgTypeDef *pCanRxMsg = NULL;
	#define CAN_TEST_DELAY_COUNT_MAX (10)
	uint32_t i = 0 , count = 0;
	#define CAN_TEST_STR "\x10\x14\x49\x02\x01\x31\x47\x31"
	char buf[sizeof(CAN_TEST_STR)] = {0};

	PreLoadFileProcessing();

	factorymodeindicatefalg = TRUE;

	ATCMD_DEF_PRINT("Enter CAN test mode!");

	CAN1_Normal_Init(500000);

	while (factorymodeindicatefalg == TRUE)
	{
		HAL_Delay(100);
		CAN1_Transmit("\x02\x09\x02\0\0\0\0\0", 8, 0x07df, 0, CAN_ID_STD, CAN_RTR_DATA);
		HAL_Delay(50);
		if (CAN1ReceiveCell.input != CAN1ReceiveCell.output)
		{
			pCanRxMsg = &(CAN1ReceiveCell.MsgBuf[CAN1ReceiveCell.output]);
			CAN1ReceiveCell.output++;
        	CAN1ReceiveCell.output %= CAN1_MAX_RECEIVE_BUF_NUM;

			if (pCanRxMsg->DLC == 8)
			{
				memset(buf, 0, sizeof(buf));
				for (i = 0; i < pCanRxMsg->DLC; i++)
				{
					buf[i] = pCanRxMsg->Data[i];
				}

				if (0 == strcmp(CAN_TEST_STR, buf))
				{
					ATCMD_DEF_PRINT("CAN data received, LEN(%d) ID(%x)", pCanRxMsg->DLC, pCanRxMsg->StdId);
					ATCMD_DEF_PRINT("OK\r\n");

					PostLoadFileProcessing();
					factorymodeindicatefalg = FALSE;
					return;
				}
				else
				{
					ATCMD_DEF_PRINT("ERROR\r\n");
					factorymodeindicatefalg = FALSE;
					PostLoadFileProcessing();
					return;
				}
			}
			else
			{
				ATCMD_DEF_PRINT("ERROR\r\n");
				factorymodeindicatefalg = FALSE;
				PostLoadFileProcessing();
				return;
			}
		}

		count++;
		if (count == CAN_TEST_DELAY_COUNT_MAX)
		{
			factorymodeindicatefalg = FALSE;
		}
	}
	ATCMD_DEF_PRINT("Time Out");
	ATCMD_DEF_PRINT("ERROR\r\n");
	PostLoadFileProcessing();
}

static void ATCmdDefUart1FactoryTest(void)
{
	#define ENTER_UART1_TEST_MODE_INFO "\r\nEnter uart1 test mode!\r\n"
	#define EXIT_UART1_TEST_MODE_INFO "\r\nExit uart1 test mode!\r\n"

	PreLoadFileProcessing();

	factorymodeindicatefalg = TRUE;

	HAL_Delay(10);
    
	UART_Init(UART1_NUMBER, 115200);
	
	ATCMD_DEF_PRINT("%s", ENTER_UART1_TEST_MODE_INFO);
	

	while (factorymodeindicatefalg == TRUE)
	{
		HAL_Delay(50);
	}

	ATCMD_DEF_PRINT("%s", EXIT_UART1_TEST_MODE_INFO);

	PostLoadFileProcessing();
}

static void ATCmdIoFactoryTest(void)
{
	char *str[2] = {"LOW", "HIGH"};
	uint8_t i = 0;
	for (i = 0; i < IO_CONTROL_NUMBER_MAX; i++)
	{
		ATCMD_DEF_PRINT("%s(Num %u)): %s", IO_INFO_STR[i], i, str[IO_Read(i)]);
	}
	ATCMD_DEF_PRINT("OK\r\n");
}

extern int ChmodemReceiveLoadFile(void);
void ATCmdDefFactoryChmodem(void)
{
	ATCMD_DEF_PRINT("Factory Chmodem mode enter!");

	PreLoadFileProcessing();

	ChmodemReceiveLoadFile();

	PostLoadFileProcessing();

	ATCMD_DEF_PRINT("Mode exit!\r\n");
}

static void ATCmdDefFactoryYmodem(void)
{
	uint8_t FileName[FILE_NAME_LENGTH];
	uint8_t TransferFlag = 0;

	ATCMD_DEF_PRINT("Ymodem mode enter!");
	ATCMD_DEF_PRINT("Input T to transfer file. Press ESC to exit!\r\n");

	// vTaskSuspendAll();
	PreLoadFileProcessing();

	factorymodeindicatefalg = TRUE;

	ymodem_init();
	while (factorymodeindicatefalg == TRUE)
	{
		while (1)
		{
			if (uartFactoryRec == 'T')
			{
				uartFactoryRec = 0;
				TransferFlag = 1;

				// Clear Data
				memset(Uart4RxBuffer, 0, sizeof(Uart4RxBuffer));
				Uart4RxCount = 0;
				
				ATCMD_DEF_PRINT("Start Transfer!\r\n");
			}

			if (TransferFlag == 1)
			{
				break;
			}

			if (factorymodeindicatefalg != TRUE)
			{
				break;
			}
			
			HAL_Delay(1000);
		}

		if (factorymodeindicatefalg != TRUE)
		{
			break;
		}
		
		if (TransferFlag == 1)
		{
			TransferFlag = 0;

			SerialDownload(FileName);
		}
	}
	
	ymodem_deinit();

	// xTaskResumeAll();
	PostLoadFileProcessing();

	ATCMD_DEF_PRINT("Mode exit!\r\n");
}

extern uint8_t FileSystemMountFlag;
extern Disk_drvTypeDef disk;
void ATCmdDefFactoryMakeFatfs(void)
{
	FRESULT res;
	#define MAKFATFS_BUF_SIZE (1024)
	uint8_t *pBUF = pvPortMalloc(MAKFATFS_BUF_SIZE);

	if (pBUF == NULL)
	{
		ATCMD_DEF_PRINT("Factory make FatFS Malloc Fail!");
	}

	memset(pBUF, 0, MAKFATFS_BUF_SIZE);

	ATCMD_DEF_PRINT("Factory make FatFS mode enter!");

	PreLoadFileProcessing();

	memset(&disk, 0, sizeof(disk));

	MX_FATFS_Init();

	res = f_mount(&EMMCFatFS, EMMCPath, 1);
	if (FR_OK == res)
	{
		ATCMD_DEF_PRINT("Factory make FatFS mount success!");
	}
	else
	{
		ATCMD_DEF_PRINT("Factory make FatFS mount fail: %d", res);
	}

	ATCMD_DEF_PRINT("Factory make FatFS start, this may take a few minutues!");

	res = f_mkfs(_TEXT("0:"), FM_EXFAT, EMMC_CLUST_SIZE_BYTE, pBUF, MAKFATFS_BUF_SIZE);
	if (res == FR_OK)
	{
		ATCMD_DEF_PRINT("Factory make FatFS success!");
	}
	else
	{
		ATCMD_DEF_PRINT("Factory make FatFS failed: %d", res);
	}

	if (pBUF != NULL)
	{
		vPortFree(pBUF);
		pBUF = NULL;
	}

	f_mount(NULL, EMMCPath, 0);
	FileSystemMountFlag = 0;
	FATFS_UnLinkDriver(EMMCPath);

	PostLoadFileProcessing();
}

static void ATCmdDefFactoryFatfsInfo(void)
{
	DWORD nclst;
	FATFS *fatfs = &EMMCFatFS;
	FRESULT res;

	ATCMD_DEF_PRINT("Factory FatfsInfo Start");
	if (FileSystemMountFlag == 0)
	{
		MX_FATFS_Init();
		res = f_mount(&EMMCFatFS, EMMCPath, 1);
		if (FR_OK == res)
		{
			ATCMD_DEF_PRINT("Factory FatfsInfo mount success!");
			FileSystemMountFlag = 1;
		}
		else
		{
			ATCMD_DEF_PRINT("Factory FatfsInfo mount fail: %d", res);
			return;
		}
	}

	res = f_getfree(EMMCPath, &nclst, &fatfs);
	if (FR_OK == res)
	{
		ATCMD_DEF_PRINT("FreeClust Num(%d) ClustSize (%d) Bytes Total Sectors (%d)"
		, nclst, EMMC_CLUST_SIZE_BYTE, EMMC_SECTOR_COUNT);
		ATCMD_DEF_PRINT("FileSystem Left(%d)M (%d)K (%u)B"
		, (nclst * EMMC_CLUST_SIZE_BYTE) / (1024 * 1024)
		, (nclst * EMMC_CLUST_SIZE_BYTE) / 1024
		, (nclst * EMMC_CLUST_SIZE_BYTE));

		ATCMD_DEF_PRINT("OK\r\n");
	}
	else
	{
		ATCMD_DEF_PRINT("Err(%d)\r\n", res);
	}
}

void ATCmdDefFactoryFirmwareUpdate(void)
{
	// vTaskSuspendAll();

	FirmwareUpdateFlagSet(IAP_BOOT_FLASH);
	
	// xTaskResumeAll();
}

static void ATCmdDefFactoryBootChmodem(void)
{
	FirmwareUpdateFlagSet(IAP_BOOT_CHMODEM);
}

static void ATCmdDefFactoryBootYmodem(void)
{
	FirmwareUpdateFlagSet(IAP_BOOT_UART);
}

#include "prot.h"
#include "file_operation.h"
#define FILE_IO_TEST_LINES (5)
static uint32_t GetSystickIncrement(uint32_t oldSystick)
{
	uint32_t tmp = HAL_GetTick();
	return (tmp >= oldSystick) ? (tmp - oldSystick) : (0xffffffff - tmp + oldSystick);
}
static void ATCmdDefFactoryFileIOTest(void)
{
	OemValFsiHandleT File1, File2;
	OemValFsiFileOpenModeT Mode = OEM_FSI_FILE_OPEN_SHARE;
	char Buf[64] = {0};
	uint32_t startSystick = HAL_GetTick();
	uint32_t increment = 0;
	if (OEM_FSI_SUCCESS == FileOpen(&File1, "test1.txt", Mode) 
		&& OEM_FSI_SUCCESS == FileOpen(&File2, "test2.txt", Mode))
	{
		increment = GetSystickIncrement(startSystick);
		ATCMD_DEF_PRINT("FileOpen(%d ms) OK", increment);
		uint32_t size = strlen("HELLOd\r\n");
		uint16_t i = 0;

		while (i < FILE_IO_TEST_LINES)
		{
			memset(Buf, 0, sizeof(Buf));
			sprintf(Buf, "HELLO%d\r\n", i);
			startSystick = HAL_GetTick();
			if (OEM_FSI_SUCCESS == FileWrite(Buf, 1, &size, File1))
			{
				increment = GetSystickIncrement(startSystick);
				ATCMD_DEF_PRINT("File(%d) Write(%d ms) OK:%d", File1, increment, i);
			}
			else
			{
				ATCMD_DEF_PRINT("File(%d) Write Fail:%d", File1, i);
			}
			i++;
		}

		startSystick = HAL_GetTick();
		if (OEM_FSI_SUCCESS == FileClose(File1))
		{
			increment = GetSystickIncrement(startSystick);
			ATCMD_DEF_PRINT("File(%d) Close(%d ms) OK", File1, increment);
			
			if (OEM_FSI_SUCCESS == FileOpen(&File1, "test1.txt", Mode))
			{
				ATCMD_DEF_PRINT("FileIO Test Start");

				i = 0;
				while (i < FILE_IO_TEST_LINES)
				{
					memset(Buf, 0, sizeof(Buf));
					startSystick = HAL_GetTick();
					if (OEM_FSI_SUCCESS == FileRead(Buf, 1, &size, File1))
					{
						increment = GetSystickIncrement(startSystick);
						ATCMD_DEF_PRINT("File(%d) Read(%d ms) OK: %s", File1, increment, Buf);
						startSystick = HAL_GetTick();
						if (OEM_FSI_SUCCESS == FileWrite(Buf, 1, &size, File2))
						{
							increment = GetSystickIncrement(startSystick);
							ATCMD_DEF_PRINT("File(%d) Write(%d ms) OK(%d)", File2, increment, i);
						}
					}
					else
					{
						ATCMD_DEF_PRINT("File(%d) Read Err", File1);
						break;
					}

					i++;
				}

				startSystick = HAL_GetTick();
				if (OEM_FSI_SUCCESS == FileClose(File1))
				{
					increment = GetSystickIncrement(startSystick);
					ATCMD_DEF_PRINT("File(%d) Close(%d ms) OK", File1, increment);
				}
				else
				{
					ATCMD_DEF_PRINT("File(%d) Close Err", File1);
				}

				startSystick = HAL_GetTick();
				if (OEM_FSI_SUCCESS == FileClose(File2))
				{
					increment = GetSystickIncrement(startSystick);
					ATCMD_DEF_PRINT("File(%d) Close(%d ms) OK", File2, increment);
				}
				else
				{
					ATCMD_DEF_PRINT("File(%d) Close Err", File2);
				}

				startSystick = HAL_GetTick();
				if (OEM_FSI_SUCCESS == FileRename("test1.txt", "rename.txt"))
				{
					increment = GetSystickIncrement(startSystick);
					ATCMD_DEF_PRINT("File Rename(%d ms) (%s) to (%s)OK", increment,  "test1.txt", "rename.txt");
				}

				startSystick = HAL_GetTick();
				if (OEM_FSI_SUCCESS == FileRemove("rename.txt"))
				{
					increment = GetSystickIncrement(startSystick);
					ATCMD_DEF_PRINT("File Remove(%d ms) (%s) OK", increment, "rename.txt");
				}

				startSystick = HAL_GetTick();
				if (OEM_FSI_SUCCESS == FileOpen(&File2, "test2.txt", Mode))
				{
					increment = GetSystickIncrement(startSystick);
					ATCMD_DEF_PRINT("FileIO Open(%d ms) (%s) OK", increment, "test2.txt");

					i = 0;
					while (i < FILE_IO_TEST_LINES)
					{
						memset(Buf, 0, sizeof(Buf));
						startSystick = HAL_GetTick();
						if (OEM_FSI_SUCCESS == FileRead(Buf, 1, &size, File2))
						{
							increment = GetSystickIncrement(startSystick);
							ATCMD_DEF_PRINT("File(%d) Read(%d ms): %s", File2, increment, Buf);
						}
						else
						{
							ATCMD_DEF_PRINT("File(%d) Read Err", File2);
							break;
						}

						i++;
					}

					startSystick = HAL_GetTick();
					if (OEM_FSI_SUCCESS == FileClose(File2))
					{
						increment = GetSystickIncrement(startSystick);
						ATCMD_DEF_PRINT("File(%d) Close(%d ms) OK", File2, increment);
					}
					else
					{
						ATCMD_DEF_PRINT("File(%d) Close Err", File2);
					}
				}

				if (i == FILE_IO_TEST_LINES)
				{
					ATCMD_DEF_PRINT("FileIO Test End OK");
				}
				else
				{
					ATCMD_DEF_PRINT("FileIO Test End Err");
				}

				return;
			}
			else
			{
				ATCMD_DEF_PRINT("File Open test1.txt Err");
			}
		}

		if (OEM_FSI_SUCCESS == FileClose(File2))
		{
			ATCMD_DEF_PRINT("File(%d) Close OK", File2);
		}
		else
		{
			ATCMD_DEF_PRINT("File(%d) Close Err", File2);
		}

		// if (OEM_FSI_SUCCESS == FileVerify("test.txt"))
		// {
		// 	ATCMD_DEF_PRINT("FileVerify OK");
		// }
	}
	else
	{
		ATCMD_DEF_PRINT("FileOpen Fail");
	}
}

static void ATCmdDefDefault(void)
{
	DebugLog("ERROR");
}

static void ATCmdDefReset(void)
{
	// Print Out
	ATCMD_DEF_PRINT("*RST:Done\r\n");
	ATCMD_DEF_PRINT("OK\r\n");
	// Jump to bootloader
	MCUReset();
}

static void ATCmdDefSleep(uint8_t Len, int32_t Param)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}
	else
	{
		if (Param > 0)
		{
			ATCMD_DEF_PRINT("[%s] SLEEP(%dS)\r\n", FmtTimeShow(), Param);
			// Set Alarm Time
			MCUDeepSleep(Param);
		}
	}
	ATCMD_DEF_PRINT("[%s] WAKE UP OK\r\n", FmtTimeShow());
}

static void ATCmdDefPwrCtl(uint8_t Len, int32_t Param, uint8_t *dataBuf)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("*PWR: (%s)", "DEBUG_PWR_INFO");
	}
	else
	{
		switch (Param)
		{
		default:
			ATCMD_DEF_PRINT("*PWR: (%s)", "DEBUG_PWR_INFO");
			break;
		}
	}
	
	ATCMD_DEF_PRINT("OK\r\n");
}

static void ATCmdDefPrototype(void)
{
	uint8_t Count = 0;

	ATCMD_DEF_PRINT("==SUPPORT AT AS FOLLOW==\r\n");
	for (Count = 0; Count < (sizeof(ATCmdTypeTable) / sizeof(ATCmdTypeTable[0])); Count++)
	{
		ATCMD_DEF_PRINT("    %s", ATCmdTypeTable[Count].cmdStr);
	}
	ATCMD_DEF_PRINT("==END OF THE AT LIST==");
}

static void ATCmdGsensorTest(uint8_t Len, int Param)
{
	//uint8_t response;

	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}
	else
	{
		switch (Param)
		{
		case 0:
		{
			GsensorStop();
		}
		break;

		case 1:
		{
			GsensorStart();
		}
		break;

		case 2:
		{
			LIS2DH_GetWHO_AM_I();
		}
		break;

		case 3:
		{
			GsensorGetRawAxes();
		}
		break;
		}
	}
	ATCMD_DEF_PRINT("OK\r\n");
}

static void ATCmdDefGPIOREAD(uint8_t Len, int Param, uint8_t *dataBuf)
{
	bool val;

	if (Len == 0)
	{
		ATCMD_DEF_PRINT("[%s] Error ", FmtTimeShow());
		return;
	}

	val = IO_Read(Param);
	ATCMD_DEF_PRINT("val %d \r\n", val);
	ATCMD_DEF_PRINT("OK\r\n");
}

static void ATCmdDefGPIOWRITE(uint8_t Len, int32_t Param, uint8_t *dataBuf)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("[%s] Error ", FmtTimeShow());
		return;
	}

	if (dataBuf != NULL)
	{
		if (dataBuf[0] == '0')
			IO_Write(Param, FALSE);
		if (dataBuf[0] == '1')
			IO_Write(Param, TRUE);
	}

	ATCMD_DEF_PRINT("OK\r\n");
}

static void ATCmdDefWatchDog(uint8_t Len, int32_t Param, uint8_t *dataBuf)
{
	uint32_t milliseconds;

	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}
	else
	{
		switch (Param)
		{
		case 0:
		{
			WatchdogTick();
			// Print Out
			ATCMD_DEF_PRINT("[%s] *WDT: Tick", FmtTimeShow());
		}
		break;

		case 1:
		{
			milliseconds = atoi((const char *)dataBuf);

			WatchdogEnable(milliseconds);
			// Print Out
			ATCMD_DEF_PRINT("[%s] *WDT: Start with %d ms", FmtTimeShow(), milliseconds);
		}
		break;

		default:
			break;
		}
	}
	ATCMD_DEF_PRINT("OK\r\n");
}

static void ATCmdDefDbgCtl(uint8_t Len, int32_t Param, uint8_t *dataBuf)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}
	else
	{
		if (Param < LOG_EN_MAX)
		{
			if (*dataBuf == '0')
			{
				LogEnInfoControl(Param, FALSE);
			}
			else
			{
				LogEnInfoControl(Param, TRUE);
			}
		}
		else if (Param == 255)
		{
			uint8_t *pdbgctl = (uint8_t *)&DbgCtl;
			uint8_t i = 0;

			for (i = 0; i < (sizeof(DbgCtl)); i++)
			{
				if (*dataBuf == '0')
				{
					pdbgctl[i] = FALSE;
				}
				else
				{
					pdbgctl[i] = TRUE;
				}
			}
		}
		else
		{
			ATCMD_DEF_PRINT("Param Error\r\n");
			return;
		}
	}
	ATCMD_DEF_PRINT("OK\r\n");
}

static void ATCmdDefCanBaudTest(uint8_t Len, int32_t Param)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}
	else
	{
		ATCMD_DEF_PRINT("CAN TEST Param(%d)", Param);
		CAN1_Normal_Init(Param * 1000);
	}
	ATCMD_DEF_PRINT("OK\r\n");
}

extern int ChmodemReceiveLoadFile(void);
static void ATCmdDefChmodem(uint8_t Len, int32_t Param)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}

	ATCMD_DEF_PRINT("Chmodem mode enter, baudrate = %d", Param);
	UART_Init(UART4_NUMBER, Param);

	PreLoadFileProcessing();

	ChmodemReceiveLoadFile();

	PostLoadFileProcessing();

	UART_Init(UART4_NUMBER, 115200);

	ATCMD_DEF_PRINT("Mode exit!\r\n");
}

static void ATCmdDefWifiMode(uint8_t Len, int32_t Param)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}

	switch (Param)
	{
		case 0:
			ATCMD_DEF_PRINT("Wifi Run.\r\n");
			WifiStartUp();
			break;

		case 1:
			ATCMD_DEF_PRINT("Wifi Enter Flash Download Mode.\r\n");
			WifiEnterFlashDownLoadMode();
			break;

		default:
			ATCMD_DEF_PRINT("Param Error\r\n");
			return;
	}

	ATCMD_DEF_PRINT("OK\r\n");
}

static void ATCmdDefWifiOperations(uint8_t Len, int32_t Param, uint8_t *dataBuf)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}

	if (Param == 255)
	{
		uint16_t i = 0;
		ATCMD_DEF_PRINT("========List WIFI Operations========\r\n");
		for (i = 0; i < WIFI_CMD_MAX; i++)
		{
			ATCMD_DEF_PRINT("[%d]: %s", i, WIFICmdStrArray[i]);
		}
		ATCMD_DEF_PRINT("========List WIFI Operations End====\r\n");

		return;
	}

	switch (Param)
	{
		case WIFI_CMD_RAW_DATA:
		{
			uint16_t len = atoi((char *)dataBuf);
			uint8_t *ptemp = pvPortMalloc(WIFI_MAX_ITEM_SIZE);
			uint16_t i = 0;

			if (ptemp == NULL)
			{
				ATCMD_DEF_PRINT("ATCmdDefWifiOperations WIFI_CMD_RAW_DATA malloc fail\r\n");
			}

			if (len > WIFI_MAX_ITEM_SIZE)
			{
				len = WIFI_MAX_ITEM_SIZE;
			}

			for (i = 0; i < WIFI_MAX_ITEM_SIZE; i++)
			{
				ptemp[i] = (i % 9 + '0');
			}

			if (WIFISendRawData(ptemp, len))
			{
				ATCMD_DEF_PRINT("ATCmdDefWifiOperations ERROR(%d)\r\n", WIFI_CMD_RAW_DATA);
				return;
			}

			vPortFree(ptemp);
		}
		break;

		case WIFI_CMD_SET_SSID_KEY:
		break;

		case WIFI_CMD_GET_IPV4_ADDR:
		break;

		case WIFI_CMD_SPI_BUS_ERROR_QUERY:
		break;

		default:
		ATCMD_DEF_PRINT("ATCmdDefWifiOperations cmd err\r\n");
		// break;
		return;
	}

	ATCMD_DEF_PRINT("OK\r\n");
}

static void ATCmdDefBleOperations(uint8_t Len, int32_t Param, uint8_t *dataBuf)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}

	if (Param == 255)
	{
		uint16_t i = 0;
		ATCMD_DEF_PRINT("========List BLE Operations========\r\n");
		for (i = 0; i < BLE_CMD_MAX; i++)
		{
			ATCMD_DEF_PRINT("[%d]: %s", i, BLECmdStrArray[i]);
		}
		ATCMD_DEF_PRINT("========List BLE Operations End====\r\n");

		return;
	}

	switch (Param)
	{
		case BLE_CMD_SET_ADV_NAME:
		BLESetAdvName((char *)dataBuf, (strlen((char *)dataBuf) > BLE_NAME_LEN_MAX) ? BLE_NAME_LEN_MAX : strlen((char *)dataBuf));
		break;

		case BLE_CMD_GET_ADV_ADDR:
		if (BLEGetAdvAddr())
		{
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_GET_ADV_ADDR);
			return;
		}
		break;

		case BLE_CMD_START_ADV:
		BLESetAdvStart();
		break;

		case BLE_CMD_NUS_DATA_SEND:
		if (BLESendToNus(dataBuf, strlen((char *)dataBuf)))
		{
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_NUS_DATA_SEND);
			return;
		}
		break;

		case BLE_CMD_BIN_INFO_SEND:
		#if RTOS_AND_FS_SUPPORT
		if (BLESendBinInfo((char *)dataBuf, 0, 0, 240))//0x1B000
		{
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_BIN_INFO_SEND);
			return;
		}
		#else
		{
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR RTOS_AND_FS_SUPPORT(%d)\r\n", RTOS_AND_FS_SUPPORT);
			return;
		}
		#endif /* RTOS_AND_FS_SUPPORT */
		break;

		case BLE_CMD_BIN_DATA_SEND:
		ATCMD_DEF_PRINT("ATCmdDefBleOperations not support(%d)\r\n", BLE_CMD_BIN_DATA_SEND);
		break;

		case BLE_CMD_ENTER_BOOT:
		if (BLEEnterBoot())
		{
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_ENTER_BOOT);
			return;
		}
		break;

		case BLE_CMD_RESET:
		if (BLEReset())
		{
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_RESET);
			return;
		}
		break;

		case BLE_CMD_READY_QUERY:
		if (BLEReadyQuery())
		{
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_READY_QUERY);
			return;
		}
		break;

		case BLE_CMD_VERION_QUERY:
		if (BLEVersionQuery())
		{
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_VERION_QUERY);
			return;
		}
		break;

		case BLE_CMD_BUS_ERROR_QUERY:
		if (BLEBusErrorQuery())
		{
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_BUS_ERROR_QUERY);
			return;
		}
		break;

		case BLE_CMD_BUS_RAW_DATA:
		if (BLESendRawData(240))
		{
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_BUS_RAW_DATA);
			return;
		}
		break;

		case BLE_CMD_GPIO_SET_CLEAR:
		{
			uint32_t set = 0;
			if (*dataBuf == '0')
			{
				set = 0;
			}
			else
			{
				set = 1;
			}

			if (BLEGpioSetOrClear(1, set))
			{
				ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_GPIO_SET_CLEAR);
				return;
			}
		}
		break;

		case BLE_CMD_GPIO_SET_INPUT:
		{
			uint32_t pull = atoi((char *)dataBuf);
			ATCMD_DEF_PRINT("ATCmdDefBleOperations ble gpio set input pull(%d)\r\n", pull);
			if (BLEGpioSetInput(30, pull))
			{
				ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_GPIO_SET_INPUT);
				return;
			}
		}
		break;

		case BLE_CMD_GPIO_READ:
		{
			if (BLEGpioRead(1))
			{
				ATCMD_DEF_PRINT("ATCmdDefBleOperations ERROR(%d)\r\n", BLE_CMD_GPIO_READ);
				return;
			}
		}
		break;

		default:
		ATCMD_DEF_PRINT("ATCmdDefBleOperations cmd err\r\n");
		// break;
		return;
	}

	ATCMD_DEF_PRINT("OK\r\n");
}

static void ATCmdDefUpdateFirmwareFromFile(uint8_t Len, int32_t Param, uint8_t *dataBuf)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}

	ATCMD_DEF_PRINT("ATCmdDefUpdateFirmwareFromFile Start");
	extern uint8 at_update(char *filename);
	if (!at_update((char *)dataBuf))
	{
		ATCMD_DEF_PRINT("Update Failed\r\n");
	}
	else
	{
		ATCMD_DEF_PRINT("OK\r\n");
	}
}

static void ATCmdDefHttpTest(uint8_t Len, int32_t Param, uint8_t *dataBuf)
{
	if (Len == 0)
	{
		ATCMD_DEF_PRINT("ERROR\r\n");
		return;
	}

	if (Param == 1)
	{
		void HttpInitForPostTest(void);
		ATCMD_DEF_PRINT("ATCmdDefHttpTest post\r\n");
		HttpInitForPostTest();
		ATCMD_DEF_PRINT("OK\r\n");
		return;
	}
	
	ATCMD_DEF_PRINT("ERROR\r\n");
}

extern uint8_t FileSystemMountFlag;
void AtReadFileAndPrintOut(uint8_t *pFileName)
{
	FRESULT res;
	char namebuf[64] = {0};
	FIL file;
	uint32_t rcount = 0;
	uint8_t buf[32] = {0};

	PreLoadFileProcessing();

	memset(&file, 0, sizeof(file));

	ATCMD_DEF_PRINT("AtReadFileAndPrintOut Start\r\n");

	if (pFileName == NULL)
	{
		ATCMD_DEF_PRINT("pFileName == NULL\r\n");
		PostLoadFileProcessing();
		return;
	}

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
			ATCMD_DEF_PRINT("Mount failed: %d", res);
			PostLoadFileProcessing();
			return;
		}
	}

	namebuf[0] = '0';
	namebuf[1] = ':';
	namebuf[2] = '/';
	memcpy(namebuf + 3, pFileName, ((strlen((char *)pFileName) + 4) < sizeof(namebuf)) ? strlen((char *)pFileName): (sizeof(namebuf) - 4));
	res = f_open(&file, namebuf, FA_READ | FA_OPEN_EXISTING);
	if (res != FR_OK)
	{
		ATCMD_DEF_PRINT("Open file(%s) failed: %d", namebuf, res);
	}
	else
	{
		if (FR_OK == f_sync(&file))
		{
			if (FR_OK == f_lseek(&file, 0))
			{
				for (;;)
				{
					rcount = 0;
					memset(buf, 0, sizeof(buf));
					res = f_read(&file, buf, sizeof(buf) - 1, &rcount);
					if (res == FR_OK)
					{
						if (rcount != 0)
							ATCmdPrintf(TRUE, "%s", buf);
						else
							break;
					}
					else
					{
						break;
					}
				}
			}
			else
			{
				f_close(&file);
				ATCMD_DEF_PRINT("Read lseek failed");
			}
		}
		else
		{
			f_close(&file);
			ATCMD_DEF_PRINT("Read sync failed");
		}
	}

	if (FileSystemMountFlag == 0)
	{
		FATFS_UnLinkDriver(EMMCPath);

		res = f_mount(NULL, EMMCPath, 0);
		if (res == FR_OK)
		{
		}
		else
		{
			ATCMD_DEF_PRINT("Unmount failed:%d", res);
			PostLoadFileProcessing();
			return;
		}
	}

	PostLoadFileProcessing();
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
