#include <string.h>
#include "flash.h"
#include "fatfs.h"
#include "ymodem.h"
#include "usart.h"
#include "uart_api.h"
#include "usrtimer.h"
#include "SdioEmmcDrive.h"

#define Serial_PutString(format, ...) DebugPrintf(DbgCtl.FileInfoEn, "\r\n" format, ##__VA_ARGS__)

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

extern void CheckUARTRecTimerCallback(uint8_t Status);
extern uint8_t FileSystemMountFlag;

/* USER CODE BEGIN PV */
void SerialDownload(uint8_t *FileName)
{
	uint8_t tab_1024[1024] = {0}; 
	int32_t Size = 0;
	uint8_t Number[10] = {0};

	memset(FileName, 0, FILE_NAME_LENGTH);
	FileName[0] = '0';
	FileName[1] = ':';
	FileName[2] = '/';
	
	ymodem_enter();
	HAL_Delay(100);
	// Print Out
	Size = Ymodem_Receive(&tab_1024[0], FileName);
	ymodem_exit();
	HAL_Delay(100);
	if (Size > 0)
	{
		ymodem_close();
		// Send Successfully Info To UART
		Serial_PutString(">>File download Successfully");
		Serial_PutString(">>File Name:%s", FileName);
		Int2Str(Number, Size);
		Serial_PutString(">>File Size:%s Bytes", Number);
	}
	else
	{
		// Show Fail Programming
		Serial_PutString("Filedownload Fail");
		if (Size == -1)
			Serial_PutString("The file size is to big!");
		else if (Size == -2)
			Serial_PutString("File write failed!");
		else if (Size == -3)
			Serial_PutString("File open failed!");
		else if (Size == -5)
			Serial_PutString("Aborted by user.");
		else
			Serial_PutString("Failed to receive the file");

		if (Size == -3)
		{

		}
		else
		{
			ymodem_close();
		}
	}
	Serial_PutString("\r\nInput T to transfer file. Press ESC to exit!\r\n");
}

uint8_t ymodem_init(void)
{
	FRESULT res;

	if (FileSystemMountFlag == 0)
	{
		MX_FATFS_Init();

		res = f_mount(&EMMCFatFS, EMMCPath, 1);
		if (res == FR_OK)
		{
			FileSystemMountFlag = 1;
			return 0;
		}
		else
		{
			FATFS_UnLinkDriver(EMMCPath);
			Serial_PutString("ymodem_openfile mount failed: %d", res);
			return 1;
		}
	}
	return 0;
}

uint8_t ymodem_deinit(void)
{
	// FRESULT res;

	// if (FileSystemMountFlag == 1)
	// {
	// 	FATFS_UnLinkDriver(EMMCPath);

	// 	res = f_mount(NULL, EMMCPath, 0);
	// 	if (res == FR_OK)
	// 	{
	// 		FileSystemMountFlag = 0;
	// 		return 0;
	// 	}
	// 	else
	// 	{
	// 		Serial_PutString("ymodem_deinit unmount failed:%d", res);
	// 		return 1;
	// 	}
	// }
	return 0;
}

void ymodem_enter(void)
{
	UART_Interrupt_DeInit(&huart4);
}

void ymodem_exit(void)
{
	UART_Interrupt_Init(&huart4);
}

uint8_t ymodem_openfile(char * filename)
{
	FRESULT res;

	res = f_open(&EMMCFile, filename, FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
	if (res == FR_OK)
	{
		// Serial_PutString("ymodem_openfile open success!");
		return 0;
	}
	else
	{
		// Serial_PutString("ymodem_openfile open failed!");
		return res;
	}
}

uint32_t ymodem_write(const char * wbuf, uint16_t wlen)
{
	FRESULT res;
	uint32_t wcount = 0;

	res = f_write(&EMMCFile, wbuf, wlen, &wcount);
	if (res == FR_OK)
	{
		return wcount;
	}
	else
	{
		return 0;
	}
}

uint16_t ymodem_close(void)
{
	FRESULT res;

	res = f_close(&EMMCFile);
	if (res == FR_OK)
	{
		return 0;
	}
	else
	{
		Serial_PutString("ymodem_close failed!");
		return 1;
	}
}

int32_t Receive_Byte(uint8_t *c, uint32_t timeout)
{
  SoftwareTimerStart(&UARTRecTimer);

  while (1)
  {
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) != RESET)
    {
      *c = (uint8_t)(huart4.Instance->RDR & (uint8_t)0xff);
      SoftwareTimerReset(&UARTRecTimer, CheckUARTRecTimerCallback, CHECK_UART4_REC_TIMEOUT);
      SoftwareTimerStop(&UARTRecTimer);
      return 0;
    }

    if (IsSoftwareTimeOut(&UARTRecTimer) == TRUE)
    {
      break;
    }
  }

  SoftwareTimerReset(&UARTRecTimer, CheckUARTRecTimerCallback, CHECK_UART4_REC_TIMEOUT);
  
  SoftwareTimerStop(&UARTRecTimer);
  return -1;
}

uint32_t Send_Byte(uint8_t c)
{
  huart4.Instance->TDR = c;
  while (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TXE) == RESET)
  {
  }
  
  return 0;
}

// uint32_t FLASH_GetWRP(void)
// {
// 	/* Return the FLASH write protection Register value */
// 	return (uint32_t)(READ_REG(FLASH->WRPR));
// }

void FLASH_If_Init(void)
{
	/* Unlock the Program memory */
	HAL_FLASH_Unlock();

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Clear all FLASH flags */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_ALL_ERRORS);
}

void FLASH_BUSY_WAIT(void)
{
	while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY))
	{

	}
}

// uint32_t FLASH_If_GetWriteProtectionStatus(void)
// {
// 	/* Test if any page of Flash memory where program user will be loaded is write protected */
// 	if ((FLASH_GetWRP() & FLASH_PROTECTED_PAGES) != FLASH_PROTECTED_PAGES)
// 	{
// 		return 1;
// 	}
// 	else
// 	{
// 		return 0;
// 	}
// }

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;
  
  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }
  
  return bank;
}
HAL_StatusTypeDef FlashMemoryErasePage(uint32_t StartAddr, uint32_t EndAddr)
{
	// #define ADDR_FLASH_PAGE_128   ((uint32_t)0x08040000) /* Base @ of Page 128, 2 Kbytes */
	FLASH_EraseInitTypeDef EraseInitStruct;
	HAL_StatusTypeDef Status = HAL_OK;
	uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0, another = 0;
	uint32_t PAGEError = 0, addr1 = 0, addr2 = 0;

	if (EndAddr >= ((uint32_t)0x08040000))
	{
		addr1 = StartAddr;
		addr2 = ((uint32_t)0x08040000) - 1;
		another = 1;
	}
	else
	{
		addr1 = StartAddr;
		addr2 = EndAddr;
	}

ERASE_ANOTHER_BANK:
	/* Get the 1st page to erase */
	FirstPage = GetPage(addr1);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = GetPage(addr2) - FirstPage + 1;
	/* Get the bank */
	BankNumber = GetBank(addr1);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = BankNumber;
	EraseInitStruct.Page = FirstPage;
	EraseInitStruct.NbPages = NbOfPages;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
	Status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	if (Status != HAL_OK)
	{
		/*
      Error occurred while page erase.
      User can add here some code to deal with this error.
      PAGEError will contain the faulty page and then to know the code error on this page,
      user can call function 'HAL_FLASH_GetError()'
    */
		return Status;
	}

	if (another)
	{
		another--;
		addr1 = (uint32_t)0x08040000;
		addr2 = EndAddr;
		goto ERASE_ANOTHER_BANK;
	}

	return Status;
}

uint32_t FLASH_If_Write(__IO uint32_t *FlashAddress, uint64_t *Data, uint16_t DataLength)
{
	uint32_t i = 0;

	for (i = 0; (i < DataLength) && (*FlashAddress <= (USER_FLASH_END_ADDRESS - 8)); i++)
	{
		/* the operation will be done by word */
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, *FlashAddress, *(Data + i)) == HAL_OK)
		{

			/* Check the written value */
			if (*(uint64_t *)*FlashAddress != *(Data + i))
			{
				return (2);
			}
			/* Increment FLASH destination address */
			*FlashAddress += 8;
		}
		else
		{
			return (1);
		}
	}
	return (0);
}

void MemoryFlashUnlock(void)
{
	/* Unlock the Flash Program Erase controller */
    FLASH_If_Init();

	// /* Test if any sector of Flash memory where user application will be loaded is write protected */
    // if (FLASH_If_GetWriteProtectionStatus() != 0)
    // {
    //   extern FLASH_ProcessTypeDef pFlash;
    //   /* Unlock the Program memory */
    //   __HAL_UNLOCK(&pFlash);
    // }
}

extern void MCUReset(void);
uint8_t FirmwareUpdateFlagSet(uint32_t flag)
{
	uint32_t writeaddr = 0;
	FirmwareInfoTypeDef FirmwareInfo;
	memset(&FirmwareInfo, 0, sizeof(FirmwareInfo));

	FirmwareInfo.IAPFlag = flag;
	FirmwareInfo.FirmwareSize = ((FirmwareInfoTypeDef *)IAP_FLAG)->FirmwareSize;
	strncpy(FirmwareInfo.Version, ((FirmwareInfoTypeDef *)IAP_FLAG)->Version, strlen(APP_DEFAULT_VERSION));
	FirmwareInfo.SizeCrc16 = ((FirmwareInfoTypeDef *)IAP_FLAG)->SizeCrc16;

	__disable_irq();
	MemoryFlashUnlock();
	FLASH_BUSY_WAIT();
	FlashMemoryErasePage(ADDR_FLASH_PAGE_15, ADDR_FLASH_PAGE_15 + FLASH_PAGE_SIZE - 1);
	FLASH_BUSY_WAIT();
	writeaddr = IAP_FLAG;
	FLASH_If_Write(&writeaddr, (uint64_t *)&FirmwareInfo, sizeof(FirmwareInfo) / 8);
	FLASH_BUSY_WAIT();

	HAL_FLASH_Lock();
	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Clear all FLASH flags */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_ALL_ERRORS);
	FLASH_BUSY_WAIT();
	__ISB();
	__DSB();
	__DMB();
	__enable_irq();
	HAL_Delay(50);

	//Check  the written value
	if ((((FirmwareInfoTypeDef *)IAP_FLAG)->IAPFlag == (uint32_t)flag))
	{
		Serial_PutString(">>Flag Set Success Flag Value(%d)", flag);
		// MCUReset();
		return 0;
	}
	else
	{
		Serial_PutString(">>Flag Set Fail");
		return 1;
	}
}

uint8_t InitEmmc(void)
{
	if (EmmcInit())
	{
		Serial_PutString("\r\nEmmc Init error!");
		return 1;
	}
	else
	{
		uint16_t waitCount = 0;
		while (EMMC_CARD_TRANSFER != EmmcGetState());
		{
			waitCount++;
			if (waitCount == 0xfff)
			{
				return 2;
			}
		}
	}

	return 0;
}

uint8_t FirmwareAreaErase(uint32_t address, FirmwareInfoTypeDef *pFirmwareInfo)
{
	// uint16_t count = 0, i = 0;
	// uint32_t eraseaddr = 0xffffffff;

	if ((address < EMMC_FIRMWARE_INFO_START_ADDR) || (address > (EMMC_FIRMWARE_START_ADDR + EMMC_FIRMWARE_MAX_SIZE)))
	{
		Serial_PutString("FirmwareAreaErase param error!");
		return 1;
	}

	// count = pFirmwareInfo->FirmwareSize / W25Q128_PHYSICAL_SECTOR_SIZE_BYTE;
	// count++;
	// if (pFirmwareInfo->FirmwareSize % W25Q128_PHYSICAL_SECTOR_SIZE_BYTE)
	// 	count++;

	// eraseaddr = address & (~W25Q128_SECTOR_ADDRESS_ALIGNMENT_MASK);
	// for (i = 0; i < count; i++)
	// {
	// 	if (W25Q128_EraseSector(eraseaddr))
	// 	{
	// 		Serial_PutString("FirmwareAreaErase error!");
	// 		return 2;
	// 	}
	// 	eraseaddr += W25Q128_PHYSICAL_SECTOR_SIZE_BYTE;
	// }
	
	return 0;
}

uint8_t FirmwareInfoSet(uint32_t address, FirmwareInfoTypeDef *pFirmwareInfo)
{
	if ((address < EMMC_FIRMWARE_INFO_START_ADDR) || (address > (EMMC_FIRMWARE_INFO_START_ADDR + EMMC_PHYSICAL_SECTOR_SIZE_BYTE)))
	{
		Serial_PutString("FirmwareInfoSet param error!");
		return 1;
	}

	if (Emmc_WriteData((uint8_t *)pFirmwareInfo, sizeof(FirmwareInfoTypeDef), address))
	{
		Serial_PutString("FirmwareInfoSet error!");
		return 2;
	}
	return 0;
}

uint8_t FirmwareSizeCrc16Read(uint32_t address, FirmwareSizeCrc16TypeDef *pSizeCrc16)
{
	if ((address < EMMC_FIRMWARE_INFO_START_ADDR) || (address > (EMMC_FIRMWARE_INFO_START_ADDR + EMMC_PHYSICAL_SECTOR_SIZE_BYTE)))
	{
		Serial_PutString("FirmwareSizeCrc16Read param error!");
		return 1;
	}

	if (Emmc_ReadData((uint8_t *)pSizeCrc16, sizeof(FirmwareSizeCrc16TypeDef), address))
	{
		Serial_PutString("FirmwareInfoSet error!");
		return 2;
	}

	return 0;
}

extern uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte);
uint8_t FirmwareCrc16Cal(uint32_t address, uint32_t size, uint16_t *pCrc16)
{
	uint32_t crc = 0;
	const uint32_t AddrEnd = address + size;
	uint8_t pBuf[FIRMWARE_CRC16_CAL_BUF_SIZE] = {0};
	uint16_t readsize = 0, i = 0;

	while (address < AddrEnd)
	{
		memset(pBuf, 0, sizeof(pBuf));

		if ((AddrEnd - address) > FIRMWARE_CRC16_CAL_BUF_SIZE)
		{
			readsize = FIRMWARE_CRC16_CAL_BUF_SIZE;
		}
		else
		{
			readsize = AddrEnd - address;
		}

		if (Emmc_ReadData((uint8_t *)pBuf, readsize, address))
		{
			Serial_PutString("FirmwareCrc16Cal error!");
			return 1;
		}
		address += readsize;

		for (i = 0; i < readsize; i++)
		{
			crc = UpdateCRC16(crc, pBuf[i]);
		}
	}

	crc = UpdateCRC16(crc, 0);
	crc = UpdateCRC16(crc, 0);

	*pCrc16 =  (crc & 0xffffu);

	return 0;
}

uint8_t FirmwareReceiveInit(FirmwareInfoTypeDef *pInfo, const char *name, uint32_t firmAddr)
{
	char *ptmp = NULL;
	uint8_t ret = 0, res = 0;

	ptmp = strstr(name, MODEM_VERSION_HEAD);
	if (NULL != ptmp)
	{
		strncpy(pInfo->Version, (char *)(ptmp + strlen(MODEM_VERSION_HEAD)), strlen(APP_DEFAULT_VERSION));
	}
	else
	{
		strcpy(pInfo->Version, APP_DEFAULT_VERSION);
	}

	res = InitEmmc();
	if (res != 0)
	{
		// Serial_PutString("[%s] FirmwareReceiveInit InitEmmc Fail(%d)", FmtTimeShow(), res);
		ret = 1;
		return ret;
	}

	res = FirmwareAreaErase(EMMC_FIRMWARE_INFO_START_ADDR, pInfo);
	if (res != 0)
	{
		// Serial_PutString("[%s] FirmwareReceiveInit FirmwareAreaErase Fail(%d)", FmtTimeShow(), res);
		ret = 2;
		return ret;
	}

	return ret;
}

uint8_t FirmwareVerifyAndSetUpdateFlag(FirmwareInfoTypeDef *pInfo)
{
	uint16_t Crc16 = 0;

	if (0 != FirmwareSizeCrc16Read(EMMC_FIRMWARE_START_ADDR, &(pInfo->SizeCrc16)))
	{
		return 1;
	}

	if (0 != FirmwareCrc16Cal(EMMC_FIRMWARE_START_ADDR + pInfo->FirmwareSize - pInfo->SizeCrc16.appsize
		, pInfo->SizeCrc16.appsize, &Crc16))
	{
		return 2;
	}

	if (Crc16 != pInfo->SizeCrc16.crc16)
	{
		return 3;
	}

	if (0 != FirmwareInfoSet(EMMC_FIRMWARE_INFO_START_ADDR, pInfo))
	{
		return 4;
	}

	if (0 != FirmwareUpdateFlagSet(pInfo->IAPFlag))
	{
		return 5;
	}
	
	return 0;
}
