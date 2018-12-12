#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdlib.h>

#include "fatfs.h"
#include "uart_api.h"
#include "flash.h"
#include "SdioEmmcDrive.h"

#define DEFAULT_BL_SIZE 2048

#define NIB_MASK 0xF
#define NIB_SIZE 4
#define HEX_BASE 16
#define BYTE_MASK 0xFF
#define BYTE_SIZE 8

#define CHMODEM_RECEIVE_BUF_SIZE_BYTES (DEFAULT_BL_SIZE + 512)
#define CHMODEM_MAX_NAME_LEN (17 + 1)

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define CHMODEM_PRINT DebugPrintf

#define CHMODEM_UARTP (&huart4)

__IO uint8_t UARTChmodemRecModeFlag = FALSE;
UARTChmodemRecTypeDef UARTChmodemRec = {0};

extern uint8_t FileSystemMountFlag;

static uint8_t chmodem_init(void)
{
	FRESULT res;

	if (FileSystemMountFlag == 0)
	{
		MX_FATFS_Init();

		res = f_mount(&EMMCFatFS, EMMCPath, 1);
		if (res == FR_OK)
		{
			return 0;
		}
		else
		{
			FATFS_UnLinkDriver(EMMCPath);
			CHMODEM_PRINT(DbgCtl.FileInfoEn, "\r\nchmodem_openfile mount failed: %d", res);
			return 1;
		}
	}

	return 0;
}

static uint8_t chmodem_deinit(void)
{
	FRESULT res;

	if (FileSystemMountFlag == 0)
	{
		FATFS_UnLinkDriver(EMMCPath);

		res = f_mount(NULL, EMMCPath, 0);
		if (res == FR_OK)
		{
			return 0;
		}
		else
		{
			CHMODEM_PRINT(DbgCtl.FileInfoEn, "\r\nchmodem_deinit unmount failed:%d", res);
			return 1;
		}
	}

	return 0;
}

static int16_t readuart(uint8_t* pBuf, uint16_t size)
{
  int16_t count = 0;

  while (size)
  {
    if (UARTChmodemRec.outIndex != UARTChmodemRec.inIndex)
    {
      pBuf[count] = (UARTChmodemRec.pBuf)[UARTChmodemRec.outIndex];
      UARTChmodemRec.outIndex++;
      UARTChmodemRec.outIndex %= UARTChmodemRec.bufSize;
      count++;
      size--;
    }
    else
    {
      return count;
    }
  }

  return count;
}

static int16_t writefile(FIL *pf, uint8_t* pBuf, uint16_t size)
{
  FRESULT res;
  uint32_t wcount = 0;

  res = f_write(pf, pBuf, size, &wcount);
  if(res != FR_OK ) CHMODEM_PRINT(DbgCtl.FileInfoEn, "\r\nChmodem write error %d!", res);
	
	return res;
}

void sendOK(UART_HandleTypeDef *phuart)
{
  HAL_UART_Transmit(phuart, "OK", 2, UART_SEND_DATA_TIMEOUT);
}

void sendStr(UART_HandleTypeDef *phuart, char *msg)
{
  int bts = strlen(msg);

  HAL_UART_Transmit(phuart, (uint8_t *)msg, bts, UART_SEND_DATA_TIMEOUT);
}

int verifySize(FIL *pf, int esize)
{
  if(f_size(pf) == esize)
  {
    return TRUE;
  }
  
  return FALSE;
}

int waitPrologue(UART_HandleTypeDef *phuart)
{
  char pre[] = "Connected";
  int actual = strlen(pre);
  int i = 0;
  uint8_t *buf = (uint8_t *) pvPortMalloc(sizeof(char) * (actual + 1));

  if (buf == NULL)
  {
    return 1;
  }
  
  while(i < actual)
  {
    if(readuart(&buf[i], 1) == 1)
    {
      i++;
    }
  }
  buf[i] = '\0';
  if(strncmp(&pre[0], (char *)buf, strlen(pre)))
  {
    if (buf != NULL)
    {
      vPortFree(buf);
      buf = NULL;
    }
    return 2;
  }

  if (buf != NULL)
  {
    vPortFree(buf);
    buf = NULL;
  }

  sendOK(phuart);
  
  return 0;
}

int decodeSize(char *buf, int len)
{
  int i;
  int num = 0;
  int shift = 0;
  int factor = 1;

  for(i = 0; i < len; i++)
  {
    num += ((buf[len -1 - i] & (NIB_MASK << shift)) >> shift) * factor;
    shift += NIB_SIZE;
    factor *= HEX_BASE;
    num += ((buf[len -1 - i] & (NIB_MASK << shift)) >> shift) * factor;
    shift = 0;
    factor *= HEX_BASE;
  }
  
  return num;
}

int waitSize(UART_HandleTypeDef *phuart)
{
  uint8_t buf[4];
  int i = 0;

  while(i < sizeof(buf))
  {
    if(readuart(&buf[i], 1) == 1)
    {
      i++;
    }
  }
  sendOK(phuart);
  return decodeSize((char *)&buf[0], sizeof(buf));
}

/*
 * Side effect: allocates memery; caller is responsible to release it.
 */
char *waitFileName(UART_HandleTypeDef *phuart, int len)
{
  char *buf;
  int i = 0;
  buf = (char *)pvPortMalloc((len+1) * sizeof(char));

  if(!buf)
  {
    return NULL;
  }
  
  while(i < len)
  {
    if(readuart((uint8_t *)&buf[i], 1) == 1)
    {
      i++;
    }
  }
  buf[i] = '\0';
  // sendOK(phuart); Defer until the file is opened successfully
  return buf;
}

int waitFileNameLen(UART_HandleTypeDef *phuart)
{
  uint8_t buf;
  int num = 0;
  int factor = 1;
  int shift = 0;
  int i = 0;

  while(i < 1)
  {
    if(readuart(&buf, 1) == 1)
    {
      i++;
    }
  }

  num = ((buf & (NIB_MASK << shift)) >> shift) * factor;
  shift += NIB_SIZE;
  factor *= HEX_BASE;
  num += ((buf & (NIB_MASK << shift)) >> shift) * factor;
  sendOK(phuart); 
  return num;
}

int receiveFile(FIL *pf, int size, int block)
{
  int bread = 0;
  int left = size;
  // int consumed = 0;
  // int rc = 0;
  int i = 0, j = 0;

  uint8_t *bufp = pvPortMalloc(block * sizeof(char));
  if (!bufp)
  {
    return 1;
  }

  while (left > 0)
  {
    memset(bufp, 0, block * sizeof(char));
    
    if (left > block)
    {
      i = block;
    }
    else
    {
      i = left;
    }

    while (i > 0)
    {
      j = readuart(&bufp[bread], i);
      bread += j;
      i -= j;
    }
    
    if (bread > 0)
    {
      writefile(pf, &bufp[0], bread);
      left -= bread;
      // consumed += bread;
      // rc++;
      // if (rc == 3500)
      // {
      //   //break;
      //   rc = 0;
      //   //sendProgress(serP, consumed);
      // }
      bread = 0;
      // if (left < block)
      // {
      //   bread = readuart(&bufp[0], left);
      //   if (bread > 0)
      //   {
      //     writefile(pf, &bufp[0], bread);
      //     left -= bread;
      //   }
      // }
    }
  }

  vPortFree(bufp);
  return 0;
}

int receiveApp(uint32_t writeaddr, int size, int block)
{
  int bread = 0;
  int left = size;
  int i = 0, j = 0;

  uint8_t *bufp = pvPortMalloc(block * sizeof(char));
  if (!bufp)
  {
    return 1;
  }

  while (left > 0)
  {
    memset(bufp, 0, block * sizeof(char));
    
    if (left > block)
    {
      i = block;
    }
    else
    {
      i = left;
    }

    while (i > 0)
    {
      j = readuart(&bufp[bread], i);
      bread += j;
      i -= j;
    }
    
    if (bread > 0)
    {
      Emmc_WriteData(&bufp[0], bread, writeaddr);
      writeaddr += bread;
      left -= bread;
      
      bread = 0;
    }
  }

  vPortFree(bufp);
  return 0;
}

int ChmodemReceive(void)
{
  int file_size;
  int name_len;
  char *file_name;
  char namebuf[CHMODEM_MAX_NAME_LEN] = {0};
  int bl_size = DEFAULT_BL_SIZE;
  FRESULT res;
  FIL file;

  memset(&file, 0, sizeof(file));

  if (waitPrologue(CHMODEM_UARTP) != 0)
  {
    return 1;
  }

  file_size = waitSize(CHMODEM_UARTP);
  if (file_size <= 0)
  {
    return 2;
  }

  name_len = waitFileNameLen(CHMODEM_UARTP);
  if (name_len <= 0)
  {
    return 3;
  }

  file_name = waitFileName(CHMODEM_UARTP, name_len);
  if (file_name == NULL)
  {
    return 4;
  }

  if (NULL == strstr(file_name, ".rom"))
  {
    namebuf[0] = '0';
    namebuf[1] = ':';
    namebuf[2] = '/';
    memcpy(namebuf + 3, file_name, ((strlen(file_name) + 4) < CHMODEM_MAX_NAME_LEN) ? strlen(file_name) : (CHMODEM_MAX_NAME_LEN - 4));
    res = f_open(&file, namebuf, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
      if (file_name != NULL)
      {
        vPortFree(file_name);
        file_name = NULL;
      }
      return 5;
    }

    sendStr(CHMODEM_UARTP, "OK");

    if (0 != receiveFile(&file, file_size, bl_size))
    {
      if (file_name != NULL)
      {
        vPortFree(file_name);
        file_name = NULL;
      }
      f_close(&file);
      return 6;
    }

    if (verifySize(&file, file_size) == TRUE)
    {
      sendStr(CHMODEM_UARTP, "OK");
    }
    else
    {
      sendStr(CHMODEM_UARTP, "SM");
      if (file_name != NULL)
      {
        vPortFree(file_name);
        file_name = NULL;
      }
      f_close(&file);
      return 7;
    }

    if (file_name != NULL)
    {
      vPortFree(file_name);
      file_name = NULL;
    }

    f_close(&file);
  }
  else
  {
    FirmwareInfoTypeDef FirmwareInfo;
    uint8_t ret = 0, res = 0;

    memset(&FirmwareInfo, 0, sizeof(FirmwareInfo));
    FirmwareInfo.IAPFlag = IAP_BOOT_FLASH;
    FirmwareInfo.FirmwareSize = file_size;
    res = FirmwareReceiveInit(&FirmwareInfo, (char *)file_name, EMMC_FIRMWARE_INFO_START_ADDR);
    if (res != 0)
    {
      ret = 8;
      goto EXIT;
    }

    sendStr(CHMODEM_UARTP, "OK");

    if (0 != receiveApp(EMMC_FIRMWARE_START_ADDR, FirmwareInfo.FirmwareSize, bl_size))
    {
      ret = 9;
      goto EXIT;
    }

    res = FirmwareVerifyAndSetUpdateFlag(&FirmwareInfo);
    if (0 != res)
    {
      ret = 10;
      sendStr(CHMODEM_UARTP, "SM");
      goto EXIT;
    }
    else
    {
      sendStr(CHMODEM_UARTP, "OK");
    }

EXIT:
    if (file_name != NULL)
    {
      vPortFree(file_name);
      file_name = NULL;
    }

    return ret;
  }

  return 0;
}

#define CHMODEM_RECEIVE_INTERVAL_MS (1000)
#define CHMODEM_RECEIVE_NODATA_TIMEOUT_COUNT_MAX (1)
int ChmodemReceiveLoadFile(void)
{
  uint8_t *buf = pvPortMalloc(CHMODEM_RECEIVE_BUF_SIZE_BYTES);
  uint16_t count = 0;
  int ret = 0;

  if (buf == NULL)
  {
    return 3;
  }

  memset(&UARTChmodemRec, 0, sizeof(UARTChmodemRec));
  memset(buf, 0, CHMODEM_RECEIVE_BUF_SIZE_BYTES);
  UARTChmodemRec.pBuf = buf;
  UARTChmodemRec.bufSize = CHMODEM_RECEIVE_BUF_SIZE_BYTES;
  UARTChmodemRecModeFlag = TRUE;

  if (0 != chmodem_init())
  {
    UARTChmodemRecModeFlag = FALSE;
    if (buf != NULL)
    {
      vPortFree(buf);
    }
    return 2;
  }

  while (1)
  {
    ret = ChmodemReceive();
    if (0 == ret)
    {
      count = 0;
    }
    else
    {
      count++;
    }

    osDelay(CHMODEM_RECEIVE_INTERVAL_MS);

    if (CHMODEM_RECEIVE_NODATA_TIMEOUT_COUNT_MAX <= count)
    {
      break;
    }
  }

  UARTChmodemRecModeFlag = FALSE;
  if (buf != NULL)
  {
    vPortFree(buf);
  }

  return chmodem_deinit();
}
