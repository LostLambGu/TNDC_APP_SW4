/*******************************************************************************
* File Name          : uart_api.c
* Author             : Yangjie Gu
* Description        : This file provides all the uart_api functions.

* History:
*  10/23/2017 : uart_api V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

#include "cmsis_os.h"

#include "uart_api.h"
#include "rtcclock.h"
#include "initialization.h"
#include "common.h"

/* Private define ------------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/* Variables -----------------------------------------------------------------*/
#define TNDC_DEGUG_LOG_DELAY_MS (50)
#if TNDC_DEBUG_LOG_LOCK_MUTEX
extern SemaphoreHandle_t xOemDebugPrintMutex;
#elif TNDC_DEBUG_LOG_LOCK_BINARY
extern SemaphoreHandle_t xOemDebugPrintBinary;
#elif TNDC_DEBUG_LOG_LOCK_EXCLUSIVE
volatile unsigned int debugLock;
#endif

#if OEM_UART2_TO_MCU_UART1_MAP
uint16_t Uart1RxCount = 0;
uint8_t Uart1RxBuffer[UART1_RX_BUFFER_SIZE] = {0};
uint8_t Uart1ParseBuffer[UART1_RX_BUFFER_SIZE] = {0};
#endif /* OEM_UART2_TO_MCU_UART1_MAP */

uint16_t Uart4RxCount = 0;
uint8_t Uart4RxBuffer[UART_BUF_MAX_LENGTH] = {'\0'};

DebugCtlPrarm DbgCtl =
	{
		.ATCmdInfoEn = FALSE,
		.ATCmdDbgEn = FALSE,
		.BLEDriverDbgEn = FALSE,
		.BLEProcessDbgEn = TRUE,
		.Lis2dhDbgInfoEn = FALSE,
		.RTCDebugInfoEn = FALSE,
		.ApiDebugInfoEn = TRUE,
		.VersionDebugInfoEn = FALSE,
		.NetworkDbgInfoEn = FALSE,
		.AtCmdFileTransInfoEn = FALSE,
		.HttpInfoEn = TRUE,
		.DefaultInfoEn = FALSE,
		.FileInfoEn = TRUE,
		.FactoryTestInfoEn = TRUE,
		.WifiDriverDbgEn = TRUE,
		.WifiProcessDbgEn = TRUE,
};

const int LogEnInfoMcuDriver[] = {4, LIS2DH_DBG_INFOEN, RTC_DBG_INFOEN, BLE_DRIVER_DBG_EN, WIFI_DRIVER_DGB_EN};
const int LogEnInfoHttp[] = {1, HTTP_DBG_INFOEN};
const int LogEnInfoAtcmdDbg[] = {2, ATCMD_INFOEN, ATCMD_DBGEN};
const int LogEnInfoBLEWifi[] = {3, BLE_PROCESS_DBG_EN, NETWORK_DBG_INFOEN, WIFI_PROCESS_DBG_EN};
const int LogEnInfoDefault[] = {2, DEFAULT_INFOEN, VERSION_DBG_INFOEN};
const int LogEnInfoFile[] = {1, FILE_INFOEN};
const int LogEnInfoApi[] = {1, API_DBG_INFOEN};
const int LogEnFactoryTest[] = {1, FACTORY_TEST_INFOEN};

const int *LogEnInfoArray[LOG_EN_MAX] = 
{
	LogEnInfoMcuDriver, LogEnInfoHttp, LogEnInfoAtcmdDbg, LogEnInfoBLEWifi, LogEnInfoDefault, LogEnInfoFile, LogEnInfoApi, LogEnFactoryTest
};

/* Function definition -------------------------------------------------------*/
void LogEnInfoControl(uint8_t type, uint8_t onoff)
{
	uint16_t num = LogEnInfoArray[type][0];
	uint16_t i = 0;
	uint8_t *ptemp = (uint8_t *)(&DbgCtl);

	for (i = 0; i < num; i++)
	{
		ptemp[LogEnInfoArray[type][i + 1]] = onoff;
	}
}

void PutString(uint8_t *String)
{
	HAL_UART_Transmit(&DEBUG_UART_HANDLE, String, strlen((char *)String), UART_SEND_DATA_TIMEOUT);
}

uint8_t StringToInt(uint8_t *dataBuf, uint16_t dataLen, uint16_t *idxSearch, int32_t *rtnInt)
{
	uint16_t idx;
	uint16_t numLen;
	uint8_t numString[12];

	// Check if buffer is Number first
	if (dataLen == 0 || !IS_NUMBER(dataBuf[0]))
		return FALSE;

	numLen = 0;
	for (idx = 0; idx < dataLen; idx++)
	{
		if (dataBuf[idx] == ' ')
		{
			continue;
		}
		else if (!IS_NUMBER(dataBuf[idx]))
		{
			break;
		}
		else if (numLen == 10)
		{
			return FALSE;
		}
		numString[numLen++] = dataBuf[idx];
	}
	numString[numLen] = 0x0;

	if (idxSearch)
		*idxSearch += idx;

	*rtnInt = atoi((const char *)numString);
	return TRUE;
}

static uint8_t chartouint8(char c)
{
    if (IS_NUMBER_ONLY(c))
    {
        return (c - '0');
    }
    else if (IS_LOWER_ONLY(c))
    {
        return (c - 'a' + 10);
    }
    else
    {
        return (c - 'A' + 10);
    }
}

uint32_t HexAsciiToHexData(char *pBuf, uint8_t len)
{
	uint8_t i = 0;
	uint32_t result = 0;
	uint32_t ctmp = 0;

	if (len > 8)
	{
		return 0xffffffff;
	}

	for (i = 0; i < len; i++)
	{
		ctmp = (uint32_t)chartouint8(pBuf[i]);
		result <<= 4;
		result |= (ctmp & 0x0000000f);
	}

	return result;
}

void PutIntHex(uint32_t num)
{
	uint32_t temp = num;
	uint8_t carray[11];
	uint8_t i = 0, j = 0;
	carray[0] = '0';
	carray[1] = 'x';
	carray[10] = 0;
	for (i = 0; i < 8; i++)
	{
		j = (temp >> ((7 - i) * 4)) & 0x0ful;
		if (j > 9)
		{
			carray[2 + i] = j - 10 + 'a';
		}
		else
		{
			carray[2 + i] = j + '0';
		}
	}
	
	HAL_UART_Transmit(&DEBUG_UART_HANDLE, carray, 11, UART_SEND_DATA_TIMEOUT);
}

static void BytesOrderSwap(uint8_t *pBuf, uint16_t num)
{
    uint16_t i = 0, count = num / 2;
    uint8_t tmp = 0;

    for (i = 0; i < count; i++)
    {
        tmp = pBuf[i];
        pBuf[i] = pBuf[num - i - 1];
        pBuf[num - i - 1] = tmp;
    }
}

void PutIntDec(uint32_t num)
{
	uint32_t temp = num;
	uint8_t array[11] = {0};
	uint8_t i = 0;

	do
	{
		array[i] = temp % 10 + '0';
		temp /= 10;
		i++;
	} while (temp != 0 && i < 11);

	BytesOrderSwap(array, i);

	UARTPrintMassData(array, i);
}

void PutStrToUartDbg(char *str, uint16_t size)
{
#if TNDC_DEBUG_LOG_LOCK_MUTEX
	if (NULL != xOemDebugPrintMutex)
	{
		if (xSemaphoreTake(xOemDebugPrintMutex, TNDC_DEGUG_LOG_DELAY_MS / portTICK_PERIOD_MS) == pdTRUE)
		{
#elif TNDC_DEBUG_LOG_LOCK_BINARY
	if (NULL != xOemDebugPrintBinary)
	{
		if (xSemaphoreTake(xOemDebugPrintBinary, TNDC_DEGUG_LOG_DELAY_MS / portTICK_PERIOD_MS) == pdTRUE)
		{
#elif TNDC_DEBUG_LOG_LOCK_EXCLUSIVE
	GetExclusiveLock(&debugLock);
#endif
	
			HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t *)str, size, UART_SEND_DATA_TIMEOUT);

#if TNDC_DEBUG_LOG_LOCK_MUTEX
			xSemaphoreGive( xOemDebugPrintMutex );
		}
	}
	else
	{
		HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t *)str, size, UART_SEND_DATA_TIMEOUT);
	}
#elif TNDC_DEBUG_LOG_LOCK_BINARY
			xSemaphoreGive( xOemDebugPrintBinary );
		}
	}
	else
	{
		HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t *)str, size, UART_SEND_DATA_TIMEOUT);
	}
#elif TNDC_DEBUG_LOG_LOCK_EXCLUSIVE
	FreeExclusiveLock(&debugLock);
#endif
}

int32_t SerialDbgPrintf(uint8_t type, char *fmt, ...)
{
	if (type == TRUE)
	{
		int32_t cnt;
		char string[MAX_PRINTF_STR_SIZE + 2] = {'\0'};
		va_list ap;
		va_start(ap, fmt);

		cnt = vsnprintf(string, MAX_PRINTF_STR_SIZE, fmt, ap);
		if (cnt > 0)
		{
			if (cnt < MAX_PRINTF_STR_SIZE)
			{
				PutStrToUartDbg(string, cnt);
			}
			else
			{
				PutStrToUartDbg(string, MAX_PRINTF_STR_SIZE);
			}
		}
		va_end(ap);
		return (cnt);
	}
	return -1;
}

void PutStrToUartDbgBeforeRTOS(char *str, uint16_t size)
{
	HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t *)str, size, UART_SEND_DATA_TIMEOUT);
}

int32_t PrintfBeforeRTOS(char *fmt, ...)
{
	int32_t cnt;
	char string[MAX_PRINTF_STR_SIZE + 2] = {'\0'};
	va_list ap;
	va_start(ap, fmt);

	cnt = vsnprintf(string, MAX_PRINTF_STR_SIZE, fmt, ap);
	if (cnt > 0)
	{
		if (cnt < MAX_PRINTF_STR_SIZE)
		{
			PutStrToUartDbgBeforeRTOS(string, cnt);
		}
		else
		{
			PutStrToUartDbgBeforeRTOS(string, MAX_PRINTF_STR_SIZE);
		}
	}
	va_end(ap);
	return (cnt);
}

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t *)&ch, 1, UART_SEND_DATA_TIMEOUT);

	while (!(__HAL_UART_GET_FLAG(&DEBUG_UART_HANDLE, UART_FLAG_TXE)))
		;

	return (ch);
}

void PrintOutHexToAscii(char *str, uint16_t len)
{
	uint16_t i = 0;
	uint8_t outchar[2] = {0};
	uint8_t tmp = 0;
	for (i = 0; i < len; i++)
	{
		tmp = (str[i] >> 4) & 0x0f;
		outchar[0] =  (tmp <= 9) ? (tmp + '0') : (tmp - 10 + 'A');
		tmp = (str[i] >> 0) & 0x0f;
		outchar[1] =  (tmp <= 9) ? (tmp + '0') : (tmp - 10 + 'A');

		UARTPrintMassData(outchar, 2);
	}
}

void PintOutFileNameAndLine(char *Filename, uint32_t Line)
{
	UARTPrintMassData((uint8_t *)">>>>", 4);
	UARTPrintMassData((uint8_t *)Filename, strlen(Filename));
	
	PutIntDec(Line);
} 

void UARTPrintMassData(uint8_t *string, uint16_t slen)
{
	uint16_t i = 0;
    for (i = 0; i < slen; i++)
    {
        while (__HAL_UART_GET_FLAG(&DEBUG_UART_HANDLE, UART_FLAG_TXE) == 0)
            ;
        DEBUG_UART_HANDLE.Instance->TDR = ((uint8_t)0x00FF & string[i]);
    }
}

void printf_array(char *ptr_array_name, uint8_t *ptr_buffer, uint8_t buffer_size)
{
    uint8_t temp = 0;

    for (temp = 0; temp < buffer_size; temp++)
    {
        SerialDbgPrintf(1, "%s", ptr_array_name);
        SerialDbgPrintf(1, "[%d]=0x%02x", temp, ptr_buffer[temp]);
        SerialDbgPrintf(1, ", ");
        if (((temp + 1) % 4 == 0) && (temp != 0))
        {
            SerialDbgPrintf(1, "\r\n");
        }
    }
    if (temp % 4 != 0)
    {
       SerialDbgPrintf(1, "\r\n");
    }
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
