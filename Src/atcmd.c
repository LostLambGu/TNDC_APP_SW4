/*******************************************************************************
* File Name          : atcmd.c
* Author             : Yangjie Gu
* Description        : This file provides all the atcmd functions.

* History:
*  07/05/2018 : atcmd V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

#include "atcmd.h"
#include "rtcclock.h"

#include "prot.h"

/* Private variables ---------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/* External variables --------------------------------------------------------*/
extern DebugCtlPrarm DbgCtl;
/* Private function prototypes -----------------------------------------------*/
/* Private Const prototypes -----------------------------------------------*/
const ATCompareTable_t ATCmdTypeTable[AT_CMD_DEF_LAST] =
	{
		{"AT", AT_CMD_DEF_INITIAL, TRUE},
		{"AT*LIST", AT_CMD_DEF_PROTOTYPE, TRUE},
		{"AT*RESET", AT_CMD_DEF_RESET, TRUE},
		{"AT*SLEEP", AT_CMD_DEF_SLEEP, TRUE},
		{"AT*PWR", AT_CMD_DEF_PWRCTL, TRUE},
		{"AT*EMMC", AT_CMD_DEF_EMMC, TRUE},
		{"AT*VER", AT_CMD_DEF_VER, TRUE},
		{"AT*GSENSOR", AT_CMD_DEF_GSENSOR, TRUE},
		{"AT*SHA204", AT_CMD_DEF_SHA204, TRUE},
		{"AT*RTC", AT_CMD_DEF_RTC, TRUE},
		{"AT*CAN", AT_CMD_DEF_CAN, TRUE},
		{"AT*UART1", AT_CMD_DEF_UART1, TRUE},
		{"AT*IOINFO", AT_CMD_DEF_IOINFO, TRUE},
		{"AT*GPIOREAD", AT_CMD_DEF_GPIOREAD, TRUE},
		{"AT*GPIOWRITE", AT_CMD_DEF_GPIOWRITE, TRUE},
		{"AT*WDT", AT_CMD_DEF_WATCHDOG, TRUE},
		{"AT*DBGCTL", AT_CMD_DEF_DBGCTL, TRUE},
		{"AT*CHMODEM", AT_CMD_DEF_CHMODEM, TRUE},
		{"AT*YMODEM", AT_CMD_DEF_YMODEM, TRUE},
		{"AT*MKFATFS", AT_CMD_DEF_MAKE_FATFS, TRUE},
		{"AT*FATFSINFO", AT_CMD_DEF_FATFSINFO, TRUE},
		{"AT*FILEIOTEST", AT_CMD_DEF_FILEIOTEST, TRUE},
		{"AT*FMUPDATE", AT_CMD_DEF_FIRMWARE_UPDATE, TRUE},
		{"AT*BOOTCHMODEM", AT_CMD_DEF_BOOT_CHMODEM, TRUE},
		{"AT*BOOTYMODEM", AT_CMD_DEF_BOOT_YMODEM, TRUE},
		{"AT*WIFIMODE", AT_CMD_DEF_WIFIMODE, TRUE},
		{"AT*WIFIOP", AT_CMD_DEF_WIFIOP, TRUE},
		{"AT*TASKTEST", AT_CMD_DEF_TASKTEST, TRUE},
		{"AT*BLEOP", AT_CMD_DEF_BLEOP, TRUE},
		{"AT*FILEUPDATE", AT_CMD_DEF_FILEUPDATE, TRUE},
		{"AT*HTTPTEST", AT_CMD_DEF_HTTPTEST, TRUE}
};

/* Static function declarations ----------------------------------------------*/
static uint8_t ATCmdGetType(ATCompareTable_t *checkTable, uint8_t tableSize, uint8_t *dataBuf, uint16_t dataLen, uint8_t *idxSearch);
static uint8_t ATGetCmdType(uint8_t *dataBuf, uint8_t dataLen, uint8_t *idxSearch);

/* Public functions ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static uint8_t ATCmdGetType(ATCompareTable_t *checkTable, uint8_t tableSize, uint8_t *dataBuf, uint16_t dataLen, uint8_t *idxSearch)
{
	uint8_t idx;
	uint8_t cmdLen;
	ATCompareTable_t *pTable;

	ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] ATGetType: TableSize(%d) DataLen(%d)", FmtTimeShow(), tableSize, dataLen);
	if (dataLen == 0 || dataBuf == NULL || idxSearch == NULL || checkTable == NULL)
		return AT_CMD_DEF_NULL;

	for (idx = 0; idx < tableSize; idx++)
	{
		pTable = &checkTable[idx];
		cmdLen = strlen(pTable->cmdStr);
		if (strncmp((char *)dataBuf, pTable->cmdStr, cmdLen) == 0)
		{
			if (pTable->isMatchAll)
			{
				if (dataLen == cmdLen /* || !IS_ALPHABET_OR_NUMBER(dataBuf[cmdLen]) */)
				{
					*idxSearch += cmdLen;
					ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] ATGetType: Match(%d) CmdLen(%d) CmdType(%d)", FmtTimeShow(), pTable->isMatchAll, cmdLen, pTable->cmdType);
					return pTable->cmdType;
				}
			}
			else
			{
				*idxSearch += cmdLen;
				ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] ATGetType: Match(%d) CmdLen(%d) CmdType(%d)", FmtTimeShow(), pTable->isMatchAll, cmdLen, pTable->cmdType);
				return pTable->cmdType;
			}
		}
	}
	ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] ATGetType: Can not find Match Command!", FmtTimeShow());
	return AT_CMD_DEF_NULL;
}

static uint8_t ATGetCmdType(uint8_t *dataBuf, uint8_t dataLen, uint8_t *idxSearch)
{
	return ATCmdGetType((ATCompareTable_t *)ATCmdTypeTable, (sizeof(ATCmdTypeTable) / sizeof(ATCmdTypeTable[0])), dataBuf, dataLen, idxSearch);
}

static void ParseATcmdContect(uint8_t *dataBuf, uint8_t dataLen)
{
	uint8_t idx = 0;
	uint8_t idxTemp = 0;
	uint8_t matchCmdType = AT_CMD_DEF_NULL;
	uint8_t ATCmd[TBUFFER_MAX] = {'\0'};
	uint8_t ATCmdLen = 0;
	uint8_t Param2Len = 0;
	int32_t ATParamInt = 0;
	uint8_t *ATParamP = NULL;
	uint8_t isFactoryMode = FALSE;
	uint8_t isStartToGetParameter = FALSE;
	uint8_t isHaveParameter2 = FALSE;

#ifndef AT_COMMAND_USE_LOWERCASE
	// To lower all alphabet
	for (idx = 0; idx < dataLen; idx++)
	{
		if (dataBuf[idx] >= 'a' && dataBuf[idx] <= 'z')
		{
			dataBuf[idx] = (dataBuf[idx] - 'a' + 'A');
		}
		/*if( dataBuf[idx] >= 'A' && dataBuf[idx] <= 'Z' )
		{
			dataBuf[idx] = ( dataBuf[idx] - 'A' + 'a' );
		}*/
		else
		{
			// dataBuf[idx] = dataBuf[idx];
		}
	}
	dataBuf[idx] = '\0';
	ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] UartDBG:To lower  len(%d) buf(%s)", FmtTimeShow(), dataLen, dataBuf);
#endif
	// Parse AT Command
	if (dataBuf != NULL && dataLen > 0)
	{
		memset(ATCmd, 0, sizeof(ATCmd));
		ATCmdLen = 0;
		for (idx = 0; idx < dataLen && ATCmdLen < TBUFFER_MAX; idx++)
		{
			if (dataBuf[idx] == '=' || dataBuf[idx] == '$' || dataBuf[idx] == ',' || dataBuf[idx] == '\"' || dataBuf[idx] == '\0')
			{
				break;
			}
			else
			{
				if (IS_ALPHABET_NUM_POUND(dataBuf[idx]))
				{
					ATCmd[ATCmdLen++] = dataBuf[idx];
				}
				else
				{
					break;
				}
			}
		}
		ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] AT: Len (%d) AT(%s)", FmtTimeShow(), ATCmdLen, ATCmd);
		if (ATCmdLen < 2)
		{
			ATCmdPrintf(DbgCtl.ATCmdInfoEn, "ERROR");
			ATCmdPrintf(DbgCtl.ATCmdInfoEn, "[%s] ERROR", FmtTimeShow());
			return;
		}
	}
	// Get AT Command Type
	matchCmdType = ATGetCmdType(ATCmd, ATCmdLen, &idxTemp);
	ATParamP = dataBuf + ATCmdLen;
	ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] AT: Type (%d) AT(%s)", FmtTimeShow(), matchCmdType, ATParamP);
	memset((char *)ATCmd, 0, sizeof(ATCmd));
	ATCmdLen = 0;
	ATParamInt = 0;
	// Check Special AT Type
	if (dataBuf[dataLen - 1] == '?')
	{
		isFactoryMode = TRUE;
	}
	else
	{
		// Parse AT Param 1
		if (ATParamP != NULL && dataLen > 0 && matchCmdType != AT_CMD_DEF_NULL)
		{
			isStartToGetParameter = FALSE;
			for (idx = 0; idx < dataLen && ATCmdLen < TBUFFER_MAX; idx++)
			{
				if (ATParamP[idx] == ',')
				{
					isHaveParameter2 = TRUE;
					break;
				}
				else if (ATParamP[idx] == '\"' || ATParamP[idx] == '\0')
				{
					break;
				}
				else if (ATParamP[idx] == '=')
				{
					isStartToGetParameter = TRUE;
				}
				else if (isStartToGetParameter == TRUE)
				{
					if (IS_NUMBER_ONLY(ATParamP[idx]))
					{
						ATCmd[ATCmdLen++] = ATParamP[idx];
					}
					else
					{
						break;
					}
				}
			}

			if (ATCmdLen > 0 && ATCmdLen < 11)
			{
				StringToInt(ATCmd, ATCmdLen, NULL, &ATParamInt);
				ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] AT: Len(%d) Parm1 (%d) Stat(%d)", FmtTimeShow(), ATCmdLen, ATParamInt, isHaveParameter2);
			}
		}
		// Parse AT Param 2
		if (isHaveParameter2 == TRUE)
		{
			ATParamP += (ATCmdLen + 1);
			ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] AT: Check Parm2 AT(%s)", FmtTimeShow(), ATParamP);
			Param2Len = 0;
			memset((char *)ATCmd, 0, sizeof(ATCmd));
			if (ATParamP != NULL && dataLen > 0 && matchCmdType != AT_CMD_DEF_NULL)
			{
				isStartToGetParameter = FALSE;
				for (idx = 0; idx < dataLen && Param2Len < TBUFFER_MAX; idx++)
				{
					if (ATParamP[idx] == '\"' || ATParamP[idx] == '\0')
					{
						break;
					}
					else if (ATParamP[idx] == ',')
					{
						isStartToGetParameter = TRUE;
					}
					else if (isStartToGetParameter == TRUE)
					{
						// if (IS_ALPHABET_OR_NUMBER(ATParamP[idx]))
						{
							ATCmd[Param2Len++] = ATParamP[idx];
						}
						// else
						// {
						// 	break;
						// }
					}
				}
				ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] AT: Len(%d) Parm2(%s)", FmtTimeShow(), Param2Len, ATCmd);
			}
		}
	}
	// AT Process
	if (Param2Len == 0)
		ATCmdProcessing(matchCmdType, isFactoryMode, ATCmdLen, ATParamInt, NULL);
	else
		ATCmdProcessing(matchCmdType, isFactoryMode, ATCmdLen, ATParamInt, ATCmd);
}

/*****************************************************************************
******************************* AT COMMAND START ******************************
*****************************************************************************/
void ATCmdDetection(void)
{
	if (AT_RXCOUNT_VAR & UART_FINISHED_RECV)
	{
		uint8_t DataLen = 0;
		uint8_t Tmp = 0;
		uint8_t UartData[UART_BUF_MAX_LENGTH] = {'\0'};
		// Get Data
		SystemDisableAllInterrupt(); // Disable Interrupt
		DataLen = AT_RXCOUNT_VAR & UART_BUF_MAX_LENGTH;
		// Copy Data
		while ((AT_RXBUFFER_VAR[Tmp] != 'a') && (AT_RXBUFFER_VAR[Tmp] != 'A') && (DataLen > 0))
		{
			DataLen--;
			Tmp++;
		}
		memcpy((char *)UartData, (char *)(AT_RXBUFFER_VAR + Tmp), DataLen);
		UartData[DataLen] = '\0';
		// Clear Data
		memset(AT_RXBUFFER_VAR, 0, sizeof(AT_RXBUFFER_VAR));
		AT_RXCOUNT_VAR = 0;
		SystemEnableAllInterrupt(); // Enable Interrupt

		// Print Out
		ATDbgPrintf(DbgCtl.ATCmdDbgEn, "\r\n[%s] UartDBG: len(%d) buf(%s)",
					FmtTimeShow(), DataLen, UartData);

		if (DataLen >= 4)
		{
			if (UartData[2] == '+' && (UartData[3] == 'x' || UartData[3] == 'X'))
			{
				extern void OemMsgHandle(uint32 MessageId, void *MsgBufferP, uint32 size);
				ATCmdPrintf(DbgCtl.ATCmdInfoEn, "\r\n%s", UartData);
				OemMsgHandle(OEM_AT_CMD_MSG, UartData, DataLen);
				return;
			}
			else if (UartData[2] == '^')
			{
				extern void ATCMDFileTrans(char *pBuf, uint16_t len);
				ATCMDFileTrans((char *)UartData, DataLen);
				return;
			}
			else if ((UartData[2] == 'R' && UartData[3] == 'D' && UartData[4] == 'F' && UartData[5] == '='))
			{
				extern void AtReadFileAndPrintOut(uint8_t * pFileName);
				ATCmdPrintf(DbgCtl.ATCmdInfoEn, "\r\n%s", UartData);
				AtReadFileAndPrintOut(UartData + 6);
				return;
			}
			else if (UartData[2] == '*' && (UartData[3] == 't' || UartData[3] == 'T') && (UartData[4] == 'o' || UartData[4] == 'O'))
			{
				extern uint8_t WIFIPutRawDataInFifo(uint16_t dataLen, uint8_t *data);
				uint16_t len = strlen((char *)&UartData[6]);

				ATCmdPrintf(DbgCtl.ATCmdInfoEn, "\r\nAT*TO:%s", &UartData[6]);
				UartData[6 + len] = '\r';
				UartData[6 + len + 1] = '\n';

				WIFIPutRawDataInFifo(len + 2, UartData + 6);

				return;
			}
		}
		// Parse AT Cmd
		ParseATcmdContect(UartData, DataLen);
	}
}
/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
