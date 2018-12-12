/*******************************************************************************
* File Name          : initialization.c
* Author             : Yangjie Gu
* Description        : This file provides all the initialization functions.

* History:
*  10/18/2017 : initialization V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#include "cmsis_os.h"

#include "uart_api.h"
#include "initialization.h"
#include "lis2dh_driver.h"
#include "WifiDriver.h"
#include "rtcclock.h"
#include "usrtimer.h"
#include "version.h"
#include "oemmsg_queue_process.h"
#include "sha204_physical.h"
#include "BLEDriver.h"
#include "BLEProcess.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*comment to eliminate errors*/
/* comment atel_ related lines */
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define InitPrintf(format, ...) DebugPrintf(DbgCtl.DefaultInfoEn, "\r\n" format, ##__VA_ARGS__)
/*----------------------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
extern uint8_t Lis2dhMemsChipID;

/* Public functions ----------------------------------------------------------*/
void SystemInitialization(void)
{
	// Init LED

	// Show Version
	ShowSoftVersion();

	//Set RTC time using compile time
	CompileSetRTCTime();

	// Timer Init
	InitSoftwareTimers();

	// Delay
	HAL_Delay(5);

	Lis2dhMemsChipID = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_WHO_AM_I);
	if (Lis2dhMemsChipID == WHOAMI_LIS2DH_ACC) //LIS2DH_MEMS_I2C_ADDRESS
	{
		PrintfBeforeRTOS("\r\nGsensor Start!\r\n");
		// Gsensor settings
		GsensorStart();
		
		// LIS2DH_SetMode(LIS2DH_POWER_DOWN);
	}

	sha204p_init();

	BLEDriverInit();
	BLERecordReset();

	// OemSysMemInit();
}

#if OEM_UART2_TO_MCU_UART1_MAP
volatile uint16_t LastUart1DataLen = 0;
uint16_t LastUart1ParseDataLen = 0;
void CheckUART1RecTimerCallback(uint8_t Status)
{
	volatile uint16_t size = Uart1RxCount;

	if (size == 0)
	{
		// Reset Timer
		SoftwareTimerReset(&UART1RecTimer, CheckUART1RecTimerCallback, CHECK_UART1_REC_TIMEOUT);
		SoftwareTimerStart(&UART1RecTimer);
		return;
	}
	else
	{
		if (LastUart1DataLen != size)
		{
			LastUart1DataLen = size;
			// Reset Timer
			SoftwareTimerReset(&UART1RecTimer, CheckUART1RecTimerCallback, CHECK_UART1_REC_TIMEOUT);
			SoftwareTimerStart(&UART1RecTimer);
			return;
		}
	}

	LastUart1ParseDataLen = LastUart1DataLen;
	memcpy((char *)Uart1ParseBuffer, (char *)Uart1RxBuffer, Uart1RxCount);

	memset((void *)&Uart1RxBuffer, 0, sizeof(Uart1RxBuffer));
	Uart1RxCount = 0;
	LastUart1DataLen = 0;

	SendToOEMMsgQueue(OEMMSG_QUEUE_MCU_UART2_RECEIVED);

	// Reset Timer
	SoftwareTimerReset(&UART1RecTimer, CheckUART1RecTimerCallback, CHECK_UART1_REC_TIMEOUT);
	SoftwareTimerStart(&UART1RecTimer);
}

void UART1_RxCpltCallback(uint8_t Data)
{
	if (Uart1RxCount < UART1_RX_BUFFER_SIZE)
	{
		Uart1RxBuffer[Uart1RxCount] = Data;
		Uart1RxCount++;
	}
	else
	{
		Uart1RxCount = 0;
		Uart1RxBuffer[0] = Data;
		Uart1RxCount++;
	}
}
#endif /* OEM_UART2_TO_MCU_UART1_MAP */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
