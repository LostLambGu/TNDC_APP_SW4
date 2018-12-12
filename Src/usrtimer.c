/*******************************************************************************
* File Name          : UserTimer.c
* Author             : Taotao Yan
* Description        : This file provides all the UserTimer functions.

* History:
*  1/13/2016 : UserTimer V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#include "usrtimer.h"
#include "initialization.h"
#include "uart_api.h"

/*----------------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif
/*----------------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// TIMER LEDFlashTimer;
TIMER CAN1RecTimer;
TIMER UARTRecTimer;

#if OEM_UART2_TO_MCU_UART1_MAP
TIMER UART1RecTimer;
#endif /* OEM_UART2_TO_MCU_UART1_MAP */

void CheckUARTRecTimerCallback(uint8_t Status)
{
}

extern void CheckCAN1RecTimerCallback(uint8_t Status);
extern void SMSEmptyTimerCallback(uint8_t Status);

void InitSoftwareTimers(void)
{
	// CAN1RecTimer
	SoftwareTimerCreate(&CAN1RecTimer, 1, CheckCAN1RecTimerCallback, CHECK_CAN1_REC_TIMEOUT);
	SoftwareTimerStart(&CAN1RecTimer);

	SoftwareTimerCreate(&UARTRecTimer, 1, CheckUARTRecTimerCallback, CHECK_UART4_REC_TIMEOUT);
	// SoftwareTimerStart(&UARTRecTimer);

	#if OEM_UART2_TO_MCU_UART1_MAP
	// UART1RecTimer
	SoftwareTimerCreate(&UART1RecTimer, 1, CheckUART1RecTimerCallback, CHECK_UART1_REC_TIMEOUT);
	SoftwareTimerStart(&UART1RecTimer);
	#endif /* OEM_UART2_TO_MCU_UART1_MAP */
}

void SoftwareTimerCounter(void)
{
	// // LED Flashing
	// if (LEDFlashTimer.TimerStartCounter == TRUE)
	// 	LEDFlashTimer.TimeOutVal++;

	// CAN1RecTimer
	if (CAN1RecTimer.TimerStartCounter == TRUE)
		CAN1RecTimer.TimeOutVal++;

	// UARTRecTimer
	if (UARTRecTimer.TimerStartCounter == TRUE)
		UARTRecTimer.TimeOutVal++;

	#if OEM_UART2_TO_MCU_UART1_MAP
	// UART1RecTimer
	if (UART1RecTimer.TimerStartCounter == TRUE)
		UART1RecTimer.TimeOutVal++;
	#endif /* OEM_UART2_TO_MCU_UART1_MAP */
}

void SoftwareTimerCompensation(uint32_t ms)
{
	if (ms == 0)
	{
		return;
	}

	// // LED Flashing
	// if (LEDFlashTimer.TimerStartCounter == TRUE)
	// 	LEDFlashTimer.TimeOutVal += ms;

	// CAN1RecTimer
	if (CAN1RecTimer.TimerStartCounter == TRUE)
		CAN1RecTimer.TimeOutVal += ms;

	// UARTRecTimer
	if (UARTRecTimer.TimerStartCounter == TRUE)
		UARTRecTimer.TimeOutVal += ms;

	#if OEM_UART2_TO_MCU_UART1_MAP
	// UART1RecTimer
	if (UART1RecTimer.TimerStartCounter == TRUE)
		UART1RecTimer.TimeOutVal += ms;
	#endif /* OEM_UART2_TO_MCU_UART1_MAP */
}

void SoftwareCheckTimerStatus(void)
{
	// // LED Flashing
	// if (IsSoftwareTimeOut(&LEDFlashTimer) == TRUE)
	// {
	// }

	// CAN1RecTimer
	if (IsSoftwareTimeOut(&CAN1RecTimer) == TRUE)
	{
	}

	#if OEM_UART2_TO_MCU_UART1_MAP
	// UARTRecTimer
	if (IsSoftwareTimeOut(&UART1RecTimer) == TRUE)
	{
	}
	#endif /* OEM_UART2_TO_MCU_UART1_MAP */
}

void SoftwareTimerStart(TIMER *timer)
{
	timer->TimerStartCounter = TRUE;
}

void SoftwareTimerStop(TIMER *timer)
{
	timer->TimerStartCounter = FALSE;
}

void SoftwareTimerReset(TIMER *timer, void (*Routine)(uint8_t), uint32_t timeout)
{
	timer->TimerStartCounter = FALSE; //timer stop
	timer->RecTickVal = timeout;	  //softtimer setting value
	timer->TimeOutVal = 0;			  //time out value
	timer->IsTimeOverflow = FALSE;	//time out flag
	timer->Routine = Routine;
}

void SoftwareTimerCreate(TIMER *timer, uint8_t TimeId, void (*Routine)(uint8_t), uint32_t timeout)
{
	timer->TimeId = TimeId;
	timer->RecTickVal = timeout;	  //softtimer setting value
	timer->TimeOutVal = 0;			  //time out value
	timer->IsTimeOverflow = FALSE;	//time out flag
	timer->TimerStartCounter = FALSE; //timer stop
	timer->Routine = Routine;
}

uint8_t IsSoftwareTimeOut(TIMER *timer)
{
	//ET0 = 0;
	if (timer->IsTimeOverflow == FALSE)
	{
		//After gSysTick and timer->TimeOutVal overflow,
		//the software timer function can still work well
		//the next statement is equivalent to:
		//(gSysTick - timer->TimeOutVal) < 0x80000000
		if (timer->TimeOutVal > timer->RecTickVal)
		{
			timer->TimerStartCounter = FALSE;
			timer->IsTimeOverflow = TRUE;
			timer->Routine(timer->TimeId);
		}
	}
	//ET0 = 1;

	return timer->IsTimeOverflow;
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
