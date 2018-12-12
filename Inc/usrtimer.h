/*******************************************************************************
* File Name          : UserTimer.h
* Author             : Taotao Yan
* Description        : This file provides all the UserTimer functions.

* History:
*  1/13/2016 : UserTimer V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _USRTIM_H
#define _USRTIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Defines -------------------------------------------------------------------*/
#define TimeMsec(NumMsec) ((NumMsec)*1000)

#define CHECK_LED_FLASH_STAT_TIMEOUT 200
#if OEM_UART2_TO_MCU_UART1_MAP
#define CHECK_UART1_REC_TIMEOUT (30)
#else
#define CHECK_UART1_REC_TIMEOUT (5 * 1000)
#endif /* OEM_UART2_TO_MCU_UART1_MAP */
#define CHECK_CAN1_REC_TIMEOUT (50)

#define CHECK_UART4_REC_TIMEOUT (5 * 1000)

// define softtimer structure.
typedef struct _TIMER
{
	__IO uint32_t TimeOutVal; //time out value
	uint8_t TimeId;			  // time ID
	uint32_t RecTickVal;	  //softtimer setting value
	uint8_t IsTimeOverflow;   //time out flag
	uint8_t TimerStartCounter;
	void (*Routine)(uint8_t);
} TIMER;

// Variable Declared
// extern TIMER LEDFlashTimer;
extern TIMER CAN1RecTimer;
extern TIMER UARTRecTimer;

#if OEM_UART2_TO_MCU_UART1_MAP
extern TIMER UART1RecTimer;
#endif /* OEM_UART2_TO_MCU_UART1_MAP */

//Function Declare
extern void InitSoftwareTimers(void);
extern void SoftwareTimerCounter(void);
extern void SoftwareTimerCompensation(uint32_t ms);
extern void SoftwareCheckTimerStatus(void);
extern void SoftwareTimerStart(TIMER *timer);
extern void SoftwareTimerStop(TIMER *timer);
extern void SoftwareTimerReset(TIMER *timer, void (*Routine)(uint8_t), uint32_t timeout);
extern void SoftwareTimerCreate(TIMER *timer, uint8_t TimeId, void (*Routine)(uint8_t), uint32_t timeout);
extern uint8_t IsSoftwareTimeOut(TIMER *timer);

#ifdef __cplusplus
}
#endif

#endif

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2014. All rights reserved
                                End Of The File
*******************************************************************************/
