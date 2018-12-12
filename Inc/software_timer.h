/*******************************************************************************
* File Name          : software_timer.h
* Author             : Yangjie Gu
* Description        : This file provides all the software_timer functions.

* History:
*  10/24/2017 : software_timer V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SOFTWARE_TIMER_H_
#define __SOFTWARE_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#define SOFTWARE_TIMER_NUMBER_MAX (5)

/* Variables -----------------------------------------------------------------*/
extern osTimerId AutoReloadTimerId[SOFTWARE_TIMER_NUMBER_MAX];

/* Exported functions --------------------------------------------------------*/
extern void InitTimers(uint16_t timer);

extern void KillTimers(void);

extern void StartTimer(uint16_t timer, uint32_t delay, void (*handler)(uint16_t timer));

extern void StopTimer(uint16_t timer);

#ifdef __cplusplus
}
#endif

#endif /* __SOFTWARE_TIMER_H_ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
