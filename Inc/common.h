/*******************************************************************************
* File Name          : common.h
* Author             : Yangjie Gu
* Description        : This file provides all the common functions.

* History:
*  08/07/2018 : common V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMON_FUNCTIONS_
#define __COMMON_FUNCTIONS_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private define ------------------------------------------------------------*/
/* Public defines ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/

extern void DelayMsTime(uint16_t delay_time);
extern void StringToUper(char* s);
extern void SystemDisableAllInterrupt(void);
extern void SystemEnableAllInterrupt(void);
extern void GetExclusiveLock(volatile unsigned int * Lock);
extern void FreeExclusiveLock(volatile unsigned int * Lock);

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_FUNCTIONS_ */

