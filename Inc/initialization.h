/*******************************************************************************
* File Name          : initialization.h
* Author             : Yangjie Gu
* Description        : This file provides all the initialization functions.

* History:
*  10/18/2017 : initialization V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _INITIALIZATION_SOFTWARE_H
#define _INITIALIZATION_SOFTWARE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "usrtimer.h"

/* Defines -------------------------------------------------------------------*/
#define min(X, Y) ((X) < (Y) ? (X) : (Y))
#define max(X, Y) ((X) > (Y) ? (X) : (Y))

#define FunStates FunctionalState

//Function Declare
extern void SystemInitialization(void);

#if OEM_UART2_TO_MCU_UART1_MAP
extern void CheckUART1RecTimerCallback(uint8_t Status);
extern void UART1_RxCpltCallback(uint8_t Data);
#endif /* OEM_UART2_TO_MCU_UART1_MAP */

#ifdef __cplusplus
}
#endif

#endif /* _INITIALIZATION_SOFTWARE_H */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
