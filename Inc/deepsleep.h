/*******************************************************************************
* File Name          : deepsleep.h
* Author             : Yangjie Gu
* Description        : This file provides all the deepsleep functions.

* History:
*  07/05/2018 : deepsleep V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEEPSLEEP_H_
#define __DEEPSLEEP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Exported constants --------------------------------------------------------*/
#define ROM_START_ADDRESS 0x8000000

/* Exported functions --------------------------------------------------------*/
extern void MCUDeepSleep(uint32_t seconds);

extern void MCUReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEEPSLEEP_H_ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
