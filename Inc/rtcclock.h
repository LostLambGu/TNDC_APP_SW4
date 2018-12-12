/*******************************************************************************
* File Name          : rtcclock.h
* Author             : Yangjie Gu
* Description        : This file provides all the rtcclock functions.
* History:
*  09/30/2017 : rtcclock V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _RTC_CLOCK_H
#define _RTC_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Defines -------------------------------------------------------------------*/
#define LEAP_YEAR 366
#define NORM_YEAR 365
#define BASE_YEAR 1980
#define LEAP_LOOP (366 + (3 * 365))
#define BASE_OFFSET 432000
#define SECONDS_PER_YEAR (86400 * 365)
#define WAKE_S1_FOR_SECS (1 * 60)
#define WAKE_S2_FOR_SECS (4 * 60)

typedef struct
{
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} TimeTableT;

//Function Declare
extern int32_t datetoweek(int32_t yr, int32_t mn, int32_t day);
extern void DectoBCD(int32_t Dec, uint8_t *Bcd, int32_t length);
extern uint32_t Divide(uint32_t Second, uint32_t value, uint32_t *returnValue);
extern int8_t GetMonthFromDays(uint32_t days, uint32_t year, uint32_t *returnDays);
extern TimeTableT SecondsToTimeTable(uint32_t seconds);
extern uint32_t TimeTableToSeconds(TimeTableT timeTable);
extern char *FmtTimeShow(void);
extern TimeTableT GetRTCDatetime(void);
extern int32_t SetRTCDatetime(TimeTableT *tm);
extern void SetRTCAlarmTime(uint32_t seconds, uint8_t Status);
extern void CompileSetRTCTime(void);

#ifdef __cplusplus
}
#endif

#endif

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
