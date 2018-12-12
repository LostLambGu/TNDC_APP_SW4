/*******************************************************************************
* File Name          : software_timer.c
* Author             : Yangjie Gu
* Description        : This file provides all the software_timer functions.

* History:
*  10/24/2017 : software_timer V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "software_timer.h"

/* Variables -----------------------------------------------------------------*/
osTimerId AutoReloadTimerId[SOFTWARE_TIMER_NUMBER_MAX] = {0};

/* Function prototypes -------------------------------------------------------*/
void InitTimers(uint16_t timer)
{
    if (timer >= SOFTWARE_TIMER_NUMBER_MAX)
        return;

    AutoReloadTimerId[timer] = NULL;
}

void KillTimers(void)
{
    uint16_t timer = 0;

    for (timer = 0; timer < SOFTWARE_TIMER_NUMBER_MAX; timer++)
    {
        if (AutoReloadTimerId[timer] != NULL)
        {
            osTimerDelete(AutoReloadTimerId[timer]);
            AutoReloadTimerId[timer] = NULL;
        }
    }
}

void StartTimer(uint16_t timer, uint32_t delay, void (*handler)(uint16_t timer))
{
    if (timer >= SOFTWARE_TIMER_NUMBER_MAX)
        return;

    if (AutoReloadTimerId[timer] == NULL)
    {
        taskENTER_CRITICAL();

        osTimerDef(AutoReloadTimer, (void (*)(void const *))handler);
        AutoReloadTimerId[timer] = osTimerCreate(osTimer(AutoReloadTimer), osTimerPeriodic, (void *)timer);

        taskEXIT_CRITICAL();

        if (AutoReloadTimerId[timer] != NULL)
            osTimerStart(AutoReloadTimerId[timer], delay);
    }
    else
    {
        osTimerStart(AutoReloadTimerId[timer], delay);
        return;
    }
}

void StopTimer(uint16_t timer)
{
    if (timer >= SOFTWARE_TIMER_NUMBER_MAX)
        return;

    if (AutoReloadTimerId[timer] != NULL)
    {
        osTimerStop(AutoReloadTimerId[timer]);
    }
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
