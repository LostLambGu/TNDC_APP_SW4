/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "atcmd.h"
#include "usrtimer.h"
#include "rtcclock.h"
#include "lis2dh_driver.h"
#include "uart_api.h"
#include "oemmsg_apis.h"
#include "oemmsg_queue_process.h"
#include "BLEDriver.h"
#include "BLEProcess.h"
#include "WifiDriver.h"
#include "WifiProcess.h"

extern void mnt_init( void );

/* Private define ------------------------------------------------------------*/
#define FREERTOS_LOG(format, ...) DebugPrintf(DbgCtl.DefaultInfoEn, "\r\n" format, ##__VA_ARGS__)

#define TNDC_TEST_WITH_OEM_HANDLE (1)

#if INCLUDE_uxTaskGetStackHighWaterMark
#define TASK_STACK_WATCH_TASK (0)
#endif

/* Variables -----------------------------------------------------------------*/
char *TNDCFwVerStr = "181211_01";

osThreadId defaultTaskHandle = NULL;
osThreadId OEMMsgTaskHandle = NULL;
osThreadId WifiTaskHandle = NULL;
#if TASK_STACK_WATCH_TASK
osThreadId TaskStackWatchHandle = NULL;
#endif /* TASK_STACK_WATCH_TASK */

#if TNDC_DEBUG_LOG_LOCK_MUTEX
SemaphoreHandle_t xOemDebugPrintMutex = NULL;
static SemaphoreHandle_t xOemDebugPrintMutextmp = NULL;
#elif TNDC_DEBUG_LOG_LOCK_BINARY
SemaphoreHandle_t xOemDebugPrintBinary = NULL;
static SemaphoreHandle_t xOemDebugPrintBinarytmp = NULL;
#endif

__IO uint8_t SuspendAppFlag = FALSE;
__IO uint8_t APPHaltModeIndicateFlag = FALSE;

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartOEMMsgTask(void const *argument);
void StartWifiProcess(void const *argument);

#if TASK_STACK_WATCH_TASK
void StartTaskStackWatch(void const *argument);
#endif /* TASK_STACK_WATCH_TASK */

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void vApplicationMallocFailedHook( void )
{
  FREERTOS_LOG("[%s] ---->>>> Application Malloc Failed!", FmtTimeShow());
}

void PreLoadFileProcessing(void)
{
	taskENTER_CRITICAL();
#if TNDC_DEBUG_LOG_LOCK_MUTEX
	xOemDebugPrintMutextmp = xOemDebugPrintMutex;
	xOemDebugPrintMutex = NULL;
#elif TNDC_DEBUG_LOG_LOCK_BINARY
	xOemDebugPrintBinarytmp = xOemDebugPrintBinary;
	xOemDebugPrintBinary = NULL;
#endif
	taskEXIT_CRITICAL();


  if (OneSenondTimer != NULL)
  {
    APPHaltModeIndicateFlag = TRUE;
    // osTimerStop(OneSenondTimer);
  }

  if (SuspendAppFlag == TRUE)
  {
    if (OEMMsgTaskHandle != NULL)
    {
      osThreadSuspend(OEMMsgTaskHandle);
    }
  }
  else
  {
    if (defaultTaskHandle != NULL)
    {
      osThreadSuspend(defaultTaskHandle);
    }
  }

  if (WifiTaskHandle != NULL)
  {
    osThreadSuspend(WifiTaskHandle);
  }

#if TASK_STACK_WATCH_TASK
  if (TaskStackWatchHandle != NULL)
  {
    osThreadSuspend(TaskStackWatchHandle);
  }
#endif /* TASK_STACK_WATCH_TASK */

#ifdef MODEM_DEEPSLEEP_MODE
  ModemWakeUpTickFwpTimer();
#endif /* MODEM_DEEPSLEEP_MODE */
}

void PostLoadFileProcessing(void)
{
  if (OneSenondTimer != NULL)
  {
    APPHaltModeIndicateFlag = FALSE;
    // osTimerStart(OneSenondTimer, HEARTBEAT_PERIOD);
  }

  if (SuspendAppFlag == TRUE)
  {
    if (OEMMsgTaskHandle != NULL)
    {
      osThreadResume(OEMMsgTaskHandle);
    }
  }
  else
  {
    if (defaultTaskHandle != NULL)
    {
      osThreadResume(defaultTaskHandle);
    }
  }

  if (WifiTaskHandle != NULL)
  {
    osThreadResume(WifiTaskHandle);
  }

#if TASK_STACK_WATCH_TASK
  if (TaskStackWatchHandle != NULL)
  {
    osThreadResume(TaskStackWatchHandle);
  }
#endif /* TASK_STACK_WATCH_TASK */

  taskENTER_CRITICAL();
#if TNDC_DEBUG_LOG_LOCK_MUTEX
  xOemDebugPrintMutex = xOemDebugPrintMutextmp;
#elif TNDC_DEBUG_LOG_LOCK_BINARY
  xOemDebugPrintBinary = xOemDebugPrintBinarytmp;
#endif
  taskEXIT_CRITICAL();

#ifdef MODEM_DEEPSLEEP_MODE
  ModemWakeUpTickFwpTimer();
#endif /* MODEM_DEEPSLEEP_MODE */
}

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}

/* Init FreeRTOS */
void MX_FREERTOS_Init(void) {
  taskENTER_CRITICAL();

#if TNDC_DEBUG_LOG_LOCK_MUTEX
  xOemDebugPrintMutex = xSemaphoreCreateMutex();
#elif TNDC_DEBUG_LOG_LOCK_BINARY
  xOemDebugPrintBinary = xSemaphoreCreateBinary();
  xSemaphoreGive( xOemDebugPrintBinary );
#endif

  xOEMMsgQueueHandle = xQueueCreate(OEMMSG_QUEUE_LENGTH, OEMMSG_QUEUE_SIZE);

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128 * 9);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of defaultTask */
  osThreadDef(OEMMsgTask, StartOEMMsgTask, osPriorityNormal, 0, 128 * 9);
  OEMMsgTaskHandle = osThreadCreate(osThread(OEMMsgTask), NULL);

  osThreadDef(WifiProcessTask, StartWifiProcess, osPriorityNormal, 0, 128 * 9);
  WifiTaskHandle = osThreadCreate(osThread(WifiProcessTask), NULL);

#if TASK_STACK_WATCH_TASK
  osThreadDef(TaskStackWatchTask, StartTaskStackWatch, osPriorityNormal, 0, 128 * 1);
  TaskStackWatchHandle = osThreadCreate(osThread(TaskStackWatchTask), NULL);
#endif /* TASK_STACK_WATCH_TASK */

  taskEXIT_CRITICAL();
}

/* StartDefaultTask function */
void StartDefaultTask(void const *argument)
{
  uint16_t count = 0;
  uint32_t i = 0;
  UNUSED(argument);

  for (;;)
  {
    SoftwareCheckTimerStatus();
    // Gsensor Interupt
    GsensorIntProcess();

    count++;
    if ((count % 500) == 0)
    {
      i++;
      if (i == 3)
      {
        FREERTOS_LOG("--------------------#################################------------------\r\n");
#if TNDC_DEBUG_LOG_LOCK_MUTEX
        FREERTOS_LOG("xOemDebugPrintMutex : 0x%x", xOemDebugPrintMutex);
#elif TNDC_DEBUG_LOG_LOCK_BINARY
        FREERTOS_LOG("xOemDebugPrintBinary : 0x%x", xOemDebugPrintBinary);
#endif
        FREERTOS_LOG("xOEMMsgQueueHandle : 0x%x", xOEMMsgQueueHandle);
        FREERTOS_LOG("defaultTaskHandle : 0x%x", defaultTaskHandle);
        FREERTOS_LOG("OEMMsgTaskHandle : 0x%x", OEMMsgTaskHandle);
        FREERTOS_LOG("OneSenondTimer : 0x%x", OneSenondTimer);
        FREERTOS_LOG("--------------------#################################------------------\r\n");
      }
      FREERTOS_LOG("[%s] Log %s Default Task Running", FmtTimeShow(), TNDCFwVerStr);
      // MCUDeepSleep(10);
    }

    BLETransportTick();
    BLEEvtProcess();

    osDelay(10);
  }
  // osThreadTerminate(NULL);
}

void StartOEMMsgTask(void const *argument)
{
  uint16_t QueueIdRec;

  UNUSED(argument);

  osDelay(2000);

  // Register oem message callback handle
  #if TNDC_TEST_WITH_OEM_HANDLE
  OemRegisterMsgHandleCallBack(OemMsgDataProcess);
  #else
  mnt_init();
  #endif /* TNDC_TEST_WITH_OEM_HANDLE */
  // Create Heart Beat Timer
  taskENTER_CRITICAL();
  osTimerDef(HeartBeatTimer, HeartBeatTimerHandle);
  OneSenondTimer = osTimerCreate(osTimer(HeartBeatTimer), osTimerPeriodic, NULL);
  taskEXIT_CRITICAL();

  if (OneSenondTimer != NULL)
      osTimerStart(OneSenondTimer, HEARTBEAT_PERIOD);

  while (1)
  {
    if (xOEMMsgQueueHandle != NULL)
    {
      if (pdTRUE == xQueueReceive(xOEMMsgQueueHandle, &QueueIdRec, portMAX_DELAY))
      {
        OemMsgProcess(QueueIdRec);
      }
      else
      {
        FREERTOS_LOG("OEMMsgQueue receive fail");
      }
    }
    else
    {
      FREERTOS_LOG("xOEMMsgQueueHandle is NULL");
      osDelay(1000);
    }
  }

  // osThreadTerminate(NULL);
}

void StartWifiProcess(void const *argument)
{
  FREERTOS_LOG("StartWifiProcess start");
  WifiStartUp();
  while (1)
  {
    WifiStateTrans();
    WifiTransportTick();
    WIFIEvtProcess();
    osDelay(1);
  }

  // osThreadTerminate(NULL);
}

#if TASK_STACK_WATCH_TASK
#if HTTP_DOWNLOAD_USE_TASK
extern osThreadId httpRecTaskHandle;
#endif /* HTTP_DOWNLOAD_USE_TASK */

void StartTaskStackWatch(void const *argument)
{
  UBaseType_t uxHighWaterMark1, uxHighWaterMark2;

  while (1)
  {
    if (defaultTaskHandle != NULL)
      uxHighWaterMark1 = uxTaskGetStackHighWaterMark(defaultTaskHandle);

    if (OEMMsgTaskHandle != NULL)
      uxHighWaterMark2 = uxTaskGetStackHighWaterMark(OEMMsgTaskHandle);

    FREERTOS_LOG("---->>>defaultTaskHandle minimum stack left size: %d Bytes", uxHighWaterMark1 * 4);
    FREERTOS_LOG("---->>>OEMMsgTaskHandle minimum stack left size: %d Bytes", uxHighWaterMark2 * 4);

    #if HTTP_DOWNLOAD_USE_TASK
    if (httpRecTaskHandle != NULL)
    {
      FREERTOS_LOG("---->>>httpRecTaskHandle minimum stack left size: %d Bytes", uxTaskGetStackHighWaterMark(httpRecTaskHandle) * 4);
    }
      
    #endif /* HTTP_DOWNLOAD_USE_TASK */

    osDelay(500);
  }
}

#endif /* TASK_STACK_WATCH_TASK */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
