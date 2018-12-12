/*******************************************************************************
* File Name          : version.c
* Author             : Yangjie Gu
* Description        : This file provides all the version functions.

* History:
*  70/05/2018 : version V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "version.h"

#include "uart_api.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAIN_VERSION_NUM 1
#define SUB_VERSION_NUM 0

#define VersionPrintf DebugPrintf

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint8_t CompanyInformation[] = "\r\nCopyright (c) 2003-2018 ATEL Corp.";
const uint8_t SoftwareInformation[] = "\r\nVERSION: TNDC";

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
void ShowSoftVersion(void)
{
	VersionPrintf(DbgCtl.VersionDebugInfoEn, "\r\n %s%sV%d.%d(%s %s)\r\n",
				  CompanyInformation,
				  SoftwareInformation,
				  MAIN_VERSION_NUM,
				  SUB_VERSION_NUM,
				  __DATE__,
				  __TIME__);
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
