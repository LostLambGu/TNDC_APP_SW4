/*******************************************************************************
* File Name          : atcmd_filetrans.h
* Author             : Yangjie Gu
* Description        : This file provides all the atcmd_filetrans functions.

* History:
*  02/27/2018 : atcmd_filetrans V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ATCMD_FILETRANS_H__
#define ATCMD_FILETRANS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "fatfs.h"

typedef enum
{
    ATCMD_FILETRANS_LIST,
    ATCMD_FILETRANS_DELETE,

    ATCMD_FILETRANS_CMD_MAX
} ATCMDFileTransTypeDef;

typedef enum
{
    ATCMD_FILETRANS_OPERATE_DEFAULT,
    ATCMD_FILETRANS_OPERATE_QUERY,
    ATCMD_FILETRANS_OPERATE_EXECUTE,

    ATCMD_FILETRANS_OPERATE_MAX
} ATCMDFileTransOperateTypeDef;

typedef struct
{
    char *str;
} ATCMDFileTransTableTypeDef;

typedef void (*ATCMDFileTransProcFuncTypeDef)(uint16_t operateType, char *pBuf, uint16_t len);

extern void ATCMDFileTrans(char *pBuf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* ATCMD_FILETRANS_H__ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
