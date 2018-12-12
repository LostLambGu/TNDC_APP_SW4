/*******************************************************************************
* File Name          : http.h
* Author             : Yangjie Gu
* Description        : This file provides all the http functions.

* History:
*  10/12/2018 : http V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HTTP_SERVICE_H
#define _HTTP_SERVICE_H
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HTTP_CFG_LOCAL_MAX_LEN (64)
#define HTTP_CFG_REMOTE_MXA_LEN (128)
#define HTTP_CFG_PATH_MXA_LEN (64)
#define HTTP_CFG_IP_MXA_LEN (64)
#define HTTP_CFG_FILENAME_MXA_LEN (64)
#define HTTP_SEND_STEP_SIZE (128 * 1)
#define HTTP_SEND_BUF_SIZE (DEFAULT_HTTP_SEND_STEP_SIZE + 64)

typedef enum
{
	HTTP_STATE_HALT = 0,
	HTTP_STATE_CFG,
	HTTP_STATE_QUERY,
	HTTP_STATE_WAIT_QUERY_RSP,
	HTTP_STATE_REC,
	HTTP_STATE_SEND,
    HTTP_STATE_WAIT_ACK,
	HTTP_STATE_WAIT_HALT,

	HTTP_STATE_MAX
} HttpStateTypeDef;

typedef struct
{
	char local[HTTP_CFG_LOCAL_MAX_LEN];
	char remote[HTTP_CFG_REMOTE_MXA_LEN];
    char path[HTTP_CFG_PATH_MXA_LEN];
	char remote_ip[HTTP_CFG_IP_MXA_LEN];
	char fileName[HTTP_CFG_FILENAME_MXA_LEN];
	uint16_t server_port;
	uint8_t mode;
	HttpStateTypeDef state;
	uint16_t status_code;
	uint32_t dataTotalLen;
	uint32_t currentDataLen;
    uint32_t dataLenOffset;
	uint8_t terminateFlag;
	uint32_t recTimeOut;
	uint32_t recSysTick;
    uint32_t waitMs;
    uint16_t err_code;
} HttpCfgTypeDef;

extern HttpCfgTypeDef HttpCfg;

#ifdef __cplusplus
}
#endif

#endif /* _HTTP_SERVICE_H */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/