/*******************************************************************************
* File Name          : oemmsg_apis.h
* Author             : Yangjie Gu
* Description        : This file provides all the oemmsg_apis functions.

* History:
*  07/05/2018 : oemmsg_apis V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OEMMSG_APIS_H__
#define __OEMMSG_APIS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Defines -------------------------------------------------------------------*/
#define AT_IMPORT_API

/* Includes ------------------------------------------------------------------*/
#include "prot.h"
#include "api.h"

/* Variables -----------------------------------------------------------------*/
extern void (*pOemMsgHandleCallBack)(uint32 MessageId, void *MsgBufferP, uint32 size);

/* Exported functions --------------------------------------------------------*/
extern void OemRegisterMsgHandleCallBack(void (*pRecvCallbackP)(uint32 MessageId, void *MsgBufferP, uint32 size));
extern void OemClearMsgHandleCallBack(void);
extern void OemMsgHandle(uint32 MessageId, void *MsgBufferP, uint32 size);

extern void OemMsgDataProcess(uint32 MsgId, void *MsgBufferP, uint32 size);

#ifdef __cplusplus
}
#endif

#endif /* __OEMMSG_APIS_H__ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
