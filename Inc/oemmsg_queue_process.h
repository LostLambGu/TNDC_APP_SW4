/*******************************************************************************
* File Name          : oemmsg_process.h
* Author             : Yangjie Gu
* Description        : This file provides all the oemmsg_process functions.

* History:
*  10/28/2017 : oemmsg_process V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OEMMSG_QUEUE_PROCESS_H__
#define __OEMMSG_QUEUE_PROCESS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
	
#include "cmsis_os.h"
	
#include "uart_api.h"

/* Defines ------------------------------------------------------------------*/

#define OEMMSG_QUEUE_LENGTH (128)
#define OEMMSG_QUEUE_SIZE (2)

#define HEARTBEAT_PERIOD (1000) // ms

#define OEMMSG_QUEUE_SEND_WAIT_TIME (50)  // ms

typedef enum
{
    OEMMSG_QUEUE_HEARTBEAT = 0,
    OEMMSG_QUEUE_MCU_ATCMD,

    OEMMSG_QUEUE_IO_NUMBER_0,

    #if OEM_UART2_TO_MCU_UART1_MAP
    OEMMSG_QUEUE_MCU_UART2_RECEIVED,
    #endif /* OEM_UART2_TO_MCU_UART1_MAP */

    OEMMSG_QUEUE_MCU_CAN1_RECEICED,

    OEMMSG_QUEUE_LAST,
    OEMMSG_QUEUE_NULL = 0xffff
} OEMMsgQueueIdDef;

extern QueueHandle_t xOEMMsgQueueHandle;

extern uint32_t HeartBeatValue;

extern osTimerId OneSenondTimer;

extern void SendToOEMMsgQueue(uint16_t sendvalue);

extern void OemSysMemInit(void);

extern void HeartBeatTimerHandle(void const * argument);

extern void SendHeartBeatImmediately(void);

extern uint8_t OemMsgProcess(uint16_t OemMsgId);

#define UDPIP_SOCKET_MIN_NUM (1)
#define UDPIP_SOCKET_MAX_NUM (5)
#define UDP_SEND_QUEUE_LENGHT_MAX (8)
#define SMS_SEND_QUEUE_LENGHT_MAX (8)

// typedef enum
// {
//     SOCKETCLOSE = 0,
//     SOCKETOPEN = 1,
//     SOCKETOPEN_REPORTONCE = 2,
// } SocketOperationTypedef;

// typedef enum
// {
//     SOCKET_CLOSE = 0,
//     SOCKET_ACTIVE, // Socket with an active data transfer connection
// } SocketStatusTypedef;

// typedef struct
// {
//   uint8_t operation;
//   uint8_t status;
//   uint16_t LocalPort;
//   uint16_t PortNum;
//   char DestAddrP[MAX_IP_ADDR_LEN];
//   uint32_t AddrNum;
// } UDPIPSocketTypedef;

// extern uint8_t UdpSocketOpenIndicateFlag;
// extern uint8_t UdpSocketCloseIndicateFlag;
// extern uint8_t UdpSocketListenIndicateFlag;
// extern UDPIPSocketTypedef UDPIPSocket[UDPIP_SOCKET_MAX_NUM];

// typedef struct
// {
//     uint16_t datalen;
//     uint8_t socketnum;
//     char *buf;
// } UdpSendUintTypedef;

// typedef struct
// {
//     uint8_t putindex;
//     uint8_t getindex;
//     uint8_t numinqueue;
//     UdpSendUintTypedef UDPIpSendUint[UDP_SEND_QUEUE_LENGHT_MAX];
// } UdpSendQueueTypedef;

// extern UdpSendQueueTypedef UdpSendQueue;

// extern void UdpSendUnitIn(UdpSendQueueTypedef *pUdpSendQueue, UdpSendUintTypedef *pUDPIpSendUint);

// extern void UdpSendUintOut(UdpSendQueueTypedef *pUdpSendQueue, UdpSendUintTypedef *pUDPIpSendUint);

#ifdef __cplusplus
}
#endif

#endif /* __OEMMSG_QUEUE_PROCESS_H__ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
