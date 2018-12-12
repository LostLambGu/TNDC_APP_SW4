/*******************************************************************************
* File Name          : oemmsg_process.c
* Author             : Yangjie Gu
* Description        : This file provides all the oemmsg_process functions.

* History:
*  07/05/2018 : oemmsg_process V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "oemmsg_queue_process.h"
#include "oemmsg_apis.h"
#include "rtcclock.h"
#include "usrtimer.h"
#include "prot.h"
#include "io_control.h"
#include "uart_api.h"
#include "iwdg.h"
#include "can.h"

#define QUEUE_PROCESS_LOG(format, ...) DebugPrintf(DbgCtl.ApiDebugInfoEn, "\r\n" format, ##__VA_ARGS__)

/* Variables -----------------------------------------------------------------*/
QueueHandle_t xOEMMsgQueueHandle = NULL;

osTimerId OneSenondTimer;

uint32_t HeartBeatValue = 0;

TimeTableT heartbeattimeTable = {0};

// uint8_t UdpSocketOpenIndicateFlag = 0;
// uint8_t UdpSocketCloseIndicateFlag = 0;
// uint8_t UdpSocketListenIndicateFlag = 0;
// UDPIPSocketTypedef UDPIPSocket[UDPIP_SOCKET_MAX_NUM] = {0};

// UdpSendQueueTypedef UdpSendQueue = {0};

extern void ATCmdDetection(void);

/* Function definition -------------------------------------------------------*/
void SendToOEMMsgQueue(uint16_t sendvalue)
{
    if (xOEMMsgQueueHandle != NULL)
		xQueueSendToBack(xOEMMsgQueueHandle, &sendvalue, OEMMSG_QUEUE_SEND_WAIT_TIME / portTICK_PERIOD_MS);
}

// void OemResetUDPIPSocket(void)
// {
//     memset(&UDPIPSocket, 0, sizeof(UDPIPSocket));
// }

// void OemSysMemInit(void)
// {
//     memset(UDPIPSocket, 0, sizeof(UDPIPSocket));
//     memset(&UdpSendQueue, 0, sizeof(UdpSendQueue));
// }

extern __IO uint8_t APPHaltModeIndicateFlag;
void HeartBeatTimerHandle(void const *argument)
{
    if (APPHaltModeIndicateFlag == FALSE)
    {
        UNUSED(argument);

        SendToOEMMsgQueue(OEMMSG_QUEUE_HEARTBEAT);
    }
    else
    {
        WatchdogTick();
    }
    
}

void SendHeartBeatImmediately(void)
{
    if (xOEMMsgQueueHandle != NULL)
    {
        SendToOEMMsgQueue(OEMMSG_QUEUE_HEARTBEAT);
    }
}

typedef struct
{
	uint8_t	HwdGpIntId;
	uint8_t	GpioValue;
} OemHwdGpIntMsgT;

static void OemMsgQueIOInputStateChangeProcess(uint32_t IONumber)
{
    OemHwdGpIntMsgT OemHwdGpIntMsg = {0};
    DebugPrintf(DbgCtl.DefaultInfoEn, "====>>>>>>IOInputStateChange IONumber(%d)\r\n\r\n", IONumber);

    OemHwdGpIntMsg.HwdGpIntId = IONumber;
    OemHwdGpIntMsg.GpioValue = IOInoutStatRecArray[IONumber].laststate;

    OemMsgHandle(OEM_HWD_INT_MSG, &OemHwdGpIntMsg, sizeof(OemHwdGpIntMsgT));
}

#if OEM_UART2_TO_MCU_UART1_MAP
extern uint16_t LastUart1ParseDataLen;
extern uint8_t Uart1ParseBuffer[UART1_RX_BUFFER_SIZE];
static void OemMsgQueUart2ReceivedDataProcess(void)
{
    OemMsgHandle(OEM_UART_INPUT_DATA_MSG, Uart1ParseBuffer, LastUart1ParseDataLen);
    LastUart1ParseDataLen = 0;
    memset(Uart1ParseBuffer, 0, sizeof(Uart1ParseBuffer));
}
#endif /* OEM_UART2_TO_MCU_UART1_MAP */

static void OemMsgQueCan1ReceivedDataProcess(void)
{
    uint32_t i = 0;
    MonetCANMsgStruct MonetCANMsg = {0};
    MonentCanData CanData = {0};
    CanRxMsgTypeDef *pCanRxMsg = NULL;

    while (CAN1ReceiveCell.input != CAN1ReceiveCell.output)
    {
        pCanRxMsg = &(CAN1ReceiveCell.MsgBuf[CAN1ReceiveCell.output]);

        CanData.StdId = pCanRxMsg->StdId;
        CanData.ExtId = pCanRxMsg->ExtId;
        CanData.IDE = pCanRxMsg->IDE;
        CanData.RTR = pCanRxMsg->RTR;
        CanData.DLC = pCanRxMsg->DLC;

        for (i = 0; i < CanData.DLC; i++)
        {
            CanData.Data[i] = pCanRxMsg->Data[i];
        }

        MonetCANMsg.msgtype = CAN_REVICE_MESSAGE;
        MonetCANMsg.pData = &CanData;
        
        CAN1ReceiveCell.output++;
        CAN1ReceiveCell.output %= CAN1_MAX_RECEIVE_BUF_NUM;

        // DebugPrintf(DbgCtl.DefaultInfoEn, "====>>>>>>Can1ReceivedData DLC(%d) Data0(%d)\r\n\r\n", CanData.DLC, CanData.Data[0]);

        OemMsgHandle(OEM_CAN_MSG, &MonetCANMsg, sizeof(MonetCANMsg));
    }
}

uint8_t OemMsgProcess(uint16_t OemMsgId)
{
    switch (OemMsgId)
    {
    case OEMMSG_QUEUE_HEARTBEAT:
        heartbeattimeTable = GetRTCDatetime();
        HeartBeatValue = TimeTableToSeconds(heartbeattimeTable);
        OemMsgHandle(OEM_HEART_BEAT_MSG, &HeartBeatValue, sizeof(HeartBeatValue));
        break;

    case OEMMSG_QUEUE_MCU_ATCMD:
        // Debug Info
        ATCmdDetection();
        break;

    case OEMMSG_QUEUE_IO_NUMBER_0:
        OemMsgQueIOInputStateChangeProcess(OemMsgId - OEMMSG_QUEUE_IO_NUMBER_0);
        break;

    #if OEM_UART2_TO_MCU_UART1_MAP
    case OEMMSG_QUEUE_MCU_UART2_RECEIVED:
        OemMsgQueUart2ReceivedDataProcess();
        break;
    #endif /* OEM_UART2_TO_MCU_UART1_MAP */

    case OEMMSG_QUEUE_MCU_CAN1_RECEICED:
        OemMsgQueCan1ReceivedDataProcess();
        break;

    default:
        QUEUE_PROCESS_LOG("Error MsgQueu ID %d", OemMsgId);
        break;
    }

    return 0;
}

// void UdpSendUnitIn(UdpSendQueueTypedef *pUdpSendQueue, UdpSendUintTypedef *pUDPIpSendUint)
// {
//     if ((pUdpSendQueue == NULL) || (pUDPIpSendUint == NULL))
//     {
//         QUEUE_PROCESS_LOG("UdpSendUnitIn param err");
//         return;
//     }

//     {
//         taskENTER_CRITICAL();
//         UdpSendUintTypedef *ptemp = &(pUdpSendQueue->UDPIpSendUint[pUdpSendQueue->putindex % UDP_SEND_QUEUE_LENGHT_MAX]);
//         ptemp->datalen = pUDPIpSendUint->datalen;
//         ptemp->socketnum = pUDPIpSendUint->socketnum;
//         ptemp->buf = pUDPIpSendUint->buf;

//         pUdpSendQueue->putindex++;
//         pUdpSendQueue->numinqueue++;
//         if (pUdpSendQueue->numinqueue > UDP_SEND_QUEUE_LENGHT_MAX)
//         {
//             pUdpSendQueue->numinqueue = UDP_SEND_QUEUE_LENGHT_MAX;
//         }
//         taskEXIT_CRITICAL();
//     }
// }

// void UdpSendUintOut(UdpSendQueueTypedef *pUdpSendQueue, UdpSendUintTypedef *pUDPIpSendUint)
// {
//     if ((pUdpSendQueue == NULL) || (pUDPIpSendUint == NULL))
//     {
//         QUEUE_PROCESS_LOG("UdpSendUintOut param err");
//         return;
//     }

//     if (pUdpSendQueue->numinqueue > 0)
//     {
//         taskENTER_CRITICAL();
//         UdpSendUintTypedef *ptemp = &(pUdpSendQueue->UDPIpSendUint[pUdpSendQueue->getindex % UDP_SEND_QUEUE_LENGHT_MAX]);
//         pUDPIpSendUint->datalen = ptemp->datalen;
//         pUDPIpSendUint->socketnum = ptemp->socketnum;
//         pUDPIpSendUint->buf = ptemp->buf;

//         pUdpSendQueue->getindex++;
//         pUdpSendQueue->numinqueue--;
//         if (pUdpSendQueue->numinqueue > UDP_SEND_QUEUE_LENGHT_MAX)
//         {
//             pUdpSendQueue->numinqueue = 0;
//         }
//         taskEXIT_CRITICAL();
//     }

// }

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
