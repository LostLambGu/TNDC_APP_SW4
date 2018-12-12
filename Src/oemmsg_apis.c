/*******************************************************************************
* File Name          : oemmsg_apis.c
* Author             : Yangjie Gu
* Description        : This file provides all the oemmsg_apis functions.

* History:
*  10/28/2017 : oemmsg_apis V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

#include "can.h"
#include "usart.h"
#include "rtcclock.h"

#include "oemmsg_apis.h"
#include "oemmsg_queue_process.h"
#include "prot.h"

/* Private define ------------------------------------------------------------*/
#define OEM_MAX_AT_CMD_LEN (128)
#define OEM_MAX_AT_RSP_LEN (128)

#define MAX_OEM_MSG_SIZE (128)
#define IS_HEX_ONLY(x) ((x >= '0' && x <= '9') || (x >= 'a' && x <= 'f') || (x >= 'A' && x <= 'F'))
#define IS_NUMBER_ONLY(x) (x >= '0' && x <= '9')
#define IS_UPPER_ONLY(x) (x >= 'A' && x <= 'F')
#define IS_LOWER_ONLY(x) (x >= 'a' && x <= 'f')

/* Types ---------------------------------------------------------------------*/
typedef struct
{
    uint16 position;
    uint16 length;
    char character[OEM_MAX_AT_CMD_LEN];
} OemAtcmdLine;

typedef struct
{
    char *Name;
    uint8 (*CmdProcessFunc)(OemAtcmdLine *cmd_line);
} OemAtCmdParseInfo;

/* Variables -----------------------------------------------------------------*/
void (*pOemMsgHandleCallBack)(uint32 MessageId, void *MsgBufferP, uint32 size);

extern void UARTPrintMassData(uint8_t *string, uint16_t slen);

/* Function definition -------------------------------------------------------*/
void OemRegisterMsgHandleCallBack(void (*pRecvCallbackP)(uint32 MessageId, void *MsgBufferP, uint32 size))
{
    pOemMsgHandleCallBack = pRecvCallbackP;
}

void OemClearMsgHandleCallBack(void)
{
    pOemMsgHandleCallBack = NULL;
}

// static uint8 chartouint8(char c)
// {
//     if (IS_NUMBER_ONLY(c))
//     {
//         return (c - '0');
//     }
//     else if (IS_LOWER_ONLY(c))
//     {
//         return (c - 'a' + 10);
//     }
//     else
//     {
//         return (c - 'A' + 10);
//     }
// }

static uint8 ToUper(uint8 ch)
{
    if (ch >= 'a' && ch <= 'z')
        return (uint8)(ch + ('A' - 'a'));
    return ch;
}

static uint8 stricmp(const char *Str1, const char *Str2)
{
    uint16 nLen1, nLen2;
    uint16 i = 0;
    if (!Str1 || !Str2)
        return 1;
    nLen1 = strlen((char *)Str1);
    nLen2 = strlen((char *)Str2);

    if (nLen1 > nLen2)
        return 1;
    else if (nLen1 < nLen2)
        return 1;
    while (i < nLen1)
    {
        if (ToUper((uint8)Str1[i]) != ToUper((uint8)Str2[i]))
            return 1;
        i++;
    }
    return 0;
}

void OemMsgHandle(uint32 MessageId, void *MsgBufferP, uint32 size)
{
    if (pOemMsgHandleCallBack != NULL)
        (*pOemMsgHandleCallBack)(MessageId, MsgBufferP, size);
}

uint8 OemATLED(OemAtcmdLine *cmd_line)
{
    at_OutputString("\r\n----->>>>> OemATCommandHdlr OemATLED\r\n");
    return 1;
}

static bool OemATCommandHdlr(char *CmdData)
{
    char buffer[OEM_MAX_AT_CMD_LEN + 1];
    char *cmd_name, *cmdString;
    uint16 length;
    uint8 index = 0;
    OemAtcmdLine command_line;
    uint16 i;
    OemAtCmdParseInfo OemAtCmdTable[] =
        {
            {"AT+XLED", OemATLED},
        };
    uint32 OemAtCmdNum = sizeof(OemAtCmdTable) / sizeof(OemAtCmdParseInfo);
    at_OutputString("\r\nReceived AT command: ");
    if (NULL == CmdData)
    {
        return FALSE;
    }
    if (NULL == CmdData || 0 == strlen(CmdData))
    {
        at_OutputString("<empty>");
    }
    else
    {
        at_OutputString(CmdData);
    }
    at_OutputString("\r\n");

    cmd_name = buffer;

    length = strlen(CmdData);
    length = length > OEM_MAX_AT_CMD_LEN ? OEM_MAX_AT_CMD_LEN : length;
    while ((CmdData[index] != '=') &&                          //might be TEST command or EXE command
           (CmdData[index] != '?') &&                          // might be READ command
           (CmdData[index] < 0x30 || CmdData[index] > 0x39) && //PA_Execute
           (CmdData[index] != 13) &&                           //carriage return
           index < length)
    {
        cmd_name[index] = CmdData[index];
        index++;
    }
    cmd_name[index] = '\0';

    memset(command_line.character, 0x00, sizeof(char) * OEM_MAX_AT_CMD_LEN);
    strncpy(command_line.character, CmdData, OEM_MAX_AT_CMD_LEN);
    command_line.length = strlen(command_line.character);
    command_line.position = index;
    for (i = 0; i < OemAtCmdNum; i++)
    {
        cmdString = OemAtCmdTable[i].Name;
        if (stricmp(cmd_name, cmdString) == 0)
        {
            OemAtCmdTable[i].CmdProcessFunc(&command_line);
            return TRUE;
        }
    }
    // OemResultProcessResultCode(&command_line, FALSE);
    return FALSE;
}

static void OmeHeartBeatHandler(void *MsgBufferP, uint32 size)
{
}

void OemMsgDataProcess(uint32 MsgId, void *MsgBufferP, uint32 size)
{
    // at_printfDebug("\r\n-->>OemMsgDataProcess::MsgId = %d\r\n", MsgId);

    if(MsgBufferP == NULL)
         return;

    switch (MsgId)
    {
    case OEM_AT_CMD_MSG:
        OemATCommandHdlr((char *)MsgBufferP);
        break;

    case OEM_HWD_INT_MSG:
    {
        typedef struct
        {
            uint8_t HwdGpIntId;
            uint8_t GpioValue;
        } OemHwdGpIntMsgT;
        OemHwdGpIntMsgT *pGpIntMsg = (OemHwdGpIntMsgT *)MsgBufferP;

        at_printfDebug("\r\n[%s]-->>OEM_HWD_INT_MSG : MsgId(%d) size(%d)\r\n", FmtTimeShow(), MsgId, size);
        at_printfDebug("\r\n[%s]-->>OEM_HWD_INT_MSG : HwdGpIntId(%d) GpioValue(%d)\r\n", FmtTimeShow(), pGpIntMsg->HwdGpIntId, pGpIntMsg->GpioValue);
    }
    break;

    case OEM_CAN_MSG:
    {
        MonetCANMsgStruct *pCanRec = (MonetCANMsgStruct *)MsgBufferP;
        at_printfDebug("\r\n[%s]-->>OEM_CAN_MSG : MsgId(%d) size(%d) ", FmtTimeShow(), MsgId, size);
        DebugLog("MonetCANMsg.msgtype(%d)", pCanRec->msgtype);
        DebugLog("CanData.StdId(0x%x)", pCanRec->pData->StdId);
        DebugLog("CanData.ExtId(0x%x)", pCanRec->pData->ExtId);
        DebugLog("CanData.IDE(%d)", pCanRec->pData->IDE);
        DebugLog("CanData.RTR(%d)", pCanRec->pData->RTR);
        DebugLog("CanData.DLC(%d)", pCanRec->pData->DLC);
        // pCanRec->pData->Data[0] += '0';
        // pCanRec->pData->Data[1] += '0';
        HAL_UART_Transmit(&huart4, (uint8_t *)pCanRec->pData->Data, pCanRec->pData->DLC, 50);
    }
    break;

    case OEM_UART_INPUT_DATA_MSG:
    DebugLog("MCU UART1 Receiced Data(Len %d) :", size);
    HAL_UART_Transmit(&huart4, (uint8_t *)MsgBufferP, size, 50);
    at_OutputStringUart2("\r\nEcho:", 7);
    at_OutputStringUart2(MsgBufferP, size);
    at_OutputStringUart2("\r\n", 2);
    break;

    case OEM_HEART_BEAT_MSG:
    OmeHeartBeatHandler(MsgBufferP, size);
    break;
    default:
        break;
    }
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
