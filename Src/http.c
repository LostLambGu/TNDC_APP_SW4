/*******************************************************************************
* File Name          : http.c
* Author             : Yangjie Gu
* Description        : This file provides all the http functions.

* History:
*  10/12/2018 : http V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "fatfs.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "http.h"
#include "rtcclock.h"
#include "uart_api.h"

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define HTTP_PRINT(format, ...) DebugPrintf(DbgCtl.HttpInfoEn, "\r\n" format, ##__VA_ARGS__)

#define HTTP_ACTION_NONE (0)
#define HTTP_ACTION_POST (1)
#define HTTP_ACTION_GET (2)

#define HTTP_DEFAULT_TIMEOUT (2000)
#define HTTP_WAIT_QUERY_RSP_TIMEOUT (50 * 1000)
#define HTTP_WAIT_QUERY_RSP_DELAY (5)
#define HTTP_DATA_WAIT_TIMEOUT (1000 * 60)
#define HTTP_DATA_WAIT_ACK_DELAY (1)
#define HTTP_DATA_WAIT_ACK_TIMEOUT (1000 * 60)

#define HTTP_WAIT_RSP_OK (0)

uint8_t httpActionRec = 0;
uint16_t httpRspRec = 0xffff;
uint16_t httpAckRec = 0xffff;
HttpCfgTypeDef HttpCfg = 
{
	{0},
	{0},
    {0},
    {0},
    {0},
    0,
	0xff,
	HTTP_STATE_HALT,
	0,
	0,
	0,
    0,
	FALSE,
	0,
    0,
    0,
    0
};

osThreadId httpTaskHandle = NULL;

extern uint8_t WifiQueryHttpState(char *ip, char *path, uint16_t port, char *fileName, uint32_t fileSize, uint8_t mode);

void HttpProcessStateChange(HttpStateTypeDef next, uint32_t timeout, uint32_t delay);
uint8_t HttpCheckTimeout(void);
void HttpQueryState(void);

void HttpProcess(void)
{
    while (1)
    {
        if (HttpCfg.terminateFlag == TRUE)
        {
            HTTP_PRINT("[%s] HttpProcess Terminated state(%d)", FmtTimeShow(), HttpCfg.state);
            HttpProcessStateChange(HTTP_STATE_WAIT_HALT, HTTP_DEFAULT_TIMEOUT, 0);
        }

        switch (HttpCfg.state)
        {
        case HTTP_STATE_CFG:
            HTTP_PRINT("[%s] HttpProcess HTTP_STATE_CFG", FmtTimeShow());
            HttpProcessStateChange(HTTP_STATE_QUERY, HTTP_DEFAULT_TIMEOUT, 0);
            break;

        case HTTP_STATE_QUERY:
            HTTP_PRINT("[%s] HttpProcess HTTP_STATE_QUERY", FmtTimeShow());
            HttpQueryState();
            HttpProcessStateChange(HTTP_STATE_WAIT_QUERY_RSP, HTTP_WAIT_QUERY_RSP_TIMEOUT, HTTP_WAIT_QUERY_RSP_DELAY);
            break;

        case HTTP_STATE_WAIT_QUERY_RSP:
            if (httpRspRec == 0xffff)
            {
                // Wait until timeout.
            }
            else if (httpRspRec == HTTP_WAIT_RSP_OK)
            {
                if (HttpCfg.mode == HTTP_ACTION_POST)
                {
                    HttpProcessStateChange(HTTP_STATE_SEND, HTTP_DATA_WAIT_TIMEOUT, 0);
                }
                else if (HttpCfg.mode == HTTP_ACTION_GET)
                {
                    HttpProcessStateChange(HTTP_STATE_REC, HTTP_DATA_WAIT_TIMEOUT, 0);
                }
                else
                {
                    HTTP_PRINT("[%s] HttpProcess WAIT_QUERY_RSP mode(%d) err", FmtTimeShow(), HttpCfg.mode);
                    HttpProcessStateChange(HTTP_STATE_WAIT_HALT, HTTP_DEFAULT_TIMEOUT, 0);
                }
                httpRspRec = 0xffff;
            }
            else
            {
                HTTP_PRINT("[%s] HttpProcess WAIT_QUERY_RSP rsp(%d) err", FmtTimeShow(), httpRspRec);
                HttpProcessStateChange(HTTP_STATE_WAIT_HALT, HTTP_DEFAULT_TIMEOUT, 0);
                httpRspRec = 0xffff;
            }
            break;

        case HTTP_STATE_REC:
            HTTP_PRINT("[%s] HttpProcess HTTP_STATE_REC", FmtTimeShow());
            break;

        case HTTP_STATE_SEND:
            HTTP_PRINT("[%s] HttpProcess HTTP_STATE_SEND", FmtTimeShow());
            HttpProcessStateChange(HTTP_STATE_WAIT_ACK, HTTP_DATA_WAIT_ACK_TIMEOUT, HTTP_DATA_WAIT_ACK_DELAY);
            break;

        case HTTP_STATE_WAIT_ACK:
            HTTP_PRINT("[%s] HttpProcess HTTP_STATE_WAIT_ACK", FmtTimeShow());
            if (httpAckRec == 0xffff)
            {
                // wait until timeout
            }
            else if (httpAckRec == 1)
            {
                HTTP_PRINT("[%s] HttpProcess ack received", FmtTimeShow());
                HttpProcessStateChange(HTTP_STATE_SEND, HTTP_DEFAULT_TIMEOUT, 0);
                httpAckRec = 0xffff;
            }
            else
            {
                if (httpAckRec == 2) // finish
                {
                    HTTP_PRINT("[%s] HttpProcess post ack finish", FmtTimeShow());
                }
                else if (httpAckRec == 3) // error
                {
                    HTTP_PRINT("[%s] HttpProcess post ack fail", FmtTimeShow());
                }
                else
                {
                    HTTP_PRINT("[%s] HttpProcess post ack err", FmtTimeShow());
                }
                HttpProcessStateChange(HTTP_STATE_WAIT_HALT, HTTP_DEFAULT_TIMEOUT, 0);
                httpAckRec = 0xffff;
            }
            break;

        case HTTP_STATE_WAIT_HALT:
            HTTP_PRINT("[%s] HttpProcess HTTP_STATE_WAIT_HALT", FmtTimeShow());
            memset(&HttpCfg, 0, sizeof(HttpCfg));
            HttpProcessStateChange(HTTP_STATE_HALT, HTTP_DEFAULT_TIMEOUT, 0);
            break;

        case HTTP_STATE_HALT:
            if (httpActionRec != HTTP_ACTION_NONE)
            {
                HttpProcessStateChange(HTTP_STATE_CFG, HTTP_DEFAULT_TIMEOUT, 0);
            }
            HttpCfg.mode = httpActionRec;
            httpActionRec = HTTP_ACTION_NONE;
            break;

        default:
            HTTP_PRINT("[%s] Http Process State Err", FmtTimeShow());
            break;
        }

        if (HttpCfg.state == HTTP_STATE_HALT)
        {
            HTTP_PRINT("[%s] Http Process task end", FmtTimeShow());
            break;
        }

        if (HttpCfg.waitMs)
        {
            osDelay(HttpCfg.waitMs);
        }

        if (HttpCheckTimeout())
        {
            HTTP_PRINT("[%s] Http Process timeout state(%d)", FmtTimeShow(), HttpCfg.state);
            HttpProcessStateChange(HTTP_STATE_WAIT_HALT, HTTP_DEFAULT_TIMEOUT, 0);
        }
        
    }
}

void HttpProcessStateChange(HttpStateTypeDef next, uint32_t timeout, uint32_t delay)
{
    HttpCfg.state = next;
    HttpCfg.recTimeOut = timeout;
    HttpCfg.recSysTick = HAL_GetTick();
    HttpCfg.waitMs = delay;
}

uint8_t HttpCheckTimeout(void)
{
    if ((HAL_GetTick() - HttpCfg.recSysTick) > HttpCfg.recTimeOut)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void HttpQueryState(void)
{
    if (WifiQueryHttpState(HttpCfg.remote_ip, HttpCfg.path, HttpCfg.server_port, HttpCfg.fileName, HttpCfg.dataTotalLen, HttpCfg.mode))
    {
        HTTP_PRINT("[%s] HttpQueryState err", FmtTimeShow());
    }
}

void StarthttpTask(void const *argument)
{
    HTTP_PRINT("\r\n[%s] Http TASK START", FmtTimeShow());
	HttpProcess();
	HTTP_PRINT("\r\n[%s] Http TASK END", FmtTimeShow());
	httpTaskHandle = NULL;
	osThreadTerminate(NULL);
}

void createHttpTask(void)
{
	taskENTER_CRITICAL();
	osThreadDef(httpTask, StarthttpTask, osPriorityNormal, 0, 128 * 5);
  	httpTaskHandle = osThreadCreate(osThread(httpTask), NULL);
	taskEXIT_CRITICAL();
}

void HttpInitForPostTest(void)
{
    httpActionRec = HTTP_ACTION_POST;

    memset(HttpCfg.remote_ip, 0, sizeof(HttpCfg.remote_ip));
    memset(HttpCfg.path, 0, sizeof(HttpCfg.path));
    memset(HttpCfg.fileName, 0, sizeof(HttpCfg.fileName));

    memcpy(HttpCfg.remote_ip, "180.166.175.194", strlen("180.166.175.194"));
    memcpy(HttpCfg.path, "/post/", strlen("/post/"));
    HttpCfg.server_port = 8080;
    memcpy(HttpCfg.fileName, "test.txt", strlen("test.txt"));
    HttpCfg.dataTotalLen = 128;

    createHttpTask();
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/