/*******************************************************************************
* File Name          : uart_api.h
* Author             : Yangjie Gu
* Description        : This file provides all the uart_api functions.

* History:
*  07/03/2018 : uart_api V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_API_
#define __UART_API_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "stm32l4xx_hal.h"

#include "usart.h"

#define UART_SEND_DATA_TIMEOUT 50
#define MAX_PRINTF_STR_SIZE 254

#define IS_SMSNUMBER_OR_PLUS(x) (IS_NUMBER_ONLY(x) || x == '+')
#define IS_ALPHABET_OR_NUMBER(x) (IS_NUMBER(x) || IS_ALPHABET(x))
#define IS_ALPHABET_NUM_POUND(x) (IS_ALPHABET(x) || IS_NUMBER_ONLY(x) || IS_POUND_CHAR(x))
#define IS_ALPHABET(x) ((x >= 'a' && x <= 'z') || (x >= 'A' && x <= 'Z'))
#define IS_LOWER_ONLY(x) (x >= 'a' && x <= 'f')
#define IS_NUMBER(x) ((x >= '0' && x <= '9') || x == '+' || x == '-' || x == '/' || x == '=' || x == '?')
#define IS_NUMBER_ONLY(x) (x >= '0' && x <= '9')
#define IS_IP_ADRESS(x) ((x >= '0' && x <= '9') || x == '.')
#define IS_SPACE_CHAR(x) (x == ' ' || x == '\r' || x == '\n')
#define IS_POUND_CHAR(x) (x == '#' || x == '*' || x == '$')

#define UART_END_CHAR_OD 0x0D
#define UART_END_CHAR_OA 0x0A
#define UART_FINISHED_RECV 0x200
#define UART_FIRST_END_CHAR 0x100
#define UART_BUF_MAX_LENGTH 0xFF

#if OEM_UART2_TO_MCU_UART1_MAP
#define UART1_RX_BUFFER_SIZE (256)
extern uint16_t Uart1RxCount;
extern uint8_t Uart1RxBuffer[UART1_RX_BUFFER_SIZE];
extern uint8_t Uart1ParseBuffer[UART1_RX_BUFFER_SIZE];
#endif /* OEM_UART2_TO_MCU_UART1_MAP */

extern uint16_t Uart4RxCount;
extern uint8_t Uart4RxBuffer[UART_BUF_MAX_LENGTH];

#define AT_RXCOUNT_VAR Uart4RxCount
#define AT_RXBUFFER_VAR Uart4RxBuffer
#define DEBUG_UART_HANDLE huart4

enum
{
  ATCMD_INFOEN = 0,
  ATCMD_DBGEN,
  BLE_DRIVER_DBG_EN,
  BLE_PROCESS_DBG_EN,
  LIS2DH_DBG_INFOEN,
  RTC_DBG_INFOEN,
  API_DBG_INFOEN,
  VERSION_DBG_INFOEN,
  NETWORK_DBG_INFOEN,
  ATCMD_FILETRANS_INFOEN,
  HTTP_DBG_INFOEN,
  DEFAULT_INFOEN,
  FILE_INFOEN,
  FACTORY_TEST_INFOEN,
  WIFI_DRIVER_DGB_EN,
  WIFI_PROCESS_DBG_EN,
};

enum
{
  LOG_EN_MCU_DRIVER = 0,
  LOG_EN_HTTP,
  LOG_EN_ATCMD_DBG,
  LOG_EN_MODEM,
  LOG_EN_DEFAULT,
  LOG_EN_FILE,
  LOG_EN_API,
  LOG_EN_FACTORY_TEST,

  LOG_EN_MAX
};

typedef struct
{
    uint8_t ATCmdInfoEn;
    uint8_t ATCmdDbgEn;
    uint8_t BLEDriverDbgEn;
    uint8_t BLEProcessDbgEn;
    uint8_t Lis2dhDbgInfoEn;
    uint8_t RTCDebugInfoEn;
    uint8_t ApiDebugInfoEn;
    uint8_t VersionDebugInfoEn;
    uint8_t NetworkDbgInfoEn;
    uint8_t AtCmdFileTransInfoEn;
    uint8_t HttpInfoEn;
    uint8_t DefaultInfoEn;
    uint8_t FileInfoEn;
    uint8_t FactoryTestInfoEn;
    uint8_t WifiDriverDbgEn;
    uint8_t WifiProcessDbgEn;
} DebugCtlPrarm;

extern DebugCtlPrarm DbgCtl;

extern void LogEnInfoControl(uint8_t type, uint8_t onoff);
extern void PutString(uint8_t *String);
extern uint32_t HexAsciiToHexData(char *pBuf, uint8_t len);
extern void PutIntHex(uint32_t num);
extern void PutIntDec(uint32_t num);
extern uint8_t StringToInt(uint8_t *dataBuf, uint16_t dataLen, uint16_t *idxSearch, int32_t *rtnInt);
extern void PutStrToUartDbg(char *str, uint16_t size);
extern int32_t SerialDbgPrintf(uint8_t type, char *fmt, ...);
extern int32_t PrintfBeforeRTOS(char *fmt, ...);

/* at handle return status */
#define STATUS_ERR -1
#define STATUS_OK 0
#define STATUS_BAD_ARGUMENTS 1
#define STATUS_TIME_OUT 2
#define STATUS_BAD_AT_CMD 3
#define STATUS_RELAYING 4

extern void PrintOutHexToAscii(char *str, uint16_t len);
extern void PintOutFileNameAndLine(char *Filename, uint32_t Line);
extern void UARTPrintMassData(uint8_t *string, uint16_t slen);
extern void printf_array(char *ptr_array_name, uint8_t *ptr_buffer, uint8_t buffer_size);

typedef struct
{
  uint32_t inIndex;
  uint32_t outIndex;
  uint32_t bufSize;
  uint8_t *pBuf;
} UARTChmodemRecTypeDef;

#define PRODUCT_DEBUG_VERSION (2)
#define PRODUCT_RELEASE_VERSION (3)
#define DEBUG_UART PRODUCT_DEBUG_VERSION

#if (DEBUG_UART == 1)
#define __FILE_NAME (strchr(__FILE__, '\\') ? (strchr(__FILE__, '\\') + 1) : (strchr(__FILE__, '/') ? (strchr(__FILE__, '/') + 1) : __FILE__))
#define FILE_NAME (strchr(__FILE_NAME, '\\') ? (strchr(__FILE_NAME, '\\') + 1) : (strchr(__FILE_NAME, '/') ? (strchr(__FILE_NAME, '/') + 1) : __FILE_NAME))
#define DebugLog(format, ...) SerialDbgPrintf(1, "[File:%s, Line:%4u]" format "\r\n", FILE_NAME, __LINE__, ##__VA_ARGS__)
#define DebugPrintf(value, format, ...) SerialDbgPrintf(value, "[File:%s, Line:%4u]" format "\r\n", FILE_NAME, __LINE__, ##__VA_ARGS__)
#elif (DEBUG_UART == PRODUCT_DEBUG_VERSION)
#define DebugLog(format, ...) SerialDbgPrintf(1, "\r\n" format, ##__VA_ARGS__)
#define DebugPrintf(value, format, ...) SerialDbgPrintf(value, format, ##__VA_ARGS__)
#elif (DEBUG_UART == PRODUCT_RELEASE_VERSION)
#define DebugLog(format, ...) SerialDbgPrintf(1, "\r\n" format, ##__VA_ARGS__)
#define DebugPrintf(value, format, ...) SerialDbgPrintf(value, format, ##__VA_ARGS__)
#else
#define DebugLog(format, ...)
#define DebugPrintf(value, format, ...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __UART_API_ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
