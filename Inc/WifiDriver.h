/*******************************************************************************
* File Name          : WifiDriver.h
* Author             : Yangjie Gu
* Description        : This file provides all the WifiDriver functions.

* History:
*  07/18/2018 : WifiDriver V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WIFI_DRIVER_
#define __WIFI_DRIVER_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "stm32l4xx_hal.h"

#include "uart_api.h"

/* Private define ------------------------------------------------------------*/
/* Public defines ------------------------------------------------------------*/
#define WIFI_RESET_PORT PH1_WIFI_RESET_GPIO_Port
#define WIFI_RESET_PIN PH1_WIFI_RESET_Pin
#define WIFI_CHIP_EN_PORT PA5_WIFI_EN_GPIO_Port
#define WIFI_CHIP_EN_PIN PA5_WIFI_EN_Pin
#define WIFI_FLASH_MODE_PORT PA15_GPIO0_GPIO_Port
#define WIFI_FLASH_MODE_PIN PA15_GPIO0_Pin
#define WIFI_MTDO_GPIO15_PORT PB7_WIFI_SPI1_CS_GPIO_Port
#define WIFI_MTDO_GPIO15_PIN PB7_WIFI_SPI1_CS_Pin
#define WIFI_IRQ_PORT PC13_WIFI_IRQ_GPIO_Port
#define WIFI_IRQ_PIN PC13_WIFI_IRQ_Pin
#define WIFI_CS_PORT PB14_WIFI_WAKE_GPIO_Port
#define WIFI_CS_PIN PB14_WIFI_WAKE_Pin

#define WIFI_RESET_HOLD_MS (1000)
#define WIFI_RESET()  do { \
    HAL_GPIO_WritePin(WIFI_RESET_PORT, WIFI_RESET_PIN, GPIO_PIN_RESET); \
    HAL_Delay(WIFI_RESET_HOLD_MS); \
    HAL_GPIO_WritePin(WIFI_RESET_PORT, WIFI_RESET_PIN, GPIO_PIN_SET); \
} while (0U);

#define WIFI_CHIP_ENABLE() HAL_GPIO_WritePin(WIFI_CHIP_EN_PORT, WIFI_CHIP_EN_PIN, GPIO_PIN_SET)
#define WIFI_CHIP_DISABLE() HAL_GPIO_WritePin(WIFI_CHIP_EN_PORT, WIFI_CHIP_EN_PIN, GPIO_PIN_RESET)

#define WIFI_FLASH_MODE_RUN() HAL_GPIO_WritePin(WIFI_FLASH_MODE_PORT, WIFI_FLASH_MODE_PIN, GPIO_PIN_SET)
#define WIFI_FLASH_MODE_DOWNLOAD() HAL_GPIO_WritePin(WIFI_FLASH_MODE_PORT, WIFI_FLASH_MODE_PIN, GPIO_PIN_RESET)

#define WIFI_MTDO_GPIO15_HIGH() HAL_GPIO_WritePin(WIFI_MTDO_GPIO15_PORT, WIFI_MTDO_GPIO15_PIN, GPIO_PIN_SET)
#define WIFI_MTDO_GPIO15_LOW() HAL_GPIO_WritePin(WIFI_MTDO_GPIO15_PORT, WIFI_MTDO_GPIO15_PIN, GPIO_PIN_RESET)

#define WIFI_IRQ_SET_HIGH() HAL_GPIO_WritePin(WIFI_IRQ_PORT, WIFI_IRQ_PIN, GPIO_PIN_SET)
#define WIFI_IRQ_SET_LOW() HAL_GPIO_WritePin(WIFI_IRQ_PORT, WIFI_IRQ_PIN, GPIO_PIN_RESET)

#define WIFI_CS_PIN_READ() HAL_GPIO_ReadPin(WIFI_CS_PORT, WIFI_CS_PIN)

extern volatile uint8_t wifiRestart;

typedef enum {
  WIFI_SPI_PROT_INIT_STATE = 0,                  /* Initialization phase         */
  WIFI_SPI_PROT_CONFIGURED_STATE,                /* Configuration phase          */
  WIFI_SPI_PROT_SLEEP_STATE,                     /* Sleep phase                  */
  WIFI_SPI_PROT_CONFIGURED_HOST_REQ_STATE,       /* Host request phase           */
  WIFI_SPI_PROT_CONFIGURED_EVENT_PEND_STATE,     /* Event pending phase          */
  WIFI_SPI_PROT_WAITING_HEADER_STATE,            /* Waiting header phase         */
  WIFI_SPI_PROT_HEADER_RECEIVED_STATE,           /* Header received phase        */
  WIFI_SPI_PROT_WAITING_DATA_STATE,              /* Waiting data phase           */
  WIFI_SPI_PROT_TRANS_COMPLETE_STATE,            /* Transaction complete phase   */
  WIFI_SPI_PROT_TRANS_WAIT_COMPLETE_STATE,            /* Transaction complete phase   */
  WIFI_SPI_PROT_TRANS_ERROR_STATE
} WifiSpiProtoType;

extern volatile WifiSpiProtoType wifi_spi_proto_state;
#define WIFI_SPI_STATE_TRANSACTION(NEWSTATE)        wifi_spi_proto_state = NEWSTATE
#define WIFI_SPI_STATE_CHECK(STATE)                (wifi_spi_proto_state==STATE)
#define WIFI_SPI_STATE_FROM(STATE)                 (wifi_spi_proto_state>=STATE)

/* External function declaration ---------------------------------------------*/
extern void WifiInit(void);
extern void WifiEnterFlashDownLoadMode(void);
extern void WifiStartUp(void);
extern void WifiStateTrans(void);
extern void WifiTransportTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_DRIVER_ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
