/*******************************************************************************
* File Name          : BLEDriver.h
* Author             : Yangjie Gu
* Description        : This file provides all the BLEDriver functions.

* History:
*  07/20/2018 : BLEDriver V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_DRIVER_
#define __BLE_DRIVER_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>

#include "stm32l4xx_hal.h"

#include "fifo.h"
#include "uart_api.h"

/* Private define ------------------------------------------------------------*/
/* Public defines ------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

typedef enum {
  SPI_PROT_INIT_STATE = 0,                  /* Initialization phase         */
  SPI_PROT_CONFIGURED_STATE,                /* Configuration phase          */
  SPI_PROT_SLEEP_STATE,                     /* Sleep phase                  */
  SPI_PROT_CONFIGURED_HOST_REQ_STATE,       /* Host request phase           */
  SPI_PROT_CONFIGURED_EVENT_PEND_STATE,     /* Event pending phase          */
  SPI_PROT_WAITING_HEADER_STATE,            /* Waiting header phase         */
  SPI_PROT_HEADER_RECEIVED_STATE,           /* Header received phase        */
  SPI_PROT_WAITING_DATA_STATE,              /* Waiting data phase           */
  SPI_PROT_TRANS_COMPLETE_STATE,            /* Transaction complete phase   */
  SPI_PROT_TRANS_ERROR_STATE
} SpiProtoType;

extern volatile SpiProtoType spi_proto_state;
#define SPI_STATE_TRANSACTION(NEWSTATE)        spi_proto_state = NEWSTATE
#define SPI_STATE_CHECK(STATE)                (spi_proto_state==STATE)
#define SPI_STATE_FROM(STATE)                 (spi_proto_state>=STATE)

#define BLE_WAKEUP_PORT PA7_WAKE_BLE_GPIO_Port
#define BLE_WAKEUP_PIN PA7_WAKE_BLE_Pin
#define BLE_SPI_IRQ_PORT PB0_BLE_IRQ_GPIO_Port
#define BLE_SPI_IRQ_PIN PB0_BLE_IRQ_Pin
#define BLE_SPI_CS_PORT PB12_BLE_CS_GPIO_Port
#define BLE_SPI_CS_PIN PB12_BLE_CS_Pin

#define BLE_SPI_WAKEUP_PIN_SET() HAL_GPIO_WritePin(BLE_WAKEUP_PORT, BLE_WAKEUP_PIN, GPIO_PIN_SET)
#define BLE_SPI_WAKEUP_PIN_RESET() HAL_GPIO_WritePin(BLE_WAKEUP_PORT, BLE_WAKEUP_PIN, GPIO_PIN_RESET)
#define BLE_SPI_WAKEUP_PIN_READ() HAL_GPIO_ReadPin(BLE_WAKEUP_PORT, BLE_WAKEUP_PIN)

#define BLE_SPI_IRQ_PIN_SET() HAL_GPIO_WritePin(BLE_SPI_IRQ_PORT, BLE_SPI_IRQ_PIN, GPIO_PIN_SET)
#define BLE_SPI_IRQ_PIN_RESET() HAL_GPIO_WritePin(BLE_SPI_IRQ_PORT, BLE_SPI_IRQ_PIN, GPIO_PIN_RESET)
#define BLE_SPI_IRQ_PIN_READ() HAL_GPIO_ReadPin(BLE_SPI_IRQ_PORT, BLE_SPI_IRQ_PIN)

#define BLE_SPI_CS_READ() HAL_GPIO_ReadPin(BLE_SPI_CS_PORT, BLE_SPI_CS_PIN)

#define BLE_SPI_ITEM_MAX_SIZE 252
#define BLE_CMD_BUFFER_SIZE 1024
#define BLE_EVT_BUFFER_SIZE ((BLE_SPI_ITEM_MAX_SIZE + 4) * 2)
#define BLE_TMP_BUFFER_SIZE (BLE_SPI_ITEM_MAX_SIZE + 4)

extern circular_fifo_t bleCmdFifo;
extern circular_fifo_t bleEvtFifo;

extern void BLEDriverInit(void);
extern void BLEStateTrans(void);
extern void BLETransportTick(void);
extern HAL_StatusTypeDef BLEWaitSpiTransferCpl(uint32_t timeout, uint32_t tickStart);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_DRIVER_ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2018. All rights reserved
                                End Of The File
*******************************************************************************/
