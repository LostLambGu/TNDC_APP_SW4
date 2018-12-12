/*******************************************************************************
* File Name          : io_control.h
* Author             : Yangjie Gu
* Description        : This file provides all the io_control functions.

* History:
*  10/18/2017 : io_control V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IO_CONTROL_H_
#define __IO_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Exported constants --------------------------------------------------------*/
#define IO_PC13_WIFI_IRQ (0)
#define IO_PH0_MCU_PH0 (1)
#define IO_PH1_WIFI_RESET (2)
#define IO_PA2_3V7_PWR_EN (3)
#define IO_PA3_eMMC_PWR_EN (4)
#define IO_PA4_SPI3_NSS (5)
#define IO_PA5_WIFI_EN (6)
#define IO_PA6_ACC_IN2 (7)
#define IO_PA7_WAKE_BLE (8)
#define IO_PB0_BLE_IRQ (9)
#define IO_PB1_ACC_IRQ (10)
#define IO_PB12_BLE_CS (11)
#define IO_PB14_WIFI_WAKE (12)
#define IO_PB15_CAN0_ENn (13)
#define IO_PA8_eMMC_RST (14)
#define IO_PA15_GPIO0 (15)
#define IO_PB6_CAN_PWR_EN (16)
#define IO_PB7_WIFI_SPI1_CS (17)
#define IO_CONTROL_NUMBER_MAX (18)

#define IO_LEVEL_LOW (0)
#define IO_LEVEL_HIGH (1)

#define IO_IN_OUT_STATE_IN (0x10)
#define IO_IN_OUT_STATE_OUT (0x11)

#define READ_IO(port, pin) ((uint8_t)HAL_GPIO_ReadPin(port, pin))
#define WRITE_IO(port, pin, state) HAL_GPIO_WritePin(port, pin, state)

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
} IO_ControlTypeDef;

typedef struct
{
    uint8_t inoutstate;
    uint8_t laststate;
} IOInOutStateCellTypeDef;

/* Variables -----------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
extern IOInOutStateCellTypeDef IOInoutStatRecArray[IO_CONTROL_NUMBER_MAX];
extern const char * IO_INFO_STR[IO_CONTROL_NUMBER_MAX];

extern uint8_t IO_Read(uint32_t IONumber);

extern void IO_Write(uint32_t IONumber, uint8_t level);

extern uint8_t IO_SetDirInput(uint32_t IONumber, uint32_t mode, uint32_t pull);

extern uint8_t IO_SetDirOutput(uint32_t IONumber, uint32_t speed, GPIO_PinState PinState);

extern void IO_Initialization(void);

#ifdef __cplusplus
}
#endif

#endif /* __IO_CONTROL_H_ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
