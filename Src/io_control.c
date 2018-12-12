/*******************************************************************************
* File Name          : io_control.c
* Author             : Yangjie Gu
* Description        : This file provides all the io_control functions.

* History:
*  10/18/2017 : io_control V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "io_control.h"

/* Variables -----------------------------------------------------------------*/
static const IO_ControlTypeDef IO_Control[IO_CONTROL_NUMBER_MAX] =
    {
        [IO_PC13_WIFI_IRQ] = { // 0
            .GPIOx = PC13_WIFI_IRQ_GPIO_Port,
            .GPIO_Pin = PC13_WIFI_IRQ_Pin,
        },
        [IO_PH0_MCU_PH0] = { // 1
            .GPIOx = PH0_MCU_PH0_GPIO_Port,
            .GPIO_Pin = PH0_MCU_PH0_Pin,
        },
        [IO_PH1_WIFI_RESET] = { // 2
            .GPIOx = PH1_WIFI_RESET_GPIO_Port,
            .GPIO_Pin = PH1_WIFI_RESET_Pin,
        },
        [IO_PA2_3V7_PWR_EN] = { // 3
            .GPIOx = PA2_3V7_PWR_EN_GPIO_Port,
            .GPIO_Pin = PA2_3V7_PWR_EN_Pin,
        },
        [IO_PA3_eMMC_PWR_EN] = { // 4
            .GPIOx = PA3_eMMC_PWR_EN_GPIO_Port,
            .GPIO_Pin = PA3_eMMC_PWR_EN_Pin,
        },
        [IO_PA4_SPI3_NSS] = { // 5
            .GPIOx = PA4_SPI3_NSS_GPIO_Port,
            .GPIO_Pin = PA4_SPI3_NSS_Pin,
        },
        [IO_PA5_WIFI_EN] = { // 6
            .GPIOx = PA5_WIFI_EN_GPIO_Port,
            .GPIO_Pin = PA5_WIFI_EN_Pin,
        },
        [IO_PA6_ACC_IN2] = { // 7
            .GPIOx = PA6_ACC_IN2_GPIO_Port,
            .GPIO_Pin = PA6_ACC_IN2_Pin,
        },
        [IO_PA7_WAKE_BLE] = { // 8
            .GPIOx = PA7_WAKE_BLE_GPIO_Port,
            .GPIO_Pin = PA7_WAKE_BLE_Pin,
        },
        [IO_PB0_BLE_IRQ] = { // 9
            .GPIOx = PB0_BLE_IRQ_GPIO_Port,
            .GPIO_Pin = PB0_BLE_IRQ_Pin,
        },
        [IO_PB1_ACC_IRQ] = { // 10
            .GPIOx = PB1_ACC_IRQ_GPIO_Port,
            .GPIO_Pin = PB1_ACC_IRQ_Pin,
        },
        [IO_PB12_BLE_CS] = { // 11
            .GPIOx = PB12_BLE_CS_GPIO_Port,
            .GPIO_Pin = PB12_BLE_CS_Pin,
        },
        [IO_PB14_WIFI_WAKE] = { // 12
            .GPIOx = PB14_WIFI_WAKE_GPIO_Port,
            .GPIO_Pin = PB14_WIFI_WAKE_Pin,
        },
        [IO_PB15_CAN0_ENn] = { // 13
            .GPIOx = PB15_CAN0_ENn_GPIO_Port,
            .GPIO_Pin = PB15_CAN0_ENn_Pin,
        },
        [IO_PA8_eMMC_RST] = { // 14
            .GPIOx = PA8_eMMC_RST_GPIO_Port,
            .GPIO_Pin = PA8_eMMC_RST_Pin,
        },
        [IO_PA15_GPIO0] = { // 15
            .GPIOx = PA15_GPIO0_GPIO_Port,
            .GPIO_Pin = PA15_GPIO0_Pin,
        },
        [IO_PB6_CAN_PWR_EN] = { // 16
            .GPIOx = PB6_CAN_PWR_EN_GPIO_Port,
            .GPIO_Pin = PB6_CAN_PWR_EN_Pin,
        },
        [IO_PB7_WIFI_SPI1_CS] = { // 17
            .GPIOx = PB7_WIFI_SPI1_CS_GPIO_Port,
            .GPIO_Pin = PB7_WIFI_SPI1_CS_Pin,
        },
};

const char * IO_INFO_STR[IO_CONTROL_NUMBER_MAX] = {
    "IO_PC13_WIFI_IRQ",
    "IO_PH0_MCU_PH0",
    "IO_PH1_WIFI_RESET",
    "IO_PA2_3V7_PWR_EN",
    "IO_PA3_eMMC_PWR_EN",
    "IO_PA4_SPI3_NSS",
    "IO_PA5_WIFI_EN",
    "IO_PA6_ACC_IN2",
    "IO_PA7_WAKE_BLE",
    "IO_PB0_BLE_IRQ",
    "IO_PB1_ACC_IRQ", 
    "IO_PB12_BLE_CS", 
    "IO_PB14_WIFI_WAKE", 
    "IO_PB15_CAN0_ENn", 
    "IO_PA8_eMMC_RST", 
    "IO_PA15_GPIO0", 
    "IO_PB6_CAN_PWR_EN", 
    "IO_PB7_WIFI_SPI1_CS", 
};

/* Function definition -------------------------------------------------------*/
uint8_t IO_Read(uint32_t IONumber)
{
    if (IONumber >= IO_CONTROL_NUMBER_MAX)
        return 0;

    return HAL_GPIO_ReadPin(IO_Control[IONumber].GPIOx, IO_Control[IONumber].GPIO_Pin);
}

void IO_Write(uint32_t IONumber, uint8_t level)
{
    if (IONumber >= IO_CONTROL_NUMBER_MAX)
        return;

    HAL_GPIO_WritePin(IO_Control[IONumber].GPIOx, IO_Control[IONumber].GPIO_Pin, (GPIO_PinState)level);
}

IOInOutStateCellTypeDef IOInoutStatRecArray[IO_CONTROL_NUMBER_MAX] = {0};

uint8_t IO_SetDirInput(uint32_t IONumber, uint32_t mode, uint32_t pull)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // IO_IN_OUT_STATE_IN (0x10) IO_IN_OUT_STATE_OUT (0x11)
    if (IONumber >= IO_CONTROL_NUMBER_MAX)
        return 1; // Err

    GPIO_InitStruct.Pin = IO_Control[IONumber].GPIO_Pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    HAL_GPIO_Init(IO_Control[IONumber].GPIOx, &GPIO_InitStruct);

    IOInoutStatRecArray[IONumber].inoutstate = IO_IN_OUT_STATE_IN;
    IOInoutStatRecArray[IONumber].laststate = READ_IO(IO_Control[IONumber].GPIOx, IO_Control[IONumber].GPIO_Pin);

    return 0;// Ok
}

uint8_t IO_SetDirOutput(uint32_t IONumber, uint32_t speed, GPIO_PinState PinState)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // IO_IN_OUT_STATE_IN (0x10) IO_IN_OUT_STATE_OUT (0x11)
    if (IONumber >= IO_CONTROL_NUMBER_MAX)
        return 1; // Err

    WRITE_IO(IO_Control[IONumber].GPIOx, IO_Control[IONumber].GPIO_Pin, PinState);

    GPIO_InitStruct.Pin = IO_Control[IONumber].GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = speed;
    HAL_GPIO_Init(IO_Control[IONumber].GPIOx, &GPIO_InitStruct);

    IOInoutStatRecArray[IONumber].inoutstate = IO_IN_OUT_STATE_OUT;

    return 0;// Ok
}

void IO_Initialization(void)
{
    // Always out and other Pins were set in void MX_GPIO_Init(void)
}

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
