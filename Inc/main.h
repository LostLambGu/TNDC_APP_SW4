/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PC13_WIFI_IRQ_Pin GPIO_PIN_13
#define PC13_WIFI_IRQ_GPIO_Port GPIOC
#define PH0_MCU_PH0_Pin GPIO_PIN_0
#define PH0_MCU_PH0_GPIO_Port GPIOH
#define PH1_WIFI_RESET_Pin GPIO_PIN_1
#define PH1_WIFI_RESET_GPIO_Port GPIOH
#define PC0_I2C_SCL3_EXT_Pin GPIO_PIN_0
#define PC0_I2C_SCL3_EXT_GPIO_Port GPIOC
#define PC1_I2C_SDA3_EXT_Pin GPIO_PIN_1
#define PC1_I2C_SDA3_EXT_GPIO_Port GPIOC
#define PC2_BLE_MISO_Pin GPIO_PIN_2
#define PC2_BLE_MISO_GPIO_Port GPIOC
#define PC3_BLE_MOSI_Pin GPIO_PIN_3
#define PC3_BLE_MOSI_GPIO_Port GPIOC
#define PA0_MCU_TX4_Pin GPIO_PIN_0
#define PA0_MCU_TX4_GPIO_Port GPIOA
#define PA1_MCU_RX4_Pin GPIO_PIN_1
#define PA1_MCU_RX4_GPIO_Port GPIOA
#define PA2_3V7_PWR_EN_Pin GPIO_PIN_2
#define PA2_3V7_PWR_EN_GPIO_Port GPIOA
#define PA3_eMMC_PWR_EN_Pin GPIO_PIN_3
#define PA3_eMMC_PWR_EN_GPIO_Port GPIOA
#define PA4_SPI3_NSS_Pin GPIO_PIN_4
#define PA4_SPI3_NSS_GPIO_Port GPIOA
#define PA5_WIFI_EN_Pin GPIO_PIN_5
#define PA5_WIFI_EN_GPIO_Port GPIOA
#define PA6_ACC_IN2_Pin GPIO_PIN_6
#define PA6_ACC_IN2_GPIO_Port GPIOA
#define PA7_WAKE_BLE_Pin GPIO_PIN_7
#define PA7_WAKE_BLE_GPIO_Port GPIOA
#define PC4_MCU_TX3_Pin GPIO_PIN_4
#define PC4_MCU_TX3_GPIO_Port GPIOC
#define PC5_MCU_RX3_Pin GPIO_PIN_5
#define PC5_MCU_RX3_GPIO_Port GPIOC
#define PB0_BLE_IRQ_Pin GPIO_PIN_0
#define PB0_BLE_IRQ_GPIO_Port GPIOB
#define PB1_ACC_IRQ_Pin GPIO_PIN_1
#define PB1_ACC_IRQ_GPIO_Port GPIOB
#define PB2_eMMC_RCLK_Pin GPIO_PIN_2
#define PB2_eMMC_RCLK_GPIO_Port GPIOB
#define PB10_I2C2_SCL_GSEN_Pin GPIO_PIN_10
#define PB10_I2C2_SCL_GSEN_GPIO_Port GPIOB
#define PB11_I2C2_SDA_GSEN_Pin GPIO_PIN_11
#define PB11_I2C2_SDA_GSEN_GPIO_Port GPIOB
#define PB12_BLE_CS_Pin GPIO_PIN_12
#define PB12_BLE_CS_GPIO_Port GPIOB
#define PB13_BLE_SCLK_Pin GPIO_PIN_13
#define PB13_BLE_SCLK_GPIO_Port GPIOB
#define PB14_WIFI_WAKE_Pin GPIO_PIN_14
#define PB14_WIFI_WAKE_GPIO_Port GPIOB
#define PB15_CAN0_ENn_Pin GPIO_PIN_15
#define PB15_CAN0_ENn_GPIO_Port GPIOB
#define PC6_eMMC_DAT6_Pin GPIO_PIN_6
#define PC6_eMMC_DAT6_GPIO_Port GPIOC
#define PC7_eMMC_DAT7_Pin GPIO_PIN_7
#define PC7_eMMC_DAT7_GPIO_Port GPIOC
#define PC8_eMMC_DAT0_Pin GPIO_PIN_8
#define PC8_eMMC_DAT0_GPIO_Port GPIOC
#define PC9_eMMC_DAT1_Pin GPIO_PIN_9
#define PC9_eMMC_DAT1_GPIO_Port GPIOC
#define PA8_eMMC_RST_Pin GPIO_PIN_8
#define PA8_eMMC_RST_GPIO_Port GPIOA
#define PA9_P09_MCU_TX1_Pin GPIO_PIN_9
#define PA9_P09_MCU_TX1_GPIO_Port GPIOA
#define PA10_P10_MCU_RX1_Pin GPIO_PIN_10
#define PA10_P10_MCU_RX1_GPIO_Port GPIOA
#define PA11_RXCAN0_Pin GPIO_PIN_11
#define PA11_RXCAN0_GPIO_Port GPIOA
#define PA12_TXCAN0_Pin GPIO_PIN_12
#define PA12_TXCAN0_GPIO_Port GPIOA
#define PA15_GPIO0_Pin GPIO_PIN_15
#define PA15_GPIO0_GPIO_Port GPIOA
#define PC10_eMMC_DAT2_Pin GPIO_PIN_10
#define PC10_eMMC_DAT2_GPIO_Port GPIOC
#define PC11_eMMC_DAT3_Pin GPIO_PIN_11
#define PC11_eMMC_DAT3_GPIO_Port GPIOC
#define PC12_eMMC_CLK_Pin GPIO_PIN_12
#define PC12_eMMC_CLK_GPIO_Port GPIOC
#define PD2_eMMC_CMD_Pin GPIO_PIN_2
#define PD2_eMMC_CMD_GPIO_Port GPIOD
#define PB3_SPI3_SCK_Pin GPIO_PIN_3
#define PB3_SPI3_SCK_GPIO_Port GPIOB
#define PB4_SPI3_MISO_Pin GPIO_PIN_4
#define PB4_SPI3_MISO_GPIO_Port GPIOB
#define PB5_SPI3_MOSI_Pin GPIO_PIN_5
#define PB5_SPI3_MOSI_GPIO_Port GPIOB
#define PB6_CAN_PWR_EN_Pin GPIO_PIN_6
#define PB6_CAN_PWR_EN_GPIO_Port GPIOB
#define PB7_WIFI_SPI1_CS_Pin GPIO_PIN_7
#define PB7_WIFI_SPI1_CS_GPIO_Port GPIOB
#define PB8_eMMC_DAT4_Pin GPIO_PIN_8
#define PB8_eMMC_DAT4_GPIO_Port GPIOB
#define PB9_eMMC_DAT5_Pin GPIO_PIN_9
#define PB9_eMMC_DAT5_GPIO_Port GPIOB

#define TNDC_DEBUG_LOG_LOCK_BINARY (1)
#define TNDC_DEBUG_LOG_LOCK_MUTEX (0)
#define TNDC_DEBUG_LOG_LOCK_EXCLUSIVE (0)

#define GSENSOR_I2C_USE_GPIO_SIMULATION (1)

#define FATFS_SDMMC_USE_DMA (1)
#if (!FATFS_SDMMC_USE_DMA)
#define EMMC_NO_DMA_SUPPORT_OS (1)
#endif /* FATFS_SDMMC_USE_DMA */

#define OEM_UART2_TO_MCU_UART1_MAP (1)

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
