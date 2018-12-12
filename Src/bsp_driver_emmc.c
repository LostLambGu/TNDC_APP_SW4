/**
 ******************************************************************************
  * @file    bsp_driver_emmc.c for L4 (based on stm32l4r9i_eval_sd.c)
  * @brief   This file includes a generic uSD card driver.
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

#ifdef OLD_API
/* kept to avoid issue when migrating old projects. */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#else
/* USER CODE BEGIN FirstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END FirstSection */
/* Includes ------------------------------------------------------------------*/
#include "bsp_driver_emmc.h"

#include "SdioEmmcDrive.h"
#include "uart_api.h"

#define BSP_EMMC_DRIVER_DEBUG (0)

/* Extern variables ---------------------------------------------------------*/ 
/* USER CODE BEGIN BeforeInitSection */
/* can be used to modify / undefine following code or add code */
/* USER CODE END BeforeInitSection */
/**
  * @brief  Initializes the EMMC card device.
  * @retval EMMC status
  */
uint8_t BSP_EMMC_Init(void)
{
  uint8_t sd_state = MEMMC_OK;
  uint32_t count = 0;
  static uint8_t initialized = 0;
  /* Check if the EMMC card is plugged in the slot */
  if (initialized == 0)
  {
    if (EmmcInit() != MEMMC_OK)
    {
#if BSP_EMMC_DRIVER_DEBUG
      DebugLog("EmmcInit Failed!");
#endif /* BSP_EMMC_DRIVER_DEBUG */
      return MEMMC_ERROR_EMMC_NOT_PRESENT;
    }
    else
    {
      while (EMMC_CARD_TRANSFER != EmmcGetState())
      {
        count++;
        if (count == 0xfff)
        {
#if BSP_EMMC_DRIVER_DEBUG
          DebugLog("EmmcInit TimeOut!");
#endif /* BSP_EMMC_DRIVER_DEBUG */
          return MEMMC_ERROR;
        }
        HAL_Delay(1);
      }
#if BSP_EMMC_DRIVER_DEBUG
      DebugLog("EmmcInit OK!");
#endif /* BSP_EMMC_DRIVER_DEBUG */
      initialized = 1;
      return MEMMC_OK;
    }
  }

#if BSP_EMMC_DRIVER_DEBUG
      DebugLog("EmmcInit Already Initialized!");
#endif /* BSP_EMMC_DRIVER_DEBUG */
  
  return sd_state;
}
/* USER CODE BEGIN AfterInitSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END AfterInitSection */

/**
  * @brief  Configures Interrupt mode for EMMC detection pin.
  * @retval Returns 0 in success otherwise 1. 
  */
// uint8_t BSP_EMMC_ITConfig(void)
// {  
//   /* TBI: add user code here depending on the hardware configuration used */
  
//   return (uint8_t)0;
// }

/** @brief  EMMC detect IT treatment
  * @retval None
  */
// void BSP_EMMC_DetectIT(void)
// {
//   /* EMMC detect IT callback */
//   BSP_EMMC_DetectCallback();
  
// }

/** @brief  EMMC detect IT detection callback
  * @retval None
  */
// __weak void BSP_EMMC_DetectCallback(void)
// {
//   /* NOTE: This function Should not be modified, when the callback is needed,
//   the BSP_EMMC_DetectCallback could be implemented in the user file
//   */ 
  
// }

/* USER CODE BEGIN BeforeReadBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadBlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an EMMC card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of EMMC blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval EMMC status
  */
uint8_t BSP_EMMC_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MEMMC_OK;

#if BSP_EMMC_DRIVER_DEBUG
  DebugLog("ReadBlocks ReadAddr(%d) NumOfBlocks(%d)", ReadAddr, NumOfBlocks);
#endif /* BSP_EMMC_DRIVER_DEBUG */

#if FATFS_SDMMC_USE_DMA
  if (EmmcReadBlocksDMA((uint8_t *)pData, ReadAddr, NumOfBlocks, EMMC_OPERATION_TIMEOUT) != 0)
#else
  if (EMMC_ReadBlocks((uint8_t *)pData, ReadAddr, NumOfBlocks, EMMC_OPERATION_TIMEOUT) != HAL_OK)
#endif /* FATFS_SDMMC_USE_DMA */
  {
    sd_state = MEMMC_ERROR;
  }

  return sd_state;  
}

/* USER CODE BEGIN BeforeWriteBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteBlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an EMMC card, in polling mode. 
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of EMMC blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval EMMC status
  */
uint8_t BSP_EMMC_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MEMMC_OK;

#if BSP_EMMC_DRIVER_DEBUG
  DebugLog("WriteBlocks WriteAddr(%d) NumOfBlocks(%d)", WriteAddr, NumOfBlocks);
#endif /* BSP_EMMC_DRIVER_DEBUG */

#if FATFS_SDMMC_USE_DMA
  if (EmmcWriteBlocksDMA((uint8_t *)pData, WriteAddr, NumOfBlocks, EMMC_OPERATION_TIMEOUT) != 0)
#else
  if (EMMC_WriteBlocks((uint8_t *)pData, WriteAddr, NumOfBlocks, EMMC_OPERATION_TIMEOUT) != HAL_OK)
#endif /* FATFS_SDMMC_USE_DMA */
  {
    sd_state = MEMMC_ERROR;
  }

  return sd_state;  
}
// uint8_t BSP_EMMC_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
// {
//   uint8_t sd_state = MEMMC_OK;

//   if (HAL_SD_WriteBlocks(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) != HAL_OK) 
//   {
//     sd_state = MEMMC_ERROR;
//   }

//   return sd_state;  
// }

/* USER CODE BEGIN BeforeReadDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadDMABlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an EMMC card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of EMMC blocks to read 
  * @retval EMMC status
  */
// uint8_t BSP_EMMC_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
// {
//   uint8_t sd_state = MEMMC_OK;
  
//   /* Read block(s) in DMA transfer mode */
//   if (HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
//   {
//     sd_state = MEMMC_ERROR;
//   }
  
//   return sd_state; 
// }

/* USER CODE BEGIN BeforeWriteDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteDMABlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an EMMC card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of EMMC blocks to write 
  * @retval EMMC status
  */
// uint8_t BSP_EMMC_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
// {
//   uint8_t sd_state = MEMMC_OK;
  
//   /* Write block(s) in DMA transfer mode */
//   if (HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
//   {
//     sd_state = MEMMC_ERROR;
//   }
  
//   return sd_state; 
// }

/* USER CODE BEGIN BeforeEraseSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeEraseSection */
/**
  * @brief  Erases the specified memory area of the given EMMC card. 
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval EMMC status
  */
// uint8_t BSP_EMMC_Erase(uint32_t StartAddr, uint32_t EndAddr)
// {
//   uint8_t sd_state = MEMMC_OK;

//   if (HAL_SD_Erase(&hsd1, StartAddr, EndAddr) != HAL_OK)  
//   {
//     sd_state = MEMMC_ERROR;
//   }

//   return sd_state; 
// }

/* USER CODE BEGIN BeforeHandlersSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeHandlersSection */
/**
  * @brief  Handles EMMC card interrupt request.
  * @retval None
  */
// void BSP_EMMC_IRQHandler(void)
// {
//   HAL_SD_IRQHandler(&hsd1);
// }

/* USER CODE BEGIN BeforeGetCardStateSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeGetCardStateSection */

/**
  * @brief  Gets the current EMMC card data status.
  * @param  None
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  EMMC_TRANSFER_OK: No data transfer is acting
  *            @arg  EMMC_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t BSP_EMMC_GetCardState(void)
{
  return ((EMMC_CARD_TRANSFER == EmmcGetState()) ? EMMC_TRANSFER_OK : EMMC_TRANSFER_BUSY);
}

/**
  * @brief  Get EMMC information about specific EMMC card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None 
  */
void BSP_EMMC_GetCardInfo(BSP_EMMC_CardInfo *CardInfo)
{
  /* Get EMMC card Information */
  CardInfo->BlockNbr = EMMC_SECTOR_COUNT;
  CardInfo->BlockSize = EMMC_BLOCK_SIZE_BYTE;
  CardInfo->LogBlockNbr = EMMC_SECTOR_COUNT;
  CardInfo->LogBlockSize = EMMC_BLOCK_SIZE_BYTE;
}

/* USER CODE BEGIN BeforeCallBacksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeCallBacksSection */
/**
  * @brief EMMC Abort callbacks
  * @param hsd: EMMC handle
  * @retval None
  */
// void HAL_EMMC_AbortCallback(SD_HandleTypeDef *hsd)
// {
//   BSP_EMMC_AbortCallback();
// }

/**
  * @brief Tx Transfer completed callback
  * @param hsd: EMMC handle
  * @retval None
  */
// void EMMC_TxCpltCallback(SDMMC_TypeDef *SDMMCx)
// {
//   UNUSED(SDMMCx);
//   BSP_EMMC_WriteCpltCallback();
// }

/**
  * @brief Rx Transfer completed callback
  * @param hsd: EMMC handle
  * @retval None
  */
// void EMMC_RxCpltCallback(SDMMC_TypeDef *SDMMCx)
// {
//   UNUSED(SDMMCx);
//   BSP_EMMC_ReadCpltCallback();
// }

/* USER CODE BEGIN CallBacksSection_C */
/**
  * @brief BSP EMMC Abort callback
  * @retval None
  */
// __weak void BSP_EMMC_AbortCallback(void)
// {

// }

/**
  * @brief BSP Tx Transfer completed callback
  * @retval None
  */
// __weak void BSP_EMMC_WriteCpltCallback(void)
// {

// }

/**
  * @brief BSP Rx Transfer completed callback
  * @retval None
  */
// __weak void BSP_EMMC_ReadCpltCallback(void)
// {

// }
/* USER CODE END CallBacksSection_C */
#endif

/**
 * @brief  Detects if EMMC card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if EMMC is detected or not
 */
// uint8_t BSP_EMMC_IsDetected(void)
// {
//   __IO uint8_t status = EMMC_PRESENT;

//   /* USER CODE BEGIN 1 */
//   /* user code can be inserted here */
//   /* USER CODE END 1 */    	

//   return status;
// }

/* USER CODE BEGIN AdditionalCode */
/* user code can be inserted here */
/* USER CODE END AdditionalCode */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
