/**
  ******************************************************************************
  * @file    emmc_diskio.c (based on sd_diskio_dma_rtos_template.c v2.0.1)
  * @brief   EMMC Disk I/O driver
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
/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection*/

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "emmc_diskio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
   
#define QUEUE_SIZE         (uint32_t) 10
#define READ_CPLT_MSG      (uint32_t) 1
#define WRITE_CPLT_MSG     (uint32_t) 2
/*
 * the following Timeout is useful to give the control back to the applications
 * in case of errors in either BSP_EMMC_ReadCpltCallback() or BSP_EMMC_WriteCpltCallback()
 * the value by default is 30 Secs but it may be adjusted depending on the application 
 * use case
 */
#define EMMC_TIMEOUT (1000 * 30)

/* 
 * when using cachable memory region, it may be needed to maintain the cache
 * validity. Enable the define below to activate a cache maintenance at each
 * read and write operation.
 * Notice: This is applicable only for cortex M7 based platform.
 */

/* #define ENABLE_EMMC_DMA_CACHE_MAINTENANCE  1 */

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
// static osMessageQId EMMCQueueID;
/* Private function prototypes -----------------------------------------------*/
static DSTATUS EMMC_CheckStatus(BYTE lun);
DSTATUS EMMC_initialize (BYTE);
DSTATUS EMMC_status (BYTE);
DRESULT EMMC_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT EMMC_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT EMMC_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef  EMMC_Driver =
{
  EMMC_initialize,
  EMMC_status,
  EMMC_read,
#if  _USE_WRITE == 1
  EMMC_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  EMMC_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/
static DSTATUS EMMC_CheckStatus(BYTE lun)
{
  Stat = STA_NOINIT;

  // if(BSP_EMMC_GetCardState() == MEMMC_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS EMMC_initialize(BYTE lun)
{
  EMMC_CheckStatus(lun);
  return BSP_EMMC_Init();
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS EMMC_status(BYTE lun)
{
  return EMMC_CheckStatus(lun);
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */
/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT EMMC_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;

  if(BSP_EMMC_ReadBlocks((uint32_t*)buff, (uint32_t) (sector), count) == MEMMC_OK)
  {
      res = RES_OK;
  }

  return res;
}

// DRESULT EMMC_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
// {
//   DRESULT res = RES_ERROR;
//   osEvent event;
//   uint32_t timer;
// #if (ENABLE_EMMC_DMA_CACHE_MAINTENANCE == 1)
//   uint32_t alignedAddr;
// #endif

//   if(BSP_EMMC_ReadBlocks_DMA((uint32_t*)buff,
//                            (uint32_t) (sector),
//                            count) == MEMMC_OK)
//   {
//     /* wait for a message from the queue or a timeout */
//     event = osMessageGet(EMMCQueueID, EMMC_TIMEOUT);

//     if (event.status == osEventMessage)
//     {
//       if (event.value.v == READ_CPLT_MSG)
//       {
//         timer = osKernelSysTick() + EMMC_TIMEOUT;
//         /* block until SDIO IP is ready or a timeout occur */
//         while(timer > osKernelSysTick())
//         {
//           if (BSP_EMMC_GetCardState() == EMMC_TRANSFER_STATE_OK)
//           {
//             res = RES_OK;
// #if (ENABLE_EMMC_DMA_CACHE_MAINTENANCE == 1)
//             /*
//                the SCB_InvalidateDCache_by_Addr() requires a 32-Bit aligned address,
//                adjust the address and the D-Cache size to invalidate accordingly.
//              */
//             alignedAddr = (uint32_t)buff & ~3;
//             SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, count*BLOCKSIZE + ((uint32_t)buff - alignedAddr));
// #endif
//             break;
//           }
//         }
//       }
//     }
//   }

//   return res;
// }

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */
/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT EMMC_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;

  if(BSP_EMMC_WriteBlocks((uint32_t*)buff, (uint32_t) (sector), count) == MEMMC_OK)
  {
    res = RES_OK;
  }

  return res;
}
#endif /* _USE_WRITE == 1 */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */
/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT EMMC_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_EMMC_CardInfo CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_EMMC_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    BSP_EMMC_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    BSP_EMMC_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

/* USER CODE BEGIN callbackSection */ 
/* can be used to modify / following code or add new code */
/* USER CODE END callbackSection */
/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: EMMC handle
  * @retval None
  */
 /*
   ===============================================================================
    Select the correct function signature depending on your platform.
    please refer to the file "stm32xxxx_eval_sd.h" to verify the correct function
    prototype
   ===============================================================================
  */
//void BSP_SD_WriteCpltCallback(uint32_t SdCard)
// void BSP_EMMC_WriteCpltCallback()
// {
//   osMessagePut(EMMCQueueID, WRITE_CPLT_MSG, osWaitForever);
// }

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: EMMC handle
  * @retval None
  */

  /*
   ===============================================================================
    Select the correct function signature depending on your platform.
    please refer to the file "stm32xxxx_eval_sd.h" to verify the correct function
    prototype
   ===============================================================================
  */
//void BSP_SD_ReadCpltCallback(uint32_t SdCard)
// void BSP_EMMC_ReadCpltCallback()
// {
//   osMessagePut(EMMCQueueID, READ_CPLT_MSG, osWaitForever);
// }

/* USER CODE BEGIN lastSection */ 
/* can be used to modify / undefine previous code or add new code */
/* USER CODE END lastSection */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
