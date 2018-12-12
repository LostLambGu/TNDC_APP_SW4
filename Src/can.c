/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"

#include "usrtimer.h"
#include "oemmsg_queue_process.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CANReceiveCellTypeDef CAN1ReceiveCell = {0};

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_3TQ;
  hcan1.Init.BS2 = CAN_BS2_4TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    // Enable Can Transceiver
    HAL_GPIO_WritePin(GPIOB, PB15_CAN0_ENn_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = PB15_CAN0_ENn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = PA11_RXCAN0_Pin|PA12_TXCAN0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, PA11_RXCAN0_Pin|PA12_TXCAN0_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

void CAN1_Normal_Init(uint32_t baudrate)
{
  CAN_FilterConfTypeDef sFilterConfig;
  
  HAL_CAN_DeInit(&hcan1);

  hcan1.Instance = CAN1;

  if (baudrate == 1000000U)
  {
    hcan1.Init.Prescaler = 4;
  }
  else if (baudrate == 500000U)
  {
    hcan1.Init.Prescaler = 8;
  }
  else if (baudrate == 250000U)
  {
    hcan1.Init.Prescaler = 16;
  }
  else // Default baudrate 125,000
  {
    hcan1.Init.Prescaler = 32;
  }

  hcan1.Init.SJW = CAN_SJW_2TQ;
  hcan1.Init.BS1 = CAN_BS1_9TQ;
  hcan1.Init.BS2 = CAN_BS2_6TQ;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sFilterConfig.FilterIdHigh = 0;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 0;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FOV0 | CAN_IT_FMP0);
}

int CAN1_filter(uint32_t id, uint32_t mask, uint8_t format, int32_t handle) 
{
		CAN_FilterConfTypeDef sFilterConfig;	
    int retval=0;

    // filter for CANAny format cannot be configured for STM32
    if((format == 0) || (format == 1)) {
        sFilterConfig.FilterNumber = handle;
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

        if(format == 0) {
            sFilterConfig.FilterIdHigh = id << 5;
            sFilterConfig.FilterIdLow =  0x0;
            sFilterConfig.FilterMaskIdHigh = mask << 5;
            sFilterConfig.FilterMaskIdLow = 0x0;    // allows both remote and data frames
        }
        else if(format == 1){
            sFilterConfig.FilterIdHigh = id >> 13;  // EXTID[28:13]
            sFilterConfig.FilterIdLow = (0x00FF & (id << 3)) | (1 << 2);  // EXTID[12:0]
            sFilterConfig.FilterMaskIdHigh = mask >> 13;
            sFilterConfig.FilterMaskIdLow = (0x00FF & (mask << 3)) | (1 << 2);
        }

        sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.BankNumber = handle;

			 if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			} 
        retval = handle;
    }
    return retval;
}


void CAN1_Normal_InitEx(uint32_t baudrate, CANFILTER_T *filters, uint8_t filetersize)
{
  uint8_t i;
	
  HAL_CAN_DeInit(&hcan1);

  hcan1.Instance = CAN1;

  if (baudrate == 1000000U)
  {
    hcan1.Init.Prescaler = 4;
  }
  else if (baudrate == 500000U)
  {
    hcan1.Init.Prescaler = 8;
  }
  else if (baudrate == 250000U)
  {
    hcan1.Init.Prescaler = 16;
  }
  else // Default baudrate 125,000
  {
    hcan1.Init.Prescaler = 32;
  }

  hcan1.Init.SJW = CAN_SJW_2TQ;
  hcan1.Init.BS1 = CAN_BS1_9TQ;
  hcan1.Init.BS2 = CAN_BS2_6TQ;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

	if(filetersize)
	{
		for(i=0; i<filetersize; i++)
		{
			if(filters[i].id)
					CAN1_filter(filters[i].id, filters[i].mask, filters[i].mode, filters[i].handle);
		}
	}
	else
	{
		CAN1_filter(0,0xFFFFFFFF,0,0);
	}

  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FOV0 | CAN_IT_FMP0);

}

uint8_t CAN1_Transmit(uint8_t *pData, uint32_t dataLen, uint32_t stdId, uint32_t extId, uint32_t idType, uint32_t dataType)
{
  /*CAN TEST*/
  uint8_t i = 0;
  CanTxMsgTypeDef pTxMsg = {0};
  HAL_StatusTypeDef retStatus = HAL_OK;

  if ((pData == NULL) || (dataLen == 0) || (dataLen > sizeof(hcan1.pTxMsg->Data)))
  {
    return 0x10;
  }

  hcan1.pTxMsg = &pTxMsg;
  hcan1.pTxMsg->StdId = stdId;
  hcan1.pTxMsg->ExtId = extId;
  hcan1.pTxMsg->IDE = idType;
  hcan1.pTxMsg->RTR = dataType;
  hcan1.pTxMsg->DLC = dataLen;
  for (i = 0; i < dataLen; i++)
  {
    hcan1.pTxMsg->Data[i] = pData[i];
  }
  
  retStatus = HAL_CAN_Transmit(&hcan1, MCU_MAX_CAN_TRANSMIT_TIMEOUT_MS);
  // #include "uart_api.h"
  // DebugLog("--->>>CAN1_Transmit retStatus %d", retStatus);
  if (retStatus != HAL_OK)
  {
    HAL_CAN_DeInit(&hcan1);

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
  }

  return retStatus;
}

uint8_t CANSendDataWithLen(uint8_t *pmsg, uint32_t len)
{
    #define CAN1_DEFAULT_STDID (0xAB)
    #define CAN1_DEFALUT_EXTID (0x00)
    uint32_t i = 0, j = 0, count = 0, left = 0;
    CanTxMsgTypeDef pTxMsg = {0};

    if ((pmsg == NULL) || (len == 0))
    {
        return 0;
    }

    count = len / CAN1_MAX_FRAME_LEN_BYTES;
    left = len % CAN1_MAX_FRAME_LEN_BYTES;
    
    hcan1.pTxMsg = &pTxMsg;
    hcan1.pTxMsg->StdId = CAN1_DEFAULT_STDID;
    hcan1.pTxMsg->ExtId = CAN1_DEFALUT_EXTID;
    hcan1.pTxMsg->IDE = CAN_ID_STD;
    hcan1.pTxMsg->RTR = CAN_RTR_DATA;
    hcan1.pTxMsg->DLC = CAN1_MAX_FRAME_LEN_BYTES;

    while (count)
    {
        for (i = 0; i < CAN1_MAX_FRAME_LEN_BYTES; i++)
        {
            hcan1.pTxMsg->Data[i] = pmsg[i + j * CAN1_MAX_FRAME_LEN_BYTES];
        }

        if (HAL_OK != HAL_CAN_Transmit(&hcan1, MCU_MAX_CAN_TRANSMIT_TIMEOUT_MS))
        {
          HAL_CAN_DeInit(&hcan1);

          if (HAL_CAN_Init(&hcan1) != HAL_OK)
          {
            _Error_Handler(__FILE__, __LINE__);
          }
          return 0;
        }

        j++;
        count--;
    }

    if (left != 0)
    {
        hcan1.pTxMsg->DLC = left;

        for (i = 0; i < left; i++)
        {
            hcan1.pTxMsg->Data[i] = pmsg[i + j * CAN1_MAX_FRAME_LEN_BYTES];
        }

        if (HAL_OK != HAL_CAN_Transmit(&hcan1, MCU_MAX_CAN_TRANSMIT_TIMEOUT_MS))
        {
          HAL_CAN_DeInit(&hcan1);

          if (HAL_CAN_Init(&hcan1) != HAL_OK)
          {
            _Error_Handler(__FILE__, __LINE__);
          }
          return 0;
        }
    }

    return 1;
}

void CheckCAN1RecTimerCallback(uint8_t Status)
{
  if (CAN1ReceiveCell.input == CAN1ReceiveCell.output)
  {
    // Reset Timer
    SoftwareTimerReset(&CAN1RecTimer, CheckCAN1RecTimerCallback, CHECK_CAN1_REC_TIMEOUT);
    SoftwareTimerStart(&CAN1RecTimer);
    return;
  }

  SendToOEMMsgQueue(OEMMSG_QUEUE_MCU_CAN1_RECEICED);

  // Reset Timer
  SoftwareTimerReset(&CAN1RecTimer, CheckCAN1RecTimerCallback, CHECK_CAN1_REC_TIMEOUT);
  SoftwareTimerStart(&CAN1RecTimer);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
