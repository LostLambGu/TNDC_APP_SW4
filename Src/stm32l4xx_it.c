/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
#include "cmsis_os.h"

#include "can.h"
#include "spi.h"
#include "uart_api.h"
#include "usrtimer.h"
#include "oemmsg_queue_process.h"
#include "BLEDriver.h"
#include "WifiDriver.h"

/* Private define ------------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern RTC_HandleTypeDef hrtc;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern SPI_HandleTypeDef hspi3;
// extern DMA_HandleTypeDef hdma_sdmmc1_rx;
// extern DMA_HandleTypeDef hdma_sdmmc1_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern __IO uint8_t factorymodeindicatefalg;
extern __IO uint8_t uartFactoryRec;

extern QueueHandle_t xOEMMsgQueueHandle;

extern __IO uint8_t UARTChmodemRecModeFlag;
extern UARTChmodemRecTypeDef UARTChmodemRec;

extern uint16_t wifiSpiCsLowCount;
extern uint16_t wifiSpiCsHighCount;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  SoftwareTimerCounter();
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
  extern void SetAccIntHappenStatus(uint8_t status);
  SetAccIntHappenStatus(TRUE);
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel4 global interrupt.
*/
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
* @brief This function handles CAN1 RX0 interrupt.
*/
void CAN1_RX0_IRQHandler(void)
{
  if ((__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_FMP0) != RESET))
  {
    // uint32_t tmp1 = 0U;
    CAN_HandleTypeDef *phcan = &hcan1;
    CanRxMsgTypeDef *pRxMsg = NULL;

    /* Set RxMsg pointer */
    pRxMsg = &(CAN1ReceiveCell.MsgBuf[CAN1ReceiveCell.input]);

    /* Get the Id */
    pRxMsg->IDE = (uint8_t)0x04U & phcan->Instance->sFIFOMailBox[CAN_FIFO0].RIR;
    if (pRxMsg->IDE == CAN_ID_STD)
    {
      pRxMsg->StdId = 0x000007FFU & (phcan->Instance->sFIFOMailBox[CAN_FIFO0].RIR >> 21U);
    }
    else
    {
      pRxMsg->ExtId = 0x1FFFFFFFU & (phcan->Instance->sFIFOMailBox[CAN_FIFO0].RIR >> 3U);
    }

    pRxMsg->RTR = (uint8_t)0x02U & phcan->Instance->sFIFOMailBox[CAN_FIFO0].RIR;
    /* Get the DLC */
    pRxMsg->DLC = (uint8_t)0x0FU & phcan->Instance->sFIFOMailBox[CAN_FIFO0].RDTR;
    /* Get the FIFONumber */
    pRxMsg->FIFONumber = CAN_FIFO0;
    /* Get the FMI */
    pRxMsg->FMI = (uint8_t)0xFFU & (phcan->Instance->sFIFOMailBox[CAN_FIFO0].RDTR >> 8U);
    /* Get the data field */
    pRxMsg->Data[0] = (uint8_t)0xFFU & phcan->Instance->sFIFOMailBox[CAN_FIFO0].RDLR;
    pRxMsg->Data[1] = (uint8_t)0xFFU & (phcan->Instance->sFIFOMailBox[CAN_FIFO0].RDLR >> 8U);
    pRxMsg->Data[2] = (uint8_t)0xFFU & (phcan->Instance->sFIFOMailBox[CAN_FIFO0].RDLR >> 16U);
    pRxMsg->Data[3] = (uint8_t)0xFFU & (phcan->Instance->sFIFOMailBox[CAN_FIFO0].RDLR >> 24U);
    pRxMsg->Data[4] = (uint8_t)0xFFU & phcan->Instance->sFIFOMailBox[CAN_FIFO0].RDHR;
    pRxMsg->Data[5] = (uint8_t)0xFFU & (phcan->Instance->sFIFOMailBox[CAN_FIFO0].RDHR >> 8U);
    pRxMsg->Data[6] = (uint8_t)0xFFU & (phcan->Instance->sFIFOMailBox[CAN_FIFO0].RDHR >> 16U);
    pRxMsg->Data[7] = (uint8_t)0xFFU & (phcan->Instance->sFIFOMailBox[CAN_FIFO0].RDHR >> 24U);
    /* Release the FIFO */
    /* Release FIFO0 */
    __HAL_CAN_FIFO_RELEASE(phcan, CAN_FIFO0);

    CAN1ReceiveCell.input++;
    CAN1ReceiveCell.input %= CAN1_MAX_RECEIVE_BUF_NUM;
  }
  else
  {
    HAL_CAN_IRQHandler(&hcan1);
  }
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles I2C2 event interrupt.
*/
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
* @brief This function handles SPI2 global interrupt.
*/
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
* @brief This function handles USART global interrupt.
*/
static void UARTx_IRQ_Com_Handler(UART_HandleTypeDef *huart)
{
  /* UART parity error interrupt occurred ------------------------------------*/
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE) != RESET)
  {
    __HAL_UART_CLEAR_FEFLAG(huart);
    huart->ErrorCode |= HAL_UART_ERROR_PE;
  }
  /* UART frame error interrupt occurred -------------------------------------*/
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) != RESET)
  {
    __HAL_UART_CLEAR_FEFLAG(huart);
    huart->ErrorCode |= HAL_UART_ERROR_FE;
  }
  /* UART noise error interrupt occurred -------------------------------------*/
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE) != RESET)
  {
    __HAL_UART_CLEAR_NEFLAG(huart);
    huart->ErrorCode |= HAL_UART_ERROR_NE;
  }
  /* UART Over-Run interrupt occurred ----------------------------------------*/
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
  }
  /* UART wakeup from Stop mode interrupt occurred -------------------------------------*/
  if(__HAL_UART_GET_FLAG(huart, UART_IT_WUF) != RESET)
  { 
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_WUF);
  }

  if (huart->ErrorCode != HAL_UART_ERROR_NONE)
  {
  }
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  #if (OEM_UART2_TO_MCU_UART1_MAP == 0)
  HAL_UART_IRQHandler(&huart1);
  #else /* OEM_UART2_TO_MCU_UART1_MAP */
  UART_HandleTypeDef *huart = &huart1;

  UARTx_IRQ_Com_Handler(huart);

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
  {
    extern __IO uint8_t factorymodeindicatefalg;
    if (factorymodeindicatefalg == TRUE)
    {
      uint8_t RevData = (uint8_t)(huart->Instance->RDR & (uint8_t)0x00FF);
      while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) == RESET)
        ;
      huart->Instance->TDR = RevData;
    }
    else
    {
      extern void UART1_RxCpltCallback(uint8_t Data);
      UART1_RxCpltCallback((uint8_t)(huart->Instance->RDR & (uint8_t)0x00FF));
    }

    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
  }
#endif /* OEM_UART2_TO_MCU_UART1_MAP */
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);

    /* CS pin rising edge - close SPI communication */
    if (HAL_GPIO_ReadPin(PB12_BLE_CS_GPIO_Port, PB12_BLE_CS_Pin) != GPIO_PIN_RESET)
    {
      // DEBUG_NOTES(GPIO_CS_RISING);
      if (SPI_STATE_CHECK(SPI_PROT_WAITING_DATA_STATE))
      {
        BLE_SPI_IRQ_PIN_RESET();
        
        SPI_STATE_TRANSACTION(SPI_PROT_TRANS_COMPLETE_STATE);
      }
    }
    /* CS pin falling edge - start SPI communication */
    else
    {
      // DEBUG_NOTES(GPIO_CS_FALLING);
      if (SPI_STATE_CHECK(SPI_PROT_CONFIGURED_EVENT_PEND_STATE))
      {
        SPI_STATE_TRANSACTION(SPI_PROT_WAITING_HEADER_STATE);
      }
      else if (SPI_STATE_CHECK(SPI_PROT_SLEEP_STATE) || SPI_STATE_CHECK(SPI_PROT_CONFIGURED_STATE))
      {
        if (BLE_SPI_WAKEUP_PIN_READ() != GPIO_PIN_RESET)
        {
          SPI_STATE_TRANSACTION(SPI_PROT_CONFIGURED_HOST_REQ_STATE);
        }
      }
    }
  }

  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);

    /* CS pin rising edge - close SPI communication */
    if (HAL_GPIO_ReadPin(PB14_WIFI_WAKE_GPIO_Port, PB14_WIFI_WAKE_Pin) != GPIO_PIN_RESET)
    {
      // DEBUG_NOTES(GPIO_CS_RISING);
      wifiSpiCsHighCount++;
    }
    /* CS pin falling edge - start SPI communication */
    else
    {
      // DEBUG_NOTES(GPIO_CS_FALLING);
      wifiSpiCsLowCount++;
    }
  }

  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles RTC alarm interrupt through EXTI line 18.
*/
void RTC_Alarm_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_Alarm_IRQn 0 */

  /* USER CODE END RTC_Alarm_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_Alarm_IRQn 1 */

  /* USER CODE END RTC_Alarm_IRQn 1 */
}

/**
* @brief This function handles SPI3 global interrupt.
*/
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

// /**
// * @brief This function handles SDMMC1 global interrupt.
// */
// void SDMMC1_IRQHandler(void)
// {
//   /* USER CODE BEGIN SDMMC1_IRQn 0 */

//   /* USER CODE END SDMMC1_IRQn 0 */
//   // HAL_SD_IRQHandler(&hsd1);
//   extern void EMMC_IRQHandler(void);
//   EMMC_IRQHandler();
//   /* USER CODE BEGIN SDMMC1_IRQn 1 */

//   /* USER CODE END SDMMC1_IRQn 1 */
// }

/**
* @brief This function handles UART4 global interrupt.
*/
void UART4_IRQHandler(void)
{
  UART_HandleTypeDef *huart = &DEBUG_UART_HANDLE;
  UARTx_IRQ_Com_Handler(huart);
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
  {
    uint8_t RevData;
    uint16_t sendvalue;
    BaseType_t higherprioritytowake;
    /* Read one byte from the receive data register and send it back */
    RevData = (uint8_t)(huart->Instance->RDR & (uint8_t)0x00FF);

    if (UARTChmodemRecModeFlag == TRUE)
    {
      (UARTChmodemRec.pBuf)[UARTChmodemRec.inIndex] = RevData;
      UARTChmodemRec.inIndex++;
      UARTChmodemRec.inIndex %= UARTChmodemRec.bufSize;
      __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
      return;
    }

    if (0x1b == RevData)
    {
      factorymodeindicatefalg = FALSE;
      __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
      return;
    }

    if (factorymodeindicatefalg)
    {
      uartFactoryRec = RevData;
      __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
      return;
    }

    huart->Instance->TDR = RevData;

    if (RevData == UART_END_CHAR_OD)
    {
      AT_RXCOUNT_VAR |= UART_FINISHED_RECV; // Finished
      huart->Instance->TDR = '\n';
      if (NULL != xOEMMsgQueueHandle)
      {
        sendvalue = OEMMSG_QUEUE_MCU_ATCMD;
        xQueueSendToBackFromISR(xOEMMsgQueueHandle, &sendvalue, &higherprioritytowake);
        portYIELD_FROM_ISR(higherprioritytowake);
      }
    }
    else if ((RevData != 0x08) && (RevData != 0x7f))
    {
      AT_RXBUFFER_VAR[AT_RXCOUNT_VAR & UART_BUF_MAX_LENGTH] = RevData;
      AT_RXCOUNT_VAR++;
      if (AT_RXCOUNT_VAR >= UART_BUF_MAX_LENGTH)
      {
        AT_RXCOUNT_VAR = 0; // Error
      }
    }
    else
    {
      if (AT_RXCOUNT_VAR != 0)
      {
        AT_RXCOUNT_VAR--;

        if (RevData == 0x7f)
        {
          while (__HAL_UART_GET_FLAG(&DEBUG_UART_HANDLE, UART_FLAG_TXE) == 0);
          huart->Instance->TDR = 0x08;
        }
        while (__HAL_UART_GET_FLAG(&DEBUG_UART_HANDLE, UART_FLAG_TXE) == 0);
        huart->Instance->TDR = ' ';
        while (__HAL_UART_GET_FLAG(&DEBUG_UART_HANDLE, UART_FLAG_TXE) == 0);
        huart->Instance->TDR = 0x08;
      }
      
      AT_RXBUFFER_VAR[AT_RXCOUNT_VAR & UART_BUF_MAX_LENGTH] = 0;
    }

    /* Clear RXNE interrupt flag */
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
  }
}

/**
* @brief This function handles DMA2 channel1 global interrupt.
*/
void DMA2_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel1_IRQn 0 */

  /* USER CODE END DMA2_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA2_Channel1_IRQn 1 */

  /* USER CODE END DMA2_Channel1_IRQn 1 */
}

/**
* @brief This function handles DMA2 channel2 global interrupt.
*/
void DMA2_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel2_IRQn 0 */

  /* USER CODE END DMA2_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA2_Channel2_IRQn 1 */

  /* USER CODE END DMA2_Channel2_IRQn 1 */
}

// /**
// * @brief This function handles DMA2 channel4 global interrupt.
// */
// void DMA2_Channel4_IRQHandler(void)
// {
//   /* USER CODE BEGIN DMA2_Channel4_IRQn 0 */

//   /* USER CODE END DMA2_Channel4_IRQn 0 */
//   HAL_DMA_IRQHandler(&hdma_sdmmc1_rx);
//   /* USER CODE BEGIN DMA2_Channel4_IRQn 1 */

//   /* USER CODE END DMA2_Channel4_IRQn 1 */
// }

// /**
// * @brief This function handles DMA2 channel5 global interrupt.
// */
// void DMA2_Channel5_IRQHandler(void)
// {
//   /* USER CODE BEGIN DMA2_Channel5_IRQn 0 */

//   /* USER CODE END DMA2_Channel5_IRQn 0 */
//   HAL_DMA_IRQHandler(&hdma_sdmmc1_tx);
//   /* USER CODE BEGIN DMA2_Channel5_IRQn 1 */

//   /* USER CODE END DMA2_Channel5_IRQn 1 */
// }

/**
* @brief This function handles I2C3 event interrupt.
*/
void I2C3_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_EV_IRQn 0 */

  /* USER CODE END I2C3_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_EV_IRQn 1 */

  /* USER CODE END I2C3_EV_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
