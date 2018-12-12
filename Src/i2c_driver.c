/*******************************************************************************
* File Name          : i2c_driver.c
* Author               : Jevon
* Description        : This file provides all the i2c_driver functions.

* History:
*  04/01/2015 : i2c_driver V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "initialization.h"
#include "i2c_driver.h"

#if GSENSOR_I2C_USE_GPIO_SIMULATION
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define EXTIIC_DELAY __NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
					 __NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
					 __NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
					 __NOP();__NOP();__NOP();
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void ExtIicDisableAllInterrupt(void)
{
	// BaseType_t SchedulerState = xTaskGetSchedulerState();
	// if (SchedulerState == taskSCHEDULER_NOT_STARTED)
	// {
	// 	__disable_irq();
	// }
	// else
	// {
	// 	taskENTER_CRITICAL();
	// }
}

static void ExtIicEnableAllInterrupt(void)
{
	// BaseType_t SchedulerState = xTaskGetSchedulerState();
	// if (SchedulerState == taskSCHEDULER_NOT_STARTED)
	// {
	// 	__enable_irq();
	// }
	// else
	// {
	// 	taskEXIT_CRITICAL();
	// }
}

void ExtensionI2cInit(void)
{
	GPIO_InitTypeDef EXTIICGPIO;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	EXTIIC_SET_SCL_OUT();
	EXTIIC_SET_SDA_OUT();
}

void ExtI2cStartCondition(void)
{ 
	GPIO_InitTypeDef EXTIICGPIO;
	/***************************************************
	START: make sure here SDIO_DIR =OUT, SCLK = 1,	SDIO = 1
	****************************************************/
	EXTIIC_SET_SDA_OUT(); 		/*SDIO_DIR = OUT;*/
	EXTIIC_SCK_HIGH(); 			/*SCLK = 1;*/
	EXTIIC_SDA_HIGH(); 			/*SDIO = 1;*/
	EXTIIC_DELAY;
	EXTIIC_SDA_LOW(); 			/*SDIO = 0;*/
	EXTIIC_DELAY;
	EXTIIC_SCK_LOW(); 			/*SCLK = 0;*/
	EXTIIC_DELAY;
}

void ExtI2cStopCondition(void)
{
	GPIO_InitTypeDef EXTIICGPIO;
	EXTIIC_SET_SDA_OUT(); 		/*SDIO_DIR = OUT;*/
	EXTIIC_SDA_LOW(); 			/*SDIO = 0;*/
	EXTIIC_DELAY;
	EXTIIC_SCK_HIGH(); 			/*SCLK = 1;*/
	EXTIIC_DELAY;
	EXTIIC_SDA_HIGH(); 			/*SDIO = 1;*/
	EXTIIC_DELAY;
}

void ExtI2cWriteACK(etAck ack)
{
	GPIO_InitTypeDef EXTIICGPIO;
	EXTIIC_SET_SDA_OUT(); 		/*SDIO_DIR = OUT;*/
	if (ack == NACK)
	{
		EXTIIC_SDA_HIGH(); 		/*SDIO = 1;*/
	}
	else
	{
		EXTIIC_SDA_LOW(); 		/*SDIO = 0;*/
	}
	EXTIIC_DELAY;
	EXTIIC_SCK_HIGH(); 			/*SCLK = 1;*/
	EXTIIC_DELAY;
	EXTIIC_SCK_LOW(); 			/*SCLK = 0;*/
	EXTIIC_DELAY;
}

uint8_t ExtI2cWaitACK(void)
{
	GPIO_InitTypeDef EXTIICGPIO;
	uint8_t errtime=50,error=NO_ERROR;
	EXTIIC_SDA_HIGH(); 		/*SDIO = 1;*/
	EXTIIC_SET_SDA_IN(); 		/*SDIO_DIR = IN;*/
	EXTIIC_DELAY;
	EXTIIC_SCK_HIGH(); 			/*SCLK = 1;*/
	EXTIIC_DELAY;
	while(EXTIIC_SDA_READ() == GPIO_PIN_SET)
	{
		errtime--;
		if(!errtime) 
		{
			//ExtI2cStopCondition();
			error=ACK_ERROR; //check ack from i2c slave
			break;
		}
	}
	EXTIIC_SCK_LOW(); 			/*SCLK = 0;*/
	EXTIIC_DELAY;
	return error;
}

uint8_t ExtI2cWriteByte(uint8_t wdata)
{
	GPIO_InitTypeDef EXTIICGPIO;
	uint8_t i,error=NO_ERROR;
	EXTIIC_SET_SDA_OUT(); 		/*SDIO_DIR = OUT;*/
	for(i=0;i<8;i++)
	{
		if(wdata&0x80)
			EXTIIC_SDA_HIGH(); 		/*SDIO = 1;*/
		else
			EXTIIC_SDA_LOW(); 		/*SDIO = 0;*/
		wdata<<=1;
		EXTIIC_DELAY;
		EXTIIC_SCK_HIGH(); 			/*SCLK = 1;*/
		EXTIIC_DELAY;
		EXTIIC_SCK_LOW(); 			/*SCLK = 0;*/
	}
	error = ExtI2cWaitACK();
	return error;
}

uint8_t ExtI2cReadByte(etAck ack)
{
	GPIO_InitTypeDef EXTIICGPIO;
	uint8_t i = 0;
	uint8_t bytedata = 0;
	EXTIIC_SDA_HIGH(); 		/*SDIO = 1;*/
	EXTIIC_SET_SDA_IN(); 		/*SDIO_DIR = IN;*/
	EXTIIC_DELAY;
	for(i=0;i<8;i++)
	{
		EXTIIC_SCK_HIGH(); 			/*SCLK = 1;*/
		EXTIIC_DELAY;
		bytedata = (bytedata << 1) | (EXTIIC_SDA_READ());
		EXTIIC_SCK_LOW(); 			/*SCLK = 0;*/
		EXTIIC_DELAY;
	}
	ExtI2cWriteACK(ack);
	return(bytedata);
}

void ExtI2cWriteRegister(uint8_t address, uint8_t reg,uint8_t val)
{
	ExtIicDisableAllInterrupt() ;// Disable Interrupt
	ExtI2cStartCondition();
	ExtI2cWriteByte(address&I2C_WRITE);
	ExtI2cWriteByte(reg);
	ExtI2cWriteByte(val);
	ExtI2cStopCondition();
	ExtIicEnableAllInterrupt();	// Enable Interrupt
}

void ExtI2cWriteSerialRegister(uint8_t address, uint8_t reg, uint8_t count, uint8_t * buff)
{
	ExtIicDisableAllInterrupt() ;// Disable Interrupt
	ExtI2cStartCondition();
	ExtI2cWriteByte(address&I2C_WRITE);
	ExtI2cWriteByte(reg);
	while(count--)
	{
		ExtI2cWriteByte(*buff);
		buff++;
	}
	ExtI2cStopCondition();
	ExtIicEnableAllInterrupt();	// Enable Interrupt
}

uint8_t  ExtI2cReadRegister(uint8_t address, uint8_t reg)
{
	uint8_t rdata;
	ExtIicDisableAllInterrupt() ;// Disable Interrupt
	ExtI2cStartCondition();
	ExtI2cWriteByte(address&I2C_WRITE);
	ExtI2cWriteByte(reg);
	ExtI2cStartCondition();
	ExtI2cWriteByte(address|I2C_READ);
	rdata=ExtI2cReadByte(NACK);
	ExtI2cStopCondition();
	ExtIicEnableAllInterrupt();	// Enable Interrupt
	return(rdata);
}

void ExtI2cReadSerialData(uint8_t address, uint8_t reg,uint8_t count,uint8_t * buff)
{
	uint8_t i;
	ExtIicDisableAllInterrupt() ;// Disable Interrupt
	ExtI2cStartCondition();
	ExtI2cWriteByte(address&I2C_WRITE);
	ExtI2cWriteByte(reg);
	ExtI2cStartCondition();
	ExtI2cWriteByte(address|I2C_READ);
	for(i=0;i<count;i++)
	{
		if(i<count-1) 
			buff[i]=ExtI2cReadByte(ACK);
		else
			buff[i]=ExtI2cReadByte(NACK);
	}
	ExtI2cStopCondition();
	ExtIicEnableAllInterrupt();	// Enable Interrupt
}

#endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */

/*******************************************************************************
End Of The File
*******************************************************************************/

