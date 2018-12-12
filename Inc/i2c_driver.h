/*******************************************************************************
* File Name          : i2c_driver.h
* Author               : Jevon
* Description        : This file provides all the i2c_driver functions.

* History:
*  04/01/2015 : i2c_driver V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _IICBUSCOMMUNICATION_H
#define _IICBUSCOMMUNICATION_H
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#if GSENSOR_I2C_USE_GPIO_SIMULATION

typedef enum {
	ACK = 1,
	NACK = 2,
	NOACK = 3,
} etAck;

// Error codes
typedef enum {
	NO_ERROR = 0x00,
	ACK_ERROR = 0x01,
	TIME_OUT_ERROR = 0x02, // timeout error
	CHECKSUM_ERROR = 0x04, // checksum mismatch error
	UNIT_ERROR = 0x08,
	PARM_ERROR = 0x80, // parameter out of range error
					   //CHECKSUM_ERROR 	= 0x04,

} etError;

#define EXT_IIC_SCL_PORT PB10_I2C2_SCL_GSEN_GPIO_Port
#define EXT_IIC_SCL_PIN PB10_I2C2_SCL_GSEN_Pin

#define EXT_IIC_SDA_PORT PB11_I2C2_SDA_GSEN_GPIO_Port
#define EXT_IIC_SDA_PIN PB11_I2C2_SDA_GSEN_Pin

#define EXTIIC_SCK_LOW()		HAL_GPIO_WritePin(EXT_IIC_SCL_PORT, EXT_IIC_SCL_PIN, GPIO_PIN_RESET)
#define EXTIIC_SCK_HIGH()		HAL_GPIO_WritePin(EXT_IIC_SCL_PORT, EXT_IIC_SCL_PIN, GPIO_PIN_SET)
#define EXTIIC_SDA_LOW()		HAL_GPIO_WritePin(EXT_IIC_SDA_PORT, EXT_IIC_SDA_PIN, GPIO_PIN_RESET)
#define EXTIIC_SDA_HIGH()		HAL_GPIO_WritePin(EXT_IIC_SDA_PORT, EXT_IIC_SDA_PIN, GPIO_PIN_SET)

#define EXTIIC_SCL_READ()		HAL_GPIO_ReadPin(EXT_IIC_SCL_PORT,EXT_IIC_SCL_PIN)	
#define EXTIIC_SDA_READ()		HAL_GPIO_ReadPin(EXT_IIC_SDA_PORT,EXT_IIC_SDA_PIN)	

#define EXTIIC_SET_SCL_OUT()	 EXTIICGPIO.Pin = EXT_IIC_SCL_PIN;	\
								 EXTIICGPIO.Mode = GPIO_MODE_OUTPUT_PP;	\
								 EXTIICGPIO.Pull = GPIO_NOPULL;	\
								 EXTIICGPIO.Speed = GPIO_SPEED_FREQ_LOW;	\
								 HAL_GPIO_Init(EXT_IIC_SCL_PORT, &EXTIICGPIO)

#define EXTIIC_SET_SCL_IN()		EXTIICGPIO.Pin = EXT_IIC_SCL_PIN;	\
								EXTIICGPIO.Mode = GPIO_MODE_INPUT;	\
								EXTIICGPIO.Pull = GPIO_NOPULL;	\
								EXTIICGPIO.Speed = GPIO_SPEED_FREQ_LOW;	\
								HAL_GPIO_Init(EXT_IIC_SCL_PORT, &EXTIICGPIO)

#define EXTIIC_SET_SDA_OUT()	EXTIICGPIO.Pin = EXT_IIC_SDA_PIN;	\
								EXTIICGPIO.Mode = GPIO_MODE_OUTPUT_PP;	\
								EXTIICGPIO.Pull = GPIO_NOPULL;	\
								EXTIICGPIO.Speed = GPIO_SPEED_FREQ_LOW;	\
								HAL_GPIO_Init(EXT_IIC_SDA_PORT, &EXTIICGPIO)

#define EXTIIC_SET_SDA_IN()		EXTIICGPIO.Pin = EXT_IIC_SDA_PIN;	\
								EXTIICGPIO.Mode = GPIO_MODE_INPUT;	\
								EXTIICGPIO.Pull = GPIO_NOPULL;	\
								EXTIICGPIO.Speed = GPIO_SPEED_FREQ_LOW;	\
								HAL_GPIO_Init(EXT_IIC_SDA_PORT, &EXTIICGPIO)

#define I2C_READ  0x01
#define I2C_WRITE 0xFE

//Function Declare
extern void ExtensionI2cInit(void);
extern void ExtI2cStartCondition(void);
extern void ExtI2cStopCondition(void);
extern void ExtI2cWriteACK(etAck ack);
extern uint8_t ExtI2cWaitACK(void);
extern uint8_t ExtI2cWriteByte(uint8_t wdata);
extern uint8_t ExtI2cReadByte(etAck ack);
extern void ExtI2cWriteRegister(uint8_t address, uint8_t reg,uint8_t val);
extern void ExtI2cWriteSerialRegister(uint8_t address, uint8_t reg, uint8_t count, uint8_t * buff);
extern uint8_t  ExtI2cReadRegister(uint8_t address, uint8_t reg);
extern void ExtI2cReadSerialData(uint8_t address, uint8_t reg,uint8_t count,uint8_t * buff);

#endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */

#endif /* _IICBUSCOMMUNICATION_H */

/*******************************************************************************
End Of The File
*******************************************************************************/

