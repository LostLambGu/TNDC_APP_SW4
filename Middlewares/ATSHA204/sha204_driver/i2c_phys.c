/** \file
 *  \brief Functions of Hardware Dependent Part of ATSHA204 Physical Layer
 *         Using I<SUP>2</SUP>C For Communication
 *  \author Atmel Crypto Products
 *  \date  January 11, 2013
 * \copyright Copyright (c) 2013 Atmel Corporation. All rights reserved.
 *
 * \atsha204_library_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel integrated circuit.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \atsha204_library_license_stop
 */

#include "i2c_phys.h"     // definitions and declarations for the hardware dependent I2C module
#include "i2c_driver.h"
#include "software_timer_utilities.h"

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define NACK   TRUE
#define ACK    FALSE

typedef uint8_t bool;

#define SDA_GPIO_OUT_LOW() HAL_GPIO_WritePin(PB11_I2C2_SDA_GSEN_GPIO_Port, PB11_I2C2_SDA_GSEN_Pin, GPIO_PIN_RESET)	
#define SDA_GPIO_OUT_HIGH() HAL_GPIO_WritePin(PB11_I2C2_SDA_GSEN_GPIO_Port, PB11_I2C2_SDA_GSEN_Pin, GPIO_PIN_SET)

#define SCL_GPIO_OUT_LOW() HAL_GPIO_WritePin(PB10_I2C2_SCL_GSEN_GPIO_Port, PB10_I2C2_SCL_GSEN_Pin, GPIO_PIN_RESET)
#define SCL_GPIO_OUT_HIGH() HAL_GPIO_WritePin(PB10_I2C2_SCL_GSEN_GPIO_Port, PB10_I2C2_SCL_GSEN_Pin, GPIO_PIN_SET)

void set_sda_pin_output(void);
void set_sda_pin_input(void);
bool read_sda_pin_level(void);
void software_i2c_quarter_period(void);
void scl_out_low(void);
void scl_out_high(void);
void software_i2c_init(void);
bool software_i2c_read_ack(void);
void software_i2c_send_ack_nack(bool ack_nack);
void software_i2c_send_byte(uint8_t data);
uint8_t software_i2c_read_byte(void);


void set_sda_pin_output(void)
{
	GPIO_InitTypeDef EXTIICGPIO;
	EXTIIC_SET_SDA_OUT();
}
void set_sda_pin_input(void)
{
	GPIO_InitTypeDef EXTIICGPIO;
	EXTIIC_SET_SDA_IN();
}

bool read_sda_pin_level(void)
{
	return (bool)HAL_GPIO_ReadPin(PB11_I2C2_SDA_GSEN_GPIO_Port, PB11_I2C2_SDA_GSEN_Pin);
}

void software_i2c_quarter_period(void)
{
	software_delay_us(1);
}

void scl_out_low(void)
{
	software_i2c_quarter_period();
	SCL_GPIO_OUT_LOW();
	software_i2c_quarter_period();	
}

void scl_out_high(void)
{
	software_i2c_quarter_period();
	SCL_GPIO_OUT_HIGH();
	software_i2c_quarter_period();	
}



void software_i2c_init(void)
{
	ExtensionI2cInit();
}

bool software_i2c_read_ack(void)
{
	uint8_t error_times = 0;
	
	scl_out_low();

	set_sda_pin_input();

	scl_out_high();

	while(read_sda_pin_level()==NACK)
	{
		if(error_times >= 4)
		{//max wait time = 4*period/2
			scl_out_low();

			return (bool)NACK;
		}
		error_times++;
		software_i2c_quarter_period();
	}
	scl_out_low();
	
	return (bool)ACK;	
}

void software_i2c_send_ack_nack(bool ack_nack)
{
	scl_out_low();

	set_sda_pin_output();
	
	if(NACK==ack_nack)
	{
		SDA_GPIO_OUT_HIGH();//SDA out high
	}
	else
	{
		SDA_GPIO_OUT_LOW();//SDA out low
	}
	
	scl_out_high();
	scl_out_low();
}

void software_i2c_send_byte(uint8_t data)
{
	uint8_t byte_bit_count = 0;
	set_sda_pin_output();
	
	for(byte_bit_count = 8; byte_bit_count > 0; byte_bit_count--)
	{
		scl_out_low();
		
		if((data >> (byte_bit_count - 1)) & 0x01)
		{
			SDA_GPIO_OUT_HIGH();
		}
		else
		{
			SDA_GPIO_OUT_LOW();
		}
		
		scl_out_high();
	}	
}

uint8_t software_i2c_read_byte(void)
{
	uint8_t byte_bit_count = 0;
	uint8_t data = 0;

	set_sda_pin_input();
	
	for(byte_bit_count = 0;byte_bit_count < 8;byte_bit_count++)
	{
		scl_out_low();
		scl_out_high();
			
		data = data << 1;
		
		if(read_sda_pin_level())
		{
			data |= 0x01;
		}
	}
	
	scl_out_low();	
	
	return data;

}

void i2c_enable(void)
{
	software_i2c_init();
}


void i2c_disable(void)
{
	software_i2c_init();
}

//SDA  --------_______
//SCL  ---__------_____
uint8_t i2c_send_start(void)
{
	scl_out_low();//SCL out low

	set_sda_pin_output();
	SDA_GPIO_OUT_HIGH();//SDA out high

	scl_out_high();	//SCL out high
	
	SDA_GPIO_OUT_LOW();	//SDA out low

	scl_out_low();//SCL out low
	
	return I2C_FUNCTION_RETCODE_SUCCESS;
}

//SDA_____------
//SCL___--------
uint8_t i2c_send_stop(void)
{
	scl_out_low();//SCL out low

	set_sda_pin_output();
	SDA_GPIO_OUT_LOW();//SDA out low

	scl_out_high();	//SCL out high

	SDA_GPIO_OUT_HIGH();//SDA out high
	
	return I2C_FUNCTION_RETCODE_SUCCESS;
}

uint8_t i2c_send_bytes(uint8_t count, uint8_t *data)
{
	uint8_t ack_nack;
	uint8_t temp;

	for(temp = 0; temp < count; temp++)
	{
		software_i2c_send_byte(*data++);
		ack_nack = software_i2c_read_ack();
		if(NACK == ack_nack)
		{
			return I2C_FUNCTION_RETCODE_NACK;
		}
	}

	return I2C_FUNCTION_RETCODE_SUCCESS;
}

uint8_t i2c_receive_byte(uint8_t *data)
{
	//uint8_t timeout_counter = I2C_BYTE_TIMEOUT;

	*data = software_i2c_read_byte();
	software_i2c_send_ack_nack((bool)ACK);

	return I2C_FUNCTION_RETCODE_SUCCESS;
}

uint8_t i2c_receive_bytes(uint8_t count, uint8_t *data)
{
	uint8_t temp;
	//uint8_t timeout_counter;

	// Acknowledge all bytes except the last one.
	for(temp = 0;temp < count -1;temp++)
	{
		*data++ = software_i2c_read_byte();
		software_i2c_send_ack_nack((bool)ACK);
	}

	*data = software_i2c_read_byte();
	software_i2c_send_ack_nack((bool)NACK);

	return i2c_send_stop();
}
