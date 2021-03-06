﻿/*
 * atsha204_device_configuration.h
 *
 * Created: 2013/7/2 21:50:04
 *  Author: Tony
 */ 


#ifndef ATSHA204_DEVICE_CONFIGURATION_H_
#define ATSHA204_DEVICE_CONFIGURATION_H_

#include "atsha204_defines.h"

/*!
 *	*** DEVICE INFORMATION  ***
 *
 *	The first 16 bytes of the configuration region holds non-modifiable device information.
 */

/*!
 *  Device Operation Modes and Parameters
 * 
 *	Configure the I2C Address, Temperature sensor offset, OTP mode, and Selector Mode bytes
 */	

//													 { I2C_Address,	TempOffset,	   OTPmode,	SelectorMode}; 
uint8_t DEVICE_MODES[WRITE_BUFFER_SIZE_LONG] = {	      0xC9,       0x00,       0x55,         0x00};								


/*!
 *  *** SLOT CONFIGURATION ***
 *
 * Configure the access restrictions governing each slot
 *
 */
//													       { LSB,  MSB,    LSB,  MSB};				
uint8_t SLOT_CONFIG_00_01[WRITE_BUFFER_SIZE_SHORT] = {0x0F, 0x20,   0xAF, 0x80};  /* Access rights for slots 0 and 1   */
uint8_t SLOT_CONFIG_02_03[WRITE_BUFFER_SIZE_SHORT] = {0x8F, 0x80,   0xAF, 0x80};  /* Access rights for slots 2 and 3   */
uint8_t SLOT_CONFIG_04_05[WRITE_BUFFER_SIZE_SHORT] = {0x8F, 0xB1,   0x8F, 0xA2};  /* Access rights for slots 4 and 5   */
uint8_t SLOT_CONFIG_06_07[WRITE_BUFFER_SIZE_SHORT] = {0xC3, 0x44,   0xCF, 0x00};  /* Access rights for slots 6 and 7   */
uint8_t SLOT_CONFIG_08_09[WRITE_BUFFER_SIZE_SHORT] = {0x0F, 0x00,   0xC1, 0x00};  /* Access rights for slots 8 and 9   */
uint8_t SLOT_CONFIG_10_11[WRITE_BUFFER_SIZE_SHORT] = {0x0F, 0x42,   0x8F, 0xB2};  /* Access rights for slots 10 and 11 */
uint8_t SLOT_CONFIG_12_13[WRITE_BUFFER_SIZE_SHORT] = {0x8F, 0x30,   0x8F, 0xA2};  /* Access rights for slots 12 and 13 */
uint8_t SLOT_CONFIG_14_15[WRITE_BUFFER_SIZE_SHORT] = {0x8F, 0x20,   0xF0, 0x80};  /* Access rights for slots 14 and 15 */

/*!
 *	*** Cconfugre USE FLAGS AND UPDATE COUNT bytes ***
 *
 * Use flags and update counts apply restrictions to enable limits on key usage.
 *	- For each 4-byte word:
 *		- Byte 0 is the UseFlag byte for the lower slot
 *		- Byte 1 is the UpdateCount byte for the lower slot
 *		- Byte 2 is the UseFlag byte for the upper slot
 *		- Byte 3 is the UpdateCount byte for the upper slot
 */
uint8_t SLOT_0_1_USE_UPDATE[WRITE_BUFFER_SIZE_SHORT] = {0xFF, 0xFF,  0xFF, 0xFF};
uint8_t SLOT_2_3_USE_UPDATE[WRITE_BUFFER_SIZE_SHORT] = {0xFF, 0xFF,  0x1F, 0xFF};
uint8_t SLOT_4_5_USE_UPDATE[WRITE_BUFFER_SIZE_SHORT] = {0xFF, 0x07,  0xFF, 0x3F};
uint8_t SLOT_6_7_USE_UPDATE[WRITE_BUFFER_SIZE_SHORT] = {0xFF, 0xFF,  0xFF, 0xFF};


/*!
 *	***	LAST KEY USE ***
 *
 *	Control limited use for KeyID 15.  Factory defaults are 0xFF
 *
 */
uint8_t LAST_KEY_USE [LAST_KEY_USE_BYTE_SIZE] = {	0x00, 0x00, 0x00, 0x00,		// Bytes 68 - 71
													0x00, 0x00, 0x00, 0x00,		// Bytes 72 - 75
													0x00, 0x00, 0x00, 0x80,		// Bytes 76 - 79
													0xFF, 0xFF, 0xFF, 0xFF		// Bytes 80 - 83																				
												};

/*!
 *	***	Configure USER EXTRA and SELECTOR bytes ***
 *
 *	- Byte 0 configures UserExtra
 *	- Byte 1 configures Selector
 *	- Bytes 2 and 3 are ignored as they can only be modified via LOCK commands.
 *
 * These bytes are modifiable only through UpdateExtra and Lock commands only
 */

/*!
 *	*** INITIAL OTP CONTENT ***
 *
 *	512 Bits in total
 */
uint8_t OTP[2 * WRITE_BUFFER_SIZE_LONG] = {	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
											};

/*!
 *	*** INITIAL SLOT CONTENT ***
 *
 *	The initial slot content can be keys or data depending on custom security configuration.
 * 
 * The slots in this example are populated with values easy to remember for easy illustration.
 * Atmel strongly advices use of random values preferably from the high quality on-chip RNG 
 * (random number generator) for initial key values.
 *
 */

uint8_t SLOT_00_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	'A', 't', 'm', 'e' , 'l' , ' ', 'W', 'i',
														'd', 'g', 'e', 't', ' ', 'M', 'o', 'd',
														'e', 'l', ' ', '5', '5', '5', '5', 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
													};

uint8_t SLOT_01_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
															0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
															0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
															0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
														};

uint8_t SLOT_02_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
															0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
															0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
															0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02
														};

uint8_t SLOT_03_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
															0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
															0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
															0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03
														};

uint8_t SLOT_04_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
															0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
															0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
															0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04
														};

uint8_t SLOT_05_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
															0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
															0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
															0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05
														};

uint8_t SLOT_06_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
															0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
															0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
															0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06
														};

uint8_t SLOT_07_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
															0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
															0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
															0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07
														};

uint8_t SLOT_08_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
															0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
															0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
															0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08
														};

 uint8_t SLOT_09_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
															0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
															0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
															0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09
														};

uint8_t SLOT_10_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0xA,
															0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0xA,
															0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0xA,
															0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0xA
														};

uint8_t SLOT_11_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
															0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
															0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
															0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B
														};

uint8_t SLOT_12_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
															0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
															0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
															0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C
														};

uint8_t SLOT_13_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D,
															0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D,
															0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D,
															0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D
														};

uint8_t SLOT_14_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E,
															0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E,
															0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E,
															0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E
														};

uint8_t SLOT_15_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
															0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
															0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
															0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
														};

#endif /* ATSHA204_DEVICE_CONFIGURATION_H_ */
