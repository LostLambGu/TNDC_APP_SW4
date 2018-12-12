#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32l4xx_hal.h"

#define MCU_FLASH_START_ADDR (FLASH_BASE)
#define JUMP_TO_BOOTLOADER_ADDRESS (MCU_FLASH_START_ADDR)
#define JUMP_TO_APPLICATION_ADDRESS (0x08008000)

#define IAP_BOOT_FLASH (uint32_t)(0xAA) // 170
#define IAP_BOOT_UART (uint32_t)(0xBB) // 187
#define IAP_BOOT_CHMODEM (uint32_t)(0x5C) // 92
#define IAP_APP_FLASH (uint32_t)(0xCC)  // 204
#define IAP_APP_UART (uint32_t)(0xDD)  // 221
#define IAP_APP_CHMODEM (uint32_t)(0xEE)  // 238
#define IAP_RESET (uint32_t)(0xFF)     // 255
#define IAP_FLAG (uint32_t)(JUMP_TO_APPLICATION_ADDRESS - 0x400)

#define ADDR_FLASH_PAGE_15 (15 * FLASH_PAGE_SIZE + MCU_FLASH_START_ADDR)
#define ADDR_FLASH_PAGE_16 (16 * FLASH_PAGE_SIZE + MCU_FLASH_START_ADDR)
#define FLASH_USER_END_ADDR (256 * FLASH_PAGE_SIZE + MCU_FLASH_START_ADDR)

/* Get the number of Sector from where the user program will be loaded */
#define FLASH_PAGE_NUMBER (uint32_t)((JUMP_TO_APPLICATION_ADDRESS - JUMP_TO_BOOTLOADER_ADDRESS) >> 12)

/* Compute the mask to test if the Flash memory, where the user program will be
   loaded, is write protected */
#define FLASH_PROTECTED_PAGES ((uint32_t) ~((1 << FLASH_PAGE_NUMBER) - 1))

#define USER_FLASH_END_ADDRESS FLASH_USER_END_ADDR

/* define the user application size */
#define USER_FLASH_SIZE (USER_FLASH_END_ADDRESS - JUMP_TO_APPLICATION_ADDRESS + 1)

#define FIRMWARE_VERSION_LEN_MAX (32)
#define FIRMWARE_CRC16_CAL_BUF_SIZE (512)
typedef struct FirmwareSizeCrc16
{
	uint32_t appsize;
	uint32_t crc16;
} FirmwareSizeCrc16TypeDef;
typedef struct FirmwareInfo
{
	uint32_t IAPFlag;
	uint32_t FirmwareSize;
	char Version[FIRMWARE_VERSION_LEN_MAX];
	FirmwareSizeCrc16TypeDef SizeCrc16;
} FirmwareInfoTypeDef;
#define MODEM_VERSION_HEAD "VER_" // Filename format: VER_00.00.00.000
#define APP_DEFAULT_VERSION "00.00.00.000"

#define FILE_NAME_LENGTH (32)

extern void SerialDownload(uint8_t *FileName);
extern uint8_t ymodem_init(void);
extern uint8_t ymodem_deinit(void);
extern void ymodem_enter(void);
extern void ymodem_exit(void);
extern uint8_t ymodem_openfile(char * filename);
extern uint32_t ymodem_write(const char * wbuf, uint16_t wlen);
extern uint16_t ymodem_close(void);
extern int32_t Receive_Byte(uint8_t *c, uint32_t timeout);
extern uint32_t Send_Byte(uint8_t c);

extern uint8_t FirmwareUpdateFlagSet(uint32_t flag);
extern uint8_t InitEmmc(void);
extern uint8_t FirmwareAreaErase(uint32_t address, FirmwareInfoTypeDef *pFirmwareInfo);
extern uint8_t FirmwareInfoSet(uint32_t address, FirmwareInfoTypeDef *pFirmwareInfo);
extern uint8_t FirmwareSizeCrc16Read(uint32_t address, FirmwareSizeCrc16TypeDef *pSizeCrc16);
extern uint8_t FirmwareCrc16Cal(uint32_t address, uint32_t size, uint16_t *pCrc16);
extern uint8_t FirmwareReceiveInit(FirmwareInfoTypeDef *pInfo, const char *name, uint32_t firmAddr);
extern uint8_t FirmwareVerifyAndSetUpdateFlag(FirmwareInfoTypeDef *pInfo);

#endif
