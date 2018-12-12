
#include "string.h"
#include "stm32l4xx_hal.h"

#include "spi.h"
#include "i2c.h"
#include "uart_api.h"

#include "BLEDriver.h"


#include "SdioEmmcDrive.h"

// #define SPI_BLE_TEST
// #define SDMMC_DRIVER_TEST
// #define ATSHA204_DRIVER_TEST
// #define FATFS_MKFATFS_TEST
// #define EMMC_DMA_TEST
// #define I2C3_API_TEST
// #define FIFO_API_TEST
// #define BLE_SPI_TRANSFER_TEST
// #define SPI_WIFI_TEST
// #define WIFI_VALUE_INSERT_TEST

#ifdef BLE_SPI_TRANSFER_TEST
#include "BLEDriver.h"
#include "BLEProcess.h"
#include "fifo.h"
extern circular_fifo_t event_fifo, command_fifo;
extern void BLETransportSendHeader(uint16_t data_length, uint8_t *header);
extern void BLETransportSendData(uint8_t *data, uint16_t data_length);
extern circular_fifo_t bleCmdFifo, bleEvtFifo;
void BLESpiTransferTest(void)
{
    BLEDriverInit();
    HAL_Delay(10);
    uint8_t Rxbuf[8] = {0}, i = 0;
    uint16_t size = 0;
    uint8_t buffer[64] = {0};
    uint32_t sysTickRec = 0;

    // BLEPutDataInFifo(32, "12345678123456781234567812345678");
    // BLEPutDataInFifo(32, "12345678123456781234567812345678");
    // BLEPutDataInFifo(32, "12345678123456781234567812345678");
    // BLEPutDataInFifo(32, "12345678123456781234567812345678");
    // BLEPutDataInFifo(32, "12345678123456781234567812345678");
    // BLEPutDataInFifo(32, "12345678123456781234567812345678");
    // BLEPutDataInFifo(32, "12345678123456781234567812345678");

    while (BLE_SPI_CS_READ() == 0)
    {
        HAL_Delay(5);
    }

    // BLE_SPI_IRQ_PIN_SET();
    // HAL_Delay(50);
    // BLE_SPI_IRQ_PIN_RESET();

    // for (i = 0; i < 6; i++)
    // {
    //     if (HAL_SPI_TransmitReceive_DMA(&hspi2, "87654321", Rxbuf, i + 1))
    //     {
    //         DebugLog("HAL_SPI_TransmitReceive_DMA Err");
    //     }
    //     sysTickRec = HAL_GetTick();
    //     while (BLE_SPI_CS_READ() != 0)
    //         ;
    //     while (BLE_SPI_CS_READ() == 0)
    //         ;
    //     sysTickRec = HAL_GetTick() - sysTickRec;
    //     DebugLog("==>> Wait for %d ms", sysTickRec);
    //     HAL_SPI_DMAStop(&hspi2);
    // }

    SPI_STATE_TRANSACTION(SPI_PROT_CONFIGURED_STATE);
    
    // HAL_Delay(20000);

    while (1)
    {
        // HAL_Delay(20);
        // DebugLog("tail :%d, head: %d -->>1", bleCmdFifo.tail, bleCmdFifo.head);
        // BLEPutDataInFifo(32, "12345678123456781234567812345678");
        BLEPutDataInFifo(8, "12345678");
        BLETransportTick();
        // DebugLog("tail :%d, head: %d -->>2", bleCmdFifo.tail, bleCmdFifo.head);
        // DebugLog("tail :%d, head: %d -->>3", bleCmdFifo.tail, bleCmdFifo.head);
        fifo_discard_var_len_item(&bleCmdFifo);
        fifo_discard_var_len_item(&bleEvtFifo);
        // DebugLog("tail :%d, head: %d 3", bleCmdFifo.tail, bleCmdFifo.head);
        HAL_Delay(2000);
        // if (fifo_get_var_len_item(&event_fifo, &size, buffer) == 0)
        // {
        //     // DEBUG_NOTES(PARSE_EVENT_PEND);

        //     SPI_STATE_TRANSACTION(SPI_PROT_CONFIGURED_EVENT_PEND_STATE);

        //     if (SPI_STATE_CHECK(SPI_PROT_CONFIGURED_EVENT_PEND_STATE) && 
        //         BLE_SPI_CS_READ() == GPIO_PIN_RESET)
        //     {
        //         SPI_STATE_TRANSACTION(SPI_PROT_WAITING_HEADER_STATE);
        //     }

        //     // DEBUG_NOTES("BLETransportSendHeader Size: (%d)", size);
        //     BLETransportSendHeader(size);

        //     BLE_SPI_IRQ_PIN_SET(); /* Issue the SPI communication request */
        //     while (BLE_SPI_CS_READ() != 0)
        //     ;
        //     BLE_SPI_IRQ_PIN_RESET();
        //     while (BLE_SPI_CS_READ() == 0)
        //     ;
        //     HAL_SPI_DMAStop(&hspi2);
        // }

        // BLETransportSendData(buffer, size);

        // while (BLE_SPI_CS_READ() != 0)
        //     ;
        // while (BLE_SPI_CS_READ() == 0)
        //     ;
        // HAL_SPI_DMAStop(&hspi2);
    }
}

// void BLESpiTransferTest(void)
// {
//     BLEDriverInit();
//     HAL_Delay(10);
//     uint8_t Rxbuf[8] = {0}, i = 0;
//     uint16_t size = 0;
//     uint8_t buffer[64] = {0};

//     BLEPutDataInFifo(8, "12345678");

//     while (BLE_SPI_CS_READ() == 0)
//     {
//         HAL_Delay(5);
//     }

//     BLE_SPI_IRQ_PIN_SET();
//     HAL_Delay(50);
//     BLE_SPI_IRQ_PIN_RESET();

//     for (i = 0; i < 6; i++)
//     {
//         if (HAL_SPI_TransmitReceive_DMA(&hspi2, "87654321", Rxbuf, i + 1))
//         {
//             DebugLog("HAL_SPI_TransmitReceive_DMA Err");
//         }
//         while (BLE_SPI_CS_READ() != 0)
//             ;
//         while (BLE_SPI_CS_READ() == 0)
//             ;
//         HAL_SPI_DMAStop(&hspi2);
//     }

//     while (1)
//     {
//         // HAL_Delay(20);
//         // BLETransportTick();
//         // HAL_Delay(1000);
//         if (fifo_get_var_len_item(&event_fifo, &size, buffer) == 0)
//         {
//             // DEBUG_NOTES(PARSE_EVENT_PEND);

//             SPI_STATE_TRANSACTION(SPI_PROT_CONFIGURED_EVENT_PEND_STATE);

//             if (SPI_STATE_CHECK(SPI_PROT_CONFIGURED_EVENT_PEND_STATE) && 
//                 BLE_SPI_CS_READ() == GPIO_PIN_RESET)
//             {
//                 SPI_STATE_TRANSACTION(SPI_PROT_WAITING_HEADER_STATE);
//             }

//             // DEBUG_NOTES("BLETransportSendHeader Size: (%d)", size);
//             BLETransportSendHeader(size);

//             BLE_SPI_IRQ_PIN_SET(); /* Issue the SPI communication request */
//             while (BLE_SPI_CS_READ() != 0)
//             ;
//             BLE_SPI_IRQ_PIN_RESET();
//             while (BLE_SPI_CS_READ() == 0)
//             ;
//             HAL_SPI_DMAStop(&hspi2);
//         }

//         BLETransportSendData(buffer, size);

//         while (BLE_SPI_CS_READ() != 0)
//             ;
//         while (BLE_SPI_CS_READ() == 0)
//             ;
//         HAL_SPI_DMAStop(&hspi2);
//     }
// }

//   BLEDriverInit();
//   while (1)
//   {
//     HAL_GPIO_TogglePin(GPIOB, PB0_BLE_IRQ_Pin);
//     HAL_GPIO_TogglePin(GPIOA, PA7_WAKE_BLE_Pin);
//     HAL_Delay(1000);
//   }

#endif /* BLE_SPI_TRANSFER_TEST */

#ifdef FIFO_API_TEST
#include "fifo.h"

#define TEST_FIFO_MAX_SIZE_BYTE (64)
#define FIFO_SIZE_ALIGNMENT (2)
uint8_t fifoBuf[TEST_FIFO_MAX_SIZE_BYTE] = {0};
uint8_t testBuf[TEST_FIFO_MAX_SIZE_BYTE];
circular_fifo_t fifoTest;
void FifoApiTest(void)
{
    uint16_t i = 0, j = 0;
    uint16_t getSize = 0;

    fifo_init(&fifoTest, TEST_FIFO_MAX_SIZE_BYTE, fifoBuf, FIFO_SIZE_ALIGNMENT);

TEST_FIFO_AGAIN:
    for (i = 0; i < (32); i++)
    {
        for (j = 1; j < ((i + 2) % 11); j++)
        {
            testBuf[j - 1] = j + '0';
        }
        j--;

        if((fifo_left_size(&fifoTest) > j) && j > 0)
        {
            if (fifo_put_var_len_item(&fifoTest, j, testBuf))
            {
                DebugLog("fifo_put_var_len_item err!");
            }
            else
            {
                DebugLog("line: %d ==>> tail: %d head: %d max_size: %d buffer: %x alignment: %d",
                         i, fifoTest.tail, fifoTest.head, fifoTest.max_size, fifoTest.buffer, fifoTest.alignment);
                DebugLog("leftsize: %d", fifo_left_size(&fifoTest) + 2);
            }
        }
        else
        {
            DebugLog("no enough room line: %d", i);
        }
    }

    while (fifo_size(&fifoTest) > 32)
    {
        memset(testBuf, 0, sizeof(testBuf));

        if (fifo_get_var_len_item(&fifoTest, &getSize, testBuf))
        {
            DebugLog("fifo_get_var_len_item err!");
        }
        else
        {
            // DebugLog("line: %d ==>> tail: %d head: %d max_size: %d buffer: %x alignment: %d",
            //          i, fifoTest.tail, fifoTest.head, fifoTest.max_size, fifoTest.buffer, fifoTest.alignment);
            DebugLog("leftsize: %d getSize: %d", fifo_left_size(&fifoTest) + 2, getSize);
            DebugLog("testBuf:(%s)", testBuf);
        }
        
        if (fifo_discard_var_len_item(&fifoTest))
        {
            DebugLog("fifo_discard_var_len_item err!");
        }
        else
        {
            // DebugLog("line: %d ==>> tail: %d head: %d max_size: %d buffer: %x alignment: %d",
            //          i, fifoTest.tail, fifoTest.head, fifoTest.max_size, fifoTest.buffer, fifoTest.alignment);
            // DebugLog("leftsize: %d", fifo_left_size(&fifoTest) + 2);
        }
    }

    goto TEST_FIFO_AGAIN;

    while (1);
}
#endif /* FIFO_API_TEST */

#ifdef I2C3_API_TEST

#include "prot.h"

void I2c3ApiTest(void)
{
    MX_I2C3_Init();

    while (1)
    {
        at_I2cWrite(0x34, 0x08, "hello", 5, 0);
        HAL_Delay(10);
    }
    
}
#endif /* I2C3_API_TEST */

#ifdef FATFS_MKFATFS_TEST
// #define _FS_REENTRANT    set to 0, this test run in no os
// #define EMMC_SUPPORT_OS 0
#include "fatfs.h"

#define MAKFATFS_BUF_SIZE (4096)
uint8_t pBUF[MAKFATFS_BUF_SIZE];
extern uint8_t FileSystemMountFlag;
extern Disk_drvTypeDef disk;
void FatFsMakeFatTest(void)
{
    FRESULT res;

    if (0 == EmmcInit())
    {
        while (EMMC_CARD_TRANSFER != EmmcGetState())
            ;
        DebugLog("Emmc Init OK");
    }
    else
    {
        DebugLog("Emmc Init Fail");
    }

    memset(pBUF, 0, MAKFATFS_BUF_SIZE);

	DebugLog("Factory make FatFS mode enter!");

	memset(&disk, 0, sizeof(disk));

	MX_FATFS_Init();

	res = f_mount(&EMMCFatFS, EMMCPath, 1);
	if (FR_OK == res)
	{
		DebugLog("Factory make FatFS mount success!");
	}
	else
	{
		DebugLog("Factory make FatFS mount fail: %d", res);
	}

	DebugLog("Factory make FatFS start, this may take a few minutues!");

	res = f_mkfs(_TEXT("0:"), FM_EXFAT, 0, pBUF, MAKFATFS_BUF_SIZE);
	if (res == FR_OK)
	{
		DebugLog("Factory make FatFS success!");
	}
	else
	{
		DebugLog("Factory make FatFS failed: %d", res);
	}

	f_mount(NULL, EMMCPath, 0);
	FileSystemMountFlag = 0;
	FATFS_UnLinkDriver(EMMCPath);
}
#endif /* FATFS_MKFATFS_TEST */


#ifdef ATSHA204_DRIVER_TEST
#include "stdlib.h"

#include "software_timer_utilities.h" 
#include "atsha204_device_configuration.h" 
#include "sha204_physical.h"  
#include "atsha204_defines.h"
#include "sha204_comm.h"
#include "sha204_helper.h"
#include "sha204_lib_return_codes.h"

#include "atsha204_read_sn.h"
#include "atsha204_mac.h"

#include "atsha204_enc_read.h"
#include "atsha204_enc_write.h"

void ATSHA204_TEST(void)
{
	uint8_t sha204_lib_return = SHA204_SUCCESS;
	uint8_t serial_number[9] = {0};

	uint8_t secret_key_id = KEY_ID_0;
	uint8_t secret_key[32] = {0x11,0x77,0x16,0x20,0x82,0xde,0xad,0x8c,0xe9,0x14,0x21,0x87,0xf5,0x94,0x6e,0xcd,0x0c,0x75,0x5c,0xd5,0x57,0x3c,0x3a,0x40,0x9a,0xdf,0xdb,0x83,0x55,0x1b,0xd0,0xd1};
	uint8_t num_in[32] = {0};
	uint8_t challenge[32] = {0};
	uint8_t wakeup_response_buffer[4] = {0};

	uint8_t random_number = 0;	

	sha204p_init();
	
	DebugLog("\r\n");		
	DebugLog("----- Test ATSHA204(I2C mode) function base on STM32F103C8T6 -----!\r\n");
	DebugLog("\r\n");	
	
	//Wake-up ATASHA204
	software_delay_ms(10);	
	DebugLog("-Test ATSHA204 function:Wake-up ATASHA204--BEGIN!\r\n");
	sha204_lib_return = sha204c_wakeup(wakeup_response_buffer);
	if(SHA204_SUCCESS != sha204_lib_return)
	{
		DebugLog("-Test ATSHA204 function:Wake-up ATASHA204--FALIED!\r\n");	
		while(1)
		{
			;
		}			
	}
	else
	{
		DebugLog("-Test ATSHA204 function:Wake-up ATASHA204--SUCESS!\r\n");	
		printf_array("buff", wakeup_response_buffer, 4);	
	}
	sha204p_sleep();	
	
	DebugLog("\r\n");	
	
	//Read ATSHA204 serial number
	software_delay_ms(10);	
	DebugLog("--Test ATSHA204 function:Read ATSHA204 serial number!\r\n");	
	
	sha204_lib_return = atsha204_read_sn(serial_number);
	if(SHA204_SUCCESS!=sha204_lib_return)
	{
		DebugLog("----Read ATSHA204 serial number: FAILED!!\r\n");	
		while(1)
		{
			;
		}				
	}
	DebugLog("----Read ATSHA204 serial number: SUCCESS!!\r\n");	
	printf_array("SN", serial_number, sizeof(serial_number));	

	DebugLog("\r\n");

	//Authentication
	software_delay_ms(10);	
	DebugLog("--Test ATSHA204 function:Authentication!\r\n");
	
	random_number = rand() % 256;
    DebugLog("random_number %d\r\n", random_number);
	memset(num_in,(uint8_t)(random_number),32);	
	
	// random_number = rand();
	memset(challenge,(uint8_t)(random_number),32);

    struct sha204h_nonce_in_out NonceParam;
    struct sha204h_temp_key tempKey;

    memset(&tempKey, 0, sizeof(tempKey));

    NonceParam.mode = NONCE_MODE_NO_SEED_UPDATE;
    NonceParam.num_in = num_in;
    NonceParam.rand_out = challenge;
    NonceParam.temp_key = &tempKey;

    sha204h_nonce(&NonceParam);

    printf_array("NONCE", tempKey.value, SHA204_KEY_SIZE);
	
	// sha204_lib_return = atsha204_mac(secret_key_id, secret_key, num_in, challenge);
	// if(SHA204_SUCCESS!=sha204_lib_return)
	// {
	// 	DebugLog("----Authentication: FAILED!!\r\n");
	// 	while(1)
	// 	{
	// 		;
	// 	}		
	// }
	// DebugLog("----Authentication: SUCCESS!!\r\n");

	DebugLog("\r\n");
}
#endif /* ATSHA204_DRIVER_TEST */

#ifdef SPI_BLE_TEST
void SpiTest(void)
{
    uint8_t *pTxData = "St dic";
    uint8_t buf[32] = {0};
    uint8_t *pRxData = buf;
    HAL_Delay(100);
    PrintfBeforeRTOS("\r\nSpi Start!\r\n");
    while (1)
    {
        HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, sizeof("St dic"), 500);

        if (HAL_GPIO_ReadPin(PB12_BLE_CS_GPIO_Port, PB12_BLE_CS_Pin) == GPIO_PIN_SET)
        {
            PrintfBeforeRTOS("\r\nbleCsPin == 1\r\n");
            while (bleCsPin == 1)
            {
            }
            PrintfBeforeRTOS("\r\nbleCsPin == 0\r\n");
        }
        else
        {
            PrintfBeforeRTOS("\r\nbleCsPin == 0\r\n");
        }

        while (bleCsPin == 0)
        {
        }

        PrintfBeforeRTOS("\r\nbleCsPin == 1\r\n");
        PrintfBeforeRTOS("\r\nRxData: %s\r\n", pRxData);
    }
}
#endif /* SPI_BLE_TEST */

#ifdef SPI_WIFI_TEST
void WifiSpiTest(void)
{
    #define WIFI_SPI_TEST_STR "00aaaaaaaabbbbbbbbaaaaaaaabbbbbbbbaaaaaaaabbbbbbbb"
    uint8_t *pTxData = WIFI_SPI_TEST_STR;
    uint8_t buf[40] = {0};
    uint8_t *pRxData = buf;
    uint8_t i = 0;
    uint32_t systicRec = 0;
    HAL_Delay(100);
    PrintfBeforeRTOS("\r\nWifi Spi Start!\r\n");
    while (1)
    {
        PrintfBeforeRTOS("\r\nWifi CS wait high1\r\n");
        while (HAL_GPIO_ReadPin(GPIOA, PA4_SPI3_NSS_Pin) == GPIO_PIN_RESET)
        {
        }

        systicRec = HAL_GetTick();
        HAL_SPI_TransmitReceive_DMA(&hspi3, pTxData, pRxData, 34);

        PrintfBeforeRTOS("\r\nWifi CS wait low\r\n");
        while (HAL_GPIO_ReadPin(GPIOA, PA4_SPI3_NSS_Pin) == GPIO_PIN_SET)
        {
        }

        PrintfBeforeRTOS("\r\nWifi CS wait high2\r\n");
        while (HAL_GPIO_ReadPin(GPIOA, PA4_SPI3_NSS_Pin) == GPIO_PIN_RESET)
        {
            // if ((HAL_GetTick() - systicRec) > 1000)
            // {
            //     PrintfBeforeRTOS("\r\nWifi CS Low timeout\r\n");
            //     break;
            // }
        }

        HAL_SPI_DMAStop(&hspi3);

        PrintfBeforeRTOS("\r\nRxData:\r\n");

        for (i = 0; i < 34; i++)
        {
            PrintfBeforeRTOS("%x ", pRxData[i]);
        }
        memset(pRxData, 0, 40);
    }
}
#endif /* SPI_WIFI_TEST */

#ifdef SDMMC_DRIVER_TEST
uint8_t WBuf[1024] = {1,2,3,4,5,6,7,8,9,0};
uint8_t RBuf[1024] = {0};

void SdioEmmcTest(void)
{
    EmmcError err = EMMC_OK;
    uint32_t error = HAL_OK;
    uint16_t i = 0;
    // EmmcChipEnControl(DISABLE);
    // EmmcChipEnControl(ENABLE);
    // while (1);
    EmmcInit();
    // HAL_Delay(10);
    while (EMMC_CARD_TRANSFER != EmmcGetState());
    for (i = 0; i < 1024; i++)
    {
        WBuf[i] = i;
    }
    extern uint32_t EMMC_WriteBlocks(uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
    extern uint32_t EMMC_ReadBlocks(uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
    error = EMMC_WriteBlocks(WBuf, 0, 1, 5000);
    // while (EMMC_CARD_TRANSFER != EmmcGetState());
    error = EMMC_ReadBlocks(RBuf, 0, 2, 5000);
    // while (EMMC_CARD_TRANSFER != EmmcGetState());
    for (i = 0; i < 1024; i++)
    {
        WBuf[i] = i + 2;
    }
    error = EMMC_WriteBlocks(WBuf, 1, 2, 5000);
    // while (EMMC_CARD_TRANSFER != EmmcGetState());
    error = EMMC_ReadBlocks(RBuf, 1, 2, 5000);
    // err = EmmcReadBlocksDMA(RBuf, 0, 512, 1);
    while (1);
}
#endif /* SDMMC_DRIVER_TEST */

#ifdef EMMC_DMA_TEST
uint8_t WBuf[1024] = {1,2,3,4,5,6,7,8,9,0};
uint8_t RBuf[1024] = {0};
void EmmcDmaTest(void)
{
    uint32_t error = HAL_OK;
    uint16_t i = 0;

    EmmcInit();
    while (EMMC_CARD_TRANSFER != EmmcGetState());
    for (i = 0; i < 1024; i++)
    {
        WBuf[i] = i + 3;
    }
    
    error = EmmcReadBlocksDMA(RBuf, 0, 1, 5000);
    error = EmmcWriteBlocksDMA(WBuf, 0, 1, 5000);
    error = EmmcReadBlocksDMA(RBuf, 0, 1, 5000);
    for (i = 0; i < 1024; i++)
    {
        WBuf[i] = i + 2;
    }
    error = EMMC_WriteBlocks(WBuf, 1, 2, 5000);
    error = EMMC_ReadBlocks(RBuf, 63, 2, 5000);
    error = EmmcReadBlocksDMA(RBuf, 0, 1, 5000);
    while (1);
}

#endif /* EMMC_DMA_TEST */

#ifdef WIFI_VALUE_INSERT_TEST

extern uint16_t WifiGetLenAfterInsert(uint16_t len);
extern uint16_t WifiGetLenAfterDelInsert(uint16_t len);
extern void WifiInsertValue(uint8_t *data, uint16_t len);
extern void WifiDelInsertValue(uint8_t *data, uint16_t len);
uint8_t wifiInsertBuf[256];
void WifiValueInsertTest(void)
{
    #define WIFI_VALUE_INSERT_TEST_LEN (128 + 3)
    
    uint16_t i = 0;
    uint16_t len = WIFI_VALUE_INSERT_TEST_LEN;

    for (i = 0; i < len; i++)
    {
        wifiInsertBuf[i] = i % 8 + 1;
    }

    PrintfBeforeRTOS("\r\nBefore Insert len (%d):\r\n", len);
    for (i = 0; i < len; i++)
    {
        if (((i % 32) == 0) && (i != 0))
        {
            PrintfBeforeRTOS("\r\n");
        }

        PrintfBeforeRTOS("%d ", wifiInsertBuf[i]);
    }

    WifiInsertValue(wifiInsertBuf, len);
    PrintfBeforeRTOS("\r\nLen After Insert: %d\r\n", WifiGetLenAfterInsert(len));
    len = WifiGetLenAfterInsert(len);
    PrintfBeforeRTOS("\r\nAfter Insert:\r\n");
    for (i = 0; i < len; i++)
    {
        if (((i % 34) == 0) && (i != 0))
        {
            PrintfBeforeRTOS("\r\n");
        }

        PrintfBeforeRTOS("%d ", wifiInsertBuf[i]);
    }

    WifiDelInsertValue(wifiInsertBuf, len);
    PrintfBeforeRTOS("\r\nLen After Del Insert: %d\r\n", WifiGetLenAfterDelInsert(len));
    len = WifiGetLenAfterDelInsert(len);
    PrintfBeforeRTOS("\r\nAfter Del Insert:\r\n");
    for (i = 0; i < len; i++)
    {
        if (((i % 32) == 0) && (i != 0))
        {
            PrintfBeforeRTOS("\r\n");
        }

        PrintfBeforeRTOS("%d ", wifiInsertBuf[i]);
    }
}

#endif /* WIFI_VALUE_INSERT_TEST */

void DriverTest(void)
{
    #ifdef FATFS_MKFATFS_TEST
    FatFsMakeFatTest();
    #endif /* FATFS_MKFATFS_TEST */

    #ifdef ATSHA204_DRIVER_TEST
    ATSHA204_TEST();
    #endif /* ATSHA204_DRIVER_TEST */

    #if SPI_BLE_TEST
    SpiTest();
    #endif /* SPI_BLE_TEST */

    #ifdef SDMMC_DRIVER_TEST
    SdioEmmcTest();
    #endif /* SDMMC_DRIVER_TEST */

    #ifdef EMMC_DMA_TEST
    EmmcDmaTest();
    #endif /* SDMMC_DRIVER_TEST */

    #ifdef I2C3_API_TEST
    I2c3ApiTest();
    #endif /* I2C3_API_TEST */

    #ifdef FIFO_API_TEST
    FifoApiTest();
    #endif /* FIFO_API_TEST */

    #ifdef BLE_SPI_TRANSFER_TEST
    BLESpiTransferTest();
    #endif /* BLE_SPI_TRANSFER_TEST */

    #ifdef SPI_WIFI_TEST
    WifiSpiTest();
    #endif /* SPI_WIFI_TEST */

    #ifdef WIFI_VALUE_INSERT_TEST
    WifiValueInsertTest();
    #endif /* WIFI_VALUE_INSERT_TEST */
}

// #define FATFS_OPERATION_TEST
#define EXCLUSIVE_DEBUG_LOCK_TEST

#ifdef FATFS_OPERATION_TEST
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "prot.h"
#include "file_operation.h"
#define FILE_NUMBER (5)
static uint32_t GetSystickIncrement(uint32_t oldSystick)
{
	uint32_t tmp = HAL_GetTick();
	return (tmp >= oldSystick) ? (tmp - oldSystick) : (0xffffffff - tmp + oldSystick);
}
void FileOperationTestTask(void const *argument)
{
    OemValFsiHandleT File[FILE_NUMBER];
	OemValFsiFileOpenModeT Mode = OEM_FSI_FILE_OPEN_SHARE;
	char Buf[1024] = {0};
	uint32_t startSystick = HAL_GetTick();
	uint32_t increment = 0, i, j, k;
    uint32_t bufSize = sizeof(Buf);

	if (OEM_FSI_SUCCESS == FileOpen(&File[1 - 1], "test1.txt", Mode) 
		&& OEM_FSI_SUCCESS == FileOpen(&File[2 - 1], "test2.txt", Mode)
        && OEM_FSI_SUCCESS == FileOpen(&File[3 - 1], "test3.txt", Mode)
        && OEM_FSI_SUCCESS == FileOpen(&File[4 - 1], "test4.txt", Mode)
        && OEM_FSI_SUCCESS == FileOpen(&File[5 - 1], "test5.txt", Mode))
	{
		increment = GetSystickIncrement(startSystick);
		DebugLog("FileOpen(%d ms) OK", increment);

        for (i = 0; i < 128; i++)
        {
            for (j = 0; j < FILE_NUMBER; j++)
            {
                memset(Buf, 0, sizeof(Buf));
                for (k = 0; k < 1024; k++)
                {
                    Buf[k] = (k + j + i) % 9 + '0';
                }
                startSystick = HAL_GetTick();
                if (OEM_FSI_SUCCESS == FileWrite(Buf, 1, &bufSize, File[j]))
                {
                    increment = GetSystickIncrement(startSystick);
                    DebugLog("File(%d) Write(%d ms) OK:%d", File[j], increment, i);
                }
                else
                {
                    DebugLog("File(%d) Write Fail:%d", File[j], i);
                }
            }
        }
		
        for (j = 0; j < FILE_NUMBER; j++)
        {
            startSystick = HAL_GetTick();
            if (OEM_FSI_SUCCESS == FileClose(File[j]))
            {
                increment = GetSystickIncrement(startSystick);
                DebugLog("File(%d) Close(%d ms) OK", File[j], increment);
            }
            else
            {
                DebugLog("File(%d) Close Err", File[j]);
            }
        }	
	}
	else
	{
		DebugLog("FileOpen Fail");
	}
}
#endif /* FATFS_OPERATION_TEST */

#ifdef EXCLUSIVE_DEBUG_LOCK_TEST
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "sha204_lib_return_codes.h"
#include "atsha204_read_sn.h"
#include "software_timer_utilities.h"
#include "sha204_physical.h"
#include "sha204_comm.h"
#include "lis2dh_driver.h"

#define EXCLUSIVE_DEBUG_LOCK_TEST_TOTAL_LINES 250

uint8_t sha204_lib_return = SHA204_SUCCESS;
uint8_t serial_number[9] = {0};
uint8_t wakeup_response_buffer[4] = {0};

static uint32_t GetSystickIncrement(uint32_t oldSystick)
{
	uint32_t tmp = HAL_GetTick();
	return (tmp >= oldSystick) ? (tmp - oldSystick) : (0xffffffff - tmp + oldSystick);
}

void ExclusiveDebugLockTest1(void const *argument)
{
    uint32_t i = 0;
    uint32_t startSystick = HAL_GetTick();
    uint32_t increment = 0;

    sha204p_init();

	DebugLog("-Test ATSHA204 function:Wake-up ATASHA204--BEGIN!\r\n");
	sha204_lib_return = sha204c_wakeup(wakeup_response_buffer);
	if(SHA204_SUCCESS != sha204_lib_return)
	{
		DebugLog("-Test ATSHA204 function:Wake-up ATASHA204--FALIED!\r\n");	
		while(1)
		{
			;
		}			
	}
	else
	{
		DebugLog("-Test ATSHA204 function:Wake-up ATASHA204--SUCESS!\r\n");	
		printf_array("buff", wakeup_response_buffer, 4);	
	}
	sha204p_sleep();	
	
	DebugLog("\r\n");	
	
	software_delay_ms(10);	

    for (i = 0; i < EXCLUSIVE_DEBUG_LOCK_TEST_TOTAL_LINES; i++)
    {
        memset(serial_number, 0, sizeof(serial_number));
        DebugLog("ExclusiveDebugLockTest==1 Total(%u) Line: %u", EXCLUSIVE_DEBUG_LOCK_TEST_TOTAL_LINES, i);
        sha204_lib_return = atsha204_read_sn(serial_number);
        if (SHA204_SUCCESS != sha204_lib_return)
        {
            DebugLog("----Read ATSHA204 serial number: FAILED!!\r\n");
        }
        // DebugLog("----Read ATSHA204 serial number: SUCCESS!!\r\n");
        printf_array("SN", serial_number, sizeof(serial_number));
        if (i % 10 == 0)
        {
            osDelay(1);
        }
    }

    osDelay(5);
    increment = GetSystickIncrement(startSystick);
    DebugLog("ExclusiveDebugLockTest==1 total time(%u ms)", increment);
    
    osThreadTerminate(NULL);
}

void ExclusiveDebugLockTest2(void const *argument)
{
    uint32_t i = 0;
    uint32_t startSystick = HAL_GetTick();
    uint32_t increment = 0;
    uint8_t ChipID;

    osDelay(100);

    for (i = 0; i < EXCLUSIVE_DEBUG_LOCK_TEST_TOTAL_LINES; i++)
    {
        DebugLog("ExclusiveDebugLockTest==2 Total(%u) Line: %u", EXCLUSIVE_DEBUG_LOCK_TEST_TOTAL_LINES, i);
        /* Check Chip ID */
        ChipID = AccelReadRegister(LIS2DH_MEMS_I2C_ADDRESS, LIS2DH_WHO_AM_I);

        if (ChipID == WHOAMI_LIS2DH_ACC)
        {
            DebugLog("GSENSOR: PASS, 0x%x", ChipID);
        }
        else
        {
            DebugLog("GSENSOR: =========================FAIL==========, 0x%x", ChipID);
        }
        if (i % 10 == 0)
        {
            osDelay(1);
        }
    }

    osDelay(5);
    increment = GetSystickIncrement(startSystick);
    DebugLog("ExclusiveDebugLockTest==2 total time(%u ms)", increment);
    
    osThreadTerminate(NULL);
}

void ExclusiveDebugLockTest(void const *argument)
{
    osThreadDef(ExclusiveLockTest1, ExclusiveDebugLockTest1, osPriorityNormal, 0, 128 * 2);
	osThreadCreate(osThread(ExclusiveLockTest1), NULL);

    osThreadDef(ExclusiveLockTest2, ExclusiveDebugLockTest2, osPriorityNormal, 0, 128 * 2);
	osThreadCreate(osThread(ExclusiveLockTest2), NULL);
}
#endif /* EXCLUSIVE_DEBUG_LOCK_TEST */

void OperationTestTask(void const *argument)
{
    #ifdef FATFS_OPERATION_TEST
    FileOperationTestTask(argument);
    #endif /* FATFS_OPERATION_TEST */

    #ifdef EXCLUSIVE_DEBUG_LOCK_TEST
    ExclusiveDebugLockTest(argument);
    #endif /* EXCLUSIVE_DEBUG_LOCK_TEST */

    osThreadTerminate(NULL);
}
