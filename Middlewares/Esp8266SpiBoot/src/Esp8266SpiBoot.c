/* SDIO example, host (uses sdmmc host driver)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "uart_api.h"
#include "eagle_fw.h"
#include "fw_boot.h"
#include "spi_cfg.h"

#define ESP_BOOT_LOG(format, ...) DebugPrintf(1, "\r\n" format, ##__VA_ARGS__)

#define ESP_SLAVE_CMD53_END_ADDR 0x1f800

#define ERR_OK 0
#define ESP_OK 0
#define ESP_FAIL -1

#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_NOT_SUPPORTED 0x106
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_INVALID_RESPONSE 0x108
#define ESP_ERR_INVALID_CRC 0x109

typedef int32_t esp_err_t;

char espspitxbuffer[128 * 7] = {0};
extern char *tx_cmd;

#define CMD_RESP_SIZE 10

#define BLOCK_W_DATA_RESP_SIZE_EACH (20)        //For each data write resp size, in block write
#define BLOCK_W_DATA_RESP_SIZE_FINAL (112 + 40) //For final data write resp size, in block write ,max :112

#define BLOCK_R_DATA_RESP_SIZE_1ST (123 + 40) //For each data read resp size, in block read ,max: 123
#define BLOCK_R_DATA_RESP_SIZE_EACH (20)      //For each data read resp size, in block read

esp_err_t sdspi_write_sync(uint8_t *rawbuf, uint32_t len)
{
    int err_ret = 0;
    int i, j;
    int n;
    uint32_t count = 0;
    int pos;
    int find_w_rsp = 0;
    uint32_t block_size = 512;
    uint32_t addr = 0x20000 - 0x800 - len;

    count = len / block_size + 1;
    ESP_BOOT_LOG("len : %d, count : %d\n", len, count);

    tx_cmd[0] = 0x75;

    if (count <= 0)
    {
        err_ret = -1;
        return err_ret;
    }

    if (addr >= (1 << 17))
    {
        err_ret = -2;
        return err_ret;
    }
    else
    {
        tx_cmd[1] = 0x90 | 0x0C | (addr >> 15);
        tx_cmd[2] = addr >> 7;

        if (count >= 512)
        {
            tx_cmd[3] = (addr << 1 | 0x0);
            tx_cmd[4] = 0x00;
        }
        else
        {
            tx_cmd[3] = (addr << 1 | (count >> 8 & 0x01));
            tx_cmd[4] = count & 0xff;
        }
    }
    tx_cmd[5] = 0x01;

    pos = 5 + 1;
    //Add cmd respon
    memset(tx_cmd + pos, 0xff, CMD_RESP_SIZE);
    pos = pos + CMD_RESP_SIZE;

    for (j = 0; j < count; j++)
    {
        //Add token
        tx_cmd[pos] = 0xFC;
        pos = pos + 1;
        //Add data
        memcpy(tx_cmd + pos, rawbuf + j * block_size, block_size);
        pos = pos + block_size;
        //Add data respon
        if (j == (count - 1))
        {
            memset(tx_cmd + pos, 0xff, BLOCK_W_DATA_RESP_SIZE_FINAL);
            pos = pos + BLOCK_W_DATA_RESP_SIZE_FINAL;
            continue;
        }
        memset(tx_cmd + pos, 0xff, BLOCK_W_DATA_RESP_SIZE_EACH);
        pos = pos + BLOCK_W_DATA_RESP_SIZE_EACH;
    }

    ESP_BOOT_LOG("******* Trans length : %d **********\n", pos);

    if (HAL_OK != STSpiWrite(tx_cmd, pos))
    {
        ESP_BOOT_LOG("spi transmit send error %d", __LINE__);
        return -1;
    }

    //Judge Write cmd resp, and 1st block data resp.
    pos = 5 + 1;
    for (i = 0; i < CMD_RESP_SIZE; i++)
    {
        if (tx_cmd[pos + i] == 0x00 && tx_cmd[pos + i - 1] == 0xff)
        {
            if (tx_cmd[pos + i + 1] == 0x00 && tx_cmd[pos + i + 2] == 0xff)
                break;
        }
    }

    if (i >= CMD_RESP_SIZE)
    {
        ESP_BOOT_LOG("spierr 1st block write cmd resp 0x00 no recv, %d\n", count);
        err_ret = -3;
        return err_ret;
    }

    pos = pos + CMD_RESP_SIZE;

    for (j = 0; j < count; j++)
    {
        find_w_rsp = 0;
        pos = pos + 1;
        pos = pos + block_size;

        if (j == (count - 1))
            n = BLOCK_W_DATA_RESP_SIZE_FINAL;
        else
            n = BLOCK_W_DATA_RESP_SIZE_EACH;

        for (i = 0; i < n; i++)
        {
            if ((tx_cmd[pos + i] & 0x0F) == 0x05)
            {
                find_w_rsp = 1;
                break;
            }
        }

        if (find_w_rsp == 0)
        {
            err_ret = -6;
            ESP_BOOT_LOG("spierr %s block%d write data no data res error, %d\n", __func__, j + 1, count);
            return err_ret;
        }
        pos = pos + n;
    }
    return err_ret;
}

esp_err_t sip_write_memory(uint8_t *rawbuf, uint32_t txseq, uint32_t addr, const uint8_t *buf, uint16_t len)
{
    struct sip_cmd_write_memory *cmd;
    struct sip_hdr *chdr;
    uint16_t remains, hdrs, bufsize;
    uint32_t loadaddr;
    const uint8_t *src;
    int err = 0;

    memset(rawbuf, 0x0, SIP_BOOT_BUF_SIZE);

    chdr = (struct sip_hdr *)rawbuf;
    SIP_HDR_SET_TYPE(chdr->fc[0], SIP_CTRL);
    chdr->cmdid = SIP_CMD_WRITE_MEMORY;

    remains = len;
    hdrs = sizeof(struct sip_hdr) + sizeof(struct sip_cmd_write_memory);

    while (remains)
    {
        src = &buf[len - remains];
        loadaddr = addr + (len - remains);

        if (remains < (SIP_BOOT_BUF_SIZE - hdrs))
        {
            /* aligned with 4 bytes */
            bufsize = roundup(remains, 4);
            memset(rawbuf + hdrs, 0x0, bufsize);
            remains = 0;
        }
        else
        {
            bufsize = SIP_BOOT_BUF_SIZE - hdrs;
            remains -= bufsize;
        }

        chdr->len = bufsize + hdrs;
        chdr->seq = txseq++;
        cmd = (struct sip_cmd_write_memory *)(rawbuf + SIP_CTRL_HDR_LEN);
        cmd->len = bufsize;
        cmd->addr = loadaddr;

        memcpy(rawbuf + hdrs, src, bufsize);

        err = sdspi_write_sync(rawbuf, chdr->len);
        if (err)
        {
            ESP_BOOT_LOG("Send buffer failed\n");
            return ESP_FAIL;
        }

        // 1ms is enough, in fact on dell-d430, need not delay at all.
        vTaskDelay(1);
    }

    return ESP_OK;
}

esp_err_t sip_send_cmd(uint32_t txseq, int cid, uint32_t cmdlen, void *cmd)
{
    int err = 0;
    struct sip_hdr *chdr;
    uint8_t *buffer = (uint8_t *)pvPortMalloc(sizeof(struct sip_hdr) + sizeof(struct sip_cmd_bootup));
    if (buffer == NULL)
    {
        ESP_BOOT_LOG("sip_send_cmd malloc failed");
        return ESP_FAIL;
    }
    memset(buffer, 0x0, (sizeof(struct sip_hdr) + sizeof(struct sip_cmd_bootup)));
    chdr = (struct sip_hdr *)buffer;
    SIP_HDR_SET_TYPE(chdr->fc[0], SIP_CTRL);

    chdr->len = SIP_CTRL_HDR_LEN + cmdlen;
    chdr->seq = txseq++;
    chdr->cmdid = cid;

    memcpy(buffer + sizeof(struct sip_hdr), (uint8_t *)cmd, cmdlen);

    err = sdspi_write_sync(buffer, chdr->len);
    if (err)
    {
        ESP_BOOT_LOG("Send buffer failed");
        if (buffer != NULL)
        {
            vPortFree(buffer);
            buffer = NULL;
        }
        return ESP_FAIL;
    }

    if (buffer != NULL)
    {
        vPortFree(buffer);
        buffer = NULL;
    }

    return ESP_OK;
}

esp_err_t esp_download_fw()
{
    const unsigned char *fw_buf = NULL;
    uint32_t offset = 0;
    int ret = ESP_OK;
    uint8_t blocks;
    struct esp_fw_hdr *fhdr;
    struct esp_fw_blk_hdr *bhdr = NULL;
    struct sip_cmd_bootup bootcmd;

    memset(espspitxbuffer, 0, sizeof(espspitxbuffer));
    tx_cmd = espspitxbuffer;

    uint32_t txseq = 0;
    fw_buf = &eagle_fw[0];

    uint8_t *rawbuf = (uint8_t *)pvPortMalloc(SIP_BOOT_BUF_SIZE);
    if (rawbuf == NULL)
    {
        ESP_BOOT_LOG("sip rawbuf memory not enough\n");
        return ESP_ERR_NO_MEM;
    }

    fhdr = (struct esp_fw_hdr *)fw_buf;

    if (fhdr->magic != 0xE9)
    {
        ESP_BOOT_LOG("wrong magic! \n");
        if (rawbuf != NULL)
        {
            vPortFree(rawbuf);
            rawbuf = NULL;
        }
        return ESP_FAIL;
    }

    blocks = fhdr->blocks;
    offset += sizeof(struct esp_fw_hdr);

    while (blocks)
    {
        bhdr = (struct esp_fw_blk_hdr *)(&fw_buf[offset]);
        offset += sizeof(struct esp_fw_blk_hdr);

        ESP_BOOT_LOG("blocks:%x ->%x,%x\n", blocks, bhdr->load_addr, bhdr->data_len);

        ret = sip_write_memory(rawbuf, txseq, bhdr->load_addr, &fw_buf[offset], bhdr->data_len);

        blocks--;
        offset += bhdr->data_len;
    }

    bootcmd.boot_addr = fhdr->entry_addr;

    ret = sip_send_cmd(txseq, SIP_CMD_BOOTUP, sizeof(struct sip_cmd_bootup), &bootcmd);

    if (rawbuf != NULL)
    {
        vPortFree(rawbuf);
        rawbuf = NULL;
    }

    return ret;
}

int SpiSendCmd(char *cmd, char *resp)
{
    if (HAL_OK != STSpiWrite(cmd, 6))
    {
        ESP_BOOT_LOG("spi transmit send error %d", __LINE__);
        return -1;
    }
    memset(resp, 0xFF, 10);

    if (HAL_OK != STSpiRead(resp, 10))
    {
        ESP_BOOT_LOG("spi transmit send error %d", __LINE__);
        return -1;
    }
    ESP_BOOT_LOG(" Resp data: 0x%x, 0x%x\n", resp[2], resp[3]);

    return 0;
}

int SPI_BLOCK_SIZE = 512;
uint8_t SpiProtocolIni(void)
{
    char spi_proto_ini_status = 0;

    char SPI2RxBuf[10];
    char SPI2TxBuf[10];
    #define WIFI_STARTUP_TIMEOUT 20
    uint32_t count = 0;

    do
    {
        if (spi_proto_ini_status == 0)
        {

            // cmd0  data + 0xff + 0xff + response
            do
            {
                memset(SPI2TxBuf, 0xFF, 10);
                SPI2TxBuf[0] = 0x40;
                SPI2TxBuf[1] = 0x00;
                SPI2TxBuf[2] = 0x00;
                SPI2TxBuf[3] = 0x00;
                SPI2TxBuf[4] = 0x00;
                SPI2TxBuf[5] = 0x95;
                ESP_BOOT_LOG("CMD0 \r\n");
                SpiSendCmd(SPI2TxBuf, SPI2RxBuf);
                vTaskDelay(200);
                count++;
                if (count == WIFI_STARTUP_TIMEOUT)
                {
                    return 1;
                }
            } while (SPI2RxBuf[2] != 0x01); // response R1, 0x01 means idle
        }
        else if (spi_proto_ini_status == 1)
        {
            SPI2TxBuf[0] = 0x45;
            SPI2TxBuf[1] = 0x00;
            SPI2TxBuf[2] = 0x20; //0x04;
            SPI2TxBuf[3] = 0x00;
            SPI2TxBuf[4] = 0x00;
            SPI2TxBuf[5] = 0x01;
            ESP_BOOT_LOG("CMD 5 1st\r\n");
            SpiSendCmd(SPI2TxBuf, SPI2RxBuf);
        }
        else if (spi_proto_ini_status == 2)
        {
            SPI2TxBuf[0] = 0x45;
            SPI2TxBuf[1] = 0x00;
            SPI2TxBuf[2] = 0x20;
            SPI2TxBuf[3] = 0x00;
            SPI2TxBuf[4] = 0x00;
            SPI2TxBuf[5] = 0x01;
            ESP_BOOT_LOG("CMD5 2nd\r\n");
            SpiSendCmd(SPI2TxBuf, SPI2RxBuf);
        }
        else if (spi_proto_ini_status == 3) //CMD 52   addr 0x2,   data 0x02;
        {
            SPI2TxBuf[0] = 0x74;
            SPI2TxBuf[1] = 0x80;
            SPI2TxBuf[2] = 0x00;
            SPI2TxBuf[3] = 0x04;
            SPI2TxBuf[4] = 0x02;
            SPI2TxBuf[5] = 0x01;
            ESP_BOOT_LOG("CMD52 Write  addr 02 \r\n");
            SpiSendCmd(SPI2TxBuf, SPI2RxBuf);
        }
        else if (spi_proto_ini_status == 4)
        {
            SPI2TxBuf[0] = 0x74;
            SPI2TxBuf[1] = 0x80;
            SPI2TxBuf[2] = 0x00;
            SPI2TxBuf[3] = 0x08;
            SPI2TxBuf[4] = 0x03;
            SPI2TxBuf[5] = 0x01;
            ESP_BOOT_LOG("CMD52 Write  addr 04 \r\n");
            SpiSendCmd(SPI2TxBuf, SPI2RxBuf);
        }
        else if (spi_proto_ini_status == 5)
        {
            SPI2TxBuf[0] = 0x74;
            SPI2TxBuf[1] = 0x00;
            SPI2TxBuf[2] = 0x00;
            SPI2TxBuf[3] = 0x04;
            SPI2TxBuf[4] = 0x00;
            SPI2TxBuf[5] = 0x01;
            ESP_BOOT_LOG("CMD52 Read  addr 0x2 \r\n");
            SpiSendCmd(SPI2TxBuf, SPI2RxBuf);
        }
        else if (spi_proto_ini_status == 6)
        {
            SPI2TxBuf[0] = 0x74;
            SPI2TxBuf[1] = 0x00;
            SPI2TxBuf[2] = 0x00;
            SPI2TxBuf[3] = 0x08;
            SPI2TxBuf[4] = 0x00;
            SPI2TxBuf[5] = 0x01;
            ESP_BOOT_LOG("CMD52 Read addr 0x4 \r\n");
            SpiSendCmd(SPI2TxBuf, SPI2RxBuf);
        }
        else if (spi_proto_ini_status > 6 && spi_proto_ini_status < 15)
        {
            SPI2TxBuf[0] = 0x74;
            SPI2TxBuf[1] = 0x10;
            SPI2TxBuf[2] = 0x00;
            SPI2TxBuf[3] = 0xF0 + 2 * (spi_proto_ini_status - 7);
            SPI2TxBuf[4] = 0x00;
            SPI2TxBuf[5] = 0x01;
            SpiSendCmd(SPI2TxBuf, SPI2RxBuf);
        }
        else if (spi_proto_ini_status == 15)
        {
            ESP_BOOT_LOG("CMD52 Write  Reg addr 0x110 \r\n");
            SPI2TxBuf[0] = 0x74;
            SPI2TxBuf[1] = 0x80; //func0 should be
            SPI2TxBuf[2] = 0x02;
            SPI2TxBuf[3] = 0x20;
            SPI2TxBuf[4] = (SPI_BLOCK_SIZE); //0x02;
            SPI2TxBuf[5] = 0x01;
            SpiSendCmd(SPI2TxBuf, SPI2RxBuf);

            ESP_BOOT_LOG("CMD52 Write Reg addr 0x111 \r\n");
            SPI2TxBuf[0] = 0x74;
            SPI2TxBuf[1] = 0x80;
            SPI2TxBuf[2] = 0x02;
            SPI2TxBuf[3] = 0x22;
            SPI2TxBuf[4] = (SPI_BLOCK_SIZE >> 8); //0x00;
            SPI2TxBuf[5] = 0x01;
            SpiSendCmd(SPI2TxBuf, SPI2RxBuf);
        }
        else if (spi_proto_ini_status == 16)
        {
        }
        else
        {
            break;
        }
        vTaskDelay(100);
        spi_proto_ini_status++;
    } while (1);

    return 0;
}

uint8_t Esp8266SpiBoot(void)
{
    esp_err_t err;

    ESP_BOOT_LOG("Esp8266SpiBoot host ready, start initializing slave...");

    extern void SpiHwIni(void);
    SpiHwIni();

    if (SpiProtocolIni())
    {
        ESP_BOOT_LOG("Esp8266SpiBoot  SpiProtocolIni fail...");
        return 1;
    }

    err = esp_download_fw();
    if (ESP_OK != err)
    {
        ESP_BOOT_LOG("Esp8266SpiBoot initializing slave fail...");
        return 2;
    }
    
    return 0;
}
