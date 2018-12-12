#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "uart_api.h"
#include "spi_cfg.h"
#include "WifiProcess.h"

#define ESP_SPIRW_LOG(format, ...) DebugPrintf(1, "\r\n" format, ##__VA_ARGS__)

#define CMD_RESP_SIZE (10)                      //Common respon wait time
#define BLOCK_W_DATA_RESP_SIZE_EACH (20)        //For each data write resp size, in block write
#define BLOCK_W_DATA_RESP_SIZE_FINAL (112 + 40) //For final data write resp size, in block write ,max :112
#define BLOCK_R_DATA_RESP_SIZE_1ST (123 + 40)   //For each data read resp size, in block read ,max: 123
#define BLOCK_R_DATA_RESP_SIZE_EACH (20)        //For each data read resp size, in block read

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;

struct sdio_slave_status_element
{
    u32 wr_busy : 1;
    u32 rd_empty : 1;
    u32 comm_cnt : 3;
    u32 intr_no : 3;
    u32 rx_length : 16;
    u32 res : 8;
};

extern char espspitxbuffer[128 * 7];
char espspirxbuffer[128 * 7] = {0};
char espspicopybuffer[128 * 7] = {0};
char *tx_cmd = NULL;
char *rx_cmd = NULL;
u8 *copy_buff = NULL;

#define SPI_BLOCK_SIZE (512)
#define BUFFERSIZE 512
uint16_t tx_len = 0;
u8 tx_buff[BUFFERSIZE], rx_buff[BUFFERSIZE];
u8 sdio_comm_cnt;
u32 total_recv_data_len = 0;

int sif_spi_write_bytes(u32 addr, u8 *src, u16 count, u8 func)
{
    int i;
    int pos, data_resp_size_w, total_num;
    unsigned char *tx_data = src;
    int err_ret = 0;
    data_resp_size_w = ((((((count >> 2) + 1) * 25) >> 5) * 21 + 16) >> 3) + 1 + 10; 
    
    if (count > 512)
    {
        err_ret = -1;
        return err_ret;
    }
    if (func > 7)
    {
        err_ret = -3;
        return err_ret;
    }
    tx_cmd[0] = 0x75;

    if (addr >= (1 << 17))
    {
        err_ret = -2;
        return err_ret;
    }
    else
    {
        tx_cmd[1] = 0x80 | (func << 4) | 0x04 | (addr >> 15); //0x94;
        tx_cmd[2] = addr >> 7;

        if (count == 512)
        {
            tx_cmd[3] = (addr << 1 | 0x0);
            tx_cmd[4] = 0x00; //0x08;
        }
        else
        {
            tx_cmd[3] = (addr << 1 | (count >> 8 & 0x01));
            tx_cmd[4] = count & 0xff; //0x08;
        }
    }

    tx_cmd[5] = 0x01;
    pos = 5 + 1;

    //Add cmd respon
    memset(tx_cmd + pos, 0xff, CMD_RESP_SIZE);
    pos = pos + CMD_RESP_SIZE;

    //Add token
    tx_cmd[pos] = 0xFE;
    pos = pos + 1;

    //Add data
    memcpy(tx_cmd + pos, tx_data, count);
    pos = pos + count;

    //Add data respon
    memset(tx_cmd + pos, 0xff, data_resp_size_w);
    total_num = pos + data_resp_size_w;
    memcpy(copy_buff, tx_cmd, pos);
    STSpiWrite(tx_cmd, total_num);

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
        ESP_SPIRW_LOG("sif_spi_write_bytes send:\n");
        for (i = 0; i < total_num; i++)
        {
            ESP_SPIRW_LOG("%02x,", copy_buff[i]);
            if (i == 0)
            {
                ESP_SPIRW_LOG("\n");
            }
        }
        ESP_SPIRW_LOG(" \n\n");

        ESP_SPIRW_LOG("sif_spi_write_bytes recv:\n");
        for (i = 0; i < total_num; i++)
        {
            ESP_SPIRW_LOG("%02x,", tx_cmd[i]);
            if (i == 0)
            {
                ESP_SPIRW_LOG("\n");
            }
        }
        ESP_SPIRW_LOG(" \n\n");
        err_ret = -4;
        return err_ret;
    }

    pos = pos + CMD_RESP_SIZE + count + 1;
    for (i = 0; i < data_resp_size_w; i++)
    {
        if (tx_cmd[pos + i] == 0xE5)
        {
            for (i++; i < data_resp_size_w; i++)
            {
                if (tx_cmd[pos + i] == 0xFF)
                {
                    break;
                }
            }
            break;
        }
    }
    if (i >= data_resp_size_w)
    {
        ESP_SPIRW_LOG("spierr byte write data no-busy wait byte 0xff no recv \n");
        for (i = 0; i < total_num; i++)
        {
            ESP_SPIRW_LOG("%02x,", tx_cmd[i]);
        }
        ESP_SPIRW_LOG(" \n\n");
        ESP_SPIRW_LOG(" \n\n");

        for (i = 0; i < total_num; i++)
        {
            ESP_SPIRW_LOG("%02x,", copy_buff[i]);
        }
        ESP_SPIRW_LOG(" \n\n");

        err_ret = -5;
        return err_ret;
    }

    return err_ret;
}

int sif_spi_write_blocks(u32 addr, u8 *src, u16 count, u16 block_size)
{
    int err_ret = 0;
    int i, j;
    int n;
    int pos;
    unsigned char *tx_data = src;
    int find_w_rsp = 0;

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
        memcpy(tx_cmd + pos, tx_data + j * block_size, block_size);
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

    STSpiWrite(tx_cmd, pos);

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
        ESP_SPIRW_LOG("spierr 1st block write cmd resp 0x00 no recv, %d\n", count);
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
            ESP_SPIRW_LOG("spierr %s block%d write data no data res error, %d\n", __func__, j + 1, count);
            return err_ret;
        }
        pos = pos + n;
    }
    return err_ret;
}

int sif_spi_read_bytes(u32 addr, u8 *dst, u16 count, u8 func)
{
    int pos, total_num, data_resp_size_r;
    int i;
    unsigned char *rx_data = dst;
    int err_ret = 0;
    int unexp_byte = 0;
    int find_start_token = 0;
    data_resp_size_r = ((((((count >> 2) + 1) * 25) >> 5) * 21 + 16) >> 3) + 1 + 10;

    rx_cmd[0] = 0x75;

    if (count > 512)
    {
        err_ret = -1;
        return err_ret;
    }

    if (func > 7)
    {
        err_ret = -3;
        return err_ret;
    }
    if (addr >= (1 << 17))
    {
        err_ret = -2;
        return err_ret;
    }
    else
    {
        rx_cmd[1] = (func << 4) | 0x04 | (addr >> 15); //0x14;
        rx_cmd[2] = addr >> 7;

        if (count == 512)
        {
            rx_cmd[3] = (addr << 1 | 0x0);
            rx_cmd[4] = 0x00; //0x08;
        }
        else
        {
            rx_cmd[3] = (addr << 1 | (count >> 8 & 0x01));
            rx_cmd[4] = count & 0xff;
        }
    }

    rx_cmd[5] = 0x01;

    total_num = CMD_RESP_SIZE + data_resp_size_r + count + 2;
    memset(rx_cmd + 6, 0xFF, total_num);
    total_num += 6;
    memcpy(copy_buff, rx_cmd, total_num);
    STSpiRead(rx_cmd, total_num);

    pos = 5 + 1;
    for (i = 0; i < CMD_RESP_SIZE; i++)
    {
        if (rx_cmd[pos + i] == 0x00 && rx_cmd[pos + i - 1] == 0xff)
        {
            if (rx_cmd[pos + i + 1] == 0x00 && rx_cmd[pos + i + 2] == 0xff)
                break;
        }
    }

    if (i >= CMD_RESP_SIZE)
    {
        ESP_SPIRW_LOG("spierr byte read cmd resp 0x00 no recv\n");
        err_ret = -4;

        for (i = 0; i < total_num; i++)
        {
            ESP_SPIRW_LOG("%02x ", copy_buff[i]);
            if ((i % 30) == 0)
            {
                ESP_SPIRW_LOG("\n");
            }
        }

        ESP_SPIRW_LOG("\n\n");
        ESP_SPIRW_LOG("\n\n");

        for (i = 0; i < total_num; i++)
        {
            ESP_SPIRW_LOG("%02x ", rx_cmd[i]);
            if ((i % 30) == 0)
            {
                ESP_SPIRW_LOG("\n");
            }
        }
        return err_ret;
    }

    pos = pos + i + 2;

    for (i = 0; i < data_resp_size_r; i++)
    {
        if (rx_cmd[pos + i] == 0xFE)
        {

            find_start_token = 1;
            break;
        }
        else if (rx_cmd[pos + i] != 0xff)
        {
            unexp_byte++;
            if (unexp_byte == 1)
                ESP_SPIRW_LOG("1 normal byte read not 0xFF or 0xFE  at the first time,count = %d,addr =%x \n", count, addr);
        }
    }

    if (find_start_token == 0)
    {
        ESP_SPIRW_LOG("spierr byte read start token 0xFE no recv ,count = %d,addr =%x \n", count, addr);

        err_ret = -5;

        return err_ret;
    }

    pos = pos + i + 1;
    for (i = 0; i < count; i++)
    {
        *(rx_data + i) = rx_cmd[pos + i];
    }
    return err_ret;
}

int sif_spi_read_blocks(u32 addr, u8 *dst, u16 count, u16 block_size)
{
    int err_ret = 0;
    int pos;
    int i, j;
    unsigned char *rx_data = dst;
    int total_num;
    int find_start_token = 0;

    rx_cmd[0] = 0x75;

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
        rx_cmd[1] = 0x10 | 0x0C | (addr >> 15);
        rx_cmd[2] = addr >> 7;

        if (count >= 512)
        {
            rx_cmd[3] = (addr << 1 | 0x0);
            rx_cmd[4] = 0x00;
        }
        else
        {
            rx_cmd[3] = (addr << 1 | (count >> 8 & 0x01));
            rx_cmd[4] = count & 0xff;
        }
    }
    rx_cmd[5] = 0x01;
    total_num = CMD_RESP_SIZE + BLOCK_R_DATA_RESP_SIZE_1ST + block_size + 2 + (count - 1) * (BLOCK_R_DATA_RESP_SIZE_EACH + block_size + 2);
    memset(rx_cmd + 6, 0xFF, total_num);
    total_num += 6;
    STSpiRead(rx_cmd, total_num);

    pos = 5 + 1;
    for (i = 0; i < CMD_RESP_SIZE; i++)
    {
        if (rx_cmd[pos + i] == 0x00 && rx_cmd[pos + i - 1] == 0xff)
        {
            if (rx_cmd[pos + i + 1] == 0x00 && rx_cmd[pos + i + 2] == 0xff)
                break;
        }
    }

    if (i >= CMD_RESP_SIZE)
    {
        ESP_SPIRW_LOG("spierr block read cmd resp 0x00 no recv\n");

        err_ret = -3;
        return err_ret;
    }

    pos = pos + i + 2;

    for (i = 0; i < BLOCK_R_DATA_RESP_SIZE_1ST; i++)
    {
        if (rx_cmd[pos + i] == 0xFE)
        {
            find_start_token = 1;
            break;
        }
    }

    if (find_start_token == 0)
    {
        ESP_SPIRW_LOG("1st block read data resp 0xFE no recv,count = %d\n", count);
        err_ret = -4;
        return err_ret;
    }

    pos = pos + i + 1;

    memcpy(rx_data, rx_cmd + pos, block_size);

    pos = pos + block_size + 2;

    for (j = 1; j < count; j++)
    {

        for (i = 0; i < BLOCK_R_DATA_RESP_SIZE_EACH; i++)
        {
            if (rx_cmd[pos + i] == 0xFE)
            {
                break;
            }
        }
        if (i >= BLOCK_R_DATA_RESP_SIZE_EACH)
        {
            ESP_SPIRW_LOG("spierr block%d read data token 0xFE no recv,total:%d\n", j + 1, count);
            err_ret = -7;
            return err_ret;
        }
        pos = pos + i + 1;

        memcpy(rx_data + j * block_size, rx_cmd + pos, block_size);
        pos = pos + block_size + 2;
    }
    return err_ret;
}

void SdioRW(void *pvParameters)
{
    int ret;
    u32 intr_clr;
    struct sdio_slave_status_element sd_sta;
    if (tx_cmd == NULL)
    {
        memset(espspitxbuffer, 0, sizeof(espspitxbuffer));
        tx_cmd = espspitxbuffer;
    }
    memset(espspirxbuffer, 0, sizeof(espspirxbuffer));
    memset(espspicopybuffer, 0, sizeof(espspicopybuffer));
    rx_cmd = espspirxbuffer;
    copy_buff = (u8*)espspicopybuffer;
    ESP_SPIRW_LOG("SDIO Communication Start!");

    unsigned short blk_size = SPI_BLOCK_SIZE, xx;
    for (xx = 0; xx < 10000; xx++)
        ;
    ret = sif_spi_write_bytes(0x110, (u8 *)&blk_size, 2, 0);
    for (xx = 0; xx < 10000; xx++)
        ;

    ret = sif_spi_read_bytes(0x20, (u8 *)&sd_sta, 4, 1);
    if (ret)
    {
        ESP_SPIRW_LOG("Init Read reg err:-0x%x\n ", -ret);
    }
    sdio_comm_cnt = sd_sta.comm_cnt;
    while (1)
    {
        ret = sif_spi_read_bytes(0x20, (u8 *)&sd_sta, 4, 1);
        if (ret)
        {
            ESP_SPIRW_LOG("Read reg err:-0x%x\n ", -ret);
            continue;
        }
        intr_clr = 1;
        ret = sif_spi_write_bytes(0x30, (u8 *)&intr_clr, 4, 1);
        if (ret)
        {
            ESP_SPIRW_LOG("Write reg err:-0x%x\n ", -ret);
            continue;
        }

        tx_len = 0;
        memset(tx_buff, 0, sizeof(tx_buff));
        if (0 == WIFIGetCmdAndSize(tx_buff, &tx_len))
        {
            ESP_SPIRW_LOG("SdioRW get txbuf tx_len(%d)", tx_len);
            ESP_SPIRW_LOG("\r\n");
            PrintOutHexToAscii((char *)tx_buff, tx_len);
            ESP_SPIRW_LOG("\r\n");
        }

        if (((sd_sta.rd_empty) || (sd_sta.rx_length == 0)) && (tx_len == 0))
        {
            osDelay(50);
        }

        if ((!sd_sta.rd_empty) && (sd_sta.rx_length != 0))
        {
            int len = sd_sta.rx_length;
            u8 *ptr_buf = rx_buff;

            ESP_SPIRW_LOG("SdioRW sd_sta.rx_length(%d)", sd_sta.rx_length);

            if (len >= SPI_BLOCK_SIZE)
            {
                ret = sif_spi_read_blocks(0x1f800 - len, (u8 *)ptr_buf, len / SPI_BLOCK_SIZE, SPI_BLOCK_SIZE);
                if (ret)
                {
                    ESP_SPIRW_LOG("Read first Data err:-0x%x\n", -ret);
                    continue;
                }
                ptr_buf += sd_sta.rx_length - len % SPI_BLOCK_SIZE;
                total_recv_data_len += sd_sta.rx_length - len % SPI_BLOCK_SIZE;
                len = len % SPI_BLOCK_SIZE;
            }

            while ((len >= 512) && (ret == 0))
            {
                ret = sif_spi_read_bytes(0x1f800 - len, (u8 *)ptr_buf, 512, 1);
                len -= 512;
                ptr_buf += 512;
                if (ret)
                {
                    ESP_SPIRW_LOG("Read Data second err:-0x%x,%d\n", -ret, len);
                }
                else
                {
                    total_recv_data_len += 512;
                }
            }

            if ((len != 0) && (ret == 0))
            {
                ret = sif_spi_read_bytes(0x1f800 - len, (u8 *)ptr_buf, len, 1);
                if (ret == 0)
                {
                    total_recv_data_len += len;
                }
            }

            rx_buff[sd_sta.rx_length] = '\0';
            if (ret)
            {
                ESP_SPIRW_LOG("Read Data err:-0x%x,%d\n", -ret, len);
            }
            else
            {
                ESP_SPIRW_LOG("RecData len(%d):", sd_sta.rx_length);
                ESP_SPIRW_LOG("\r\n");
                PrintOutHexToAscii((char *)rx_buff, sd_sta.rx_length);
                ESP_SPIRW_LOG("\r\n");

                if (WIFIPutEvtInFifo(sd_sta.rx_length, rx_buff))
                {
                    ESP_SPIRW_LOG("SdioRW put in evt fifo err");
                }

                sdio_comm_cnt = (sdio_comm_cnt + 1) & 0x07;
            }
        }

        if (tx_len != 0)
        {
            if (!sd_sta.wr_busy)
            {
                uint16_t len = tx_len;
                u8 *ptr_buf = tx_buff;
                if (len >= SPI_BLOCK_SIZE)
                {
                    ret = sif_spi_write_blocks(0x1f800 - len, (u8 *)ptr_buf, len / SPI_BLOCK_SIZE, SPI_BLOCK_SIZE);
                    if (ret)
                    {
                        ESP_SPIRW_LOG("Read Data err:-0x%x\n", -ret);
                        continue;
                    }

                    ptr_buf += tx_len - len % SPI_BLOCK_SIZE;
                    len = len % SPI_BLOCK_SIZE;
                }
                if (len != 0)
                {
                    ret = sif_spi_write_bytes(0x1f800 - len, (u8 *)ptr_buf, len, 1);
                }
                tx_len = 0;
                if (ret)
                {
                    ESP_SPIRW_LOG("Write Data err:-0x%x\n ", -ret);
                }
                else
                {
                    sdio_comm_cnt = (sdio_comm_cnt + 1) & 0x07;
                }
            }
        }
    }
    // ESP_SPIRW_LOG("SDIO Communication thread delete\n");
}
