/*******************************************************************************
* File Name          : api.c
* Author             : Yangjie Gu
* Description        : This file provides all the api functions.

* History:
*  10/23/2017 : api V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "prot.h"

#include "cmsis_os.h"

#include "usart.h"
#include "i2c.h"
#include "iwdg.h"
#include "can.h"

#include "common.h"
#include "deepsleep.h"
#include "software_timer.h"
#include "io_control.h"
#include "rtcclock.h"
#include "uart_api.h"
#include "lis2dh_driver.h"
#include "file_operation.h"
#include "oemmsg_apis.h"
#include "oemmsg_queue_process.h"
#include "initialization.h"
#if GSENSOR_I2C_USE_GPIO_SIMULATION
#include "i2c_driver.h"
#endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */

#include "fatfs.h"
#include "flash.h"
#include "SdioEmmcDrive.h"

#define BIN_MAJOR 0
#define BIN_MINOR 0
#define BIN_REV  1
#define BIN_BUILD 2



#define FILE_IO_CACHE
#define MAX_FILE_BUF 1024
typedef struct {
	int8 FileP; 
	uint8*			buff;				   
	uint16			length;		
} atFile_t;

































/* Private define ------------------------------------------------------------*/
#define API_PRINT(format, ...) DebugPrintf(DbgCtl.ApiDebugInfoEn, "\r\n" format, ##__VA_ARGS__)

/* Variables -----------------------------------------------------------------*/
extern uint16_t ModemtimeZoneGet;
/* Function prototypes -------------------------------------------------------*/
// entry
void at_startTftp(void (*callback)(uint16 result), const char *local, const char *remote, const char *domain, const tftpconfig *config)
{
}

void at_stopTftp(void)
{
}

void at_queryTftp(uint16 *state, uint16 *block)
{
}

bool at_verify(char *path)
{
    return 1;
}
/*----------------------------------------------------------------------------*/

// gps
void at_closeGps(void)
{
}
/*-------------------->>>>>>>>>>>>>>>>>>>>>> parameter --------------->>>>>>> */
void at_GpsStart(uint8 fixmode, uint32 TimeBFixes)
{
}
/*----------------------------------------------------------------------------*/

// call
bool at_AnswerCall(void)
{
    return 0;
}

bool at_HangupCall(void)
{
    return 0;
}

bool at_OrigVoiceCall(char *pDialstr)
{
    return 0;
}

bool at_CallWait(bool bEnable)
{
    return 0;
}

bool at_HookFlash(uint8 *KeypadFac)
{
    return 0;
}

bool at_SendBurstDTMF(char *pDtmf, uint8 DurOn, uint8 DurOff)
{
    return 0;
}

bool at_SendContDTMF(char *pDtmf, uint8 mode)
{
    return 0;
}
/*----------------------------------------------------------------------------*/

#define MODEM_SINGLE_SMS_MAX_LEN (160)
// message
bool at_sendMessage(char *Num, char *buf)
{
    return 1;
}

RegIdT at_GetSMSRegisterId(void)
{
    return 0;
}

bool at_DeleteAllSMS(OemSMSDelMemT mem)
{
    return 1;
}
/*----------------------------------------------------------------------------*/

// ppp and socket
void at_UdpIpSocketOpen(uint8 Socket_Num, uint16 LocalPort, char *DestAddrP, uint16 PortNum)
{
}

void at_UdpIpSocketClose(uint8 Socket_Num)
{
}

void at_UdpIpSocketSendData(uint8 Socket_Num, char *buffer, uint16 len)
{
}

void at_TcpIpPppOpen(void)
{
}

void at_TcpIpPppClose(void)
{
}

OemPppStatus at_TcpIpGetPppStatus(void)
{
    return (OemPppStatus)0;
}

void at_TcpIpSocketOpen(uint8 Socket_Num, uint16 LocalPort, char *DestAddrP, uint16 PortNum)
{
}

void at_TcpIpSocketClose(uint8 Socket_Num)
{
}

void at_TcpIpSocketSendData(uint8 Socket_Num, char *buffer, uint16 len)
{
}

OemNetResultT at_DNSQueryMsg(char *address, OEM_APP_CallBackT appFunc, RegIdT regId)
{
    return (OemNetResultT)0;
}

//uint32 at_NetNToHl(uint32 netlong)
//{
//    // return t_ntohl(netlong);
//}

void at_IpAddrInt2Char(uint32 IntAddr, char *pCharIpAddr)
{
    uint32 addr1, addr2, addr3, addr4;
    addr1 = (IntAddr & 0xff000000) >> 24;
    addr2 = (IntAddr & 0x00ff0000) >> 16;
    addr3 = (IntAddr & 0x0000ff00) >> 8;
    addr4 = IntAddr & 0x000000ff;

    sprintf(pCharIpAddr, "%d.%d.%d.%d", addr1, addr2, addr3, addr4);
}

void at_TcpIpSetUdpAddrPort(uint8 Socket_Num, char *addr, uint16 port)
{
}

OemNetResultT at_UdpIpSocketSendDataNew(uint8 Socket_Num, char *buffer, uint16 *len)
{
    return OEM_NET_SUCCESS;
}
/*----------------------------------------------------------------------------*/

// heap
void *at_malloc(uint32 size)
{
    if (size > 0)
    {
        return pvPortMalloc(size);
    }
    else
    {
        return NULL;
    }
}

void at_mfree(void *Ptr)
{
    if (Ptr != NULL)
        vPortFree(Ptr);
}
/*----------------------------------------------------------------------------*/
// string
int at_sprintf(char *buffer, const char *format, ...)
{
    int n;
    va_list ap;

    va_start(ap, format);
    n = at_vsprintf(buffer, format, ap);
    va_end(ap);
    return (n);
}

int at_vsprintf(char *buffer, const char *format, va_list arg)
{
    return vsprintf(buffer, format, arg);
}
/*-------------------->>>>>>>>>>>>>>>>>>>>>> function  --------------->>>>>>> */
//uint32 at_NTOH32(uint32 val)
//{
//    // return t_ntohl(val);
//}
/*----------------------------------------------------------------------------*/

// file
OemValFsiResultT at_fopen(OemValFsiHandleT *FileP, const char *FileNameP, OemValFsiFileOpenModeT Mode)
{
	
#ifdef FILE_IO_CACHE
	atFile_t  *fileH;
	OemValFsiResultT ret;
	
	fileH = (atFile_t  *)pvPortMalloc(sizeof(atFile_t));
	fileH->buff = NULL; 
	fileH->length = 0;
	ret = FileOpen((OemValFsiHandleT *) &(fileH->FileP), FileNameP, Mode);
	if(OEM_FSI_SUCCESS == ret)
	{
		if( Mode != OEM_FSI_FILE_OPEN_READ_EXIST ) 
			fileH->buff = (uint8*) pvPortMalloc( MAX_FILE_BUF);
	}
	else
	{
		vPortFree(fileH);
		fileH = NULL;
	}
	*FileP = (OemValFsiHandleT )fileH;
//	at_printfDebug("\nat_fopen file End %s retrun %d", FileNameP, ret);
	return ret;
#else
	return FileOpen(FileP, FileNameP, Mode);
#endif	
	
    
}

OemValFsiResultT at_fclose(OemValFsiHandleT File)
{
#ifdef FILE_IO_CACHE
	OemValFsiResultT ret=OEM_FSI_SUCCESS;
	atFile_t  *fileH;
	uint32 ItemNum;

	fileH = (atFile_t  *)File;

	if(!fileH) return ret;
	if(fileH->length)
	{
		ItemNum = fileH->length;
		ret = FileWrite(fileH->buff, 1, &ItemNum, fileH->FileP);
//		at_printfDebug("\nFile Close Flush buffer byte %d\n", fileH->length);
	}
	ret |= (OemValFsiResultT)FileClose( fileH->FileP );
	if(fileH->buff)
	{
		vPortFree(fileH->buff);
	}
	vPortFree(fileH);
	return ret;
#else
	return FileClose(File);
#endif	
    
}

OemValFsiResultT at_fread(void *BufferP, uint32 ItemSize, uint32 *ItemNumP, OemValFsiHandleT File)
{
#ifdef FILE_IO_CACHE
	OemValFsiResultT ret=OEM_FSI_SUCCESS;
	atFile_t  *fileH;
	uint32 ItemNum;

	fileH = (atFile_t  *)File;
	if(!fileH) return ret;
	if(fileH->length)
	{
		ItemNum = fileH->length;
		ret = FileWrite(fileH->buff, 1, &ItemNum, fileH->FileP);
//		at_printfDebug("\nFile fread Flush buffer byte %d\n", fileH->length);
	}

	return  (ret| FileRead(BufferP, ItemSize, ItemNumP, ((atFile_t  *)File)->FileP));
#else
	return FileRead(BufferP, ItemSize, ItemNumP, File);
#endif	

}

OemValFsiResultT at_fwrite(void *BufferP, uint32 ItemSize, uint32 *ItemNumP, OemValFsiHandleT File)
{
#ifdef FILE_IO_CACHE
	OemValFsiResultT ret = OEM_FSI_SUCCESS;
	uint32 ItemNum;
	uint32 left;
	atFile_t  *fileH = (atFile_t  *)File;
	ItemSize *=(*ItemNumP);
	if(!fileH->buff) return OEM_FSI_ERR_WRITE;
	if(ItemSize > MAX_FILE_BUF)
	{
		if(fileH->length)
		{
			ItemNum = fileH->length;
			ret |= FileWrite(fileH->buff, 1, &ItemNum, fileH->FileP);	
		}
		ItemNum = ItemSize;
		ret |= FileWrite(BufferP, 1, &ItemNum, fileH->FileP);	
//		at_printfDebug("\nFile Write Flush buffer byte %d\n", ItemSize+fileH->length);
		fileH->length = 0;
	}
	else if (fileH->length + ItemSize > MAX_FILE_BUF) 
	{
		left = MAX_FILE_BUF- fileH->length;
		memcpy(fileH->buff+fileH->length, BufferP, left);
		ItemNum = MAX_FILE_BUF;
		ret = FileWrite(fileH->buff, 1, &ItemNum, fileH->FileP);
//		at_printfDebug("\nFile Write Flush buffer byte %d\n", MAX_FILE_BUF);
		fileH->length = ItemSize- left;
		memcpy(fileH->buff, (uint8*)BufferP+left, fileH->length);
	}
	else
	{
		memcpy(fileH->buff+fileH->length, BufferP, ItemSize);
		fileH->length += ItemSize;
//		at_printfDebug("\nFile Write Cache size %d, %d\n", fileH->length, ItemSize);
	}
	if(ret != OEM_FSI_SUCCESS) *ItemNumP = 0;
	return ret;
#else
	return FileWrite(BufferP, ItemSize, ItemNumP, File);
#endif	

}

OemValFsiResultT at_fseek(OemValFsiHandleT File, OemValFsiFileSeekTypeT SeekFrom, int32 MoveDistance)
{
#ifdef FILE_IO_CACHE
	OemValFsiResultT ret=OEM_FSI_SUCCESS;
	atFile_t  *fileH;
	uint32 ItemNum;

	fileH = (atFile_t  *)File;

	if(!fileH) return ret;
	if(fileH->length)
	{
		ItemNum = fileH->length;
		ret = FileWrite(fileH->buff, 1, &ItemNum, fileH->FileP);
//		at_printfDebug("\nFile fseek Flush buffer byte %d\n", fileH->length);
	}

	return ret |(OemValFsiResultT)FileSeek( ((atFile_t  *)File)->FileP, SeekFrom, MoveDistance);
#else
	return (OemValFsiResultT)FileSeek( File, SeekFrom, MoveDistance);
#endif	
   
}

OemValFsiResultT at_ftell(OemValFsiHandleT File, uint32 *PosP)
{
	
#ifdef FILE_IO_CACHE
	OemValFsiResultT ret=OEM_FSI_SUCCESS;
	atFile_t  *fileH;
	uint32 ItemNum;

	fileH = (atFile_t  *)File;

	if(!fileH) return ret;

	if(fileH->length)
	{
		ItemNum = fileH->length;
		ret = FileWrite(fileH->buff, 1, &ItemNum, fileH->FileP);
//		at_printfDebug("\nFile ftell Flush buffer byte %d\n", fileH->length);
	}

	return ret | FileTell( ((atFile_t  *)File)->FileP, PosP);
#else
	return FileTell(File, PosP);
#endif	
  
}

OemValFsiResultT at_remove(const char *NameP)
{
    return FileRemove(NameP);
}

OemValFsiResultT at_rename(const char *OldNameP, const char *NewNameP)
{
    return FileRename(OldNameP, NewNameP);
}

OemValFsiResultT at_fgetLength(const char *NameP, uint32 *FileLengthP)
{
    return FileGetLength(NameP, FileLengthP);
}
/*----------------------------------------------------------------------------*/
// output
void at_printfDebug(const char *Fmt, ...)
{
    int32_t cnt;
    char string[MAX_PRINTF_STR_SIZE + 2] = {'\0'};
    va_list ap;
    va_start(ap, Fmt);

    cnt = vsnprintf(string, MAX_PRINTF_STR_SIZE, Fmt, ap);
    if (cnt > 0)
    {
        if (cnt < MAX_PRINTF_STR_SIZE)
        {
            PutStrToUartDbg(string, cnt);
        }
        else
        {
            PutStrToUartDbg(string, MAX_PRINTF_STR_SIZE);
        }
    }
    va_end(ap);
}

void at_vprintfDebug(const char *Fmt, va_list arg)
{
}

void at_OutputString(char *buf)
{
    PutStrToUartDbg(buf, strlen(buf));
}
/*----------------------------------------------------------------------------*/

// system
void at_reset(void)
{
    MCUReset();
}

void at_readImei(uint8 *buf)
{
}

void at_readImsi(uint8 *buf)
{
}

uint16 at_readBsid(void)
{
    return 0;
}

void at_readBsidWithNeighbor(OemNghbrPilotData nghbrdata[], uint8 *num)
{
}

uint16 at_getSrvType(void)
{
    return 0;
}

void at_getBinVersion(uint8 *buf)
{
}

void at_writeAccount(const char *username, const char *passwd)
{
}

void at_setWorkMode(BOOL PowerUpCtrl)
{
}

uint8 at_getGetRegStat(void)
{
    return 0;
}

uint32 at_MonDeepSleepSuspend(uint8 Interface, uint32 BitMask)
{
    return 1;
}

uint32 at_MonDeepSleepResume(uint8 Interface, uint32 BitMask)
{
#define TNDC_MCU_SLEEP_S (5)
    uint32_t sleepseconds = TNDC_MCU_SLEEP_S;
    uint32_t seconds = TimeTableToSeconds(GetRTCDatetime());

    MCUDeepSleep(sleepseconds);

    if ((seconds + sleepseconds) > TimeTableToSeconds(GetRTCDatetime()))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

char *at_readICCID(void)
{
    return NULL;
}

void at_initTimers(uint16 timers)
{
    InitTimers(timers);
}

void at_killTimers(void)
{
    KillTimers();
}

void at_startTimer(uint16 timer, uint32 delay, void (*handler)(uint16 timer))
{
    StartTimer(timer, delay, handler);
}

void at_stopTimer(uint16 timer)
{
    StopTimer(timer);
}

bool at_IPAddrCheck(char *buf)
{
    int16 val = 0;
    uint8 index = 0;
    uint8 num = 0;
    char *address = buf;
    char ch = address[index];

    while (ch != '\0')
    {
        if (ch == '.')
        {
            if ((val < 0) || (val > 255) || (index == 0))
            {
                return FALSE;
            }
            val = 0;
            index++;
            num++;
            ch = address[index];
            if ((ch < '0') || (ch > '9'))
                return FALSE;
        }
        else if ((ch >= '0') && (ch <= '9'))
        {
            val = val * 10 + (ch - '0');
            index++;
            ch = address[index];
        }
        else
        {
            return FALSE;
        }
    }
    if (num == 3)
    {
        if ((val < 0) || (val > 255))
        {
            return FALSE;
        }
        return TRUE;
    }
    return FALSE;
}

void at_SetRTCTimer(uint8 time)
{
    API_PRINT("[%s] APP: SetRTCTimer",
                 FmtTimeShow());
    SetRTCAlarmTime(time, TRUE);
}

OemNetResultT at_NetPppDormantReq(void)
{
    return (OemNetResultT)0;
}

bool at_HFANeedStart(void)
{
    return 0;
}

void at_HFARTN(void)
{
}

uint16 at_GetECIO(void)
{
    return 0;
}
void at_GetBSCoOrdInfo(OemBSCoOrdMsgT *BSCoOrd)
{
}
/*----------------------------------------------------------------------------*/

// device
bool at_readGpio(uint32 GpioNum)
{
    if (IOInoutStatRecArray[GpioNum].inoutstate != IO_IN_OUT_STATE_IN)
    {
        uint32_t mode = GPIO_MODE_IT_RISING_FALLING;
        IO_SetDirInput(GpioNum, mode, GPIO_PULLDOWN);
    }
    
    return IO_Read(GpioNum);
}

void at_writeGpio(uint32 GpioNum, bool value)
{
    if (IOInoutStatRecArray[GpioNum].inoutstate != IO_IN_OUT_STATE_OUT)
    {
        IO_SetDirOutput(GpioNum, GPIO_SPEED_FREQ_LOW, (value ? GPIO_PIN_SET : GPIO_PIN_RESET));
    }
    else
    {
        IO_Write(GpioNum, (value ? GPIO_PIN_SET : GPIO_PIN_RESET));
    }
}

int at_CPSetFotaFlag(OemOMALOGStates OMA_Log_States)
{
    return 0;
}

int at_FlashDeltaFileErase(void)
{
    return 0;
}

int at_FlashDeltaFileWrite(uint32 Offset, const uint8 *data_ptr, uint32 NumBytes)
{
    return 0;
}

void at_I2cInit(void)
{
    GetExclusiveLock(&i2c2Lock);

#if (GSENSOR_I2C_USE_GPIO_SIMULATION)
    ExtensionI2cInit();
#else
    MX_I2C2_Init();
#endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */

    FreeExclusiveLock(&i2c2Lock);
}

void at_I2cLock(void)
{
}

void at_I2cUnlock(void)
{
}

bool at_I2cWrite(uint16 DeviceAddr, uint16 SubAddress, uint8 *DataPtr, uint16 NumBytes, uint16 ConfigBits)
{
    if (DeviceAddr == LIS2DH_MEMS_I2C_ADDRESS)
    {
        GetExclusiveLock(&i2c2Lock);

#if GSENSOR_I2C_USE_GPIO_SIMULATION
        ExtI2cWriteSerialRegister(DeviceAddr, SubAddress, NumBytes, DataPtr);
        FreeExclusiveLock(&i2c2Lock);
        return 1;
#else
        if (HAL_OK == HAL_I2C_Mem_Write(&hi2c2, DeviceAddr, SubAddress, I2C_MEMADD_SIZE_8BIT, DataPtr, NumBytes, I2C2_TIMEOUT_VALUE))
        {
            FreeExclusiveLock(&i2c2Lock);
            return 1;
        }
        else
        {
            FreeExclusiveLock(&i2c2Lock);
            return 0;
        }
#endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */
    }
    else
    {
        if (HAL_OK == HAL_I2C_Mem_Write(&hi2c3, DeviceAddr, SubAddress, I2C_MEMADD_SIZE_8BIT, DataPtr, NumBytes, I2C3_TIMEOUT_VALUE))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

bool at_I2cRead(uint16 DeviceAddr, uint16 SubAddress, uint8 *DataPtr, uint16 NumBytes, uint16 ConfigBits)
{
    if (DeviceAddr == LIS2DH_MEMS_I2C_ADDRESS)
    {
        GetExclusiveLock(&i2c2Lock);

#if GSENSOR_I2C_USE_GPIO_SIMULATION
        ExtI2cReadSerialData(DeviceAddr, SubAddress, NumBytes, DataPtr);
        FreeExclusiveLock(&i2c2Lock);
        return 1;
#else
        if (HAL_OK == HAL_I2C_Mem_Read(&hi2c2, DeviceAddr, SubAddress, I2C_MEMADD_SIZE_8BIT, DataPtr, NumBytes, I2C2_TIMEOUT_VALUE))
        {
            FreeExclusiveLock(&i2c2Lock);
            return 1;
        }
        else
        {
            FreeExclusiveLock(&i2c2Lock);
            return 0;
        }
#endif /* GSENSOR_I2C_USE_GPIO_SIMULATION */
    }
    else
    {
        if (HAL_OK == HAL_I2C_Mem_Read(&hi2c3, DeviceAddr, SubAddress, I2C_MEMADD_SIZE_8BIT, DataPtr, NumBytes, I2C3_TIMEOUT_VALUE))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

void at_UartSetBaudRate(uint32 rate)
{
    
}

bool at_UartSetbit(uint8 bit, uint8 stopbit, uint8 Parity)
{
    return 0;
}

uint16 at_WatchdogKick(void)
{
    WatchdogTick();
    return 1;
}

void at_WatchdogStart(uint16 dur)
{
    #define TNDC_WATCHDOG_MAX_KICK_TIME_MS (32000)
    WatchdogEnable(TNDC_WATCHDOG_MAX_KICK_TIME_MS);
}

void at_hwreset(void)
{
}
/*----------------------------------------------------------------------------*/

// time
int at_getTimeZone(void)
{
    return 0;
}

OemTimeTbl at_getTime(void)
{
    OemTimeTbl ret;
    return ret;
}
/*----------------------------------------------------------------------------*/

// sim
// void      mx_querySim (const mx_uint8 *path, mx_uint32 user)
// {

// }
/*----------------------------------------------------------------------------*/

// C lib functions
void at_assert(int expression)
{
    if (!expression)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

int at_atoi(const char *string)
{
    return atoi(string);
}
/*----------------------------------------------------------------------------*/

// Register Call back functions
void at_msgHandleCallBack(void (*pRecvCallbackP)(uint32 MessageId, void *MsgBufferP, uint32 size))
{
    OemRegisterMsgHandleCallBack(pRecvCallbackP);
}
/*----------------------------------------------------------------------------*/

/*oem api*/
OemPswInfo at_OemPswInfoGet(void)
{
    OemPswInfo PswInfo;

    return PswInfo;
}

OemNetResultT at_ValNetGetIpAddress(uint32 *IpAddrP)
{
    return OEM_NET_SUCCESS;
}

void at_GetAdcValue(uint8 Channel)
{
}

void at_getNetworkInfo(uint8 *buf)
{
    *buf = 1;
}

//bool at_ValGetNmeaData(void* data,uint16 size);
void at_SwitchToDbg(void)
{
}

uint16 at_AdcToMv(uint16 nChannel, uint16 AdcValue)
{
  return 0;
}

void at_PswPowerCtrl(bool bPowerUp)
{
}

int16 at_read_rf_pa_temperature(void)
{
    return 0;
}

void at_getBinVersionEx(version_t * buf)
{
}

void at_VirtualATCmd(char *cmd, uint16 len)
{

}

void at_Uart2Config(uint32 rate)
{
    UART_Init(UART1_NUMBER, rate);
}

// #define UART_WORDLEN_8B (0)
// #define UART_WORDLEN_9B (1)
// #define UART_STOPBIT_1 (0)
// #define UART_STOPBIT_2 (1)
// #define UART_PARITYNONE (0)
// #define UART_PARITYODD (1)
// #define UART_PARITYEVEN (2)
bool at_Uart2Setbit(uint8 bit,uint8 stopbit ,uint8 Parity)
{
    if (UART_SetBit(UART1_NUMBER, bit, stopbit, Parity) != 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

bool at_OutputStringUart2(char *buf,uint16 length)
{
    if ((buf == NULL) || (length == 0))
    {
        return 0;
    }
	
    if (HAL_UART_Transmit(&huart1, (uint8_t *)buf, length, UART_SEND_DATA_TIMEOUT) == HAL_OK)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void at_CANSend(MonentCanData *pmsg)
{
    MonetCANMsgStruct MonetCANMsg;

    if (pmsg == NULL)
    {
        return;
    }

    if (0 == CAN1_Transmit(pmsg->Data, pmsg->DLC, pmsg->StdId, pmsg->ExtId, pmsg->IDE, pmsg->RTR))
    {
        MonetCANMsg.msgtype = CAN_SEND_SUCCESSFUL;
    }
    else
    {
        MonetCANMsg.msgtype = CAN_SEND_FAILED;
    }
    MonetCANMsg.pData = pmsg;

    OemMsgHandle(OEM_CAN_MSG, &MonetCANMsg, sizeof(MonetCANMsg));
}

void at_CANInit(uint32 baudrate)//baudrate can be 125,000 250,000, 500,000, or 1,000,000
{
    // Default baudrate 125,000
    CAN1_Normal_Init(baudrate);
}

void at_CANInitEx(uint32 baudrate, CANFILTER_T *filters, uint8 filetersize)//baudrate can be 125,000 250,000, 500,000, or 1,000,000
{
    // Default baudrate 125,000
    CAN1_Normal_InitEx(baudrate, filters, filetersize);
}

void at_SetfotaURL(char *url)
{
}

void at_setModemConnInfo(mx_accountdata ConnInfo)
{
}

void at_readModemConnInfo(mx_accountdata * ConnInfo)
{
}

OemSimCardStatus at_isSimValid(void)
{
    return SIM_NOT_INSERTED;
}

void at_startFileTransfer(void (*callback)(uint16 result), const char *local, const char *remote, uint8 mode)
{
    
}

void at_stopFileTranser (void)
{
}

void at_queryTransfer(uint8 *progress)
{
}

void at_swmcSessionTrigger(void)
{

}

int16 at_read_ambinent_temperature(void)
{
		return 0;
}

char *at_getModemVersion(void)
{
    return NULL;
}

extern uint8_t FileSystemMountFlag;
uint8 at_update(char *filename)
{
    FRESULT res;
    FIL file;
    FirmwareInfoTypeDef FirmwareInfo;
    char namebuf[MAX_FILE_NAME_LEN] = {0};
    uint8_t ret = 0;
    #define AT_UPDATE_DATA_BUF_SIZE_BYTES (1024)
    uint8_t dataBuf[AT_UPDATE_DATA_BUF_SIZE_BYTES] = {0};
    uint32_t rcount = 0, left = 0, line = 0;
    uint32_t writeaddr = 0;

    memset(&file, 0, sizeof(file));

    if (FileSystemMountFlag == 0)
	{
		MX_FATFS_Init();

		res = f_mount(&EMMCFatFS, EMMCPath, 1);
		if (res == FR_OK)
		{
			FileSystemMountFlag = 1;
		}
		else
		{
			FATFS_UnLinkDriver(EMMCPath);
			API_PRINT("[%s] APP: at_update mount failed: %d",FmtTimeShow(), res);
			return 0;
		}
	}

    namebuf[0] = '0';
	namebuf[1] = ':';
	namebuf[2] = '/';
	memcpy(namebuf + 3, filename, ((strlen((char *)filename) + 4) < sizeof(namebuf)) ? strlen((char *)filename): (sizeof(namebuf) - 4));
	res = f_open(&file, namebuf, FA_READ | FA_OPEN_EXISTING);
    if (res != FR_OK)
	{
		API_PRINT("[%s] APP: at_update Open file(%s) failed: %d",FmtTimeShow(), namebuf, res);
        return 0;
	}

    memset(&FirmwareInfo, 0, sizeof(FirmwareInfo));
    FirmwareInfo.IAPFlag = IAP_BOOT_FLASH;
    FirmwareInfo.FirmwareSize = f_size(&file);
    API_PRINT("[%s] APP: at_update file(%s) size(%d)",FmtTimeShow(), filename, FirmwareInfo.FirmwareSize);
    if (FirmwareReceiveInit(&FirmwareInfo, (char *)filename, EMMC_FIRMWARE_INFO_START_ADDR) != 0)
    {
        ret = 0;
        goto AT_UPDATE_EXIT;
    }

    if (FR_OK != f_lseek(&file, 0))
    {
        API_PRINT("[%s] APP: at_update lseek failed",FmtTimeShow());
        ret = 0;
        goto AT_UPDATE_EXIT;
    }

    left = FirmwareInfo.FirmwareSize;
    writeaddr = EMMC_FIRMWARE_START_ADDR;
    while (left > 0)
    {
        rcount = 0;
        memset(dataBuf, 0, sizeof(dataBuf));
        res = f_read(&file, dataBuf, (left > sizeof(dataBuf)) ? sizeof(dataBuf) : left, &rcount);
        if (res == FR_OK)
        {
            if (rcount != 0)
            {
                if (Emmc_WriteData(dataBuf, rcount, writeaddr))
                {
                    API_PRINT("[%s] APP: at_update emmc write failed",FmtTimeShow());
                    ret = 0;
                    goto AT_UPDATE_EXIT;
                }
                else
                {
                    line++;
                    API_PRINT("APP: at_update emmc write (%d) bytes line(%d)",rcount, line);
                }
                left -= rcount;
                writeaddr += rcount;
            }
            else
            {
                if (left > 0)
                {
                    API_PRINT("[%s] APP: at_update emmc write left(%d)",FmtTimeShow(), left);
                    ret = 0;
                    goto AT_UPDATE_EXIT;
                }
                else
                {
                    break;
                }
            }
        }
        else
        {
            API_PRINT("[%s] APP: at_update file read failed",FmtTimeShow());
            ret = 0;
            goto AT_UPDATE_EXIT;
        }
    }

    if (0 != FirmwareVerifyAndSetUpdateFlag(&FirmwareInfo))
    {
        ret = 10;
        API_PRINT("[%s] APP: at_update firmware verify failed",FmtTimeShow());
        ret = 0;
        goto AT_UPDATE_EXIT;
    }
    else
    {
        ret = 1;
    }

AT_UPDATE_EXIT:
    f_close(&file);

    return ret;
}

/*----------------------------------------------------------------------------*/

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                               End Of The File
*******************************************************************************/
