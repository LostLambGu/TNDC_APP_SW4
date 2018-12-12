#define AT_API(r, n, p)                 \
r n p;

#define AT_RESERVED(i, c)

#define AT_API_BEGIN

#define AT_API_END

#ifdef AT_IMPORT_API
#define AT_IMPORT_API_HACKED
#undef AT_IMPORT_API
#endif


AT_API_BEGIN

// entry
AT_API(void,        at_startTftp,       (void (*callback)(uint16 result), const char *local, const char *remote, const char *domain, const tftpconfig *config))
AT_API(void,        at_stopTftp,        (void))
AT_API(void,        at_queryTftp,		(uint16 *state, uint16 *block))
AT_API(bool,        at_verify,			(char *path))
AT_RESERVED(1, 6)		// Total should be 10

#if defined(AT_IMPORT_API)
#define at_startTftp                    (at_oemapi->_at_startTftp)
#define at_stopTftp                     (at_oemapi->_at_stopTftp)
#define at_queryTftp                    (at_oemapi->_at_queryTftp)
#define at_verify						(at_oemapi->_at_verify)
#endif /* defined(AT_IMPORT_API) */

// heartbeat
AT_RESERVED(2, 5)		// Total should be 5

#if defined(AT_IMPORT_API)
#endif /* defined(AT_IMPORT_API) */

// command
AT_API(void,        at_swmcSessionTrigger,        (void))
AT_API(void,        at_swmcSessionGpsTrigger,      (void))
AT_RESERVED(3, 8)		// Total should be 10

#if defined(AT_IMPORT_API)
#define at_swmcSessionTrigger			(at_oemapi->_at_swmcSessionTrigger)
#define at_swmcSessionGpsTrigger		(at_oemapi->_at_swmcSessionGpsTrigger)
#endif /* defined(AT_IMPORT_API) */

// atc
AT_RESERVED(4, 10)		// Total should be 10

#if defined(AT_IMPORT_API)
#endif /* defined(AT_IMPORT_API) */

// gps
AT_API(void,        at_closeGps,        (void))
AT_API(void,        at_suspendGps,      (void))
AT_API(void,        at_GpsStart,        (uint8  fixmode, uint32 TimeBFixes))
AT_API(void,        at_GpsResume,       (void))
AT_API(void,        at_GpsRestart,      (void))
AT_API(void,        at_gps_powersave_mode_set,        (uint32 LabTestMode, uint32 LabTestParam))
AT_API(void,        at_GpsResetAssitDataMode,        (uint8 mode))
AT_API(void,		at_getIgpsVersion,			(uint8 * buf))
//AT_API(bool,        at_GpsConfigFixRateSet,      (uint32 InstanceID, uint32 NumFixes, uint32 TimeBFixes))
AT_RESERVED(5, 2)							// Total should be 10

#if defined(AT_IMPORT_API)
#define at_closeGps				(at_oemapi->_at_closeGps)
#define at_suspendGps			(at_oemapi->_at_suspendGps)
#define at_GpsStart				(at_oemapi->_at_GpsStart)
#define at_GpsResume			(at_oemapi->_at_GpsResume)
#define at_GpsRestart			(at_oemapi->_at_GpsRestart)
#define at_gps_powersave_mode_set             (at_oemapi->_at_gps_powersave_mode_set)
#define at_GpsResetAssitDataMode			(at_oemapi->_at_GpsResetAssitDataMode)
#define at_getIgpsVersion			(at_oemapi->_at_getIgpsVersion)
//#define at_GpsConfigFixRateSet			(at_oemapi->_at_GpsConfigFixRateSet)
#endif /* defined(AT_IMPORT_API) */

// call
AT_API(bool,     at_AnswerCall,      ( void ))
AT_API(bool,     at_HangupCall,      ( void ))
AT_API(bool,     at_OrigVoiceCall,   ( char*   pDialstr))
AT_API(bool,     at_CallWait,        ( bool    bEnable))
AT_API(bool,     at_HookFlash,       ( uint8*  KeypadFac))
AT_API(bool,     at_SendBurstDTMF,   ( char*  pDtmf,uint8 DurOn,uint8 DurOff ))
AT_API(bool,     at_SendContDTMF,    ( char*  pDtmf,uint8 mode))
AT_RESERVED(6, 3)							// Total should be 10

#if defined(AT_IMPORT_API)
#define at_AnswerCall               (at_oemapi->_at_AnswerCall)
#define at_HangupCall               (at_oemapi->_at_HangupCall)
#define at_OrigVoiceCall            (at_oemapi->_at_OrigVoiceCall)
#define at_CallWait                 (at_oemapi->_at_CallWait)
#define at_HookFlash                (at_oemapi->_at_HookFlash)
#define at_SendBurstDTMF            (at_oemapi->_at_SendBurstDTMF)
#define at_SendContDTMF             (at_oemapi->_at_SendContDTMF)
#endif /* defined(AT_IMPORT_API) */

// message
AT_API(bool,        at_sendMessage,     (char *Num, char* buf))
AT_API(RegIdT,      at_GetSMSRegisterId,(void))
AT_API(bool,        at_DeleteAllSMS,     (OemSMSDelMemT mem))

AT_RESERVED(7, 7)		// Total should be 10

#if defined(AT_IMPORT_API)
#define at_sendMessage				(at_oemapi->_at_sendMessage)
#define at_GetSMSRegisterId			(at_oemapi->_at_GetSMSRegisterId)
#define at_DeleteAllSMS			(at_oemapi->_at_DeleteAllSMSs)
#endif /* defined(AT_IMPORT_API) */

// ppp and socket
AT_API(void,			at_UdpIpSocketOpen,		(uint8 Socket_Num, uint16 LocalPort, char* DestAddrP, uint16 PortNum))
AT_API(void,			at_UdpIpSocketClose,	(uint8 Socket_Num))
AT_API(void,			at_UdpIpSocketSendData,	(uint8 Socket_Num, char* buffer, uint16 len))
AT_API(void,			at_TcpIpPppOpen,		(void))
AT_API(void,			at_TcpIpPppClose,		(void))
AT_API(OemPppStatus,	at_TcpIpGetPppStatus,	(void))
AT_API(void,        at_TcpIpSocketOpen,        (uint8 Socket_Num, uint16 LocalPort,char* DestAddrP, uint16 PortNum))
AT_API(void,        at_TcpIpSocketClose,        (uint8 Socket_Num))
AT_API(void,        at_TcpIpSocketSendData,        (uint8 Socket_Num,char* buffer, uint16 len))
AT_API(OemNetResultT,    at_DNSQueryMsg,            (char *address, OEM_APP_CallBackT appFunc, RegIdT regId))
AT_API(uint32,      at_NetNToHl,               (uint32 netlong))
AT_API(void,        at_IpAddrInt2Char,          (uint32 IntAddr, char* pCharIpAddr))
AT_API(void,        at_TcpIpSetUdpAddrPort,            (uint8 Socket_Num, char * addr, uint16 port))
AT_API(OemNetResultT, at_UdpIpSocketSendDataNew,             (uint8 Socket_Num, char* buffer, uint16* len))

AT_RESERVED(8, 6)		// Total should be 20

#if defined(AT_IMPORT_API)
#define at_UdpIpSocketOpen				(at_oemapi->_at_UdpIpSocketOpen)
#define at_UdpIpSocketClose				(at_oemapi->_at_UdpIpSocketClose)
#define at_UdpIpSocketSendData			(at_oemapi->_at_UdpIpSocketSendData)
#define at_TcpIpPppOpen					(at_oemapi->_at_TcpIpPppOpen)
#define at_TcpIpPppClose				(at_oemapi->_at_TcpIpPppClose)
#define at_TcpIpGetPppStatus			(at_oemapi->_at_TcpIpGetPppStatus)
#define at_TcpIpSocketOpen        (at_oemapi->_at_TcpIpSocketOpen)
#define at_TcpIpSocketClose        (at_oemapi->_at_TcpIpSocketClose)
#define at_TcpIpSocketSendData         (at_oemapi->_at_TcpIpSocketSendData)
#define at_DNSQueryMsg                  (at_oemapi->_at_DNSQueryMsg)
#define at_NetNToHl                   (at_oemapi->_at_NetNToHl)
#define at_IpAddrInt2Char                 (at_oemapi->_at_IpAddrInt2Char)
#define at_TcpIpSetUdpAddrPort                 (at_oemapi->_at_TcpIpSetUdpAddrPort)
#define at_UdpIpSocketSendDataNew                 (at_oemapi->_at_UdpIpSocketSendDataNew)
#endif /* defined(AT_IMPORT_API) */

// heap
AT_API(void*,		at_malloc,			(uint32 size))
AT_API(void,		at_mfree,			(void* Ptr))
AT_RESERVED(9, 3)		// Total should be 5

#if defined(AT_IMPORT_API)
#define at_malloc						(at_oemapi->_at_malloc)
#define at_mfree						(at_oemapi->_at_mfree)
#endif /* defined(AT_IMPORT_API) */

// conversion
AT_RESERVED(10, 10)		// Total should be 10

#if defined(AT_IMPORT_API)
#endif /* defined(AT_IMPORT_API) */

// string
AT_API(int,         at_sprintf,         (char *buffer, const char *format, ...))
AT_API(int,         at_vsprintf,        (char *buffer, const char *format, va_list arg))
AT_API(uint32,		at_NTOH32,			(uint32 val))
AT_RESERVED(11, 22)		// Total should be 25

#if defined(AT_IMPORT_API)
#define at_sprintf						(at_oemapi->_at_sprintf)
#define at_vsprintf						(at_oemapi->_at_vsprintf)
#define at_NTOH32						(at_oemapi->_at_NTOH32)
#endif /* defined(AT_IMPORT_API) */

// file
AT_API(OemValFsiResultT,	at_fopen,		(OemValFsiHandleT *FileP, const char *FileNameP, OemValFsiFileOpenModeT Mode))
AT_API(OemValFsiResultT,	at_fclose,		(OemValFsiHandleT File))
AT_API(OemValFsiResultT,	at_fread,		(void *BufferP, uint32 ItemSize, uint32 *ItemNumP, OemValFsiHandleT File))
AT_API(OemValFsiResultT,	at_fwrite,		(void *BufferP, uint32 ItemSize, uint32 *ItemNumP, OemValFsiHandleT File))
AT_API(OemValFsiResultT,	at_fseek,		(OemValFsiHandleT File, OemValFsiFileSeekTypeT SeekFrom, int32 MoveDistance))
AT_API(OemValFsiResultT,	at_ftell,		(OemValFsiHandleT File, uint32 *PosP))
AT_API(OemValFsiResultT,	at_remove,		(const char *NameP))
AT_API(OemValFsiResultT,	at_rename,		(const char *OldNameP, const char *NewNameP))
AT_API(OemValFsiResultT,	at_fgetLength,	(const char *NameP, uint32 *FileLengthP))
AT_RESERVED(12, 16)		// Total should be 25

#if defined(AT_IMPORT_API)
#define at_fopen						(at_oemapi->_at_fopen)
#define at_fclose						(at_oemapi->_at_fclose)
#define at_fread						(at_oemapi->_at_fread)
#define at_fwrite						(at_oemapi->_at_fwrite)
#define at_fseek						(at_oemapi->_at_fseek)
#define at_ftell						(at_oemapi->_at_ftell)
#define at_extend						(at_oemapi->_at_extend)
#define at_remove						(at_oemapi->_at_remove)
#define at_rename						(at_oemapi->_at_rename)
#define at_fgetLength					(at_oemapi->_at_fgetLength)
#endif /* defined(AT_IMPORT_API) */

// unit
AT_RESERVED(13, 5)		// Total should be 5

#if defined(AT_IMPORT_API)
#endif /* defined(AT_IMPORT_API) */

// output
AT_API(void,	at_printfDebug,		(const char *Fmt, ...))
AT_API(void,	at_vprintfDebug,	(const char *Fmt, va_list arg))
AT_API(void,	at_OutputString,	(char *buf))
AT_API(void, at_VirtualATCmd,      (char *cmd, uint16 len))
AT_API(void, at_Uart2Config,         (uint32 rate))
AT_API(bool,	at_OutputStringUart2,	(char *buf,uint16 length))
AT_API(void,	at_Uart2Chan,         (uint32 chan))
AT_API(bool,		at_Uart2Setbit,	   (uint8 bit,uint8 stopbit ,uint8 Parity))
AT_RESERVED(14, 2)		// Total should be 10


#if defined(AT_IMPORT_API)
#define at_printfDebug			(at_oemapi->_at_printfDebug)
#define at_vprintfDebug			(at_oemapi->_at_vprintfDebug)
#define at_OutputString			(at_oemapi->_at_OutputString)
#define  at_VirtualATCmd                (at_oemapi->_at_VirtualATCmd)
#define  at_Uart2Config                   (at_oemapi->_at_Uart2Config)
#define at_OutputStringUart2			(at_oemapi->_at_OutputStringUart2)
#define at_Uart2Chan			(at_oemapi->_at_Uart2Chan)
#define at_Uart2Setbit                   (at_oemapi->_at_Uart2Setbit)
#endif /* defined(AT_IMPORT_API) */

// storage
AT_RESERVED(15, 10)		// Total should be 10

#if defined(AT_IMPORT_API)
#endif /* defined(AT_IMPORT_API) */

// system
AT_API(void,		at_reset,					(void))
AT_API(void,		at_readImei,				(uint8 * buf))
AT_API(void,		at_readImsi,				(uint8 * buf))
AT_API(uint16,		at_readBsid,				(void))
AT_API(void,		at_readBsidWithNeighbor,	(OemNghbrPilotData  nghbrdata[], uint8 *num))
AT_API(uint16,		at_getSrvType,				(void))
AT_API(void,		at_getBinVersion,			(uint8 * buf))
AT_API(void,		at_writeAccount,			(const char* username,const char* passwd))
AT_API(void,		at_setWorkMode,				(BOOL PowerUpCtrl))
AT_API(uint8,		at_getGetRegStat,			(void))
AT_API(uint32,		at_MonDeepSleepSuspend,		(uint8 Interface,uint32 BitMask))
AT_API(uint32,		at_MonDeepSleepResume,		(uint8 Interface,uint32 BitMask))
AT_API(char *,	    at_readICCID,				(void))
AT_API(void,        at_initTimers,				(uint16 timers))
AT_API(void,        at_killTimers,				(void))
AT_API(void,        at_startTimer,				(uint16 timer, uint32 delay, void (*handler)(uint16 timer)))
AT_API(void,        at_stopTimer,				(uint16 timer))
AT_API(bool,           at_IPAddrCheck,      (char * buf))
AT_API(void,           at_SetRTCTimer,                  (uint8 time))
AT_API(OemNetResultT,     at_NetPppDormantReq,      ( void))
AT_API(bool,     at_HFANeedStart,      ( void))
AT_API(void,     at_HFARTN,      ( void))
AT_API(uint16,     at_GetECIO,      ( void))
AT_API(void,     at_GetBSCoOrdInfo,      ( OemBSCoOrdMsgT *BSCoOrd))
AT_API(void,		at_getBinVersionEx,			(version_t * buf))
//AT_RESERVED(16, 0)		// Total should be 25

#if defined(AT_IMPORT_API)
#define at_reset					(at_oemapi->_at_reset)
#define at_readImei					(at_oemapi->_at_readImei)
#define at_readImsi					(at_oemapi->_at_readImsi)
#define at_readBsid					(at_oemapi->_at_readBsid)
#define at_readBsidWithNeighbor		(at_oemapi->_at_readBsidWithNeighbor)
#define at_getSrvType				(at_oemapi->_at_getSrvType)
#define at_getBinVersion			(at_oemapi->_at_getBinVersion)
#define at_writeAccount				(at_oemapi->_at_writeAccount)
#define at_setWorkMode				(at_oemapi->_at_setWorkMode)
#define at_getGetRegStat			(at_oemapi->_at_getGetRegStat)
#define at_MonDeepSleepSuspend		(at_oemapi->_at_MonDeepSleepSuspend)
#define at_MonDeepSleepResume		(at_oemapi->_at_MonDeepSleepResume)
#define at_readICCID				(at_oemapi->_at_readICCID)
#define at_initTimers				(at_oemapi->_at_initTimers)
#define at_killTimers				(at_oemapi->_at_killTimers)
#define at_startTimer				(at_oemapi->_at_startTimer)
#define at_stopTimer				(at_oemapi->_at_stopTimer)
#define at_IPAddrCheck              (at_oemapi->_at_IPAddrCheck)
#define at_SetRTCTimer              (at_oemapi->_at_SetRTCTimer)
#define at_NetPppDormantReq         (at_oemapi->_at_NetPppDormantReq)
#define at_HFANeedStart             (at_oemapi->_at_HFANeedStart)
#define at_HFARTN                   (at_oemapi->_at_HFARTN)
#define at_GetECIO                  (at_oemapi->_at_GetECIO)
#define at_GetBSCoOrdInfo           (at_oemapi->_at_GetBSCoOrdInfo)
#define at_getBinVersionEx			(at_oemapi->_at_getBinVersionEx)
#endif /* defined(AT_IMPORT_API) */

// device
AT_API(bool,		at_readGpio,		        (uint32 GpioNum))
AT_API(void,		at_writeGpio,		        (uint32 GpioNum, bool value))
AT_API(int,		    at_CPSetFotaFlag,	        (OemOMALOGStates OMA_Log_States))
AT_API(int,		    at_FlashDeltaFileErase,		(void))
AT_API(int,		    at_FlashDeltaFileWrite,		(uint32 Offset, const uint8 *data_ptr, uint32 NumBytes))
AT_API(void,		at_I2cInit,    		        (void))
AT_API(void,		at_I2cLock,    		        (void))
AT_API(void,		at_I2cUnlock,    		    (void))
AT_API(bool,		at_I2cWrite,		        (uint16 DeviceAddr, uint16 SubAddress,  uint8 *DataPtr, uint16 NumBytes, uint16 ConfigBits))
AT_API(bool,	    at_I2cRead,		            (uint16 DeviceAddr, uint16 SubAddress, uint8 *DataPtr, uint16 NumBytes, uint16 ConfigBits))
AT_API(void,		at_UartSetBaudRate,		    (uint32 rate))
AT_API(bool,		at_UartSetbit,	     	    (uint8 bit,uint8 stopbit ,uint8 Parity))
AT_API(void,		at_WatchdogKill,		    (void))
AT_API(void,		at_DisableInterrupts,		(void))
AT_RESERVED(17, 6)		// Total should be 20

#if defined(AT_IMPORT_API)
#define at_readGpio						(at_oemapi->_at_readGpio)
#define at_writeGpio					(at_oemapi->_at_writeGpio)
#define at_CPSetFotaFlag                (at_oemapi->_at_CPSetFotaFlag)
#define at_FlashDeltaFileErase          (at_oemapi->_at_FlashDeltaFileErase)
#define at_FlashDeltaFileWrite          (at_oemapi->_at_FlashDeltaFileWrite)
#define at_I2cInit                      (at_oemapi->_at_I2cInit)
#define at_I2cLock                      (at_oemapi->_at_I2cLock)
#define at_I2cUnlock                    (at_oemapi->_at_I2cUnlock)
#define at_I2cWrite                     (at_oemapi->_at_I2cWrite)
#define at_I2cRead                      (at_oemapi->_at_I2cRead)
#define at_UartSetBaudRate              (at_oemapi->_at_UartSetBaudRate)
#define at_UartSetbit                   (at_oemapi->_at_UartSetbit)
#define at_WatchdogKill                 (at_oemapi->_at_WatchdogKill)
#define at_DisableInterrupts            (at_oemapi->_at_DisableInterrupts)
#endif /* defined(AT_IMPORT_API) */

// time
AT_API(int,				at_getTimeZone,		(void))
AT_API(OemTimeTbl,		at_getTime,			(void))
AT_RESERVED(18, 8)		// Total should be 10

#if defined(AT_IMPORT_API)
#define at_getTimeZone				(at_oemapi->_at_getTimeZone)
#define at_getTime					(at_oemapi->_at_getTime)
#endif /* defined(AT_IMPORT_API) */

// lcd
AT_RESERVED(19, 10)		// Total should be 10

#if defined(AT_IMPORT_API)

#endif /* defined(AT_IMPORT_API) */

// audio
AT_RESERVED(20, 10)		// Total should be 10

#if defined(AT_IMPORT_API)
#endif /* defined(AT_IMPORT_API) */

// sim
AT_RESERVED(21, 5)		// Total should be 10

#if defined(AT_IMPORT_API)
#endif /* defined(AT_IMPORT_API) */

// C lib functions
AT_API(void,		at_assert,			(int expression))
AT_API(int,			at_atoi,			(const char * string))
AT_RESERVED(22, 8)		// Total should be 10

#if defined(AT_IMPORT_API)
#define at_assert				(at_oemapi->_at_assert)
#define at_atoi					(at_oemapi->_at_atoi)
#endif /* defined(AT_IMPORT_API) */

// Register Call back functions
AT_API(void,			at_msgHandleCallBack,		(void (*pRecvCallbackP)(uint32 MessageId, void *MsgBufferP, uint32 size)))
AT_RESERVED(23, 9)		// Total should be 10

#if defined(AT_IMPORT_API)
#define at_msgHandleCallBack						(at_oemapi->_at_msgHandleCallBack)
#endif /* defined(AT_IMPORT_API) */


/*oem api*/
AT_API(OemPswInfo,		at_OemPswInfoGet,				(void))
AT_API(OemNetResultT,	at_ValNetGetIpAddress,			(uint32* IpAddrP))
AT_API(void,			at_GetAdcValue,				    (uint8 Channel))
AT_API(void,			at_getNetworkInfo,				(uint8 * buf))
//AT_API(bool,            at_ValGetNmeaData,              (void* data,uint16 size))
AT_API(void,            at_SwitchToDbg,                 (void))
AT_API(uint16,          at_AdcToMv,                     (uint16 channel, uint16 AdcValue))
AT_API(void,            at_PswPowerCtrl,                (bool bPowerUp))
AT_API(int16,            at_read_rf_pa_temperature,   (void))
AT_API(int16,            at_read_ambinent_temperature,   (void))
AT_RESERVED(24, 1)		// Total should be xx

AT_API(void,			 at_setModemConnInfo,             (mx_accountdata ConnInfo))
AT_API(void,			 at_readModemConnInfo,             (mx_accountdata * ConnInfo))
AT_API(OemSimCardStatus,			 at_isSimValid,					  (void))
AT_API(void,        at_startFileTransfer,       (void (*callback)(uint16 result), const char *local, const char *remote, uint8 mode))
AT_API(void,        at_stopFileTranser,        (void))
AT_API(void,        at_queryTransfer,		(uint8 *progress))
AT_RESERVED(17, 1)		// Total should be 32

AT_API(uint16,           at_WatchdogKick,                 (void))
AT_API(void,             at_WatchdogStart,                 (uint16 dur))
AT_API(void,             at_WatchdogKill,                 (void))


/*
at_startFileTransfer(...)
remote: remote url, for example http://backend.server.com/update/custom.rom
local: local file name on the device for example: /mnt/jfffs2/customer/custom_temp.com
mode:  0: Transfer file from server to the device. 1: Transfer file from device to server. 
*callback: the same function as tftp for callback result. We will resue the TFTP result code. 
*/

void at_Uart2ReConfig(uint32 rate, uint8 parity, uint8 stopbits);
/////////////////////////////////////////////////////////////////
// Parameter: 
// rate: baud rate
// parity: 0: NONE, 1: ODD, 2: EVEN
// stopbits:  1: 1 stop bits, 2: 2 stop bits


//Extra consideration:
//1. at_setWorkMode: to set default modem on/off at initial startup.

void at_SetfotaURL(char *url);
//Set fot package download link: https://192.168.1.15/Package 
//Package is a information file following GemTek Spec


void at_mountSIM(bool bMount);

void at_CANInitEx(uint32 baudrate, CANFILTER_T *filters, uint8 filetersize);
char *at_getModemVersion(void);

#if defined(AT_IMPORT_API)

#define at_OemPswInfoGet					(at_oemapi->_at_OemPswInfoGet)
#define at_ValNetGetIpAddress				(at_oemapi->_at_ValNetGetIpAddress)
#define at_GetAdcValue						(at_oemapi->_at_GetAdcValue)
#define at_getNetworkInfo					(at_oemapi->_at_getNetworkInfo)
//#define at_ValGetNmeaData                   (at_oemapi->_at_ValGetNmeaData)
#define at_SwitchToDbg                          (at_oemapi->_at_SwitchToDbg)
#define at_AdcToMv                             (at_oemapi->_at_AdcToMv)
#define at_PswPowerCtrl                     (at_oemapi->_at_PswPowerCtrl)
#define at_read_rf_pa_temperature    (at_oemapi->_at_read_rf_pa_temperature)
#endif /* defined(AT_IMPORT_API) */

AT_API_END

#undef AT_API_BEGIN

#undef AT_API_END

#undef AT_API

#undef AT_RESERVED

