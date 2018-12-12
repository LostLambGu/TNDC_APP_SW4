#ifndef __PFAL_H__
#define __PFAL_H__

//#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#if defined(__cplusplus)
extern "C" {
#endif
#define __arm
#if defined(__arm)
#define long_long               long long
#else
#define long_long               __int64
#endif

#ifdef AT_IMPORT_API
typedef signed char             int8;
typedef signed short            int16;
typedef signed int              int32;
typedef signed long_long        int64;

typedef unsigned char           uint8;
typedef unsigned short          uint16;
typedef unsigned int            uint32;
typedef unsigned long_long      uint64;

typedef unsigned char           bool;
typedef unsigned char           BOOL;
typedef unsigned char           byte;
typedef unsigned short          word;
typedef unsigned int            dword;
typedef unsigned long_long      qword;

typedef unsigned short          wchar;
typedef long_long               longlong;

typedef unsigned char           at_uchar;
typedef unsigned short          at_ushort;
typedef unsigned int            at_ulong;
typedef unsigned long_long      at_ulonglong;
typedef unsigned int            at_uint;
#endif

typedef uint16					tftp_sock;
typedef int16					RegIdT;
typedef uint32					OemValFsiHandleT;


#ifndef M_PI
#define M_PI                    3.14159265358979323846
#endif

#ifndef EARTH_RADIUS
#define EARTH_RADIUS            6371004.0
#endif

#ifndef FALSE
#define FALSE                   ((BOOL)0)
#endif

#ifndef TRUE
#define TRUE                    ((BOOL)1)
#endif

#ifndef MIN
#define MIN(x, y)               (((x) < (y)) ? (x) : (y))
#endif

#ifndef MAX
#define MAX(x, y)               (((x) > (y)) ? (x) : (y))
#endif

#ifndef MID
#define MID(x, y, z)            (((x) > (y)) ? (x) : ((y) < (z)) ? (y) : (z))
#endif

#ifndef ABS
#define ABS(x)                  (((x) > 0) ? (x) : (-(x)))
#endif

#ifndef COUNT_OF
#define COUNT_OF(a)             (sizeof(a) / sizeof((a)[0]))
#endif

#ifndef HEXNUM
#define HEXNUM(x)               (((x) >= 'a') ? (((x) - 'a') + 10) : ((x) >= 'A') ? (((x) - 'A') + 10) : ((x) - '0'))
#endif

#ifndef HEXUPPER
#define HEXUPPER(x)             ("0123456789ABCDEF"[x])
#endif

#ifndef HEXLOWER
#define HEXLOWER(x)             ("0123456789abcdef"[x])
#endif



#define AT_UART_LENGTH          480

#ifdef AT_IMPORT_API
typedef enum
{
   OEM_UART_REV_MSG, 
   OEM_SMS_REV_MSG, 
   OEM_UDP_REV_MSG, 
   OEM_HWD_INT_MSG,
   OEM_TX_RSP_MSG,
   OEM_PPP_UDP_STATUS_MSG,
   OEM_UDP_RECEIVE_DATA,
   OEM_TCP_RECEIVE_DATA,
   OEM_PSW_IN_SERVICE_MSG,
   OEM_AT_CMD_MSG,
   OEM_AT_ACK_MSG,
   OEM_GPS_LOC_INF_MSG,
   OEM_GPS_FIX_STATUS_MSG,
   OEM_UDP_DEMO,
   OEM_ADC_READ_RSP,
   OEM_SEND_DATA_LENGTH,
   OEM_PSW_REGISTRATION_ACCEPT_MSG,
   OEM_GPS_SIGNAL_STATUS_EVENT_MSG,
   OEM_GPS_NMEA_EVENT_MSG,
   OEM_INIT_FINISH_MSG,
   OEM_DM_SESSION_STATE_MSG,
   OEM_PSW_EVT_INCOMING_CALL_ALERT_WITH_INFO_MSG,
   OEM_PSW_EVT_CALL_CONNECTED_MSG,
   OEM_PSW_EVT_CALL_ENDED_MSG,
   OEM_OTASP_EVT_MSG,
   OEM_GPS_RESTART_EVT_MSG,
   OEM_UART_INPUT_DATA_MSG,
   OEM_VIRTUAL_AT_RSP_MSG,
   OEM_UDP_WRITE_EVT_MSG,
   OEM_HEART_BEAT_MSG,
   OEM_RESET_MSG,
   OEM_I2C_WRITE_MSG,
   OEM_UBLOX_MSG,
	 OEM_CAN_MSG,
   OEM_UDP_SOCKET_EVT_MSG,	// BOD
   OEM_UDP_SEND_EVT_MSG,	// BOD
   OEM_NUM_MSG
} OEMMsgIdT;

#define MX_IPV4_LENGTH          4
#define MX_GPRS_LENGTH          32
#define MX_APN_LENGTH           100

typedef struct {
	uint8		authType;
	char		name[MX_GPRS_LENGTH + 1];
	char		username[MX_GPRS_LENGTH + 1];
	char		password[MX_GPRS_LENGTH + 1];
	char		apn[MX_APN_LENGTH + 1];
	uint8		dns[MX_IPV4_LENGTH];
	uint8		dns2[MX_IPV4_LENGTH];
	uint8		pdpType;  //for PDP type (0/1/2 for IPv4/IPv6/IPv4IPv6)
	uint8		roaming_enable;
	char		attach_apn[MX_APN_LENGTH+1];
} mx_accountdata;

typedef struct
{
    double              Longitude;
    double              Latitude;
    double              Heading;
    double              Velocity;
	double              Altitude;
    double              hdop;
	uint16              SatelliteCount;
    bool                bGpsLock;
	
   uint16 Year;
   uint16 Month;
   uint16 DayOfWeek;
   uint16 Day;
   uint16 Hour;
   uint16 Minute;
   uint16 Second;
   uint16 Milliseconds;

   /*; GPS week as the number of whole weeks since GPS time zero*/
   uint32 GPSweek;
   /*; GPS time of week in milliseconds*/
   uint32 GPSTimeOfWeek;

   /*
   validFix.------------------ FALSE---no gps fixed
   WeakSignalvalidFix-----FALSE---signal is weak ----TRUE( validFix is FALSE), means signal is strong, but no fix yet.
   Gps2Dfix_Blocked------TRUE---2D fix, but is blocked,won't send 2D loc out.
   */
   uint8 validFix;
   uint8 WeakSignalvalidFix;
   uint8 Gps2Dfix_Blocked;
   
}OemGpsInfo;

//coypied from inc\sysdefs.h
typedef enum
{
   SYS_MODE_1xRTT  = 0,
   SYS_MODE_EVDO,
   SYS_MODE_GSM,
   SYS_MODE_MAX

} SysAirInterfaceT;

#define SYS_CP_MAX_NEIGHBOR_LIST_PILOTS   40   /* array size for neighbor pilot list */ 

/*from inc\pswapi.h(83):*/
#define PSW_GPS7560_DATA_SIZE_MAX   760


//copied from inc\monapi.h
typedef enum {
   MON_HWD_SLEEPOVER_FLAG       = 0x00000001,  /* HWD: for sleepover ISR */
   MON_HWD_SLEEP_AUDIOPATH_FLAG = 0x00000002,  /* HWD: for audio path in use */
   MON_HWD_GPIO_INT_C108_FLAG   = 0x00000004,  /* HWD: for gpio CKT108 activity */
   MON_HWD_SLEEP_USBDATA_FLAG   = 0x00000008,  /* HWD: for USB data activity */
   MON_IOP_ETS_TX_SUSPEND_FLAG  = 0x00000010,  /* IOP: for ETS Tx Uart activity */
   MON_IOP_ETS_RX_SUSPEND_FLAG  = 0x00000020,  /* IOP: for ETS Rx Uart activity */
   MON_IOP_AP_UART_RX_FLAG      = 0x00000040,  /* IOP: for AP Rx Uart activity */
   MON_BM_CHRG_FLAG             = 0x00000080,  /* MON: battery charge activity */
   MON_HSC_1X_MODEM_ACTIVE_FLAG = 0x00000100,  /* HSC: for 1x modem activity */
   MON_HSC_DO_MODEM_ACTIVE_FLAG = 0x00000200,  /* HSC: for DO modem activity */
   MON_HWD_TS_SLEEP_FLAG        = 0x00000400,  /* MON: Thouch screen is activity */
   MON_HWD_SLEEP_USB_DETECT_FLAG= 0x00000800,  /* MON: For usb detect activity */
   MON_VAL_BLUETOOTH_SLEEP_FLAG        = 0x00001000,  /* val: Bluetooth FLAG */
   MON_HWD_BLUETOOTH_WAKE_UP_FLAG        = 0x00002000,  /*HWD: for bluetooth wake up*/
   MON_IOP_TX_VETO_FLAG        = 0x00004000,  /* MON: For 4 line wakeup */
   MON_VAL_SLEEP_SND_FLAG       = 0x00008000,  /* used for sound play */
   MON_HWD_KEYPAD_ACTION_FLAG   = 0x00010000,  /* HWD: for Keypad activity */
   MON_VAL_LIGHT_FLAG           = 0x00020000,  /* MON: For LCD backlight activity */
   MON_HWD_SLEEP_DSPV_INIT_FLAG = 0x00040000,  /* used for Dspv init */
   MON_HWD_SLEEP_DSPV_BUSY_FLAG = 0x00080000,  /* HWD: for DSPv busy */
   MON_HWD_SLEEP_APRDY_FLAG     = 0x00100000,  /* HWD: for communication with AP */
   MON_HWD_GENERIC_GPINT_FLAG   = 0x00200000,  /* HWD: for generic GPINT activity */
   MON_HWD_HEADSETKEY_FLAG      = 0x00400000,  /* MON: For headset key activity */
   MON_HWD_SLEEP_UIM2_FLAG      = 0x00800000,  /* MON: for UIM2 activity*/
   MON_HWD_SLEEP_UIM_FLAG       = 0x01000000,  /* HWD: for UIM activity */
   MON_HWD_SLEEP_VIBRATE_FLAG   = 0x02000000,  /* HWD: for vibrate, not used in fact */
   MON_HWD_SLEEP_MMAPPS_FLAG    = 0x04000000,  /* HWD: for multi-media Apps */
   MON_HWD_SLEEP_AUXADC_FLAG    = 0x08000000,  /* HWD: for aux adc conversions */

   MON_LEC_GPS_ACTIVITY_FLAG    = 0x10000000,  /* LEC: for GPS activity */
   MON_EEP_GPS_ACTIVITY_FLAG    = 0x20000000,  /* EEP: for GPS activity */

   MON_IOP_RX_VETO_FLAG       = 0x40000000   /* MON: For 4 line wakeup */
   /* don't use > 0x7FFF FFFF */
} MonDeepSleepVetoT;

//copied from inc\pswvalapi.h
typedef enum
{
  VAL_PSW_IN_SERVICE,
  VAL_PSW_NO_SERVICE,
  VAL_PSW_OOSA,
  VAL_PSW_CP_DISABLED
} ValPswServiceStatusT;

//OEM_RESET_MSG reason, from inc/hwdiramdbm.h file
#define HWD_IRAM_JUMP_TO_BOOT_FLAG 0x34567890 
#define HWD_IRAM_CP_RESET_FLAG   0x98653846

//\mnt\tftputils.h(150):
typedef struct {
  uint16 port;
  uint16 type;
  uint16 mode;
  uint16 dataSize;  
}tftpconfig;

#endif

typedef enum 
{
    OEM_TCPIP_PPP_INACTIVE,
    OEM_TCPIP_PPP_CONNECTING,
    OEM_TCPIP_PPP_CONNECTED,
    OEM_TCPIP_PPP_CLOSING,
    OEM_TCPIP_PPP_INDORMANCY
}OemPppStatus;

typedef enum 
{
    OEM_PPPUDP_NET_ISCONN_STATUS,
    OEM_PPPUDP_NET_NONET_STATUS,
    OEM_PPPUDP_NET_INDORMANCY_STATUS,
    OEM_PPPUDP_UDP_SOCKET_CONNECT,
    OEM_PPPUDP_UDP_SOCKET_CLOSE,
    OEM_PPPSOCKET_TCP_SOCKET_CONNECT,
    OEM_PPPSOCKET_TCP_SOCKET_CLOSE,
    OEM_PPPUDP_PPP_UDP_STATUS_MAX
}OemPppUdpStatus;

typedef enum
{
   OEM_FSI_SUCCESS         = 0, /*No errors. Function completed successfully.*/
   OEM_FSI_ERR_PARAMETER   = 1, /*Incorrect parameter to the function.*/
   OEM_FSI_ERR_READ        = 2, /*file read operation is failed.*/
   OEM_FSI_ERR_WRITE       = 3, /*file write operation is failed.*/
   OEM_FSI_ERR_SYSTEM      = 4, /*Indicates that a system error has occurred.*/
   OEM_FSI_ERR_EXIST       = 5, /*The specified object has existed already.*/
   OEM_FSI_ERR_NOTEXIST    = 6, /*No matched object in specified media.*/
   OEM_FSI_ERR_EOF         = 7, /*file pointer reaches the end-of-file.*/
   OEM_FSI_ERR_FULL        = 8, /*Flash device is full*/
   OEM_FSI_ERR_NOSUPPORT   = 9, /*FSI does not support this function now .*/
   OEM_FSI_ERR_FORMAT      = 10, /*Volume is in the incorrect format.*/
   OEM_FSI_ERR_ACCESS_DENY = 11, /*Insufficient permissions to access object.*/
   
   /* reach to a limitation of the maximum number of the files that can be open
   simultaneously.*/
   OEM_FSI_ERR_MAX_OPEN    = 12, 
   OEM_FSI_ERR_TIMEOUT     = 13,
   OEM_FSI_ERR_INIT        = 14,
   OEM_FSI_ERR_MEMORY      = 15,
   OEM_FSI_ERR_ACCESS      = 16,
   OEM_FSI_ERR_MOUNTED     = 17,
   OEM_FSI_ERR_UNMOUNTED   = 18,
   OEM_FSI_ERR_UNKNOWN     = 255 /*Other unknowned error occar*/
} OemValFsiResultT;

typedef enum 
{
	OEM_FSI_FILE_OPEN_READ_EXIST     = 0,
	OEM_FSI_FILE_OPEN_WRITE_EXIST    = 1,
	OEM_FSI_FILE_OPEN_CREATE_NEW     = 2,
	OEM_FSI_FILE_OPEN_CREATE_ALWAYS  = 3,
	OEM_FSI_FILE_OPEN_WRITE_ALWAYS   = 4,
	OEM_FSI_FILE_OPEN_SHARE   = 5
} OemValFsiFileOpenModeT;

typedef enum 
{ 
	OEM_FSI_FILE_SEEK_START   = 1,
	OEM_FSI_FILE_SEEK_END     = 2,
	OEM_FSI_FILE_SEEK_CURRENT = 3
} OemValFsiFileSeekTypeT;

typedef enum
{
  OEM_NET_SUCCESS,   /* The operation was a success. */
  OEM_NET_EBADF,     /* Bad file number */
  OEM_NET_EFAULT,    /*  Bad address     */
  OEM_NET_EWOULDBLOCK,   /*    Operation would block */
  OEM_NET_EAFNOSUPPORT,  /*  Address family not supported by protocol. */
  OEM_NET_EPROTOTYPE,    /*  Protocol wrong type for socket */
  OEM_NET_ESOCKNOSUPPORT,/*  Socket type not supported. */
  OEM_NET_EPROTONOSUPPORT,/* Protocol not supported */
  OEM_NET_EMFILE,         /* Too many open files. */
  OEM_NET_EOPNOTSUPP,     /* Operation not supported on transport endpoint. */
  OEM_NET_EADDRINUSE,     /* Address already in use. */
  OEM_NET_EADDRREQ,       /* Destination address required */
  OEM_NET_EINPROGRESS,    /* Operation now in progress */
  OEM_NET_ESHUTDOWN,    /* Socket was closed */
  OEM_NET_EISCONN,         /* 	Transport endpoint is already connected. */
  OEM_NET_EIPADDRCHANGED,  /* Remote address changed. */
  OEM_NET_ENOTCONN,       	/* Transport endpoint is not connected. */
  OEM_NET_ECONNREFUSED,   	/* Connection refused. */ 
  OEM_NET_ETIMEDOUT,      	/* Connection timed out. */
  OEM_NET_ECONNRESET,     	/* Connection reset by peer. */
  OEM_NET_ECONNABORTED,   	/* Software caused connection abort. */
  OEM_NET_ENETDOWN,       	/* Network is down. */
  OEM_NET_EPIPE,          	/* Broken pipe. */
  OEM_NET_EMAPP,          	/* No mapping found. */
  OEM_NET_EBADAPP,        	/* RegId invalid. */
  OEM_NET_ESOCKEXIST,      	/* The socket doesn't exist. */
  OEM_NET_EINVAL,         	/* Invalid argument. */
  OEM_NET_EMSGSIZE,       	/* Message too long. */
  OEM_NET_EEOF,           	/* End of file reached. */
  OEM_NET_EHOSTNOTFOUND,  	/* The host wasn't found. */
  OEM_NET_ETRYAGAIN,      	/* Try again. */
  OEM_NET_ENORECOVERY,    	/* Can't recover from error. */
  OEM_NET_ENOADDRESS,     	/* No address given. */
  OEM_NET_SUCCESS_END,      /*Success and have no more data left to be sent to val*/
  OEM_NET_ENETEXIST
} OemNetResultT;

typedef enum 
{
  OEM_NET_EVT_SOCKET,
  OEM_NET_EVT_NET,
  OEM_NET_EVT_DNS_LOOKUP,
  OEM_NET_EVT_RTP_CREATE_STATUS,
  OEM_NET_EVT_RTP_DATA_RECEIVED,
  OEM_NET_EVT_MAX
} OemValNetEventIdT;

typedef void (*OEM_APP_CallBackT)(RegIdT id, OemValNetEventIdT EventId, void *EventMsgP);

typedef struct
{
	char		content[AT_UART_LENGTH + 1];
	uint16		mode;
	uint16		param;
} at_atcdata;

typedef struct
{
  uint32 year;
  uint32 month;
  uint32 day;
  uint32 hour;
  uint32 minute;
  uint32 second;
}OemTimeTbl;

typedef struct
{
  uint16      channel;          /* Current channel number */
  uint32      sid;                /* System ID */
  uint32      nid;                /* Network ID */
  uint32      baseId;            /* Base station ID */
  uint16      mcc;               /* Mobile Country Code (from extended system parameter message)*/
  uint8       mnc;        /* Mobile Network Code (from extended system parameter message)*/
  int16       rxPower;          /* Rx power */   
  int16       txPower;          /* Tx power */
  uint8       uBand;
  uint8       Csq;
	uint8				roaming;
	uint8				rssi;
 }OemPswInfo;

typedef struct
{
   int16   socketid;
   int16   len;
   uint8*  DataBufP;
   uint32  address;
   int16   port;
}OemUdpModuleReceiveDataT;

typedef struct
{
   uint8 PppUdpStatus;
   uint8 Socket_Num;
}OemUdpModuleReceiveStatusT;


typedef struct
{
        char * messagep;
        char * numberp;
}OemSmsRecvT;

typedef struct
{
   uint8   ChanId;
   uint16          MeasResult;
   uint8           Status;
} OemHwdAdcMeasResponseMsgT;

typedef enum
{
   OEM_VAL_GPS_START,
   OEM_VAL_GPS_END,
   OEM_VAL_GPS_INPRORESS,
} OemValGpsStatusEventE;

typedef enum
{
   OEM_SESS_CLOSE_USER=50,
   OEM_AP_DUN_DISCONNECTED,
   OEM_E911_CALL,
   OEM_VAL_SESS_ERR_STATE,
   OEM_AP_DATA_DISCONNECTING, 
   OEM_RES_AT_CMD=60,
   OEM_LOC_AT_CMD,
   OEM_MSS_TIMER_EXP,
   OEM_START_POS_RESP_TIMER_EXP=70,
   OEM_SESS_DONE_IND,
   OEM_CANCEL_NI_TRACKING_SESS,
   OEM_MPC_CANCELL,
   OEM_MPC_START_ERR,
   OEM_REQ_NOT_AUTH_OR_REFUSED=75,
   OEM_SERVICE_NOT_AVAIL,
   OEM_NET_ERR_STATE,
   OEM_SOCKET_ERR_STATE,
   OEM_NET_REGID_ERR,
   OEM_PPP_OPEN_STATE_ERR=80,
   OEM_PPP_ERROR_STATE,
   OEM_ERR_POS_REPORT_RESP,
   OEM_TCP_CLOSE_EVENT,
   OEM_CANCEL_BY_USER,

   OEM_EXP_60_SEC=100,
   OEM_FIX_DONE=200,
   OEM_FIX_FAIL,
   OEM_NUM_SESS_CLOSE
}OemValGpsEndReasonE;

typedef struct
{
    uint32 InstanceID;
    uint32 FixNum;
    uint32 TotalFixNum;
    OemValGpsStatusEventE event;
    OemValGpsEndReasonE status;
    
}OemValGpsFixStatusMsgT;

typedef  struct
{
	BOOL			bActivated;
	BOOL			bConnected;
	BOOL			bPending;
	char			destination[64];
	uint16			localPort;
	uint16			type;

} OemSocketStatusT;

typedef  struct
{
	uint8				ServiceStatus;
	uint8				ServiceType;
	uint8				Roam;				/* ROAM Status							*/
	uint16				Band;				/* Current Operating Band				*/
	uint16				Channel;			/* Current Channel Number				*/
	uint8				Mode;				/* current mode: PCS/CellularAnalog		*/
	uint8				Block;				/* current CDMA block (if CDMA system)	*/
	uint8				ServingSystem;		/* Serving System/Block                 */
	uint16				SysID;				/* Last-Received System ID  (sid)		*/
	uint16				LocArea;			/* Current Location Area ID (nid)		*/
	uint16				PilotPn;			/* PILOT_PN								*/ 
	uint8				pRevInUse;			/* CDMA Protocol Revision that MS uses	*/
} OemPswServiceMsgT;


typedef struct 
{
   uint16 DataLen;          
   uint8  Data[PSW_GPS7560_DATA_SIZE_MAX];
} OemPswGpsNmeaStreamMsgT;


typedef struct {
	uint16 nghbrPilotMCC;
	uint8  nghbrPilotMNC;
	uint16 nghbrPilotCELLID;
	uint16 nghbrPilotSINR;
} OemNghbrPilotData;

typedef struct {
    char* h_name;
    int16 h_addrtype;
    /*the length of each address, always 4 here*/
    int16 h_length;
    /*ended with a null pointer*/
    char ** h_addr_list;    
} OemNetHostEntT;

typedef enum
{
  OEM_PSW_IN_SERVICE,
  OEM_PSW_NO_SERVICE,
  OEM_PSW_OOSA,
  OEM_PSW_CP_DISABLED
} OemServiceStatusT;

typedef enum
{
  OEM_PSW_CDMA_SERVICE_TYPE,
  OEM_PSW_AMPS_SERVICE_TYPE
} OemUIPswServiceT;
typedef struct
{
  OemServiceStatusT    ServiceStatus;
  OemUIPswServiceT     ServiceType;
  uint8                Roam;          /* ROAM Status                          */
  uint16               Band;          /* Current Operating Band               */
  uint16               Channel;       /* Current Channel Number               */
  uint8                Mode;          /* current mode: PCS/CellularAnalog     */
  uint8                Block;         /* current CDMA block (if CDMA system)  */
  uint8                ServingSystem; /* Serving System/Block                 */
  uint16               SysID;         /* Last-Received System ID  (sid)       */
  uint16               LocArea;       /* Current Location Area ID (nid)       */
  uint16               PilotPn;       /* PILOT_PN  */ 
  uint8                pRevInUse;                 /* CDMA Protocol Revision that MS uses  */
} OemValPswServiceMsgT;


typedef  struct
{
    bool RoamIndPresent;
    uint8 RoamInd;
} OemValNwkRptRegistrationAcceptMsgT;

typedef enum
{
    OEM_OTA_PROGRAMMING_STARTED = 1,  
    OEM_OTA_PROGRAMMINGLOCK_UNLOCKED,
    OEM_OTA_NAMPARAM_DOWNLOADED_OK,
    OEM_OTA_MDN_DOWNLOADED_OK,
    OEM_OTA_IMSI_DOWNLOADED_OK,
    OEM_OTA_PRL_DOWNLOADED_OK,
    OEM_OTA_CIMMIT_SUCCESSFUL,
    OEM_OTA_PROGRAMMING_SUCCESSFUL,
    OEM_OTA_PROGRAMMING_UNSUCCESSFUL,
    OEM_OTA_NULL
} OemOtaMsgT;
 
 typedef struct
 {
    uint16   baseId;
    int32    baseLat;
    int32    baseLong;
 } OemBSCoOrdMsgT;

 typedef enum
{
    OMA_LOG_OFF          = 0xFFFF,
    OMA_LOG_ON           = 0xFCFC,
    OMA_LOG_NULL         = 0xCCCC 
}OemOMALOGStates;
typedef enum
{
    OEM_SMS_DEL_UIM= 0,
    OEM_SMS_DEL_FLASH,
    OEM_SMS_DEL_ALL 
}OemSMSDelMemT;

typedef enum
{
    OEM_PARITY_ENABLE      = 0x08,
    OEM_PARITY_EVEN        = 0x10,
    OEM_PARITY_SET         = 0x20 
}OemParity_Type;

typedef enum {

	DM_STATE_NOTIFY_HFA_START = 0,		//hfa start
	DM_STATE_NOTIFY_HFA_END,				//hfa end
	DM_STATE_NOTIFY_HFA_FAIL,			//hfa fail
	DM_STATE_NOTIFY_CIDC_START,			//CIDC start
	DM_STATE_NOTIFY_CIDC_END,			//CIDC end
	DM_STATE_NOTIFY_CIDC_FAIL,			//CIDC fail
	DM_STATE_NOTIFY_CIPRL_START,			//CIPRL start
	DM_STATE_NOTIFY_CIPRL_END,			//CIPRL  end
	DM_STATE_NOTIFY_CIPRL_FAIL,			//CIPRL  fail
	DM_STATE_NOTIFY_CIFUMO_START,		//FUMO start
	DM_STATE_NOTIFY_CIFUMO_END,			//FUMO end
	DM_STATE_NOTIFY_CIFUMO_SUCCESS,		//FUMO succeeded
	DM_STATE_NOTIFY_CIFUMO_FAIL,			//FUMO failed
	DM_STATE_NOTIFY_NI_START,			//NI start
	DM_STATE_NOTIFY_NI_END,				//NI end
	DM_STATE_NOTIFY_NI_FAIL				//NI end
} DM_SessionStateType;


typedef enum
{ // These values MUST match the TLV-protocol for the error code! Error-code field is only 1-byte
	OEM_TFTP_SUCCESS,
	OEM_TFTP_FILE_NOT_FOUND,
	OEM_TFTP_ACCESS_VIOLATION,
	OEM_TFTP_DISK_FULL,
	OEM_TFTP_ILLEGAL_OPERATION,
	OEM_TFTP_UNKNOWN_TID,
	OEM_TFTP_FILE_EXIST,
	OEM_TFTP_NO_SUCH_USER,
	OEM_TFTP_BUSY,
	OEM_TFTP_INTERRUPT,
	OEM_TFTP_ILLEGAL_ARGUMENT,
	OEM_TFTP_NETWORK_ERROR,
	OEM_TFTP_TIMEOUT,
	OEM_TFTP_INVALID_FILE	= 99
} OEM_TFTP_ERROR_CODE_e;

typedef enum {
	SIM_NOT_INSERTED = 0,
	SIM_READY,
	SIM_PIN,
	SIM_PUK,
	SIM_PIN2,
	SIM_PUK2,
	SIM_FAILURE,
	PH_NET_PIN,
	PH_NET_PUK,
	SIM_INVALID,
	SIM_UNKNOWN
} OemSimCardStatus;

typedef struct
{
    DM_SessionStateType type ;
} DmSessionMsgT;

typedef struct {
       uint8         major;
       uint8         minor;
       uint8         revision;
       uint8         carrier;            // 1 Sprint, 2 Verizon ???
       uint16        build;               
       uint8         reserve[64];
} version_t;


typedef struct {
    uint32  id;
	uint32  mask;
	uint8   mode;
	uint8   handle;
} CANFILTER_T;

#if defined(AT_IMPORT_API)

#define AT_API_DEFINITION
#include "api.h"
#undef AT_API_DEFINITION

#define AT_API_DECLARATION
#include "api.h"
#undef AT_API_DECLARATION

// custom
#define at_initCustom1          init
#define at_initCustom2          init
#define at_initCustom3          init
#define at_initCustom4          init

#else /* defined(AT_IMPORT_API) */

#define AT_API_DEFINITION
#include "api.h"
#undef AT_API_DEFINITION

#define AT_API_DECLARATION
#include "api.h"
#undef AT_API_DECLARATION

#include "api.h"

//typedef void (*at_oementry)(const at_oemapi_v1 *oemapi);
typedef void at_oementry(const at_oemapi_v1 *oemapi);

// system
void        at_dispatch(void);
void        at_init(void);
void        at_start(void);

// custom
void        at_initCustom1(void);
void        at_initCustom2(void);
void        at_initCustom3(void);
void        at_initCustom4(void);

#endif /* defined(AT_IMPORT_API) */

#if defined(__cplusplus)
}
#endif

#endif // __PFAL_H__
