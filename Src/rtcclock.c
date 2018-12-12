/*******************************************************************************
* File Name          : rtcclock.c
* Author             : Yangjie Gu
* Description        : This file provides all the rtcclock functions.
* History:
*  09/30/2017 : rtcclock V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "rtcclock.h"
#include "uart_api.h"

/* Private define ------------------------------------------------------------*/
#define RTCTimPrintf DebugPrintf

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;

/* Public variables ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const uint8_t norm_month_table[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static char SysDate[30] = {'\0'};

/* Function prototypes -------------------------------------------------------*/
int32_t datetoweek(int32_t yr, int32_t mn, int32_t day)
{
	int32_t i;
	int32_t days = 0;
	int32_t s;
	int32_t week;
	int32_t mont[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	if ((0 == yr % 4 && 0 != yr % 100) || 0 == yr % 400)
		mont[2] = 29;
	else
		mont[2] = 28;
	for (i = 0; i < mn; i++)
		days += mont[i];
	days += day;
	s = yr - 1 + (int32_t)((yr - 1) / 4) - (int32_t)((yr - 1) / 100) + (int32_t)((yr - 1) / 400) + days;
	week = s % 7;

	return week;
}

void DectoBCD(int32_t Dec, uint8_t *Bcd, int32_t length)
{
	int32_t i;
	int32_t temp;
	for (i = length - 1; i >= 0; i--)
	{
		temp = Dec % 100;
		Bcd[i] = ((temp / 10) << 4) + ((temp % 10) & 0x0F);
		Dec /= 100;
	}
}
uint32_t Divide(uint32_t Second, uint32_t value, uint32_t *returnValue)
{
	*returnValue = Second % value;
	return Second / value;
}

int8_t GetMonthFromDays(uint32_t days, uint32_t year, uint32_t *returnDays)
{
	int32_t i = 0;
	uint32_t totalday = 0;
	for (i = 0; i < 12; i++)
	{
		if (days < totalday)
		{
			break;
		}
		if (((year & 0x3) == 0) && (i == 1))
			totalday += 29;
		else
			totalday += norm_month_table[i];
	}
	if (((year & 0x3) == 0) && (i == 2))
		*returnDays = days - totalday + 29 + 1;
	else
		*returnDays = days - totalday + norm_month_table[i - 1] + 1;
	return i;
}

TimeTableT SecondsToTimeTable(uint32_t seconds)
{
	TimeTableT timeTable;
	uint32_t days;

	seconds += BASE_OFFSET;

	//get seconds, minute and hour;
	seconds = Divide(seconds, 60, &timeTable.second);
	seconds = Divide(seconds, 60, &timeTable.minute);
	seconds = Divide(seconds, 24, &timeTable.hour);

	//count how many leap_loop be included;
	uint32_t leap = Divide(seconds, LEAP_LOOP, &days);
	timeTable.year = BASE_YEAR + 4 * leap;

	//get surplus days to determine the appointed year;
	if (days < 366)
	{
	}
	else if (days < (366 + 365))
	{
		timeTable.year += 1;
		days -= (366);
	}
	else if (days < (366 + 365 * 2))
	{
		timeTable.year += 2;
		days -= (366 + 365);
	}
	else if (days < (366 + 365 * 3))
	{
		timeTable.year += 3;
		days -= (366 + 365 * 2);
	}

	timeTable.month = GetMonthFromDays(days, timeTable.year, &timeTable.day);
	return timeTable;
}

uint32_t TimeTableToSeconds(TimeTableT timeTable)
{
	uint32_t seconds = 0;
	int32_t i;

	//get total days from 1980 not included nowyear
	for (i = BASE_YEAR; i < timeTable.year; i++)
	{

		//if %4 != 0;
		if ((i & 0x3) != 0)
		{
			seconds += NORM_YEAR;
		}
		else
		{
			seconds += LEAP_YEAR;
		}
	}
	//get nowyears total days not included this month;
	for (i = 1; i < timeTable.month; i++)
	{
		//if leap year and 2th month;
		if (((timeTable.year & 0x3) == 0) && (i == 2))
			seconds += 29;
		else
			seconds += norm_month_table[i - 1];
	}
	//get this month's days;
	seconds += timeTable.day - 1;
	seconds = seconds * 24 + timeTable.hour;
	seconds = seconds * 60 + timeTable.minute;
	seconds = seconds * 60 + timeTable.second;

	seconds -= BASE_OFFSET;
	return seconds;
}

char *FmtTimeShow(void)
{
	//int32_t week = 0;
	char *tstr = SysDate;
	TimeTableT timeTable = GetRTCDatetime();
	/* Display time Format : hh:mm:ss */
	//week = datetoweek(timeTable.year, timeTable.month, timeTable.day);
	/* Show Time */
	sprintf(tstr, "%02d/%02d/%02d %02d:%02d:%02d",
			timeTable.month,
			timeTable.day,
			(timeTable.year - 2000), /*week, */
			timeTable.hour,
			timeTable.minute,
			timeTable.second);
	return tstr;
}

TimeTableT GetRTCDatetime(void)
{
	TimeTableT timeTable;
	RTC_DateTypeDef sdatestructureget = {0};
	RTC_TimeTypeDef stimestructureget = {0};

	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	/* Display time Format : hh:mm:ss */

	timeTable.year = sdatestructureget.Year + 2000;
	timeTable.month = sdatestructureget.Month;
	timeTable.day = sdatestructureget.Date;
	timeTable.hour = stimestructureget.Hours;
	timeTable.minute = stimestructureget.Minutes;
	timeTable.second = stimestructureget.Seconds;

	return timeTable;
}

typedef unsigned long	DWORD;
void GetRTCTimeForFatFs(DWORD *year, DWORD *mon, DWORD *day, DWORD *hour, DWORD * minute, DWORD *second)
{
	RTC_DateTypeDef sdatestructureget = {0};
	RTC_TimeTypeDef stimestructureget = {0};

	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	/* Display time Format : hh:mm:ss */

	*year = sdatestructureget.Year + 2000;
	*mon = sdatestructureget.Month;
	*day = sdatestructureget.Date;
	*hour = stimestructureget.Hours;
	*minute = stimestructureget.Minutes;
	*second = stimestructureget.Seconds / 2;
}

void GetRTCTimeFromFatFs(TimeTableT *ptimeTable, DWORD tm)
{
	ptimeTable->year = (((tm >> 25) & 0x7f) + 1980);
	ptimeTable->month = (tm >> 21) & 0x0f;
	ptimeTable->day = (tm >> 16) & 0x1f;
	ptimeTable->hour = (tm >> 11) & 0x1f;
	ptimeTable->minute = (tm >> 5) & 0x3f;
	ptimeTable->second = (tm & 0x1f) * 2;
}

int32_t SetRTCDatetime(TimeTableT *tm)
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	// Data
	if (tm->year < 1900)
		DectoBCD(tm->year + 2000, &sDate.Year, 1);
	else
		DectoBCD(tm->year, &sDate.Year, 1);
	DectoBCD(tm->month, &sDate.Month, 1);
	DectoBCD(tm->day, &sDate.Date, 1);
	// Time
	DectoBCD(tm->hour, &sTime.Hours, 1);
	DectoBCD(tm->minute, &sTime.Minutes, 1);
	DectoBCD(tm->second, &sTime.Seconds, 1);
	// Week
	sDate.WeekDay = datetoweek(tm->year, tm->month, tm->day);
	/*if(gDeviceConfig.DbgCtl.RTCDebugInfoEn == TRUE)
	RTCTimPrintf(NRCMD,"SetTime: [0x%x-0x%x-0x%x] [0x%x:0x%x:0x%x] [0x%x]\n", \
	sDate.Year,sDate.Month,sDate.Date, \
	sTime.Hours,sTime.Minutes,sTime.Seconds, \
	sDate.WeekDay);*/
	//Set Time: Hour:Minure:Second
	//sTime.Hours = 0x10;
	//sTime.Minutes = 0x10;
	//sTime.Seconds = 0x00;

	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	/* Set Date: Week Month Day Year */
	//sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
	//sDate.Month = RTC_MONTH_MARCH;
	//sDate.Date = 0x26;
	//sDate.Year = 0x15;
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
	// Check the time
	// CheckAccurateTiming();
	return 0;
}

void SetRTCAlarmTime(uint32_t seconds, uint8_t Status)
{
	uint8_t BCDday;
	uint8_t BCDhour;
	uint8_t BCDmin;
	uint8_t BCDSec;
	uint32_t CurrentSecs = 0;
	TimeTableT Timetable;
	RTC_AlarmTypeDef sAlarm;
	// Get seconds of current time
	CurrentSecs = TimeTableToSeconds(GetRTCDatetime());
	CurrentSecs += seconds;
	if (Status == TRUE)
	{
		RTCTimPrintf(DbgCtl.RTCDebugInfoEn, "\r\n[%s] RTC: Alm Time(%d Secs)", FmtTimeShow(), seconds);
	}
	Timetable = SecondsToTimeTable(CurrentSecs);
	// Converted to BCD Code
	DectoBCD(Timetable.day, &BCDday, 1);
	DectoBCD(Timetable.hour, &BCDhour, 1);
	DectoBCD(Timetable.minute, &BCDmin, 1);
	DectoBCD(Timetable.second, &BCDSec, 1);
	// Print Out
	/*if(gDeviceConfig.DbgCtl.RTCDebugInfoEn == TRUE)
		RTCTimPrintf(DbgCtl.RTCDebugInfoEn,"\r\n[%s] RTC: Tim(%04d-%02d-%02d %02d:%02d:%02d) BCD(0x%02X 0x%02X:0x%02X:0x%02X)", \
			FmtTimeShow(), \
			Timetable.year, \
			Timetable.month, \
			Timetable.day, \
			Timetable.hour, \
			Timetable.minute, \
			Timetable.second, \
			BCDday, \
			BCDhour, \
			BCDmin, \
			BCDSec);*/
	// Set Alarm Values
	sAlarm.AlarmTime.Hours = BCDhour;
	sAlarm.AlarmTime.Minutes = BCDmin;
	sAlarm.AlarmTime.Seconds = BCDSec;
	sAlarm.AlarmTime.SubSeconds = 0x0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = BCDday;
	sAlarm.Alarm = RTC_ALARM_A;
	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD);
}

static const char *monthstr[] = {
	"Err",
	"Jan", "Feb", "Mar", "Apr", "May", "Jun",
	"Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
};

void CompileSetRTCTime(void)
{
	TimeTableT tm = {0};
	char time[16] = {0};
	char date[16] = {0};
	char *timetemp = time;
	char *datetemp = date;
	char month[4] = {0};
	uint8_t i = 0;

	memcpy(time, __TIME__, strlen(__TIME__));
	memcpy(date, __DATE__, strlen(__DATE__));

	tm.hour = (timetemp[0] - '0') * 10 + timetemp[1] - '0';
	tm.minute = (timetemp[3] - '0') * 10 + timetemp[4] - '0';
	tm.second = (timetemp[6] - '0') * 10 + timetemp[7] - '0';

	month[0] = *datetemp++;
	month[1] = *datetemp++;
	month[2] = *datetemp++;
	datetemp++;
	for (i = 0; i < (sizeof(monthstr) / sizeof(*monthstr)); i++)
	{
		if (strcmp(month, monthstr[i]) == 0)
		{
			break;
		}
	}
	tm.month = i;

	tm.day = ((datetemp[0] >= '0' && datetemp[0] <= '3') ? (datetemp[0] - '0') * 10 : 0) + datetemp[1] - '0';
	datetemp += 3;

	tm.year = (datetemp[0] - '0') * 1000 + (datetemp[1] - '0') * 100 + (datetemp[2] - '0') * 10 + datetemp[3] - '0';

	SetRTCDatetime(&tm);
}


/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
