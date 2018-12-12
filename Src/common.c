/*******************************************************************************
* File Name          : common.c
* Author             : Yangjie Gu
* Description        : This file provides all the common functions.

* History:
*  08/07/2018 : common V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "common.h"

#include "cmsis_os.h"

#include "uart_api.h"

/* Private define ------------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/* Extern variables ----------------------------------------------------------*/
#if TNDC_DEBUG_LOG_LOCK_MUTEX
extern SemaphoreHandle_t xOemDebugPrintMutex;
#elif TNDC_DEBUG_LOG_LOCK_BINARY
extern SemaphoreHandle_t xOemDebugPrintBinary;
#endif

// DelayUsTime
void DelayUsTime(uint16_t delay_usec)
{
	volatile uint16_t temp = 0;
	
	for(;delay_usec > 0;delay_usec--)
	{
		for(temp=0;temp < 6;temp++)
		{
			;
		}
	}
}

// DelayMsTime
void DelayMsTime(uint16_t delay_time)
{
	uint16_t i;

	if (delay_time == 0)
		return;

	for (i = 0; i < delay_time; i++)
	{
		DelayUsTime(1000);
	}
}

uint8_t ToUperChar(uint8_t ch)
{
   if( ch >='a' && ch <= 'z' )
      return (uint8_t)( ch + ('A' - 'a') );
   return ch;
}

void StringToUper(char* s)
{
   uint16_t len = strlen(s);
   uint16_t idx;

   for(idx = 0; idx < len; idx++)
	s[idx] = ToUperChar(s[idx]);
}

void SystemDisableAllInterrupt(void)
{
	// portDISABLE_INTERRUPTS();
	// portENTER_CRITICAL();
	// __asm volatile( "cpsid i" );
	__disable_irq();
}

void SystemEnableAllInterrupt(void)
{
	// portENABLE_INTERRUPTS();
	// portEXIT_CRITICAL();
	// __asm volatile( "cpsie i" );
	__enable_irq();
}


void hard_fault_handler_c (unsigned int * hardfault_args)
{
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;
	unsigned int r_sp;
//	char *read = (char *)0x20000000ul;
//	char out[3] = {0};
//	char MsgId = 0;
//	unsigned int i = 0;

	r_sp = __get_PSP(); // Get SP Value

#if TNDC_DEBUG_LOG_LOCK_MUTEX
	xOemDebugPrintMutex = NULL;
#elif TNDC_DEBUG_LOG_LOCK_BINARY
	xOemDebugPrintBinary = NULL;
#endif

	stacked_r0 = ((unsigned long)hardfault_args[0]);
	stacked_r1 = ((unsigned long)hardfault_args[1]);
	stacked_r2 = ((unsigned long)hardfault_args[2]);
	stacked_r3 = ((unsigned long)hardfault_args[3]);

	stacked_r12 = ((unsigned long)hardfault_args[4]);
	stacked_lr = ((unsigned long)hardfault_args[5]);
	stacked_pc = ((unsigned long)hardfault_args[6]);
	stacked_psr = ((unsigned long)hardfault_args[7]);

	PrintfBeforeRTOS("\r\n%s", "######################");
	switch (__get_IPSR())
	{
	case 3:
		PrintfBeforeRTOS("\r\n%s", "Hard Fault");
		break;

	case 4:
		PrintfBeforeRTOS("\r\n%s", "Memory Manage");
		break;

	case 5:
		PrintfBeforeRTOS("\r\n%s", "Bus Fault");
		break;

	case 6:
		PrintfBeforeRTOS("\r\n%s", "Usage Fault");
		break;

	default:
		PrintfBeforeRTOS("\r\nUnknown Fault %ld", __get_IPSR());
		break;
	}
	PrintfBeforeRTOS("\r\n,corrupt,dump registers:\n\r");

	PrintfBeforeRTOS("\r\nR0 = 0x%08x", stacked_r0);
	PrintfBeforeRTOS("\r\nR1 = 0x%08x", stacked_r1);
	PrintfBeforeRTOS("\r\nR2 = 0x%08x", stacked_r2);
	PrintfBeforeRTOS("\r\nR3 = 0x%08x", stacked_r3);
	PrintfBeforeRTOS("\r\nR12 = 0x%08x", stacked_r12);
	PrintfBeforeRTOS("\r\nLR [R14] = 0x%08x  subroutine call return address\r\n", stacked_lr);
	PrintfBeforeRTOS("\r\nPC [R15] = 0x%08X  program counter\r\n", stacked_pc);
	PrintfBeforeRTOS("\r\nPSR = 0x%08X\r\n", stacked_psr);

	PrintfBeforeRTOS("\r\nSP(0x%08x)", r_sp);
	PrintfBeforeRTOS("\r\nSCB->VTOR(0x%08x)", SCB->VTOR);
	PrintfBeforeRTOS("\r\nSCB->CFSR(0x%08x)", SCB->CFSR);
	PrintfBeforeRTOS("\r\nSCB->HFSR(0x%08x)", SCB->HFSR);
	PrintfBeforeRTOS("\r\nSCB->DFSR(0x%08x)", SCB->DFSR);
	PrintfBeforeRTOS("\r\nSCB->MMFAR(0x%08x)", SCB->MMFAR);
	PrintfBeforeRTOS("\r\nSCB->BFAR(0x%08x)", SCB->BFAR);
	PrintfBeforeRTOS("\r\nSCB->AFSR(0x%08x)\r\n", SCB->AFSR);

    PrintfBeforeRTOS("\r\n\r\n");
    PrintfBeforeRTOS("\r\n\r\n");
    // for (i = 0;(unsigned long)read < (0x20000000ul + 0x10000);i++)
    // {
    //   MsgId = *read;
    //   out[0] = (((MsgId & 0xf0) >> 4) < 0x0a) ? ((MsgId & 0xf0) >> 4) + '0' : ((MsgId & 0xf0) >> 4) - 10 + 'a';
    //   out[1] = (((MsgId & 0x0f) >> 0) < 0x0a) ? ((MsgId & 0x0f) >> 0) + '0' : ((MsgId & 0x0f) >> 0) - 10 + 'a';
    //   out[2] = 0;
    //   read++;
    //   PrintfBeforeRTOS("%s",out);
    // }

	while (1)
		;
}

void GetExclusiveLock(volatile unsigned int * pLock)
{
    volatile int status;
	
    do
    {
        while (__LDREXW(pLock) != 0);
        status = __STREXW(1, pLock);
    } while (status != 0);
    __DMB();
    return;
}

void FreeExclusiveLock(volatile unsigned int * pLock)
{
    __DMB();
    *pLock = 0;
    return;
}

