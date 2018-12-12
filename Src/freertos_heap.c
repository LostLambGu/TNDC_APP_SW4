
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "stm32l4xx_hal.h"
/* Allocate the memory for the heap. */
#if( configAPPLICATION_ALLOCATED_HEAP == 1 )

#if   defined ( __CC_ARM )
    uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__((at(0x10000000)));
#elif defined ( __ICCARM__ )
    #pragma location=0x10000000
	uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#elif defined ( __GNUC__ )
    // uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__((address (0x10000000)));
	// uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__((section("._user_freerots_heap_stack")));
    // uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#else
    #error FreeRTOS ucHeap allocation fail
#endif 

#endif /* configAPPLICATION_ALLOCATED_HEAP */
