#include <dwt.h>

uint32_t DWT_Enable()
{
	uint32_t count;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // Enable core debugger trace

    DWT->CYCCNT = 0;                            // Reset debugger counter trace

    DWT->CTRL &= DWT_CTRL_CYCCNTENA_Msk;       // Enable cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    count = DWT->CYCCNT;

    //dummy instructions
    __asm volatile ("nop\n"
					"nop\n"
					"nop\n");

	//return difference, if DWT started result > 0
	return (DWT->CYCCNT - count);
}
