/*
 * SysTick.c
 *
 *  Created on: Aug 18, 2025
 *      Author: jtori
 */


#include "SysTick.h"
#include "hardware.h"

#define FREQ2TICKS(x) (__CORE_CLOCK__/(x)) - 1

static void (*sysTickCallback)(void) = 0;

bool SysTick_Init (void (*funcallback)(void), uint64_t freqHz)
{
	static bool initialized = false;

	SysTick->CTRL = 0x00;
	SysTick->LOAD = FREQ2TICKS(freqHz);
	SysTick->VAL = 0x00;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

	if (!initialized)
	{
		initialized = true;
		if (funcallback != 0)
		{
			sysTickCallback = funcallback;
			return true;
		}
	}
	return false;
}


__ISR__ SysTick_Handler()
{
	sysTickCallback();
}
