//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//

#include "Timer.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

uint32_t led_flash_countdown = 0;

void
timer_tick(void);

// ----------------------------------------------------------------------------

volatile uint32_t timer_delayCount;
volatile uint32_t stopwatch;

// ----------------------------------------------------------------------------

void timer_start(void) {
	// Use SysTick as reference for the delay loops.
	SysTick_Config(SystemCoreClock / TIMER_FREQUENCY_HZ);
	led_flash_countdown = 0;
}

void stopwatchStart(void) {
	stopwatch = 0;

}

uint32_t stopwatchReport(void) {
	return stopwatch;
}

void timer_sleep(uint32_t ticks) {
	timer_delayCount = ticks;

	// Busy wait until the SysTick decrements the counter to zero.
	while (timer_delayCount != 0u)
		;
}

void timer_tick(void) {
	// Decrement to zero the counter used by the delay routine.
	if (timer_delayCount != 0u) {
		--timer_delayCount;
	}

	// increment stopwatch
	stopwatch++;
	if (led_flash_countdown != 0u) {
		--led_flash_countdown;
	}

}

// ----- SysTick_Handler() ----------------------------------------------------

void SysTick_Handler(void) {
	timer_tick();
}

// ----------------------------------------------------------------------------
