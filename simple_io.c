/*
 * simple_io.c
 *
 *  Created on: Jul 11, 2012
 *      Author: Kevin
 */
#include "simple_io.h"
#include <MSP430.h>

void delay(long ms){
	while(ms > 0){						// For each millisecond counted
		delay_ticks(CLOCK_TIME_MS);		// Delay one ms' worth of clock cycles
		ms--;
	}
}

void delay_ticks(long ticks){
	ticks /= 2;						// Divide by two
	while(ticks > 0){					// For each two ticks counted
		ticks--;						// Waste two CPU instructions (compare, decrement)
	}
}
