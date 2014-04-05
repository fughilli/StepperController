/*
 * simple_io.c
 *
 *  Created on: Jul 11, 2012
 *      Author: Kevin
 *
 *
 * Copyright (C) 2012-2014  Kevin Balke
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
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
