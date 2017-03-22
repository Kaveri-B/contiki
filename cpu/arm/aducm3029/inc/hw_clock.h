/** \addtogroup sys
 * @{
 */

/**
 * \defgroup clock Clock library
 *
 * The clock library is the interface between Contiki and the platform
 * specific clock functionality. The clock library performs a single
 * function: measuring time. Additionally, the clock library provides
 * a macro, CLOCK_SECOND, which corresponds to one second of system
 * time.
 *
 * \note The clock library need in many cases not be used
 * directly. Rather, the \ref timer "timer library" or the \ref etimer
 * "event timers" should be used.
 *
 * \sa \ref timer "Timer library"
 * \sa \ref etimer "Event timers"
 *
 * @{
 */

/*
 * Copyright (c) 2004, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: clock.h,v 1.11 2009/01/24 15:20:11 adamdunkels Exp $
 */
#ifndef __CLOCK_H__
#define __CLOCK_H__

//#include "contiki-conf.h"

#if 0 /* XXX problems with signedness and use in timer_expired(). #if:ed it out for now. */
/**
 * Check if a clock time value is less than another clock time value.
 *
 * This macro checks if a clock time value is less than another clock
 * time value. This macro is needed to correctly handle wrap-around of
 * clock time values.
 *
 */
#define CLOCK_LT(a, b) ((clock_time_t)((a) - (b)) < ((clock_time_t)(~((clock_time_t)0)) >> 1))
#endif /* 0 */

#define TIMER0  0
#define TIMER1  1

typedef enum {
	ONE_SHOT = 0,
	REPEATING	
} timer_mode_t;

/**
 * Initialize the clock library.
 *
 * This function initializes the clock library and should be called
 * from the main() function of the system.
 *
 */
void tmr_init(void);

/**
 * Get the current clock time.
 *
 * This function returns the current system clock time.
 *
 * \return The current clock time, measured in system ticks.
 */
uint32_t get_current_time(void);

void clock_delay(unsigned int);

int StartTimer(u32 expiry_time);
int StopTimer(u32 timer_index);

/**
 * A second, measured in system clock time.
 *
 * \hideinitializer
 */
 
/* 

This value is used to convert microseconds to ticks calculated based on the timer interval

eg: if timer frequency is 32000 Hz timer interval is (1 / 32000) * PRESCALAR

*/
 
#define ONE_TICK_IN_us				(125)
 
#ifdef CLOCK_CONF_SECOND
#define CLOCK_SECOND CLOCK_CONF_SECOND
#else
#define CLOCK_SECOND (clock_time_t)32
#endif

int clock_fine_max(void);
unsigned short clock_fine(void);

unsigned long clock_seconds(void);
uint8_t tmr_rand(void);


#endif /* __CLOCK_H__ */

/** @} */
/** @} */
