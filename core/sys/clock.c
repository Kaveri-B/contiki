/**
 * Copyright (c) 2014, Analog Devices, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 * - Neither the name of Analog Devices, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \author Jim Paris <jim.paris@rigado.com>
 */

#include <services/tmr/adi_tmr.h>
#include <sys/etimer.h>

/* timer device instance ID */
#define TIMER_DEVICE_ID   2
#define CLOCK_RATE        (26000000/4)  /* HF has default clock divider of 4 */


/* desired interrupt rate is 1ms */
#define INTERRUPT_RATE    (1000)
#define PRELOAD_VALUE     (CLOCK_RATE/INTERRUPT_RATE)

static volatile clock_time_t current_clock = 0;
static volatile unsigned long current_seconds = 0;
static unsigned int second_countdown = CLOCK_SECOND;
static uint8_t aDeviceMemory0[ADI_TMR_MEMORY_SIZE];
static volatile bool_t bTimeOutFlag;

#define SAMPLE_STACK_POINTER
#ifdef SAMPLE_STACK_POINTER
volatile uint32_t *__min_sampled_sp = (uint32_t *)0xFFFFFFFF;
#endif


static void GPTimer2Callback(void *pCBParam, uint32_t Event, void *pArg)
{
  switch(Event)
  {
      case ADI_TMR_EVENT_TIMEOUT:
        {
          current_clock++;

          if(etimer_pending() && etimer_next_expiration_time() <= current_clock) {
            etimer_request_poll();
          }

          if (--second_countdown == 0) {
            current_seconds++;
            second_countdown = CLOCK_SECOND;		
          }
        }
        break;
      case ADI_TMR_EVENT_CAPTURED:
        break;
     default:
        break;
  }
}
/*---------------------------------------------------------------------------*/
void
clock_init()
{
 /* assume a return code */
    ADI_TMR_RESULT result = ADI_TMR_SUCCESS;
    ADI_TMR_HANDLE hDevice;
    do
    {  
       
      result = adi_tmr_Open(TIMER_DEVICE_ID,aDeviceMemory0,ADI_TMR_MEMORY_SIZE,&hDevice);
      if (ADI_TMR_SUCCESS == result)
      {  
          result = adi_tmr_RegisterCallback( hDevice, GPTimer2Callback ,hDevice);
      }
      if(ADI_TMR_SUCCESS == result)
      {
        result = adi_tmr_SetClockSource(hDevice, ADI_TMR_CLOCK_HFOSC);
      }
      if( ADI_TMR_SUCCESS == result )
      {
        result = adi_tmr_SetPrescaler(hDevice, ADI_GPT_PRESCALER_1); 
      }    
      
      if (ADI_TMR_SUCCESS == result)
      {  
          result = adi_tmr_SetLoadValue( hDevice, PRELOAD_VALUE);
      }
      
      if (ADI_TMR_SUCCESS == result)
      {  
          result = adi_tmr_SetRunMode( hDevice, ADI_TMR_PERIODIC_MODE);
      }
      if (ADI_TMR_SUCCESS == result)
      {  
          result = adi_tmr_SetCountMode(hDevice, ADI_TMR_COUNT_DOWN);
      }
      
      //adi_tmr_EnableReloading(hDevice,true);
      
      if (ADI_TMR_SUCCESS == result)
      {  
          result = adi_tmr_Enable(hDevice, true );
      }
    }while(0);
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  return current_clock;
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
	return current_seconds;
}
/*---------------------------------------------------------------------------*/
void
clock_delay(unsigned int d)
{
  /* Does not do anything. */
}
/*---------------------------------------------------------------------------*/
void
clock_time_update ( uint32_t count )
{
  current_clock += ( count*CLOCK_SECOND );
  current_seconds += count;
}

void
clock_time_update_ms ( uint32_t count )
{
    while(count)
    {
      current_clock ++;
      
      if (--second_countdown == 0) {
        current_seconds++;
        second_countdown = CLOCK_SECOND;		
      }
      count--;
    }
}
