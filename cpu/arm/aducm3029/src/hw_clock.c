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

#include "common.h"
#include <services/int/adi_int.h>
#include "critical_fn.h"
#include "hw_clock.h"
#include "hw_tmr.h"    
#include <services/gpio/adi_gpio.h>
#include <services/tmr/adi_tmr.h>


#define TIMER_MAX_VALUE	0xFFFF


typedef union
{
  uint32_t u32_latest_timer_expiry;
  uint16_t u16_latest_timer_expiry[2];
}latest_timer_expiry_t;

latest_timer_expiry_t latest_timer_expiry;

volatile uint16_t Core_Time_Tick=0;
volatile uint16_t ms_tick_calc = 0;
bool_t timer_present = false;
static ADI_TMR_HANDLE hTimer0,hTimer1;

static uint8_t aDeviceMemory0[ADI_TMR_MEMORY_SIZE];

static uint8_t aDeviceMemory1[ADI_TMR_MEMORY_SIZE];

uint16_t get_elapsed_time();

static void Tmr0_Int_Callback( void *pCBParam, uint32_t Event, void *pArg );

static void Tmr1_Int_Callback( void *pCBParam, uint32_t Event, void *pArg );

/*---------------------------------------------------------------------------*/

void Tmr0_Int_Callback( void *pCBParam, uint32_t Event, void *pArg )
{
    Core_Time_Tick++;
    if (Core_Time_Tick == 0)
        ms_tick_calc++;
    //adi_gpio_Toggle(ADI_GPIO_PORT0,ADI_GPIO_PIN_12);
    if(true == timer_present)
    {
        if(Core_Time_Tick == latest_timer_expiry.u16_latest_timer_expiry[1])
        {
            if(0 == latest_timer_expiry.u16_latest_timer_expiry[0])
            {                       
                /* Invoke Callback Function */
                Timer0_Callback(0,NULL);
            
                timer_present = false;
            }
            else
            {
                /* Start Timer1 in Priodic mode */
                adi_tmr_Enable(hTimer1, true);
            }
        }
    }
}

void Tmr1_Int_Callback( void *pCBParam, uint32_t Event, void *pArg )
{
  /* Invoke Callback Function */
  Timer0_Callback(0,NULL); 
  
  /* Disable Timer1 */
  adi_tmr_Enable(hTimer1, false);
    
  timer_present = false;
}
/*---------------------------------------------------------------------------*/
ADI_TMR_RESULT Init_timer(ADI_TMR_HANDLE *hDevice, 
                          void *handle_mem,
                          uint8_t device_num,
                          ADI_CALLBACK   const   pfCallback)
{
    
    ADI_TMR_RESULT result;
    
    result = adi_tmr_Open(device_num,handle_mem,ADI_TMR_MEMORY_SIZE,hDevice);
    
    if (ADI_TMR_SUCCESS == result)
        result = adi_tmr_RegisterCallback( *hDevice, pfCallback ,*hDevice);

    if (ADI_TMR_SUCCESS == result)
        result = adi_tmr_SetClockSource(hDevice, ADI_TMR_CLOCK_HFOSC);
    
    if (ADI_TMR_SUCCESS == result)
        result = adi_tmr_SetCountMode(*hDevice, ADI_TMR_COUNT_UP);
    
    if (ADI_TMR_SUCCESS == result)
        result = adi_tmr_SetPrescaler(*hDevice, ADI_GPT_PRESCALER_16);
    
    return result;
}
void tmr_init()
{
    ADI_TMR_RESULT result;
    
    result = Init_timer(&hTimer0, aDeviceMemory0, 0,Tmr0_Int_Callback);
    
    if (ADI_TMR_SUCCESS == result)
        result = Init_timer(&hTimer1, aDeviceMemory1, 1, Tmr1_Int_Callback);

    if (ADI_TMR_SUCCESS == result)
        result = adi_tmr_SetRunMode( hTimer0, ADI_TMR_FREE_RUNNING_MODE);
    
    if (ADI_TMR_SUCCESS == result)
        adi_tmr_SetLoadValue(hTimer0, 0);
    
    if (ADI_TMR_SUCCESS == result)
        result = adi_tmr_SetRunMode( hTimer1, ADI_TMR_PERIODIC_MODE);
    
    if (ADI_TMR_SUCCESS == result)
        adi_tmr_Enable(hTimer0, true);
}
/*---------------------------------------------------------------------------*/
uint32_t get_current_time(void)
{
	return ((((uint32_t)Core_Time_Tick) << 16) + get_elapsed_time());
}

uint32_t get_time_difference(uint32_t previous_time)
{
    uint32_t current_time = get_current_time();
    
    if(current_time - previous_time > 0x7FFFFFFF)
        return ((0xFFFFFFFF - previous_time) + current_time);
    else
        return (current_time - previous_time);
}
/*---------------------------------------------------------------------------*/
uint16_t get_elapsed_time()
{
    uint16_t timer_value;
    adi_GPT_GetCurrentValue(hTimer0, &timer_value);
    return timer_value;
}

int StopTimer(u32 timer_index)
{
  timer_present = false;
  
  return 0;
}

int StartTimer(u32 expiry_time)
{  
  
  uint32_t curr_time;
  
  /* Disable Interrupt */
  adi_int_EnterCriticalRegion();
  
  latest_timer_expiry.u32_latest_timer_expiry = expiry_time;
  timer_present = true;
  
  if(Core_Time_Tick == latest_timer_expiry.u16_latest_timer_expiry[1])
  {
      curr_time = get_current_time() & 0x0000FFFF;  
      
      if(curr_time >= latest_timer_expiry.u16_latest_timer_expiry[0])
      {         
        /* Invoke Callback Function */
        //Timer0_Callback(0,NULL);
        adi_tmr_SetLoadValue(hTimer1,(TIMER_MAX_VALUE - 20));
        
        //timer_present = false;
      }
      else
      {          
        /* Start Timer1 in Priodic mode by loading TxLD Register
         with a value of (TIMER_MAX_VALUE - u16_latest_timer_expiry[0])*/
         adi_tmr_SetLoadValue(hTimer1,(TIMER_MAX_VALUE - (latest_timer_expiry.u16_latest_timer_expiry[0] - curr_time)));
         adi_tmr_Enable(hTimer1, true);
      }
  }
  else
  {      
      /*Load TxLD Register with a value of (TIMER_MAX_VALUE - u16_latest_timer_expiry[0])*/
      adi_tmr_SetLoadValue(hTimer1, (TIMER_MAX_VALUE - latest_timer_expiry.u16_latest_timer_expiry[0]));
  }
    
  /* Enable Interrupt */
  adi_int_ExitCriticalRegion();
  
  return 0;
}

void hw_clock_init(void)
{
    tmr_init();
}
uint32_t get_current_time_us(void)
{
    uint64_t c_time = get_current_time() * 16;
	return (c_time / 26);
}

/*---------------------------------------------------------------------------*/
