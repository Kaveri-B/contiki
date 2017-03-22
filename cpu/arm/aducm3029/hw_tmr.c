/** \file hw_tmr.c
 **
 ** \brief Implements APIs and structures for use by Hardware
 *    Timer Module.
 **
 ** \cond STD_FILE_HEADER
 **
 ** COPYRIGHT(c) 2010-11 Procubed Technology Solutions Pvt Ltd.
 ** All rights reserved.
 **
 ** THIS SOFTWARE IS PROVIDED BY "AS IS" AND ALL WARRANTIES OF ANY KIND,
 ** INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR USE,
 ** ARE EXPRESSLY DISCLAIMED.  THE DEVELOPER SHALL NOT BE LIABLE FOR ANY
 ** DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE. THIS SOFTWARE
 ** MAY NOT BE USED IN PRODUCTS INTENDED FOR USE IN IMPLANTATION OR OTHER
 ** DIRECT LIFE SUPPORT APPLICATIONS WHERE MALFUNCTION MAY RESULT IN THE DIRECT
 ** PHYSICAL HARM OR INJURY TO PERSONS. ALL SUCH IS USE IS EXPRESSLY PROHIBITED.
 **
 *******************************************************************************
 **  \endcond
 */

/*******************************************************************************
* File inclusion
*******************************************************************************/
#include "telec_common.h"
#include "queue_latest.h"
#include "list_latest.h"
#include "sys/clock.h"
#include "hw_tmr.h"
#include "sw_timer.h"

/*
** ============================================================================
** Private Macro definitions
** ============================================================================
*/

/* None*/

/*
** ============================================================================
** Private Structures, Unions & enums Type Definitions
** ============================================================================
*/

/* None*/

/*
** ============================================================================
** Private Variable Definitions
** ============================================================================
*/

static uint8_t rnd_index = 0;

static uint8_t rnd_array[] =
{
    0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57, 0x74,
    0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e, 0xa6,
    0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89, 0x9d,
    0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d, 0x0f,
    0xd9, 0x3c, 0xa8, 0x35, 0xe5, 0x8f, 0x2a, 0xcd,
    0x29, 0x23, 0xb0, 0x5b, 0x1b, 0xa3, 0x2a, 0x82,
    0xae, 0x0f, 0x84, 0xcd, 0x49, 0xd6, 0x83, 0x42,
    0xf1, 0x8a, 0xfe, 0x21, 0x1b, 0x19, 0x7e, 0xe4
};

/*
** ============================================================================
** Public Variable Definitions
** ============================================================================
*/

hw_tmr_t* mp_hw_tmr_inst = NULL;

/*
** ============================================================================
** External Variable Declarations
** ============================================================================
*/

/*None*/

/*
** ============================================================================
** Private Function Prototypes
** ============================================================================
*/

//static p3time_t ConvertCyclesToMicrosecond( p3time_t from_cycles );
//
//static p3time_t ConvertMicrosecondToCycles( p3time_t from_sys_time );

/*
** ============================================================================
** Public Function Definitions
** ============================================================================
*/

void hw_tmr_init( void *hw_tmr_ins,  void *sw_tmr_mod_ins )
{
	mp_hw_tmr_inst = (hw_tmr_t*)hw_tmr_ins;

    clock_init();
}
/******************************************************************************/

void hw_tmr_uninit( void *hw_tmr_ins,  void *sw_tmr_mod_ins )
{

}

/******************************************************************************/
//#pragma section ("L1_code")
p3time_t hw_tmr_get_time( void *hw_tmr_ins )
{
	p3time_t current_time;

    current_time = get_current_time();

	return current_time;
}

/******************************************************************************/

void hw_tmr_start( void *hw_tmr_ins, p3time_t exp_time_in_us )
{

    uint32_t current_time;

    current_time = get_current_time();

    if(IsTimeLess(exp_time_in_us, current_time))
    {
        mp_hw_tmr_inst->cb(mp_hw_tmr_inst->ctx);
    }
    else
    {
        StartTimer
        (
            exp_time_in_us
        );
    }
}

/******************************************************************************/

void hw_tmr_stop( void *hw_tmr_ins )
{
	StopTimer(((hw_tmr_t*)hw_tmr_ins)->tmr_channel);
}

/******************************************************************************/

uint8_t hw_tmr_rand( void *hw_tmr_ins )
{
	p3time_t seed = 0x00;

	rnd_index += 1;

    seed = hw_tmr_get_time(hw_tmr_ins);

    return (rnd_array[ (rnd_index & 0x3f) ] ^ (seed & 0xff));

}

/******************************************************************************/

//#pragma section ("L1_code")

void hw_tmr_delay( void *hw_tmr_ins, uint32_t duration_tks )
{

	p3time_t time_now,target_time;

    adi_int_EnterCriticalRegion();

    target_time = get_current_time();

	target_time += duration_tks;

	do
	{
        time_now = get_current_time();
	} while( IsTimeLess(target_time, time_now) );

    adi_int_ExitCriticalRegion();
}

//#pragma section ("L1_code")
uint32_t hw_tmr_get_symbols( uint32_t time )
{
	return (time / HWTIMER_SYMBOL_LENGTH);  //time /20 usecs
}
/******************************************************************************/

/*
** ============================================================================
** Private Function Definitions
** ============================================================================
*/

void Timer0_Callback(int id, void *data)
{
    //adi_GPIO_Toggle(ADI_GPIO_PORT_0, ADI_GPIO_PIN_0);
	mp_hw_tmr_inst->cb(mp_hw_tmr_inst->ctx);
}

/******************************************************************************/
bool_t IsTimeLess(uint32_t time1, uint32_t time2)
{
    if(time1 <= time2)
    {
        if((time2 - time1) > 0x7FFFFFFF)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    else
    {
      if((time1 - time2) > 0x7FFFFFFF)
      {
          return true;
      }
      else
      {
          return false;
      }
    }
}


