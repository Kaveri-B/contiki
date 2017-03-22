/** \file timer_service.c
 *******************************************************************************
 ** \brief Implements the OS dependant part of the timer service
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
#ifdef ADI_IAR
#endif
#include "telec_common.h"
#include "queue_latest.h"
#include "list_latest.h"
#include "sts.h"
#ifndef ADI_IAR
#include "bf_timer.h"
#else
#include "sys/clock.h"
#endif
#include "hw_tmr.h"
#include "sw_timer.h"
#include "timer_service.h"
#include "event_manager.h"


/*
** ============================================================================
** Private Macro definitions
** ============================================================================
*/

/**< Indicates a software timer is just created  */
#define SW_TMR_CREATED 1

/**< Indicates that a software timer is active/running  */
#define SW_TMR_ACTIVE  2

/**< Indicates that a software timer is expired  */
#define SW_TMR_EXPIRED 3

/**< Indicates that a software timer is stopped */
#define SW_TMR_STOPPED 4

/**< Indicates that the software timer was requested to be stopped from timer
*    interrupt context */
#define SW_TMR_ISR_REQ_STOP  1

/**< Indicates that the software timer was requested to be start from timer
*    interrupt context */
#define SW_TMR_ISR_REQ_START 2

/**< Indicates software timer expiry event */
#define SW_TMR_EXPIRE_EVENT    SST_HIGHEST_PRIO_EVENT

/**< Indicates software timer ISR request event */
#define SW_TMR_ISR_REQ_EVENT   0x02

/*
** ============================================================================
** Private Structures, Unions & enums Type Definitions
** ============================================================================
*/

/*None*/


/*
** ============================================================================
** Private Variable Definitions
** ============================================================================
*/

static sw_tmr_module_t tmr_mod_ins;
static hw_tmr_t hw_tmr_ins;
volatile unsigned char timer_trigger = 0;

/*
** ============================================================================
** Public Variable Definitions
** ============================================================================
*/

sw_tmr_module_t* gpTmr_mod_ins;


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

static bool tmr_start( sw_tmr_t *pTmr_ins );

static void timer_expiry_notify( sw_tmr_module_t *pTmr_mod_ins );

/*
** ============================================================================
** Public Function Definitions
** ============================================================================
*/

void MAC_SoftInterruptHandler(void *pCBParam, uint32_t Event, void *pArg);

bool tmr_service_init(void)
{
  if(adi_gpio_RegisterCallback(MAC_SOFT_INT_NUM, MAC_SoftInterruptHandler, NULL) != 0)
  {
      return 1;
  }
  ADI_ENABLE_INT(MAC_SOFT_INT_NUM);
  
  gpTmr_mod_ins = &tmr_mod_ins;

  hw_tmr_ins.tmr_channel = HW_TIMER_CHANNEL_0;
  hw_tmr_ins.cb  = (cb_routine_t)timer_expiry_notify;
  hw_tmr_ins.ctx = ( void* )&tmr_mod_ins;

  gpTmr_mod_ins->hw_timer_if.pHw_tmr_ins = &hw_tmr_ins;
  gpTmr_mod_ins->hw_timer_if.hw_tmr_get_time = hw_tmr_get_time;
  gpTmr_mod_ins->hw_timer_if.hw_tmr_init = hw_tmr_init;
  gpTmr_mod_ins->hw_timer_if.hw_tmr_start = hw_tmr_start;
  gpTmr_mod_ins->hw_timer_if.hw_tmr_stop = hw_tmr_stop;
  gpTmr_mod_ins->hw_timer_if.hw_tmr_delay = hw_tmr_delay;
  gpTmr_mod_ins->hw_timer_if.hw_tmr_rand = hw_tmr_rand;
  gpTmr_mod_ins->prio = TIMER_THREAD_PRIO;

  sw_tmr_init(gpTmr_mod_ins);

// create a thred in case of an OS being used
  return TRUE;
}

/******************************************************************************/



/*
** ============================================================================
** Private Function Definitions
** ============================================================================
*/


void
timer_task( void )
{


    adi_int_EnterCriticalRegion();

    if( timer_trigger )
    {
        timer_trigger--;

        update_expired_tmr_list(gpTmr_mod_ins);
    }

    adi_int_ExitCriticalRegion();

    invoke_expired_timer_cbs(gpTmr_mod_ins);

    adi_int_EnterCriticalRegion();

    if(!timer_trigger)
    {
        event_clear(TIMER_EXPIRY_EVENT);
    }
    adi_int_ExitCriticalRegion();

    return;
}
/******************************************************************************/
bool tmr_create_one_shot_timer( sw_tmr_t *pTmr_ins,stime_t period,sw_tmr_cb_t cb,void* param )
{
	return sw_tmr_create( gpTmr_mod_ins, pTmr_ins, SW_TMR_ONESHOT, period, cb, param );
}

/******************************************************************************/

bool
tmr_start_relative( sw_tmr_t *pTmr_ins )
{
  p3time_t curr  = gpTmr_mod_ins->hw_timer_if.hw_tmr_get_time(
                      gpTmr_mod_ins->hw_timer_if.pHw_tmr_ins );

  if( pTmr_ins->state >= SW_TMR_CREATED )
  {
    pTmr_ins->exp_time = (pTmr_ins->period * 26 / 16) + curr;

    return tmr_start( pTmr_ins );
  }
  return FALSE;
}

/******************************************************************************/

bool
tmr_start_absolute( sw_tmr_t *pTmr_ins, p3time_t expire_time)
{
  if( pTmr_ins->state >= SW_TMR_CREATED )
  {
    pTmr_ins->exp_time = expire_time;
    return tmr_start( pTmr_ins );
  }
  return FALSE;
}


/******************************************************************************/

void tmr_delay(uint32_t delay_tks)
{
    delay_tks = delay_tks * 26 / 16;
	if ( delay_tks !=0 )
	{
		sw_tmr_delay(gpTmr_mod_ins, delay_tks);
	}
	return;
}

/******************************************************************************/

static bool
tmr_start( sw_tmr_t *pTmr_ins )
{

    adi_int_EnterCriticalRegion();
//    irq_disable();

    sw_tmr_add_to_active_list( gpTmr_mod_ins, pTmr_ins );

    adi_int_ExitCriticalRegion();
//    irq_enable();

    return TRUE;
}

/******************************************************************************/

bool
tmr_stop( sw_tmr_t *pTmr_ins )
{

    adi_int_EnterCriticalRegion();

    if( pTmr_ins != NULL )
    {
    	if( pTmr_ins->state != SW_TMR_ACTIVE )
        goto sw_tmr_stop_exit;

    	sw_tmr_stop(gpTmr_mod_ins,pTmr_ins );
    }
    else
    {
        adi_int_ExitCriticalRegion();
    	return false;
    }


sw_tmr_stop_exit:
    pTmr_ins->state = SW_TMR_STOPPED;
    adi_int_ExitCriticalRegion();
    return TRUE;
}

/******************************************************************************/
static void
timer_expiry_notify( sw_tmr_module_t *pTmr_mod_ins )
{
	timer_trigger += 1;
	event_set(TIMER_EXPIRY_EVENT);
	signal_event_to_mac_task();

}

void raise_int( ushort int_num )
{
  switch(int_num)
  {
    case RADIO_INT_NUM:
      NVIC_SetPendingIRQ(RADIO_INT_NUM);
      break;
    case UART_INT_NUM:
      NVIC_SetPendingIRQ(UART_INT_NUM);
      break;
    case MAC_SOFT_INT_NUM:
      NVIC_SetPendingIRQ(MAC_SOFT_INT_NUM);
      break;
  }
}

void MIL_Task( void )
{
  uint32_t event;
  while(( event = highest_prio_event_get()) != MAX_BASE_VALUE )
  {
      switch( event )
      {
        case TIMER_EXPIRY_EVENT:
          timer_task();
        break;
        default:
        break;
      }
  }
}

void signal_event_to_mac_task( void )
{
    raise_int(MAC_SOFT_INT_NUM);
}

void MAC_SoftInterruptHandler(void *pCBParam, uint32_t Event, void *pArg)
{
    MIL_Task();
}
/******************************************************************************/




