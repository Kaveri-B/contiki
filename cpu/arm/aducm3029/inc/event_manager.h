/** \file event_manager.h
 *******************************************************************************
 ** \brief Provides supporting APIs and macros for Non-RTOS task scheduling 
 ** mechanism 
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
 
#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
** ============================================================================
** Public Macro definitions
** ============================================================================
*/

typedef enum
{
	RX_IND_EVENT = 0x00,
	CCA_DONE_EVENT,
	TX_CONF_EVENT,
	SFD_DETECTED_EVENT,
	TIMER_EXPIRY_EVENT,
	HIF_RX_EVENT,	
	RX_SEC_FRAME_EVENT,
	UNSECURE_EVENT,
	BCN_RX_EVENT,
	DATA_REQ_RX_EVENT,
	CMD_OR_DATA_FRAME_RX_EVENT,
	SECURE_EVENT,
	PENDING_TX_EVENT,
	FRAME_TX_DONE_EVENT,
	MLME_EVENT,
	MCPS_EVENT,
	MAC_2_NHLE_EVENT,
	MAC_EVENT_NONE = 0xFF
} events_t;

#define MAX_EVENT_PRIO	0x12

/*
** ============================================================================
** Public Structures, Unions & enums Type Definitions
** ============================================================================
*/

/* None */

/*
** ============================================================================
** Public Variable Declarations
** ============================================================================
*/

/* None */

/*
** ============================================================================
** Private Variable Definitions
** ============================================================================
*/

/* None */

/*
** =============================================================================
** Public Variables Definitions
** =============================================================================
**/

/* None */

/*
** ============================================================================
** External Variable Declarations
** ============================================================================
*/

/* None */

/*
**=========================================================================
**  Public Function Prototypes
**=========================================================================
*/

/**
 *******************************************************************************
 ** \brief Function to get the highest priority event
 ** \param - None
 ** \retval - priority 
 ******************************************************************************/
uint32_t highest_prio_event_get( void );

/**
 *******************************************************************************
 ** \brief Function to set event for given priority
 ** \param prio - priority 
 ** \retval - None
 ******************************************************************************/
void event_set( uint32_t prio );

/**
 *******************************************************************************
 ** \brief Function to clear event for given priority
 ** \param prio - priority 
 ** \retval - None
 ******************************************************************************/
void event_clear( uint32_t prio );

/**
 *******************************************************************************
 ** \brief Function to signel event to mac layer
 ** \param prio - None
 ** \retval - None
 ******************************************************************************/
extern void signal_event_to_mac_task( void );

/**
 *******************************************************************************
 ** \brief Function to initialize event manager module
 ** \param prio - None
 ** \retval - None
 ******************************************************************************/
void event_manager_init(void);

void raise_int( ushort int_num );

void Radio_Interrupt_Handler(void *pCBParam, uint32_t Event, void *pArg);

void Soft_Interrupt_Handler(void *pCBParam, uint32_t Event, void *pArg);

#ifdef __cplusplus
}
#endif
#endif		//EVENT_MANAGER_H

