/** \file timer_service.h
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

#ifndef TIMER_SERVICE_H
#define TIMER_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/*
** ============================================================================
** Public Macro definitions
** ============================================================================
*/

/**
 ** \defgroup timer_service  Timer Service Module
 */

/*@{*/

/**
 ** \defgroup timer_def Timer Service Definitions
 ** \ingroup timer_service
 */

/*@{*/

/* None */

/*
** ============================================================================
** Public Structures, Unions & enums Type Definitions
** ============================================================================
*/

/* None */

/*@}*/

/*
** ============================================================================
** Public Variable Declarations
** ============================================================================
*/

extern sw_tmr_module_t* gpTmr_mod_ins;

/*
** ============================================================================
** Public Function Prototypes
** ============================================================================
*/

extern void update_expired_tmr_list(sw_tmr_module_t *pTmr_mod_ins);
extern void invoke_expired_timer_cbs(sw_tmr_module_t *pTmr_mod_ins); 

/**
 ** \defgroup timer_func Timer Service APIs
 ** \ingroup timer_service
 */

/*@{*/

/**
 *****************************************************************************
 * @brief Initializes the software timer service
 * @param None
 * @retval TRUE or FALSE
 * @note This function should be called atleast once during the system 
 * initialization and only after this, the timer services should be used.
 *****************************************************************************/
bool tmr_service_init(void);

/**
 *****************************************************************************
 * @brief Implements the timer task functionality.
 * @param None
 * @retval None
 * @note This function should be called in the main functions while(1) loop.
 *****************************************************************************/
void timer_task(void);

/**
 *****************************************************************************
 * @brief Implements the delay functionality
 * @param usecs[in] - the delay in microseconds 
 * @retval None
 * @note The delay specified should be in terms of microseconds.
 *****************************************************************************/
void tmr_delay(uint32_t usecs);

/**
 *****************************************************************************
 * @brief Implements the software timer start functionality
 * @param *pTmr_ins[in] - the software timer to be started
 * @retval TRUE or FALSE
 * @note This function should be called only after creating a software timer.
 * @sa tmr_create_one_shot_timer 
 *****************************************************************************/
bool tmr_start_relative( sw_tmr_t *pTmr_ins );

/**
 *****************************************************************************
 * @brief Implements the software timer start functionality
 * @param *pTmr_ins[in] - the software timer to be started
 * @retval TRUE or FALSE
 * @note This function should be called only after creating a software timer.
 * @sa tmr_create_one_shot_timer
 *****************************************************************************/
bool tmr_start_absolute( sw_tmr_t *pTmr_ins, p3time_t expire_time );

/**
 *****************************************************************************
 * @brief Initializes an one shot software timer 
 * @param *pTmr_ins[in] - sw timer to be initilialized
 * @param  period[in] - indicates the period after which the timer will expire
 * @param  cb[in] - holds the callback function which will be invoked after the timer expires
 * @param *param[in] - holds the param used for passing to the invoked expiry callback function
 * @retval TRUE or FALSE
 * @note The memory for the softtware timer should be defined before 
 * initializing it using this function
 *****************************************************************************/
bool tmr_create_one_shot_timer( sw_tmr_t *pTmr_ins,stime_t period,sw_tmr_cb_t cb,void* param );

/**
 *****************************************************************************
 * @brief Implements the software timer stop service
 * @param *pTmr_ins[in] - active timer to be stopped
 * @retval TRUE or FALSE
 * @note None
 *****************************************************************************/
bool tmr_stop( sw_tmr_t *pTmr_ins );

/*@}*/

/*@}*/

#ifdef __cplusplus
}
#endif
#endif /* TIMER_SERVICE_H */
