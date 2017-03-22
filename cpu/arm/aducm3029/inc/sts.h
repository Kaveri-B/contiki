/** \file sts.h
 *******************************************************************************
 ** \brief Provides APIs for creating and starting threads.
 **
 ** This file contains the public APIs and structures needed for creating and 
 ** starting new threads with Simple Thread Scheduler module. This file also 
 ** provides APIs for initializing and starting the scheduler.
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

#ifndef _STS_H_
#define _STS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *****************************************************************************
 * @ingroup sysdoc
 *
 * @{
 *****************************************************************************/

/**
 *****************************************************************************
 * @defgroup sts Simple Task Scheduler Module
 *
 * @brief This module provides a simple, lightweight, prioritized, fully 
 * preemptive and deterministic run to completion real time scheduler. 
 *
 * This scheduler supports a max of 4097 threads. This prioritised scheduler 
 * allows only the  higher-priority threads to preempt the currently active 
 * thread. All threads will run to completion and cannot block as they are not 
 * based on an infinite while loop. All threads use the same stack.\n
 *
 * The threads have priorities from 0 to IDLE_THREAD_PRIO where the lower 
 * number represents the higher priority. IDLE_THREAD_PRIO is the lowest priority 
 * assigned to an idle thread. This thread can be used for performing any low 
 * power operations when all the threads are not active. As a fully preemptive 
 * scheduler, STS will ensure that at all times the highest-priority thread 
 * which is in ready state, is executed. \n STS supports two kinds of preemption.\n
 * 1)Synchronous preemption \n
 * 2) Asynchronous peremption \n
 *
 * Synchronous preemtion is where the a lower priority thread posts an event to 
 * a higher priority thread. In this scenario, STS suspends the lower priority 
 * thread and runs the higher priority thread to its completion before resuming 
 * the execution of the lower priority thread. Synchronous preemptions are not 
 * limited to only one level. If the high-priority thread posts an event to a 
 * still higher-priority thread, the high-priority thread would be synchronously
 * preempted and the scenario would recursively repeat itself at a higher level 
 * of priority.
 *
 * Asynchronous preemption is where an interrupt posts an event to a higer 
 * priority thread than the thread which is currently running. In this case the 
 * STS suspends the current running thread and executes the higher priority 
 * thread to its completion. Asynchronous peremption is also noy limited to one 
 * level.\n
 * The system using this STS has to call sts_init() for initializing the STS 
 * module before creating and running any threads.    
******************************************************************************/

/**
 *****************************************************************************
 * 
 * @} 
 *****************************************************************************/

/*
** ============================================================================
** Public Macro definitions
** ============================================================================
*/

/**
 ** \defgroup sts_defs  STS Definitions
 ** \ingroup sts
 */

/*@{*/


/*! Default Priority Value assigned of the highest priority thread*/
#define SST_HIGHEST_THREAD_PRIO     0

/*! Default Priority Value assigned to the lowest priority thread*/
#define SST_LOWEST_THREAD_PRIO      STS_MAX_THREAD_PRIO

/*! Default Priority Value assigned of the highest priority event*/
#define SST_HIGHEST_PRIO_EVENT      ( 0 )

/*! Default Priority Value assigned to the lowest priority event */
#define SST_LOWEST_PRIO_EVENT       ( sizeof(base_t) << 3 ) - 1 //31

/*! Default Priority Value assigned to the  NULL event(no event)*/
#define SST_EVENT_PRIO_NONE         0xFF

/*! Default PTimer module Thread Priority */
#define TIMER_THREAD_PRIO           0

/*! Default USART driver Thread Priority*/
#define USART_DRIVER_THREAD_PRIO    TIMER_THREAD_PRIO + 1

/*! Default SPI Thread Priority*/
#define SPI_DRIVER_THREAD_PRIO      USART_DRIVER_THREAD_PRIO + 1

/*! Default USB driver Thread Priority */
#define USB_DRIVER_THREAD_PRIO      SPI_DRIVER_THREAD_PRIO + 1

/*! Default GPIO driver Thread Priority*/
#define GPIO_DRIVER_THREAD_PRIO     USB_DRIVER_THREAD_PRIO + 1

/*! Default ADC driver Thread Priority*/
#define ADC_DRIVER_THREAD_PRIO      GPIO_DRIVER_THREAD_PRIO + 1

/*! Default PWM driver Thread Priority*/
#define PWM_DRIVER_THREAD_PRIO      ADC_DRIVER_THREAD_PRIO + 1

/*! Default I2C driver Thread Priority*/
#define I2C_DRIVER_THREAD_PRIO      PWM_DRIVER_THREAD_PRIO + 1

/*! Default SSC driver Thread Priority*/
#define SSC_DRIVER_THREAD_PRIO      I2C_DRIVER_THREAD_PRIO + 1

/*! Default Applicaiton A Thread Priority*/
#define A_THREAD_PRIO               SSC_DRIVER_THREAD_PRIO + 1

/*! Default Applicaiton B Thread Priority*/
#define B_THREAD_PRIO               A_THREAD_PRIO + 1

/*! Default Idle Thread Priority */
#define IDLE_THREAD_PRIO            B_THREAD_PRIO + 1

/*! Default Total threads in the system*/
#define STS_MAX_THREAD_PRIO         IDLE_THREAD_PRIO + 1

/*
** ============================================================================
** Public Structures, Unions & enums Type Definitions
** ============================================================================
*/  
 
/**
 *****************************************************************************
 * @enum Status retured by STS.
 *    Enum used in STS for status return.
 *****************************************************************************/
typedef enum sts_status_enum
{
    STS_SUCCESS,
    STS_FAILURE
}sts_status_t;

/**
 *****************************************************************************
 * @brief Structure for the thread entry function.Objects of this type will be 
 * used to store the address of the thread entry functions.
 *****************************************************************************/
typedef void (*sts_thread_t)( base_t event, void *pCtx );

/**
 *****************************************************************************
 * @brief Idle routine type difinition.
 *    Difines the type definition of an idle thread.
 *****************************************************************************/
typedef void (*sts_idle_routine_t)( void *pCtx );

/**
 *****************************************************************************
 * @struct Structure for the thread control block.
 *    Structure difinition for the thread control block, given by user to 
 *    create a thread.
 *****************************************************************************/
typedef struct sts_tcb_struct {
    sts_thread_t pEntry_fn; /**< Thread entry function */
    void         *pCtx;     /**< Thread entry function parameter */
    base_t       event;     /**< Stores the events for the thread */
    base_t       prio;      /**< Priority of the thread */
}sts_tcb_t;

/*@}*/

/*
** ============================================================================
** Public Variable Declarations
** ============================================================================
*/

/*None*/

/*
** ============================================================================
** Public Function Prototypes
** ============================================================================
*/

/** \defgroup sts_req STS APIs
 ** \ingroup sts
 */

/*@{*/

 /**
 *****************************************************************************
 * @brief Initializes the STS module.
 * This function does the following as part of initialisation of the STS.\n
 *   1) Clears the TCB list \n
 *   2) clears the bit mask maintained to store the list of ready threads.
 * @param None
 * @retval None
 * @note This function should be called once from main function during 
 * initialization of the system components. 
 *****************************************************************************/
void sts_init( void );

/**
 *****************************************************************************
 * @brief Registers the Idle thread function with the STS.
 * @param rutn[in] - Pointer to the idle thread function to be registered.
 * @param *pCtx[in] - Pointer to the idle thread function parameter.
 * @retval None
 * @note This function will be called for registering the Idle thread function 
 * before starting the STS.
 *****************************************************************************/
void sts_register_idle_routine( sts_idle_routine_t rutn, void *pCtx );

/**
 *****************************************************************************
 * @brief Starts the STS
 * This function executes a infinite while loop which does the following.\n
 * 1) Brings ths STS into running state \n
 * 2)  Scedule high priority threads if ready to be executed \n
 * 3) If no threads are ready, executtre the idle thread.
 * @param None
 * @retval None
 * @note This function should be called once to start the scheduler from main 
 * function after the STS has been initialized using sts_init().
 *****************************************************************************/
void sts_start(void);

/**
 *****************************************************************************
 * @brief Creates and starts a thread.Following things are done as part of 
 * initialization adn creation of a thread.\n
 * 1) initializes the thread TCB\n
 * 2) adds the TCB into the TCB list maintained by STS\n
 * 3) Add an event if any to the TCBs Q\n
 * 4) schedule the thread fo execution of  event to he Highest priority thread
 * @param *pTcb[in] - pointer to the app allocated TCB block provided to STS
 * @param entry_fn[in] - pointer to the thread function to be executed by STS 
 * @param prio[in] - priority to be g a assigned to the thread under creation.
 * @param *pCtx[in] - pointer to the context to be passed when invoking a thread 
 *                   from STS
 * @param event[in] - event(s) for the thread
 * @retval sts_status_t
 * @note This function should be called once for creating a thread. Caller has 
 * to allocate the TCB object for the thread being created and pass the same to 
 * this function.
 *****************************************************************************/
sts_status_t sts_thread_create( sts_tcb_t *pTcb, sts_thread_t entry_fn, base_t prio, 
    void *pCtx, base_t event );

/**
 *****************************************************************************
 * @brief Posts an event to the event queue of a thread.
 * @param prio[in] - priority of the thread to which event to be posted.
 * @param event[in] - event to be posted. 
 * @retval sts_status_t
 * @note This function is used for synchronous preemption if the thread posting 
 * an event to a higher priority thread. This function execuutes the thread if 
 * the event being posted is to a higher priority thread than the thread calling 
 * this function.
 *****************************************************************************/
sts_status_t sts_post_event( base_t prio, base_t event );

 /**
 *****************************************************************************
 * @brief This function provides mutex to avoid priority inversion scenarios. 
 * This function blocks any threads with priorities lower than this priority 
 * ceiling 
 * @param prio_ceiling[in] - priority to where the current thread priority is 
 * elevated.
 * @retval - actual priority of the current thread which has be elevated 
 * to a higher priority temporarily.
 * @note temporarily increasing the priority of the current thread from where 
 * this function is being called. The actual priority of the thread should be stored 
 * by the caller for restoring the priority at a later stage.
 *****************************************************************************/
base_t sts_mutex_lock(base_t prio_ceiling);

/**
 *****************************************************************************
 * @brief Mutex Unlock function. This function restores the previously elivated 
 * thread's priority.   
 * @param  actual_prio[in] - the actual priority of the thread which was elivated 
 *         to a higher priority temporarily.
 * @retval None
 * @note This function has to be called to release the previously acquired lock, 
 *once the critical section is executed to 
 *****************************************************************************/
void sts_mutex_unlock(base_t actual_prio);

 /**
 *****************************************************************************
 * @brief Provides the current running thread's priority.
 * @param None
 * @retval - the priority of the thread under execution. Returns 0xFFFFFFFF 
 * if the STS is locked as a result of an interrupt. 
 *****************************************************************************/
base_t sts_current_runniung_thread( void );

 /**
 *****************************************************************************
 * @brief  This function saves the priority of the interrupted thread on the 
 * stack and raises the current priority to the ISR level.
 * @param None
 * @retval None
 * @note This function should be called from the the IRQ handler as soon as the 
 *  IRQ is invoked by the controller.
 *****************************************************************************/
void sts_isr_entry(void);

 /**
 *****************************************************************************
 * @brief The function sends the end-of-interrupt (EOI) instruction to the 
 *  interrupt controller, restores the saved priority of the interrupted thread 
 *  from the stack, and invokes the STS.
 * @param None
 * @retval None
 * @note This function should be called as soon as the interrtup handling is completed 
 * to resume the interrupted thread execution.
 *****************************************************************************/
void sts_isr_exit(void);

/**
 *****************************************************************************
 * @brief Checks and returns the the highest priority event from the event list.
 * @param *pEvent[in] - pointer to an event list
 * @retval - the highest prioritised event
 *****************************************************************************/
base_t sts_get_highest_prio_event( base_t *pEvent );

/*@}*/

#ifdef __cplusplus
}
#endif
#endif /* STS_H_ */
