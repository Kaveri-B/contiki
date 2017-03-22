/** \file list_latest.c
 *******************************************************************************
 ** \brief This file provides the two list functionalities.
 **
 ** This file provides the functions like adding an entry at the end of the
 ** list and deleting an entry from the list.
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
#include "list_latest.h"

/*
** ============================================================================
** Private Macro definitions
** ============================================================================
*/

/*None*/

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

/*None*/

/*
** ============================================================================
** Public Variable Definitions
** ============================================================================
*/

/*None*/

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

/*None*/

/*
** ============================================================================
** Public Function Definitions
** ============================================================================
*/


void list_init_latest( list_latest_t *list )
{
    list->start = NULL;
}

/******************************************************************************/

void list_entry_add_start(
      list_latest_t *list,     /**< the list    */
      list_item_t* ptr  /**< the item to be saved */
      )
{

    adi_int_EnterCriticalRegion();

    ptr->next = NULL;

    if ( list->start == NULL)
    {
        // This is the first
        list->start = ptr;
    }
    else
    {
        // Add to front of the list
        ptr->next  = list->start;
        list->start = ptr;
    }

    adi_int_ExitCriticalRegion();
}

/******************************************************************************/

void list_entry_add_end(
      list_latest_t *list,
      list_item_t* ptr
      )
{
    list_item_t *current;


    adi_int_EnterCriticalRegion();
//    irq_disable();

    current = list->start;

    ptr->next = NULL;

    if ( list->start == NULL)
    {
        // This is the first
        list->start = ptr;
    }
    else
    {
        while( current->next != NULL)
        {
            current = current->next;
        }
        current->next = ptr;
    }

    adi_int_ExitCriticalRegion();
//    irq_enable();
}

/******************************************************************************/
 uint8_t already_in_list(list_latest_t *list, list_item_t* ptr)
 {
    list_item_t *item = list->start;

    while(item != NULL)
    {
      if(ptr == item)
        return 1;
      item = item->next;
    }
    return 0;
 }
/*volatile*/ uint16_t err_count = 0;
 void list_entry_add_after(
    list_latest_t *list,
    list_item_t* ptr,
    list_item_t* lptr
    )
{

    adi_int_EnterCriticalRegion();
    if(already_in_list(list, ptr))
    {
        adi_int_ExitCriticalRegion();
        err_count++;
        return;
    }

//    irq_disable();
    ptr->next = NULL;

	if ( lptr == NULL )
    {
        list_entry_add_start( list, ptr );
    }
    else
    {
    	ptr->next = lptr->next;
    	lptr->next = ptr;
    }

    adi_int_ExitCriticalRegion();
//    irq_enable();
}

 void list_concat(
    list_latest_t *list1,
    list_latest_t *list2
    )
{

    adi_int_EnterCriticalRegion();
//    irq_disable();

	if ( list1->start == NULL )
    {
    	*list1 = *list2;
    	adi_int_ExitCriticalRegion();
//        irq_enable();
    	return;
    }

    if( list2->start == NULL )
    {
    	adi_int_ExitCriticalRegion();
//        irq_enable();
    	return;
    }

	list_item_t *current;
    current = list1->start;

    while( current->next != NULL)
    {
        current = current->next;
    }
    current->next = list2->start;

    adi_int_ExitCriticalRegion();
//    irq_enable();
}

/******************************************************************************/

 list_item_t* list_entry_get_start(
    list_latest_t*list
    )
{
    return list->start;
}

/******************************************************************************/

  list_item_t* list_entry_get_prev(
    list_latest_t*list,
    list_item_t* ptr
    )
{

    adi_int_EnterCriticalRegion();
//    irq_disable();

    if (ptr == list->start)
    {
            //If trying to get the previous item of start
            adi_int_ExitCriticalRegion();
//            irq_enable();
            return NULL;
    }

    list_item_t *current = list->start;

    while(( current != NULL ) && ( current->next != ptr ))
    {
        current = current->next;
    }

    adi_int_ExitCriticalRegion();
//    irq_enable();

    return current;
}

/******************************************************************************/

 list_item_t* list_entry_scan_next(
    list_latest_t* list,
    list_item_t* ptr
    )
{
    return ptr->next;
}

/******************************************************************************/

  bool list_entry_is_start(
    list_latest_t*list,
    list_item_t* ptr
    )
{
    return list->start == ptr;
}

/******************************************************************************/

#if 1
list_item_t* list_entry_delete(
      list_latest_t *list,
      list_item_t* ptr
      )
{
    list_item_t *current, *ret = NULL;

    adi_int_EnterCriticalRegion();

    current = list->start;

	if (ptr == list->start)
	{
		list->start = ptr->next;
		ret = ptr;
		adi_int_ExitCriticalRegion();
		return ret;
	}

    while (( current != NULL ) && ( current->next != ptr ))
    {
        current = current->next;
    }
    if ( NULL != current )
    {
        //current->next = ptr;
		current->next = ptr->next;
        ret = ptr;
    }

    adi_int_ExitCriticalRegion();

    return ret;
}

/******************************************************************************/

 void list_entry_set_start(
    list_latest_t*list,
    list_item_t* ptr
    )
{
    list->start = ptr;
}

/*
** ============================================================================
** Private Function Definitions
** ============================================================================
*/

/*None*/

#endif
