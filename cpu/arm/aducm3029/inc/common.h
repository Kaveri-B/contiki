/*!
 *****************************************************************************
 * @file:     common.h
 * @brief:    common include file for all example 
 * @version: $Revision: 32612 $
 * @date:    $Date: 2015-11-08 13:00:01 -0500 (Sun, 08 Nov 2015) $
 *-----------------------------------------------------------------------------
 *
 * Copyright (C) 2009-2013 ARM Limited. All rights reserved.
 *
 * ARM Limited (ARM) is supplying this software for use with Cortex-M3
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************/


#ifndef __TEST_COMMON_H__
#define __TEST_COMMON_H__

#ifdef __ICCARM__
/*
* Pm106 (rule 20.9): the input/output library <stdio.h> shall not be used in
                     production code
*   The purpose of this header is to provide I/O facilities based on stdio.
*/
#pragma diag_suppress=Pm106
#endif /* __ICCARM__ */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include <services/wdt/adi_wdt.h>
#include <drivers/uart/adi_uart.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Enable REDIRECT_OUTPUT_TO_UART to send the output to UART terminal. */
/* #define REDIRECT_OUTPUT_TO_UART */

extern char_t aDebugString[150]; 

#ifdef __ICCARM__
/*
* Pm154 (rule 19.10): in the definition of a function-like macro, each instance
*                     of a parameter shall be enclosed in parentheses
*   The __VA_ARGS__ macro cannot be enclosed in parentheses.
*/
#pragma diag_suppress=Pm154
#endif /* __ICCARM__ */

#define DEBUG_MESSAGE(...) \
  do { \
    sprintf(aDebugString,__VA_ARGS__); \
    test_Perf(aDebugString); \
  } while(0)

#ifdef __ICCARM__
#pragma diag_default=Pm154
#endif /* __ICCARM__ */

#define DEBUG_RESULT(s,result,expected_value) \
  do { \
    if ((result) != (expected_value)) { \
      sprintf(aDebugString,"%s  %d", __FILE__,__LINE__); \
      test_Fail(aDebugString); \
      sprintf(aDebugString,"%s Error Code: 0x%08X\n\rFailed\n\r",(s),(result)); \
      test_Perf(aDebugString); \
      exit(0); \
    } \
  } while (0)

/********************************************************************************
* API function prototypes
*********************************************************************************/
void test_Init(void);
void test_Pass(void);
void test_Fail(char *FailureReason);
void test_Perf(char *InfoString);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_COMMON_H__ */
