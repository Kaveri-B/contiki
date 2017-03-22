/*!
 *****************************************************************************
 * @file:    ADuCM3029_cdef.h
 * @brief:   ADuCM3029 C MMR Pointer Definitions
 * @version: $Revision: 33562 $
 * @date:    $Date: 2016-02-16 04:23:40 -0500 (Tue, 16 Feb 2016) $
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2015 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef _WRAP_ADUCM3029_CDEF_H
#define _WRAP_ADUCM3029_CDEF_H

#ifdef __ICCARM__
/* IAR MISRA C 2004 error suppressions.
*
* Pm008 (rule 2.4): sections of code should not be 'commented out'.
*   Some comments are wrongly identified as code.
*
* Pm009 (rule 5.1): identifiers shall not rely on significance of more than 31 characters.
*   The YODA-generated headers rely on more. The IAR compiler supports that.
*
* Pm081 (rule 19.6): #undef should not be used
*   Needed to work around incorrect definitions in generated headers.
*/
#pragma diag_suppress=Pm008,Pm009,Pm081
#endif /* __ICCARM__ */

#include <sys/ADUCM302x_cdef.h>


/* Workarounds for sys/ADuCM302x.h issues. */

#undef BITM_TMR_LOAD_VALUE
#define BITM_TMR_LOAD_VALUE                  (_ADI_MSK(0x0000FFFF,0x0000FFFFU, uint16_t ))    /*  Load value */

#undef BITM_TMR_CURCNT_VALUE
#define BITM_TMR_CURCNT_VALUE                (_ADI_MSK(0x0000FFFF,0x0000FFFFU, uint16_t ))    /*  Current count */

#undef BITM_TMR_CAPTURE_VALUE
#define BITM_TMR_CAPTURE_VALUE               (_ADI_MSK(0x0000FFFF,0x0000FFFFU, uint16_t ))    /*  16-bit captured value */

#undef BITM_TMR_ALOAD_VALUE
#define BITM_TMR_ALOAD_VALUE                 (_ADI_MSK(0x0000FFFF,0x0000FFFFU, uint16_t ))    /*  Load value, asynchronous */

#undef BITM_TMR_ACURCNT_VALUE
#define BITM_TMR_ACURCNT_VALUE               (_ADI_MSK(0x0000FFFF,0x0000FFFFU, uint16_t ))    /*  Counter value */

#undef BITM_TMR_PWMMATCH_VALUE
#define BITM_TMR_PWMMATCH_VALUE              (_ADI_MSK(0x0000FFFF,0x0000FFFFU, uint16_t ))    /*  PWM Match Value */

#undef BITM_WDT_LOAD_VALUE
#define BITM_WDT_LOAD_VALUE                  (_ADI_MSK(0x0000FFFF,0x0000FFFFU, uint16_t ))    /*  Load value */

#undef BITM_WDT_CCNT_VALUE
#define BITM_WDT_CCNT_VALUE                  (_ADI_MSK(0x0000FFFF,0x0000FFFFU, uint16_t ))    /*  Current count value */

#undef BITM_WDT_RESTART_CLRWORD
#define BITM_WDT_RESTART_CLRWORD             (_ADI_MSK(0x0000FFFF,0x0000FFFFU, uint16_t ))    /*  Clear watchdog */

#undef BITM_UART_COMDIV_DIV
#define BITM_UART_COMDIV_DIV                 (_ADI_MSK(0x0000FFFF,0x0000FFFFU, uint16_t ))    /*  Baud rate divider */

#undef BITM_DMA_PDBPTR_ADDR
#define BITM_DMA_PDBPTR_ADDR                 (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Pointer to the base address of the primary data structure */

#undef BITM_DMA_ADBPTR_ADDR
#define BITM_DMA_ADBPTR_ADDR                 (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Base address of the alternate data structure */

#undef BITM_FLCC_KH_DATA0_VALUE
#define BITM_FLCC_KH_DATA0_VALUE             (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Lower half of 64-bit dual word data to be written on a WRITE command */

#undef BITM_FLCC_KH_DATA1_VALUE
#define BITM_FLCC_KH_DATA1_VALUE             (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Upper half of 64-bit dual word data to be written on a WRITE command */

#undef BITM_FLCC_KEY_VALUE
#undef ENUM_FLCC_KEY_USERKEY
#define BITM_FLCC_KEY_VALUE                  (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Key register */
#define ENUM_FLCC_KEY_USERKEY                (_ADI_MSK(0x676C7565,0x676C7565UL, uint32_t ))    /*  VALUE: USERKEY */

#undef BITM_FLCC_WR_ABORT_ADDR_VALUE
#define BITM_FLCC_WR_ABORT_ADDR_VALUE        (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Holds the address targeted by an ongoing write command and retains its value after an ABORT event */

#undef BITM_FLCC_WRPROT_WORD
#define BITM_FLCC_WRPROT_WORD                (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Clear bits to write protect related groups of user space pages. Once cleared these bits can only be set again by resetting the part */

#undef BITM_FLCC_SIGNATURE_VALUE
#define BITM_FLCC_SIGNATURE_VALUE            (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Provides read access to the most recently generated signature */

#undef BITM_FLCC_CACHE_KEY_VALUE
#define BITM_FLCC_CACHE_KEY_VALUE            (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Cache Key register */

#undef BITM_CRC_IPDATA_VALUE
#define BITM_CRC_IPDATA_VALUE                (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Data Input. */

#undef BITM_CRC_RESULT_VALUE
#define BITM_CRC_RESULT_VALUE                (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  CRC Residue */

#undef BITM_CRYPT_INBUF_VALUE
#define BITM_CRYPT_INBUF_VALUE               (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Input Buffer */

#undef BITM_CRYPT_OUTBUF_VALUE
#define BITM_CRYPT_OUTBUF_VALUE              (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))    /*  Output Buffer */

#undef BITM_PTI_RST_ISR_STARTADDR_VALUE
#define BITM_PTI_RST_ISR_STARTADDR_VALUE     (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))

#undef BITM_PTI_RST_STACK_PTR_VALUE
#define BITM_PTI_RST_STACK_PTR_VALUE         (_ADI_MSK(0xFFFFFFFF,0xFFFFFFFFUL, uint32_t ))


/* Workarounds for sys/ADuCM302x_cdef.h issues. */

#undef pREG_TMR0_LOAD
#undef pREG_TMR0_CURCNT
#define pREG_TMR0_LOAD                   ((volatile        uint16_t *) REG_TMR0_LOAD)                   /*  16-bit load value */
#define pREG_TMR0_CURCNT                 ((volatile const  uint16_t *) REG_TMR0_CURCNT)                 /*  16-bit timer value */

#undef pREG_TMR0_CAPTURE
#undef pREG_TMR0_ALOAD
#undef pREG_TMR0_ACURCNT
#define pREG_TMR0_CAPTURE                ((volatile const  uint16_t *) REG_TMR0_CAPTURE)                /*  Capture */
#define pREG_TMR0_ALOAD                  ((volatile        uint16_t *) REG_TMR0_ALOAD)                  /*  16-bit load value, asynchronous */
#define pREG_TMR0_ACURCNT                ((volatile const  uint16_t *) REG_TMR0_ACURCNT)                /*  16-bit timer value, asynchronous */

#undef pREG_TMR0_PWMMATCH
#define pREG_TMR0_PWMMATCH               ((volatile        uint16_t *) REG_TMR0_PWMMATCH)               /*  PWM Match Value */

#undef pREG_TMR1_LOAD
#undef pREG_TMR1_CURCNT
#define pREG_TMR1_LOAD                   ((volatile        uint16_t *) REG_TMR1_LOAD)                   /*  16-bit load value */
#define pREG_TMR1_CURCNT                 ((volatile const  uint16_t *) REG_TMR1_CURCNT)                 /*  16-bit timer value */

#undef pREG_TMR1_CAPTURE
#undef pREG_TMR1_ALOAD
#undef pREG_TMR1_ACURCNT
#define pREG_TMR1_CAPTURE                ((volatile const  uint16_t *) REG_TMR1_CAPTURE)                /*  Capture */
#define pREG_TMR1_ALOAD                  ((volatile        uint16_t *) REG_TMR1_ALOAD)                  /*  16-bit load value, asynchronous */
#define pREG_TMR1_ACURCNT                ((volatile const  uint16_t *) REG_TMR1_ACURCNT)                /*  16-bit timer value, asynchronous */

#undef pREG_TMR1_PWMMATCH
#define pREG_TMR1_PWMMATCH               ((volatile        uint16_t *) REG_TMR1_PWMMATCH)               /*  PWM Match Value */

#undef pREG_TMR2_LOAD
#undef pREG_TMR2_CURCNT
#define pREG_TMR2_LOAD                   ((volatile        uint16_t *) REG_TMR2_LOAD)                   /*  16-bit load value */
#define pREG_TMR2_CURCNT                 ((volatile const  uint16_t *) REG_TMR2_CURCNT)                 /*  16-bit timer value */

#undef pREG_TMR2_CAPTURE
#undef pREG_TMR2_ALOAD
#undef pREG_TMR2_ACURCNT
#define pREG_TMR2_CAPTURE                ((volatile const  uint16_t *) REG_TMR2_CAPTURE)                /*  Capture */
#define pREG_TMR2_ALOAD                  ((volatile        uint16_t *) REG_TMR2_ALOAD)                  /*  16-bit load value, asynchronous */
#define pREG_TMR2_ACURCNT                ((volatile const  uint16_t *) REG_TMR2_ACURCNT)                /*  16-bit timer value, asynchronous */

#undef pREG_TMR2_PWMMATCH
#define pREG_TMR2_PWMMATCH               ((volatile        uint16_t *) REG_TMR2_PWMMATCH)               /*  PWM Match Value */

#undef pREG_WDT0_LOAD
#undef pREG_WDT0_CCNT
#define pREG_WDT0_LOAD                   ((volatile        uint16_t *) REG_WDT0_LOAD)                   /*  Load value */
#define pREG_WDT0_CCNT                   ((volatile const  uint16_t *) REG_WDT0_CCNT)                   /*  Current count value */

#undef pREG_WDT0_RESTART
#define pREG_WDT0_RESTART                ((volatile        uint16_t *) REG_WDT0_RESTART)                /*  Clear interrupt */

#undef pREG_UART0_COMDIV
#define pREG_UART0_COMDIV                ((volatile        uint16_t *) REG_UART0_COMDIV)                /*  Baudrate divider */

#undef pREG_DMA0_PDBPTR
#undef pREG_DMA0_ADBPTR
#define pREG_DMA0_PDBPTR                 ((volatile        uint32_t *) REG_DMA0_PDBPTR)                 /*  DMA channel primary control data base pointer */
#define pREG_DMA0_ADBPTR                 ((volatile const  uint32_t *) REG_DMA0_ADBPTR)                 /*  DMA channel alternate control data base pointer */

#undef pREG_FLCC0_KH_DATA0
#undef pREG_FLCC0_KH_DATA1
#define pREG_FLCC0_KH_DATA0              ((volatile        uint32_t *) REG_FLCC0_KH_DATA0)              /*  WRITE Lower Data */
#define pREG_FLCC0_KH_DATA1              ((volatile        uint32_t *) REG_FLCC0_KH_DATA1)              /*  WRITE Upper Data */

#undef pREG_FLCC0_KEY
#undef pREG_FLCC0_WR_ABORT_ADDR
#undef pREG_FLCC0_WRPROT
#undef pREG_FLCC0_SIGNATURE
#define pREG_FLCC0_KEY                   ((volatile        uint32_t *) REG_FLCC0_KEY)                   /*  Key */
#define pREG_FLCC0_WR_ABORT_ADDR         ((volatile const  uint32_t *) REG_FLCC0_WR_ABORT_ADDR)         /*  Write Abort Address */
#define pREG_FLCC0_WRPROT                ((volatile        uint32_t *) REG_FLCC0_WRPROT)                /*  Write Protection */
#define pREG_FLCC0_SIGNATURE             ((volatile const  uint32_t *) REG_FLCC0_SIGNATURE)             /*  Signature */

#undef pREG_FLCC0_CACHE_KEY
#define pREG_FLCC0_CACHE_KEY             ((volatile        uint32_t *) REG_FLCC0_CACHE_KEY)             /*  Cache Key register */

#undef pREG_CRC0_IPDATA
#undef pREG_CRC0_RESULT
#define pREG_CRC0_IPDATA                 ((volatile        uint32_t *) REG_CRC0_IPDATA)                 /*  Input Data Word Register */
#define pREG_CRC0_RESULT                 ((volatile        uint32_t *) REG_CRC0_RESULT)                 /*  CRC Result Register */

#undef pREG_CRYPT0_INBUF
#undef pREG_CRYPT0_OUTBUF
#define pREG_CRYPT0_INBUF                ((volatile        uint32_t *) REG_CRYPT0_INBUF)                /*  Input Buffer */
#define pREG_CRYPT0_OUTBUF               ((volatile const  uint32_t *) REG_CRYPT0_OUTBUF)               /*  Output Buffer */

#undef pREG_PTI0_RST_ISR_STARTADDR
#undef pREG_PTI0_RST_STACK_PTR
#define pREG_PTI0_RST_ISR_STARTADDR      ((volatile        uint32_t *) REG_PTI0_RST_ISR_STARTADDR)      /*  Reset ISR Start Address */
#define pREG_PTI0_RST_STACK_PTR          ((volatile        uint32_t *) REG_PTI0_RST_STACK_PTR)          /*  Reset Stack Pointer */

#ifdef __ICCARM__
#pragma diag_default=Pm008,Pm009,Pm081
#endif /* __ICCARM__ */

#endif /* _WRAP_ADUCM3029_CDEF_H */
