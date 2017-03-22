/*!
 *****************************************************************************
 * @file:    system.c
 * @brief:   System startup code for ADuCM302x
 * @version: $Revision: 33290 $
 * @date:    $Date: 2016-01-19 09:11:48 -0500 (Tue, 19 Jan 2016) $
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2010-2015 Analog Devices, Inc.

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

/*! \addtogroup SYS_Driver System Interfaces
 *  @{
 */

#include <stdint.h>
#include "system.h"
#include <services/int/adi_int.h>
#include <adi_processor.h>

#include <intrinsics.h>

#ifdef __ICCARM__
/*
* IAR MISRA C 2004 error suppressions.
*
* Pm073 (rule 14.7): a function should have a single point of exit.
* Pm143 (rule 14.7): a function should have a single point of exit at the end of the function.
*   Multiple returns are used for error handling.
*
* Pm140 (rule 11.3): a cast should not be performed between a pointer type and an integral type
*   The rule makes an exception for memory-mapped register accesses.
*/
#pragma diag_suppress=Pm073,Pm143,Pm140
#endif /* __ICCARM__ */

#ifdef RELOCATE_IVT
extern void __relocated_vector_table;
#else
extern void __vector_table;
#endif
uint32_t SystemCoreClock;

#ifdef ADI_DEBUG
/* not needed unless its debug mode */
uint32_t lfClock = 0u;    /* "lf_clk" coming out of LF mux             */
#endif
uint32_t hfClock = 0u;    /* "root_clk" output of HF mux               */
uint32_t gpioClock = 0u;  /* external GPIO clock                       */

/*!
 * Initialize the system
 *
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the relocate vector table.
 */
void SystemInit (void)
{
    /* Switch the Interrupt Vector Table Offset Register
     * (VTOR) to point to the relocated IVT in SRAM.
     */

    ADI_ENTER_CRITICAL_REGION();  /* Do all this in safe way. */

    /* Switch from boot ROM IVT to application's IVT. */
#ifdef RELOCATE_IVT
    SCB->VTOR = (uint32_t) &__relocated_vector_table;
#else
    SCB->VTOR = (uint32_t) &__vector_table;
#endif

    /* Set all three (USGFAULTENA, BUSFAULTENA, and MEMFAULTENA) fault enable bits
     * in the System Control Block, System Handler Control and State Register
     * otherwise these faults are handled as hard faults.
     */
    SCB->SHCSR = SCB_SHCSR_USGFAULTENA_Msk |
                 SCB_SHCSR_BUSFAULTENA_Msk |
                 SCB_SHCSR_MEMFAULTENA_Msk ;

    /* Flush instruction and data pipelines to insure assertion of new settings. */
    __ISB();
    __DSB();

    ADI_EXIT_CRITICAL_REGION(); 
}

/*!
 * Update the clock. 
 *
 * @return none
 *
 * @brief  Updates the variable SystemCoreClock and must be called whenever 
           the core clock is changed during program execution.
 */
 void SystemCoreClockUpdate(void)
{
    uint32_t val, nDivisor, nMulfactor, div2, mul2;

#ifdef ADI_DEBUG
    /* "lfclock" is only used during debug checks... */
    /* LF clock is always 32k, whether osc or xtal */
    lfClock = __LFCLK;  /* for beep, wdt and lcd */
    if (lfClock == 0u)
    {
      while (1) {}
    }
#endif
    /* Update Core Clock sources */
    /* update the HF clock */
    switch (pADI_CLKG0_CLK->CTL0 & BITM_CLKG_CLK_CTL0_CLKMUX ) {

        case HFMUX_INTERNAL_OSC_VAL:
            hfClock = __HFOSC;
            break;

        case HFMUX_EXTERNAL_XTAL_VAL:
            hfClock = __HFXTAL;
            break;

        case HFMUX_SYSTEM_SPLL_VAL:
            /* Calculate System PLL output frequency */
            if ((pADI_CLKG0_CLK->CTL0 & BITM_CLKG_CLK_CTL0_SPLLIPSEL) != 0u) {
                /* PLL input from HFXTAL */
                val = __HFXTAL;
            } else {
                /* PLL input from HFOSC */
                val = __HFOSC;
            }

            /* PLL NSEL multiplier */
            nMulfactor = (pADI_CLKG0_CLK->CTL3 & BITM_CLKG_CLK_CTL3_SPLLNSEL) >> BITP_CLKG_CLK_CTL3_SPLLNSEL;
            /* PLL MSEL divider */
            nDivisor = (pADI_CLKG0_CLK->CTL3 & BITM_CLKG_CLK_CTL3_SPLLMSEL) >> BITP_CLKG_CLK_CTL3_SPLLMSEL;

            /* PLL NSEL multiplier */
            mul2 = (pADI_CLKG0_CLK->CTL3 & BITM_CLKG_CLK_CTL3_SPLLMUL2) >> BITP_CLKG_CLK_CTL3_SPLLMUL2;
            /* PLL MSEL divider */
            div2 = (pADI_CLKG0_CLK->CTL3 & BITM_CLKG_CLK_CTL3_SPLLDIV2) >> BITP_CLKG_CLK_CTL3_SPLLDIV2;
            
            val = ((val << mul2) * nMulfactor / nDivisor) >> div2;

            hfClock = val;
            break;

        case HFMUX_GPIO_VAL:
            hfClock = gpioClock;
            break;

        default:         
            return;
    } /* end switch */
 
    SystemCoreClock = hfClock;
 }
 
/*!
 * @brief  This enables or disables  the cache.
 * \n @param  bEnable : To specify whether to enable/disable cache.
 * \n              true : To enable cache.
 * \n
 * \n              false : To disable cache.
 * \n
 * @return none
 *
 */
void adi_system_EnableCache(bool_t bEnable)
{
    pADI_FLCC0_CACHE->KEY = CACHE_CONTROLLER_KEY;
    if(bEnable)
    {
        pADI_FLCC0_CACHE->SETUP |= BITM_FLCC_CACHE_SETUP_ICEN;
    }
    else
    {
        pADI_FLCC0_CACHE->SETUP &= ~BITM_FLCC_CACHE_SETUP_ICEN;
    }
}

/*!
 * @brief  This enables or disables instruction SRAM
 *
 * @param bEnable: To enable/disable the instruction SRAM.
 * \n              true : To enable cache.
 * \n
 * \n              false : To disable cache.
 * \n
 * @return none
 * @note:  Please note that respective linker file need to support the configuration.
 */
void adi_system_EnableISRAM(bool_t bEnable)
{

    if(bEnable)
    {
        pADI_PMG0_TST->SRAM_CTL |= BITM_PMG_TST_SRAM_CTL_INSTREN;
    }
    else
    {
        pADI_PMG0_TST->SRAM_CTL &= ~BITM_PMG_TST_SRAM_CTL_INSTREN;
    }
}

/*!
 * @brief  This enables/disable SRAM retention during the hibernation.
 * @param eBank:   Specify which SRAM bank. Only BANK1 and BANK2 are valid.
 * @param bEnable: To enable/disable the  retention for specified  SRAM bank.
 * \n              true : To enable retention during the hibernation.
 * \n
 * \n              false :To disable retention during the hibernation.
 * \n
 * @return : SUCCESS : Configured successfully.
 *           FAILURE :  For invalid bank.  
 * @note: Please note that respective linker file need to support the configuration. Only BANK-1 and 
          BANK-2 of SRAM is valid.
 */
uint32_t adi_system_EnableRetention(ADI_SRAM_BANK eBank,bool_t bEnable)
{
#ifdef ADI_DEBUG
    if((eBank != ADI_SRAM_BANK_1) && (eBank != ADI_SRAM_BANK_2))
    {
        return FAILURE;
    }

#endif
    pADI_PMG0->PWRKEY = PWRKEY_VALUE_KEY;
    if(bEnable)
    {
        pADI_PMG0->SRAMRET |= (uint32_t)eBank>>1;
    }
    else
    {
        pADI_PMG0->SRAMRET &= ~((uint32_t)eBank >> 1);    
    }

    return SUCCESS;
}

volatile int intCount = 0;

/* Returns the global interrupt state */
void adi_int_EnterCriticalRegion(void)
{
  if(!intCount)
     __disable_interrupt();

   intCount++;
}

/* Restore the global interrupt state */
void adi_int_ExitCriticalRegion()
{
  intCount--;

  if(!intCount)
    __enable_interrupt();
}

/*@}*/