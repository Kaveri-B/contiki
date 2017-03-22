/*!
 *****************************************************************************
 * @file:    startup.c
 * @brief:   Interrupt table and default handlers for ADuCM302x
 * @version: $Revision: 33303 $
 * @date:    $Date: 2016-01-20 10:45:16 -0500 (Wed, 20 Jan 2016) $
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

#include <startup.h>
#include <system.h>

#ifdef _USING_UCOS_II_
#include <os_cpu.h>
#endif

/*
    Stack size considerations for RTOS environments:

    In the Cortex-M class processors, the hardware has two stacks: the main stack
    and the process stack.  The main stack is used at reset and by system exception
    handlers.  In simple applications (non-RTOS) you may only use this stack.  In
    more complex systems that use an RTOS you have a split between the main stack
    which is used by the kernel and exception handlers and the process stack which
    is used by the tasks and programmable interrupts (interrupts stack on top of
    the currently active task, so tasks stacks should be allocated for worst-case
    interrupt scenarios).  In the uCOS case, the process stack is the one that gets
    context-switched and used by the tasks, where we store the registers, etc.

    As RTOS tasks specify their own stack area, a minimal size of 4 for the process
    stack in order to retain symbols, etc., seems OK and we would not need any more.
    The main stack needs to be big enough to initialize the RTOS, run interrupts etc.

    As always, stack and heap size optimizations is a balancing act between usage
    and available memory.  It is entirely application-dependant.

*/


/*
    The minimal vector table for a Cortex M3.  Note that the proper constructs
    must be placed on this to ensure that it ends up at physical address
    0x0000.0000.
*/

/*
    When operating in an RTOS context, interrupts are dispatched by the NVIC controller
    to a second level dispatch function known as the OSAL (operating system abstraction layer) dispatch.
    The OSAL dispatch has common code that will interact with the RTOS.
   
    OSAL dispatched interrupts must pass their Interrupt ID (iid) to the common OSAL dispatcher.
   
    System interrupts (Reset through SysTick) should never dispatch through the OSAL.
   
    The IntFunc typedef is used for System Interrupts handlers. The parameter list for these interrupts is always 'void'.
   
    The ADIIntFunc typdef is used for Driver Interrupt Handlers. The parameter list for these handlers
    will change depending on the RTOS context. In an RTOS context the IID (Interrupt ID) must be passed to the OSAL
    dispatcher (along with a second arg of type void * which is handler specific). In a non-RTOS context the handler
    parameters is simply 'void'.
*/

#ifdef __ICCARM__
/*
* IAR MISRA C 2004 error suppressions:
*
* Pm093 (rule 18.4): use of union - overlapping storage shall not be used.
*    Required for interrupt vector table entries.
*
* Pm140 (rule 11.3): a cast should not be performed between a pointer type and an integral type
*   The rule makes an exception for memory-mapped register accesses.
*/
#pragma diag_suppress=Pm093,Pm140
#endif /* __ICCARM__ */

/* IVT typedefs. */
typedef void  ( *IntFunc )( void );
#if (ADI_CFG_ENABLE_RTOS_SUPPORT == 1)
#ifndef USER_SPECIFIED_RTOS
/* With a user-specified RTOS, user_rtos_support.h is expected to define this type. */
typedef void  ( *ADIIntFunc )( uint32_t iid, void* handlerArg);
#endif
#else
typedef void  ( *ADIIntFunc )( void );
#endif

typedef union { ADIIntFunc __fun; void * __ptr;  IntFunc __ifunc; } IntVector;

WEAK_FUNCTION( NmiSR                    )
WEAK_FUNCTION( HardFault_Handler        )
WEAK_FUNCTION( MemManage_Handler        )
WEAK_FUNCTION( BusFault_Handler         )
WEAK_FUNCTION( UsageFault_Handler       )
WEAK_FUNCTION( SVC_Handler              )
WEAK_FUNCTION( DebugMon_Handler         )
WEAK_FUNCTION( PENDSV_HANDLER           )
WEAK_FUNCTION( SYSTICK_HANDLER          )
ADI_WEAK_FUNCTION( RTC1_Int_Handler       )
ADI_WEAK_FUNCTION( Ext_Int0_Handler         )
ADI_WEAK_FUNCTION( Ext_Int1_Handler         )
ADI_WEAK_FUNCTION( Ext_Int2_Handler         )
ADI_WEAK_FUNCTION( Ext_Int3_Handler         )
ADI_WEAK_FUNCTION( WDog_Tmr_Int_Handler     )
ADI_WEAK_FUNCTION( Vreg_over_Int_Handler     )
ADI_WEAK_FUNCTION( Battery_Voltage_Int_Handler     )	
ADI_WEAK_FUNCTION( RTC0_Int_Handler     )	
ADI_WEAK_FUNCTION( GPIO_A_Int_Handler       )
ADI_WEAK_FUNCTION( GPIO_B_Int_Handler       )
ADI_WEAK_FUNCTION( GP_Tmr0_Int_Handler      )
ADI_WEAK_FUNCTION( GP_Tmr1_Int_Handler      )
ADI_WEAK_FUNCTION( Flash0_Int_Handler       )
ADI_WEAK_FUNCTION( UART_Int_Handler         )
ADI_WEAK_FUNCTION( SPI0_Int_Handler         )
ADI_WEAK_FUNCTION( SPI2_Int_Handler         )
ADI_WEAK_FUNCTION( I2C0_Slave_Int_Handler   )
ADI_WEAK_FUNCTION( I2C0_Master_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_Err_Int_Handler      )
ADI_WEAK_FUNCTION( DMA_SPI2_TX_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_SPI2_RX_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_SPORT0A_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_SPORT0B_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_SPI0_TX_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_SPI0_RX_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_SPI1_TX_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_SPI1_RX_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_UART_TX_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_UART_RX_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_I2C0_STX_Int_Handler )
ADI_WEAK_FUNCTION( DMA_I2C0_SRX_Int_Handler )
ADI_WEAK_FUNCTION( DMA_I2C0_MX_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_AES0_IN_Int_Handler  )
ADI_WEAK_FUNCTION( DMA_AES0_OUT_Int_Handler )
ADI_WEAK_FUNCTION( DMA_FLASH0_Int_Handler   )
ADI_WEAK_FUNCTION( SPORT0A_Int_Handler      )
ADI_WEAK_FUNCTION( SPORT0B_Int_Handler      )
ADI_WEAK_FUNCTION( Crypto_Int_Handler       )
ADI_WEAK_FUNCTION( DMA_ADC0_Int_Handler     )
ADI_WEAK_FUNCTION( GP_Tmr2_Int_Handler      )
ADI_WEAK_FUNCTION( Crystal_osc_Int_Handler  )
ADI_WEAK_FUNCTION( SPI1_Int_Handler         )
ADI_WEAK_FUNCTION( PLL_Int_Handler          )
ADI_WEAK_FUNCTION( RNG_Int_Handler          )
ADI_WEAK_FUNCTION( Beep_Int_Handler         )
ADI_WEAK_FUNCTION( ADC_Int_Handler          )
/** Placeholder: IRQn = 47 is reserved on the ADuM302X                                   */
/** Placeholder: IRQn = 48 is reserved on the ADuM302x                                   */
/** Placeholder: IRQn = 49 is reserved on the ADuM302X                                   */
/** Placeholder: IRQn = 50 is reserved on the ADuM302x                                   */
/** Placeholder: IRQn = 51 is reserved on the ADuM302X                                   */
/** Placeholder: IRQn = 52 is reserved on the ADuM302x                                   */
/** Placeholder: IRQn = 53 is reserved on the ADuM302X                                   */
/** Placeholder: IRQn = 54 is reserved on the ADuM302x                                   */
/** Placeholder: IRQn = 55 is reserved on the ADuM302X                                   */
ADI_WEAK_FUNCTION( DMA_SIP0_Int_Handler     )
ADI_WEAK_FUNCTION( DMA_SIP1_Int_Handler     )
ADI_WEAK_FUNCTION( DMA_SIP2_Int_Handler     )
ADI_WEAK_FUNCTION( DMA_SIP3_Int_Handler     )
ADI_WEAK_FUNCTION( DMA_SIP4_Int_Handler     )
ADI_WEAK_FUNCTION( DMA_SIP5_Int_Handler     )
ADI_WEAK_FUNCTION( DMA_SIP6_Int_Handler     )
ADI_WEAK_FUNCTION( DMA_SIP7_Int_Handler     )
WEAK_FUNCTION( RESERVED_VECTOR              )

/* Interrupt Vector Table */
#pragma segment="CSTACK"
#pragma location = ".intvec"
const IntVector __vector_table[] =
{
    /* Grab stack pointer "end" address from ICF file */
    { .__ptr = __sfe( "CSTACK" ) },

    /* Exception mappings */
    { .__ifunc = ResetISR                 } ,  /* -15 */
    { .__ifunc = NmiSR                    } ,  /* -14 */
    { .__ifunc = HardFault_Handler        } ,  /* -13 */
    { .__ifunc = MemManage_Handler        } ,  /* -12 */
    { .__ifunc = BusFault_Handler         } ,  /* -11 */
    { .__ifunc = UsageFault_Handler       } ,  /* -10 */
    { .__ifunc = RESERVED_VECTOR          } ,  /* -09 */
    { .__ifunc = RESERVED_VECTOR          } ,  /* -08 */
    { .__ifunc = RESERVED_VECTOR          } ,  /* -07 */
    { .__ifunc = RESERVED_VECTOR          } ,  /* -06 */
    { .__ifunc = SVC_Handler              } ,  /* -05 */
    { .__ifunc = DebugMon_Handler         } ,  /* -04 */
    { .__ifunc = RESERVED_VECTOR          } ,  /* -03 */
    { .__ifunc = PENDSV_HANDLER           } ,  /* -02  (Micrium overrides weak binding if using uCOS)  */
    { .__ifunc = SYSTICK_HANDLER          } ,  /* -01  (Micrium overrides weak binding if using uCOS)  */

    /* Programmable interrupt mappings... */
    {RTC1_Int_Handler       } ,                /* 0  */
    {Ext_Int0_Handler         } ,              /* 1  */
    {Ext_Int1_Handler         } ,              /* 2  */
    {Ext_Int2_Handler         } ,              /* 3  */ 
    {Ext_Int3_Handler         } ,              /* 4  */
    {WDog_Tmr_Int_Handler     } ,              /* 5  */
    {Vreg_over_Int_Handler    } ,              /* 6  */
    {Battery_Voltage_Int_Handler    } ,        /* 7  */
    {RTC0_Int_Handler         } ,              /* 8  */
    {GPIO_A_Int_Handler       } ,              /* 9  */
    {GPIO_B_Int_Handler       } ,              /* 10 */
    {GP_Tmr0_Int_Handler      } ,              /* 11 */
    {GP_Tmr1_Int_Handler      } ,              /* 12 */
    {Flash0_Int_Handler       } ,              /* 13 */
    {UART_Int_Handler         } ,              /* 14 */
    {SPI0_Int_Handler         } ,              /* 15 */
    {SPI2_Int_Handler         } ,              /* 16 */
    {I2C0_Slave_Int_Handler   } ,              /* 17 */
    {I2C0_Master_Int_Handler  } ,              /* 18 */
    {DMA_Err_Int_Handler      } ,              /* 19 */
    {DMA_SPI2_TX_Int_Handler  } ,              /* 20 */
    {DMA_SPI2_RX_Int_Handler  } ,              /* 21 */ 
    {DMA_SPORT0A_Int_Handler  } ,              /* 22 */
    {DMA_SPORT0B_Int_Handler  } ,              /* 23 */
    {DMA_SPI0_TX_Int_Handler  } ,              /* 24 */
    {DMA_SPI0_RX_Int_Handler  } ,              /* 25 */
    {DMA_SPI1_TX_Int_Handler  } ,              /* 26 */
    {DMA_SPI1_RX_Int_Handler  } ,              /* 27 */
    {DMA_UART_TX_Int_Handler  } ,              /* 28 */
    {DMA_UART_RX_Int_Handler  } ,              /* 29 */
    {DMA_I2C0_STX_Int_Handler } ,              /* 30 */
    {DMA_I2C0_SRX_Int_Handler } ,              /* 31 */
    {DMA_I2C0_MX_Int_Handler  } ,              /* 32 */
    {DMA_AES0_IN_Int_Handler  } ,              /* 33 */
    {DMA_AES0_OUT_Int_Handler } ,              /* 34 */
    {DMA_FLASH0_Int_Handler   } ,              /* 35 */
    {SPORT0A_Int_Handler      } ,              /* 36 */
    {SPORT0B_Int_Handler      } ,              /* 37 */
    {Crypto_Int_Handler       } ,              /* 38 */
    {DMA_ADC0_Int_Handler     } ,              /* 39 */    
    {GP_Tmr2_Int_Handler      } ,              /* 40 */
    {Crystal_osc_Int_Handler  } ,              /* 41 */
    {SPI1_Int_Handler         } ,              /* 42 */
    {PLL_Int_Handler          } ,              /* 43 */
    {RNG_Int_Handler          } ,              /* 44 */
    {Beep_Int_Handler         } ,              /* 45 */
    {ADC_Int_Handler          } ,              /* 46 */    
    { .__ifunc = RESERVED_VECTOR } ,           /* 47 */
    { .__ifunc = RESERVED_VECTOR } ,           /* 48 */
    { .__ifunc = RESERVED_VECTOR } ,           /* 49 */
    { .__ifunc = RESERVED_VECTOR } ,           /* 50 */
    { .__ifunc = RESERVED_VECTOR } ,           /* 51 */
    { .__ifunc = RESERVED_VECTOR } ,           /* 52 */
    { .__ifunc = RESERVED_VECTOR } ,           /* 53 */
    { .__ifunc = RESERVED_VECTOR } ,           /* 54 */
    { .__ifunc = RESERVED_VECTOR } ,           /* 55 */
    {DMA_SIP0_Int_Handler    } ,               /* 56 */
    {DMA_SIP1_Int_Handler    } ,               /* 57 */
    {DMA_SIP2_Int_Handler    } ,               /* 58 */
    {DMA_SIP3_Int_Handler    } ,               /* 59 */
    {DMA_SIP4_Int_Handler    } ,               /* 60 */
    {DMA_SIP5_Int_Handler    } ,               /* 61 */
    {DMA_SIP6_Int_Handler    } ,               /* 62 */
    {DMA_SIP7_Int_Handler    } ,               /* 63 */
    
} ;

 #pragma location="ReadProtectedKeyHash"
__root const uint32_t ReadProKeyHsh[] =
    { 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu };        

 #pragma location="CRC_ReadProtectedKeyHash"
__root const uint32_t CrcOfReadKey = 0xA79C3203u;        

 #pragma location="NumCRCPages"
__root const uint32_t NumOfCRCPages = 0u;  


#ifdef RELOCATE_IVT
/*
    A relocated IVT is requested.  Provision for IVT relocation
    to RAM during startup.	This allows for dynamic interrupt
    vector patching required by RTOS.  Places the relocated IVT
    at the start of RAM.  Note: the IVT placement is required
    to be next power-of-two	of the vector table size.  So the
    IVT includes 61	programmable interrupts, 15 system exception
    vectors and	the main stack pointer, therefore we need
    (61 + 15 + 1)*4	= 308 bytes, which rounds up to a 512 (0x200)
    address	boundary (which address 0x20000000 satisfies).

    Also note use of the "__no_init" attribute to force the
    relocated IVT into the ".noinit" section.  This prevents
    the CRTL startup sequence from initializing the relocated
    IVT after we have activated it during the reset vector.
    so that the CRTL does not clear it.
*/
#define RELOCATION_ADDRESS   (0x10000000)// Changes Required for 6LoWPAN project
#define RELOCATION_ALIGNMENT      (0x200)

#define LENGTHOF_IVT (sizeof __vector_table / sizeof *__vector_table)

#if defined ( __ICCARM__ )
    #pragma data_alignment=RELOCATION_ALIGNMENT  /* IAR */
#elif defined (__CC_ARM)
    __align(RELOCATION_ALIGNMENT)                /* Keil */
#else
    #pragma message("WARNING: NO ALIGHMENT DEFINED FOR IVT RELOCATION")
#endif
/* reserve no-init aligned IVT space at top of RAM */
SECTION_PLACE(KEEP_VAR(__no_init IntVector __relocated_vector_table[LENGTHOF_IVT]), RELOCATION_ADDRESS);
#endif


#ifdef __ARMCC_VERSION
/* We want a warning if semi-hosting libraries are used. */
#pragma import(__use_no_semihosting_swi)
#endif

#ifdef __GNUC__
/* Symbols defined by the GNU linker. */
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss ;
extern unsigned long _ebss;
#endif

#ifdef __ICCARM__
/* IAR system startup functions. */
WEAK_PROTO( void __iar_init_core (void));
WEAK_PROTO( void __iar_init_vfp  (void));
extern void __cmain(void);
#else
/* Application Entry point. */
extern int main(void);
#endif /* __ICCARM__*/


/******************************************************************************
* Function    : ResetISR (-15)
* Description : Reset event handler
******************************************************************************/

WEAK_FUNC (void ResetISR (void)) {

    /* Reset the main SP to clean up any leftovers from the boot rom. */
    __set_MSP( (uint32_t) __vector_table[0].__ptr );

    /* Unlock the PWRMOD register by writing the two keys to the PWRKEY register. */
    pADI_PMG0->PWRKEY = PWRKEY_VALUE_KEY;
    pADI_PMG0->SRAMRET &= ~(BITM_PMG_SRAMRET_BNK2EN | BITM_PMG_SRAMRET_BNK1EN);
    /* Set the RAM0_RET bit so the entire 8K of SRAM Bank0 is hibernate-preserved. */

    adi_system_EnableRetention(ADI_SRAM_BANK_1, true);
    /* With current version, RAM size for 6LN exceeded 16K, hence to support for 
    sleepy device enabling BANK2 to retain during hibernate.*/
    adi_system_EnableRetention(ADI_SRAM_BANK_2, true);
                               
/* To disable the instruction SRAM and entire 64K of SRAM is used as DSRAM. */    
#ifdef  ADI_DISABLE_INSTRUCTION_SRAM
    adi_system_EnableISRAM(false);
#endif    

    
/* To enable the instruction cache.  */    
#ifdef  ENABLE_CACHE
    adi_system_EnableCache(true);    
#endif    
    
#ifdef RELOCATE_IVT
    /* Copy the IVT (avoid use of memcpy here so it does not become locked into flash). */
    size_t i;
    for (i = 0u; i < LENGTHOF_IVT; i++)
    {
        __relocated_vector_table[i] = __vector_table[i];
    }
#endif

#ifdef __GNUC__

    unsigned long *pulSrc, *pulDest;

    /* Copy initialised data from flash into RAM. */
    pulSrc = &_etext;
    for (pulDest = &_data; pulDest < &_edata; )
    {
        *pulDest++ = *pulSrc++;
    }

    /* Clear the bss segment. */
    for (pulDest = &_bss; pulDest < &_ebss; )
    {
        *pulDest++ = 0;
    }

    /* Call application main directly. */
    main();

#elif __ICCARM__

    /* Call IAR system startup. */
    __iar_init_core();
    __iar_init_vfp();
    __cmain();

#else

    /* Call application main directly. */
    main();

#endif

    /* Stick here if main returns. */
    while(1) {}
}
