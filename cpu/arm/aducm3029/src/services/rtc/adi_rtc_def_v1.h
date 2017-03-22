/*!
 *****************************************************************************
 * @file:    adi_rtc_def_v1.h
 * @brief:   RTC def file 
 * @version: $Revision: 33205 $
 * @date:    $Date: 2016-01-11 05:46:07 -0500 (Mon, 11 Jan 2016) $
 *-----------------------------------------------------------------------------
 *
 * Copyright (c) 2010-2015 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Modified versions of the software must be conspicuously marked as such.
 * - This software is licensed solely and exclusively for use with processors
 *   manufactured by or for Analog Devices, Inc.
 * - This software may not be combined or merged with other code in any manner
 *   that would cause the software to become subject to terms and conditions
 *   which differ from those listed here.
 * - Neither the name of Analog Devices, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 * - The use of this software may or may not infringe the patent rights of one
 *   or more patent holders.  This license does not release you from the
 *   requirement that you obtain separate licenses from these patent holders
 *   to use this software.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
 * PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef __ADI_RTC_DEF_V1_H__
#define __ADI_RTC_DEF_V1_H__ 

#include <adi_rtc_config.h>

/*! \cond PRIVATE */
#define ADI_RTC_NUM_INSTANCE 2u 

/*! Timer configuration structure used to hold the application configuration.
*/

#define ADI_RTC_INT_ENA_MASK_CR0         0XF804u

#define ADI_RTC_INT_ENA_MASK_CR1         0X1Fu

#define ADI_RTC_INT_ENA_MASK_CR2         0X7400u

#define ADI_RTC_INT_ENA_MASK_CR3         0X0200u

#define ADI_RTC_INT_SOURCE_MASK          0x007Eu


#define ADI_RTC_WRITE_STATUS_MASK        0XCF8u

#define ADI_RTC_SR2_IRQ_STATUS_MASK      0X1Fu

#define ADI_RTC_SR3_IRQ_STATUS_MASK      0X21Du

#define ADI_RTC_SR3_ICIRQ_STATUS_MASK    0X01Du


#define ADI_RTC_TRIM_MASK (BITM_RTC_TRM_VALUE | BITM_RTC_TRM_ADD|BITM_RTC_TRM_IVL | BITM_RTC_TRM_IVL2EXPMIN )

#if (ADI_RTC_CFG_ENABLE_SAFE_WRITE == 1)
    /* pause on pending writes to CR to avoid data loss */

#ifdef __ICCARM__
/*
* Pm154 (rule 19.10): in the definition of a function-like macro, each instance
*                     of a parameter shall be enclosed in parentheses
*   Parameter use without parentheses needed for struct field name in register access macro.
*/
#pragma diag_suppress=Pm154
#endif /* __ICCARM__ */

#define  PEND_BEFORE_WRITE(reg,mask) while((pDevice->pRTCRegs->reg&(mask))!=0u)\
        {\
        }

#define  SYNC_AFTER_WRITE(reg,mask) while((pDevice->pRTCRegs->reg&(mask))==0u)\
        {\
        }

#ifdef __ICCARM__
#pragma diag_default=Pm154
#endif /* __ICCARM__ */

#else
    /* pause on pending writes to CR to avoid data loss */
#define  PEND_BEFORE_WRITE(reg,mask)
#define  SYNC_AFTER_WRITE(reg,mask)        
#endif


#if (ADI_RTC_STATIC_CONFIG_SUPPORT_ENABLE == 1) 
typedef struct
{
    uint16_t    CONTROL0;           /*!< 16 bit control register-0 value */  
    uint16_t    CONTROL1;           /*!< 16 bit control register-1 value */
    uint16_t    COUNT0;              /*!< 16 bit count register value */
    uint16_t    COUNT1;              /*!< 16 bit count register value */
   
    uint16_t    ALARAM0;             /*!< 16 bit integer part of alarm value */
    uint16_t    ALARAM1;             /*!< 16 bit integer part of alarm value */
    uint16_t    ALARAM2;             /*!< 16 bit integer part of alarm value */
    uint16_t    TRIM;               /*!< 16 bit trim register value */    
    uint16_t    CONTROL2;           /*!< 16 bit control (which controls the input capture ) register-2 value */
    uint16_t    CONTROL3;           /*!< 16 bit control ( Controls enabling output compare /IRQ etc )register-3 value */
    uint16_t    CONTROL4;           /*!< 16 bit control ( controls Auto reload  and mask for output compare  ) register-4 value */
    uint16_t    MASK;               /*!<  Mask register for output compare channel */
    uint16_t    AUTORELOAD;         /*!< 16 bit Auto reload value */
    
}ADI_RTC_CONFIG;
#endif
/* Device information structure */
typedef struct _ADI_RTC_DEVICE_INFO
{
    volatile ADI_RTC_TypeDef  *pRTCRegs;   /* Base address of the  SPORT registers */
    const IRQn_Type            eIRQn;      /* IRQn */   
    ADI_RTC_HANDLE             hDevice;    /* RTC handle */
}ADI_RTC_DEVICE_INFO;

/*! RTC driver instance data */
typedef struct _ADI_RTC_DEVICE
{
    volatile ADI_RTC_TypeDef *pRTCRegs;    /* Pointer to RTC Memory Mapped Registers */

    ADI_CALLBACK              pfCallback;  /*  Function pointer for callback function. */

    void                     *pCBParam;    /*  Parameter to callback function. */
    IRQn_Type                eIRQn;      /* IRQn */       
    uint32_t  cbWatch;
    ADI_RTC_DEVICE_INFO     *pDeviceInfo;  /*  Parameter to callback function. */
} ADI_RTC_DEVICE;
ADI_INT_HANDLER(RTC0_Int_Handler);
ADI_INT_HANDLER(RTC1_Int_Handler);
#if (ADI_RTC_STATIC_CONFIG_SUPPORT_ENABLE==1)
static void rtc_init(ADI_RTC_DEVICE *pDevice,ADI_RTC_CONFIG *pConfig);
#endif
#ifdef ADI_DEBUG
static ADI_RTC_RESULT ValidateHandle( ADI_RTC_DEVICE *pInDevice);
#endif
/*! \endcond */    
#endif /* __ADI_PWR_DEF_H__ */

