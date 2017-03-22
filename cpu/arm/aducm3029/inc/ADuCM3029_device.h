/*!
 *****************************************************************************
 * @file:    ADuCM3029_device.h
 * @brief:   ADuCM3029 C Register Definitions
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

#ifndef _WRAP_ADUCM3029_DEVICE_H
#define _WRAP_ADUCM3029_DEVICE_H

#ifdef __ICCARM__
/* IAR MISRA C 2004 error suppressions:
 * Pm008 (rule 2.4): sections of code should not be 'commented out'.
 *   Some comments are wrongly identified as code.
 * Pm093 (rule 18.4): use of union - overlapping storage shall not be used.
 *    Unions are required by sys/ADUCM302x_typedefs.h.
 */
#pragma diag_suppress=Pm008,Pm093

/* The generated header does not include stdint.h for IAR. */
#include <stdint.h>

#endif /* __ICCARM__ */

/* Suppress struct definitions with incorrectly signed fields. */
#define __ADI_NO_DECL_STRUCT_ADI_TMR_TypeDef__
#define __ADI_NO_DECL_STRUCT_ADI_WDT_TypeDef__
#define __ADI_NO_DECL_STRUCT_ADI_UART_TypeDef__
#define __ADI_NO_DECL_STRUCT_ADI_DMA_TypeDef__
#define __ADI_NO_DECL_STRUCT_ADI_FLCC_TypeDef__
#define __ADI_NO_DECL_STRUCT_ADI_FLCC_CACHE_TypeDef__
#define __ADI_NO_DECL_STRUCT_ADI_CRC_TypeDef__
#define __ADI_NO_DECL_STRUCT_ADI_CRYPT_TypeDef__
#define __ADI_NO_DECL_STRUCT_ADI_PTI_TypeDef__

/* The generated header. */
#include <sys/ADuCM302x_device.h>

/* Correctly define the structs suppressed above. */
typedef struct _ADI_TMR_TypeDef
{
    __IO     uint16_t LOAD;                          /*!< 16-bit load value */
    __I __C  uint8_t  RESERVED0[2];
    __I __C  uint16_t CURCNT;                        /*!< 16-bit timer value */
    __I __C  uint8_t  RESERVED1[2];
    __IO     uint16_t CTL;                           /*!< Control */
    __I __C  uint8_t  RESERVED2[2];
    __O      uint16_t CLRINT;                        /*!< Clear Interrupt */
    __I __C  uint8_t  RESERVED3[2];
    __I __C  uint16_t CAPTURE;                       /*!< Capture */
    __I __C  uint8_t  RESERVED4[2];
    __IO     uint16_t ALOAD;                         /*!< 16-bit load value, asynchronous */
    __I __C  uint8_t  RESERVED5[2];
    __I __C  uint16_t ACURCNT;                       /*!< 16-bit timer value, asynchronous */
    __I __C  uint8_t  RESERVED6[2];
    __I __C  uint16_t STAT;                          /*!< Status */
    __I __C  uint8_t  RESERVED7[2];
    __IO     uint16_t PWMCTL;                        /*!< PWM Control Register */
    __I __C  uint8_t  RESERVED8[2];
    __IO     uint16_t PWMMATCH;                      /*!< PWM Match Value */
} ADI_TMR_TypeDef;

typedef struct _ADI_WDT_TypeDef
{
    __IO     uint16_t LOAD;                          /*!< Load value */
    __I __C  uint8_t  RESERVED0[2];
    __I __C  uint16_t CCNT;                          /*!< Current count value */
    __I __C  uint8_t  RESERVED1[2];
    __IO     uint16_t CTL;                           /*!< Control */
    __I __C  uint8_t  RESERVED2[2];
    __O      uint16_t RESTART;                       /*!< Clear interrupt */
    __I __C  uint8_t  RESERVED3[10];
    __I __C  uint16_t STAT;                          /*!< Status */
} ADI_WDT_TypeDef;

typedef struct _ADI_UART_TypeDef
{
    union {
    __I __C  uint16_t COMRX;                         /*!< Receive Buffer Register */
    __O      uint16_t COMTX;                         /*!< Transmit Holding Register */
    };
    __I __C  uint8_t  RESERVED0[2];
    __IO     uint16_t COMIEN;                        /*!< Interrupt Enable */
    __I __C  uint8_t  RESERVED1[2];
    __I __C  uint16_t COMIIR;                        /*!< Interrupt ID */
    __I __C  uint8_t  RESERVED2[2];
    __IO     uint16_t COMLCR;                        /*!< Line Control */
    __I __C  uint8_t  RESERVED3[2];
    __IO     uint16_t COMMCR;                        /*!< Modem Control */
    __I __C  uint8_t  RESERVED4[2];
    __I __C  uint16_t COMLSR;                        /*!< Line Status */
    __I __C  uint8_t  RESERVED5[2];
    __I __C  uint16_t COMMSR;                        /*!< Modem Status */
    __I __C  uint8_t  RESERVED6[2];
    __IO     uint16_t COMSCR;                        /*!< Scratch buffer */
    __I __C  uint8_t  RESERVED7[2];
    __IO     uint16_t COMFCR;                        /*!< FIFO Control */
    __I __C  uint8_t  RESERVED8[2];
    __IO     uint16_t COMFBR;                        /*!< Fractional Baud Rate */
    __I __C  uint8_t  RESERVED9[2];
    __IO     uint16_t COMDIV;                        /*!< Baudrate divider */
    __I __C  uint8_t  RESERVED10[2];
    __IO     uint16_t COMLCR2;                       /*!< second Line Control */
    __I __C  uint8_t  RESERVED11[2];
    __IO     uint16_t COMCTL;                        /*!< UART control register */
    __I __C  uint8_t  RESERVED12[2];
    __I __C  uint16_t COMRFC;                        /*!< RX FIFO byte count */
    __I __C  uint8_t  RESERVED13[2];
    __I __C  uint16_t COMTFC;                        /*!< TX FIFO byte count */
    __I __C  uint8_t  RESERVED14[2];
    __IO     uint16_t COMRSC;                        /*!< RS485 half-duplex Control */
    __I __C  uint8_t  RESERVED15[2];
    __IO     uint16_t COMACR;                        /*!< Auto Baud Control */
    __I __C  uint8_t  RESERVED16[2];
    __I __C  uint16_t COMASRL;                       /*!< Auto Baud Status (Low) */
    __I __C  uint8_t  RESERVED17[2];
    __I __C  uint16_t COMASRH;                       /*!< Auto Baud Status (High) */
} ADI_UART_TypeDef;

typedef struct _ADI_DMA_TypeDef
{
    __I __C  uint32_t STAT;                          /*!< DMA Status */
    __O      uint32_t CFG;                           /*!< DMA Configuration */
    __IO     uint32_t PDBPTR;                        /*!< DMA channel primary control data base pointer */
    __I __C  uint32_t ADBPTR;                        /*!< DMA channel alternate control data base pointer */
    __I __C  uint8_t  RESERVED0[4];
    __O      uint32_t SWREQ;                         /*!< DMA channel software request */
    __I __C  uint8_t  RESERVED1[8];
    __IO     uint32_t RMSK_SET;                      /*!< DMA channel request mask set */
    __O      uint32_t RMSK_CLR;                      /*!< DMA channel request mask clear */
    __IO     uint32_t EN_SET;                        /*!< DMA channel enable set */
    __O      uint32_t EN_CLR;                        /*!< DMA channel enable clear */
    __IO     uint32_t ALT_SET;                       /*!< DMA channel primary-alternate set */
    __O      uint32_t ALT_CLR;                       /*!< DMA channel primary-alternate clear */
    __O      uint32_t PRI_SET;                       /*!< DMA channel priority set */
    __O      uint32_t PRI_CLR;                       /*!< DMA channel priority clear */
    __I __C  uint8_t  RESERVED2[8];
    __IO     uint32_t ERRCHNL_CLR;                   /*!< DMA Per Channel Error Clear */
    __IO     uint32_t ERR_CLR;                       /*!< DMA bus error clear */
    __IO     uint32_t INVALIDDESC_CLR;               /*!< DMA Per Channel Invalid Descriptor Clear */
    __I __C  uint8_t  RESERVED3[1964];
    __IO     uint32_t BS_SET;                        /*!< DMA channel bytes swap enable set */
    __O      uint32_t BS_CLR;                        /*!< DMA channel bytes swap enable clear */
    __I __C  uint8_t  RESERVED4[8];
    __IO     uint32_t SRCADDR_SET;                   /*!< DMA channel source address decrement enable set */
    __O      uint32_t SRCADDR_CLR;                   /*!< DMA channel source address decrement enable clear */
    __IO     uint32_t DSTADDR_SET;                   /*!< DMA channel destination address decrement enable set */
    __O      uint32_t DSTADDR_CLR;                   /*!< DMA channel destination address decrement enable clear */
    __I __C  uint8_t  RESERVED5[1984];
    __I __C  uint32_t REVID;                         /*!< DMA Controller Revision ID */
} ADI_DMA_TypeDef;

typedef struct _ADI_FLCC_TypeDef
{
    __IO     uint32_t STAT;                          /*!< Status */
    __IO     uint32_t IEN;                           /*!< Interrupt Enable */
    __IO     uint32_t CMD;                           /*!< Command */
    __IO     uint32_t KH_ADDR;                       /*!< WRITE Address */
    __IO     uint32_t KH_DATA0;                      /*!< WRITE Lower Data */
    __IO     uint32_t KH_DATA1;                      /*!< WRITE Upper Data */
    __IO     uint32_t PAGE_ADDR0;                    /*!< Lower Page Address */
    __IO     uint32_t PAGE_ADDR1;                    /*!< Upper Page Address */
    __O      uint32_t KEY;                           /*!< Key */
    __I __C  uint32_t WR_ABORT_ADDR;                 /*!< Write Abort Address */
    __IO     uint32_t WRPROT;                        /*!< Write Protection */
    __I __C  uint32_t SIGNATURE;                     /*!< Signature */
    __IO     uint32_t UCFG;                          /*!< User Configuration */
    __IO     uint32_t TIME_PARAM0;                   /*!< Time Parameter 0 */
    __IO     uint32_t TIME_PARAM1;                   /*!< Time parameter 1 */
    __IO     uint32_t ABORT_EN_LO;                   /*!< IRQ Abort Enable (lower bits) */
    __IO     uint32_t ABORT_EN_HI;                   /*!< IRQ Abort Enable (upper bits) */
    __IO     uint32_t ECC_CFG;                       /*!< ECC Config */
    __I __C  uint32_t ECC_ADDR;                      /*!< ECC Status (Address) */
    __I __C  uint8_t  RESERVED0[9];
} ADI_FLCC_TypeDef;

typedef struct _ADI_FLCC_CACHE_TypeDef
{
    __I __C  uint32_t STAT;                          /*!< Cache Status register */
    __IO     uint32_t SETUP;                         /*!< Cache Setup register */
    __O      uint32_t KEY;                           /*!< Cache Key register */
    __I __C  uint8_t  RESERVED0[37];
} ADI_FLCC_CACHE_TypeDef;

typedef struct _ADI_CRC_TypeDef
{
    __IO     uint32_t CTL;                           /*!< CRC Control Register */
    __O      uint32_t IPDATA;                        /*!< Input Data Word Register */
    __IO     uint32_t RESULT;                        /*!< CRC Result Register */
    __IO     uint32_t POLY;                          /*!< Programmable CRC Polynomial */
    union {
    __O      uint8_t IPBITS[8];                     /*!< Input Data Bits */
    __O      uint8_t IPBYTE;                        /*!< Input Data Byte */
    };
} ADI_CRC_TypeDef;

typedef struct _ADI_CRYPT_TypeDef
{
    __IO     uint32_t CFG;                           /*!< Configuration Register */
    __IO     uint32_t DATALEN;                       /*!< Payload Data Length */
    __IO     uint32_t PREFIXLEN;                     /*!< Authentication Data Length */
    __IO     uint32_t INTEN;                         /*!< Interrupt Enable Register */
    __IO     uint32_t STAT;                          /*!< Status Register */
    __O      uint32_t INBUF;                         /*!< Input Buffer */
    __I __C  uint32_t OUTBUF;                        /*!< Output Buffer */
    __IO     uint32_t NONCE0;                        /*!< Nonce Bits [31:0] */
    __IO     uint32_t NONCE1;                        /*!< Nonce Bits [63:32] */
    __IO     uint32_t NONCE2;                        /*!< Nonce Bits [95:64] */
    __IO     uint32_t NONCE3;                        /*!< Nonce Bits [127:96] */
    __O      uint32_t AESKEY0;                       /*!< Key Bits[ 31:0 ] */
    __O      uint32_t AESKEY1;                       /*!< Key Bits [ 63:32 ] */
    __O      uint32_t AESKEY2;                       /*!< Key Bits [ 95:64 ] */
    __O      uint32_t AESKEY3;                       /*!< Key Bits [ 127:96 ] */
    __O      uint32_t AESKEY4;                       /*!< Key Bits [ 159:128 ] */
    __O      uint32_t AESKEY5;                       /*!< Key Bits [ 191:160 ] */
    __O      uint32_t AESKEY6;                       /*!< Key Bits [ 223:192 ] */
    __O      uint32_t AESKEY7;                       /*!< Key Bits [ 255:224 ] */
    __IO     uint32_t CNTRINIT;                      /*!< Counter Initialization Vector */
    __IO     uint32_t SHAH0;                         /*!< SHA Bits [ 31:0 ] */
    __IO     uint32_t SHAH1;                         /*!< SHA Bits [ 63:32 ] */
    __IO     uint32_t SHAH2;                         /*!< SHA Bits [ 95:64 ] */
    __IO     uint32_t SHAH3;                         /*!< SHA Bits [ 127:96 ] */
    __IO     uint32_t SHAH4;                         /*!< SHA Bits [ 159:128 ] */
    __IO     uint32_t SHAH5;                         /*!< SHA Bits [ 191:160 ] */
    __IO     uint32_t SHAH6;                         /*!< SHA Bits [ 223:192] */
    __IO     uint32_t SHAH7;                         /*!< SHA Bits [ 255:224 ] */
    __IO     uint32_t SHA_LAST_WORD;                 /*!< SHA Last Word and Valid Bits Information */
    __IO     uint32_t CCM_NUM_VALID_BYTES;           /*!< NUM_VALID_BYTES */
} ADI_CRYPT_TypeDef;

typedef struct _ADI_PTI_TypeDef
{
    __IO     uint32_t RST_ISR_STARTADDR;             /*!< Reset ISR Start Address */
    __IO     uint32_t RST_STACK_PTR;                 /*!< Reset Stack Pointer */
    __IO     uint32_t CTL;                           /*!< Parallel Test Interface Control Register */
} ADI_PTI_TypeDef;

/* Add missing ADC events. */
#define DMA0_CH24_DONE_IRQn ((IRQn_Type)39)
#define ADC0_EVT_IRQn ((IRQn_Type)46)

#ifdef __ICCARM__
#pragma diag_default=Pm008,Pm093
#endif /* __ICCARM__ */

#if defined (__CC_ARM)
/* local undef storage classes */
#undef __I
#undef __C
#undef __O
#undef __IO
#endif /* __CC_ARM */

#endif /* _WRAP_ADUCM3029_DEVICE_H__ */
