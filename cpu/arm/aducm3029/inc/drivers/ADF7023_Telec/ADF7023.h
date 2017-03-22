/*!
 ******************************************************************************
 * @file:    uhf.h
 * @brief:   UHF Device Definitions for ADuCRFxxx
 * @version: $Revision: 13580 $
 * @date:    $Date: 2012-04-06 16:16:27 -0400 (Fri, 06 Apr 2012) $
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2010 Analog Devices, Inc.

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

/*! \addtogroup ADF7023_Driver ADF7023 Driver
 *  @{
 */

#ifndef __UHF_H__
#define __UHF_H__


/* system service includes */
#include <stdint.h>
#include <services/dma/adi_dma.h>
#include <drivers/spi/adi_spi.h>
#include <services/gpio/adi_gpio.h>

#include <stdbool.h>
#include "net/netstack.h"
#include "device.h"
#include "telec_common.h"

/******************************************************
 * Header file to define 7023 bit fields and commands *
 ******************************************************/



/* Initial defines */
//#define  PATCH_802_15_4D     0x1  // flag to PATCH enable/disable

//#define PATCH_802_DBGCOUNTS  0x1    // macro to use or not use in the dbg counts in the 15d4 ISR for TX/RX

//#define  DO_IR_CALIBRATION  1
// Enable this flag for 50kbps data rate and disable for 100kbps data rate.

#define  USE_50K_DATARATE

//#define UTEST_UHF

#define CHANNEL_FREQUENCY		922100000

/*! Default ADF7023 data rate */

#ifdef USE_50K_DATARATE
	#define UHF_DEFAULT_DATA_RATE                      (500)   // in BPS 500*100 = 50kbps
	#define UHF_FREQ_DEVIATION	 			(UHF_DEFAULT_DATA_RATE * 100)/2

#else
	#define UHF_DEFAULT_DATA_RATE                      (1000)   // in BPS 1000*100 = 100kbps
	#define UHF_FREQ_DEVIATION	 			(UHF_DEFAULT_DATA_RATE * 100)/2
#endif

/*! Default ADF7023 frequency*/
#define UHF_DEFAULT_RADIO_FREQ                     (900000000UL)   //in Hz
#ifdef PATCH_802_15_4D
/*! Default start address of a TX packet in packet RAM.
 * Change this value to load packets at a different address.*/
#define PKT_RAM_TXBASE                 0x20

/*! Default start address of a RX packet in packet RAM.
 * Change this value to load packets at a different address.*/
#define PKT_RAM_RXBASE                 0x20

#else
/*! Default start address of a TX packet in packet RAM.
 * Change this value to load packets at a different address.*/
#define PKT_RAM_TXBASE                 0x10

/*! Default start address of a RX packet in packet RAM.
 * Change this value to load packets at a different address.*/
#define PKT_RAM_RXBASE                 0x10
#endif
/*! Default max packet length*/
#define MAX_PKT_LEN                    0x80

#define SPI_SCK_HERTZ                  1000000 /* 1 Mhz SPI clock */

#define ENABLE_IFBW_AUTO_SWITCH_BIT_POS	5

#define CCA_FILTER_BIT_POS 				5

#define ENABLE_IFBW_SWITCH_ON_PREAMBLE_BIT_POS 6

#define DEFAULT_AGC_CLK_DIVIDE_VAL		0x12

/**********************
 * register bitfields *
 *********************/

/* Interrupt mask 0 register */

#ifdef PATCH_802_15_4D

#define IDLEMODE		0
#define TXMODE		1
#define RXMODE		2

#ifdef TELEC_TEST_HARNESS
#define MAX_PDSU_15_4D  2047
#else
/* For 6LoWPAN compatibility MAX_PDSU_15_4D should be 150. */ 
#define MAX_PDSU_15_4D  150
#endif

#define USE_PDSU_15_4D  MAX_PDSU_15_4D

#define MAX_CSUM_SZ    0x4  //Check sum 4 bytes


#define TX_BASE_154D  PKT_RAM_TXBASE
#define TX_SZ_15_4D   20 /*0x80*/
#define RX_BASE_154D  PKT_RAM_RXBASE
#define RX_SZ_15_4D   20 /*0x80*/

#define INT_CCA                             (0x1 << 7)
#define INT_BUFF_FULL			            (0x1 << 6)
#define INT_BUFF_ALMOST_FULL			    (0x1 << 5)
#define INT_RX_EOF			                (0x1 << 4)
#define INT_TX_EOF			                (0x1 << 3)
#define INT_CRC_CORRECT				        (0x1 << 2)
#define INT_PHR_DETECT			            (0x1 << 1)
#define INT_PREAMBLE_DETECT			        (0x1 << 0)

#define ANTENNA_DIVERSITY_BIT_MASK 			1
#define ANTENNA_0							0
#define ANTENNA_1 							2
#define BIT_CONTINOUS_RX  			  		0
#define PHR_LENMSBMASK 						7
#define PHR_FCSMASK							0x10
#define PREAMBLE_SFD_DETECT					3
#define MCR_SYNTH_CAL_OVERWRITE				3
#define MCR_SYNTH_CAL_ENABLE				0

#else
/*! Interrupt_mask_0 - Bit 7 - Number of wakeups threshold exceeded interrupt */
#define INT_NUM_WAKEUPS                ((0x1)<<7)
/*! Interrupt_mask_0 - Bit 6 - Smart wake mode RSSI threshold exceeded interrupt */
#define INT_SWM_RSSI_DET               ((0x1)<<6)
/*! Interrupt_mask_0 - Bit 5 - AES operation complete interrupt */
#define INT_AES_DONE                   ((0x1)<<5)
/*! Interrupt_mask_0 - Bit 4 - TX packet end of frame interrupt */
#define INT_TX_EOF                     ((0x1)<<4)
/*! Interrupt_mask_0 - Bit 3 - Address match on RX interrupt */
#define INT_RX_ADDR_MATCH              ((0x1)<<3)
/*! Interrupt_mask_0 - Bit 2 - CRC correct interrupt */
#define INT_CRC_CORRECT                ((0x1)<<2)
/*! Interrupt_mask_0 - Bit 1 - Valid sync word detected interrupt */
#define INT_SYNC_DETECT                ((0x1)<<1)
/*! Interrupt_mask_0 - Bit 0 - Preamble detected interrupt */
#define INT_PREAMBLE_DETECT            ((0x1)<<0)
#endif

/* Interrupt mask 1 register */

/*! Interrupt_mask_1 - Bit 7 - Battery voltage less than threshold interrupt*/
#define INT_BATTERY_ALARM               ((0x1)<<15)
/*! Interrupt_mask_1 - Bit 6 - Comms processor ready to receive command interrupt */
#define INT_CMD_READY                   ((0x1)<<14)
/*! Interrupt_mask_1 - Bit 4 - WUC timer expired interrupt*/
#define INT_WUC_TIMEOUT                ((0x1)<<12)
/*! Interrupt_mask_1 - Bit 1 - SPI ready interrupt*/
#define INT_SPI_READY                  ((0x1)<<9)
/*! Interrupt_mask_1 - Bit 0 - Command completed interrupt*/
#define INT_CMD_FINISHED               ((0x1)<<8)

#define AUTO_TX_TO_RX_ENABLE				4

#define BBRam_MMemmap_Start                 0x100
#define INIT_TRSC_PORTS                     0x01

#define CLOCK_FREQ_IN_MHZ                   (26)
#define MAX_SYS_TICK_COUNT                  (104000)
#define TICK_DURATION                       (4000)
#define SYS_TICK_TO_ms_FACTOR               ((float)(1/CLOCK_FREQ_IN_MHZ))

/*----------- Transceiver error codes -------------*/
#define TRE_NO_ERR                            0x00
#define TRE_PWRUP                             0x80
#define TRE_SPI_NOTRDY                        0x81
#define TRE_NW_CMD                            0x82
#define TRE_RST_CMD                           0x83
#define TRE_WAIT_CMD                          0x84
#define TRE_SYNC_CMD                          0x85
#define TRE_XMIT_CMD                          0x86
#define TRE_MAPWR_CMD                         0x87
#define TRE_MAPRD_CMD                         0x88
#define TRE_TO_RX_STATE                       0x89
#define TRE_TO_TX_STATE                       0x8A
#define TRE_SET_TX_PWR                        0x8B
#define TRE_TX_PACK                           0x8C
#define TRE_TO_OFF_STATE                      0x8D
#define TRE_TO_SLEEP_STATE                    0x8E
#define TRE_CFG_CMD                           0x8F
#define TRE_WAIT_A_STATE                      0x90
#define TRE_GET_RSSI                          0x91
#define TRE_TO_ON_STATE                       0x92
#define TRE_CONFIG_ERR                        0xA0
#define TRE_TX_PACK_1                         0xB0
#define TRE_TX_PACK_2                         0xB1
#define TRE_TX_PACK_3                         0xB2
#define TRE_TX_PACK_4                         0xB3
#define TRE_DEBUG                             0xB4
#define I_SUPPORT_4BYTE_FCS                   0x01

#if ( I_SUPPORT_4BYTE_FCS == 0x01)
#define SIZE_CRC_FIELD 	(4)
#else
#define SIZE_CRC_FIELD 	(2)
#endif // #if ( I_SUPPORT_4BYTE_FCS == 0x01)

#define TRANSMIT_POWER                        0x06

#if (I_SUPPORT_4BYTE_FCS == 0x01)
/* Polynomial to calculate 32 bit CRC */
#define CRCPOLY2 0xEDB88320UL
#endif // I_SUPPORT_4BYTE_FCS

#define MAX_TX_PWR                  63

typedef signed long                    S32;
typedef signed short                   S16;
typedef signed char                    S8;
typedef volatile signed long           VS32;
typedef volatile signed short          VS16;
typedef volatile signed char           VS8;
typedef unsigned long                  U32;
typedef unsigned short                 U16;
typedef unsigned char                  U8;
typedef unsigned long  const           UC32;
typedef unsigned short const           UC16;
typedef unsigned char  const           UC8;
typedef volatile unsigned long         VU32;
typedef volatile unsigned short        VU16;
typedef volatile unsigned char         VU8;
typedef volatile unsigned long  const  VUC32;
typedef volatile unsigned short const  VUC16;
typedef volatile unsigned char  const  VUC8;
typedef volatile U8  *                 U8P;
typedef volatile U16 *                 U16P;
typedef volatile U32 *                 U32P;
typedef volatile S8  *                 S8P;
typedef volatile S16 *                 S16P;
typedef volatile S32 *                 S32P;

/*!
 *****************************************************************************
 * \enum ADI_ADF7023_RESULT_TYPE
 *
 * UHF error codes
 *****************************************************************************/
/*************************************************************************/
/* CMD Codes                                                             */
/*************************************************************************/
typedef enum
{
  CMD_SYNC                        = 0xA2, // Synchronizes the communication processor to the host microprocessor after reset
  CMD_PHY_OFF                     = 0xB0, // Invoke transition of device into state PHY_OFF
  CMD_PHY_ON                      = 0xB1, // Invoke transition of device into state PHY_ON
  CMD_PHY_RX                      = 0xB2, // Invoke transition of device into state PHY_RX
  CMD_PHY_TX                      = 0xB5, // Invoke transition of device into state PHY_TX
  CMD_PHY_SLEEP                   = 0xBA, // Invoke transition of device into state PHY_SLEEP
  CMD_CONFIG_DEV                  = 0xBB, // Configures the radio parameters based on the BBRAM values.
  CMD_GET_RSSI                    = 0xBC, // Performs an RSSI measurement
  CMD_BB_CAL                      = 0xBE, // Performs an calibration of the IF filter
  CMD_HW_RESET                    = 0xC8, // Performs a full hardware reset. The device enters PHY_SLEEP
  CMD_RAM_LOAD_INIT               = 0xBF, // Prepares the program RAM for a download
  CMD_RAM_LOAD_DONE               = 0xC7, // Performs a reset of the communications processor after loading RAM
  CMD_IR_CAL                      = 0xBD, // Initiates an image rejection calibration using the IR cal code stored on program RAM
  CMD_AES_ENCRYPT                 = 0xD0, // Requires the AES software module
  CMD_AES_DECRYPT_INIT            = 0xD1, // Requires the AES software module
  CMD_AES_DECRYPT                 = 0xD2, // Requires the AES software module
  SPI_MEM_WR                      = 0x18, // Sequential Write
  SPI_MEM_RD                      = 0x38, // Sequential Read
  SPI_MEMR_WR                     = 0x08, // Random Write
  SPI_MEMR_RD                     = 0x28, // Random Read
  SPI_NOP                         = 0xFF  // No operation
#ifndef DISABLE_PATCH_802_15_4D
  ,
  CMD_ENTER_15d4_MODE             = 0xC1,  // Transition from PHY_ON to PHY_ON_15d4
  CMD_EXIT_15d4_MODE              = 0xB1,  // Transition from PHY_ON_15d4 to PHY_ON
  CMD_PHY_RX_CCA_15d4             = 0xB2,  // Transition from XXXX_15d4 states to PHY_RX_CCA_15d4
  CMD_PHY_TX_15d4                 = 0xB5,  // Transition from XXXX_15d4 states to PHY_TX_15d4
  CMD_PHY_ON_15d4                 = 0xB1,  // Transition from PHY_TX_15d4,PHY_RX_CCA_15d4 states to PHY_ON_15d4
  CMD_SYNTH_CAL_15d4              = 0xEE,  // Carry out a full synth calibration, makes for fast RX,TX
  CMD_CCA_START                   = 0xB7,  // Starts the CCA timer sets the CCA_flag
  CMD_CCA_STOP                    = 0xB8,  // Stops the CCA timer clears the CCA_flag
  CMD_CCA_FILTER                  = 0xB9  // Sets the RXBB Filter to the value in BIT_CCA_FILTER_REG
#endif // DISABLE_PATCH_802_15_4D

} ADF_CmdCodes;


typedef enum
{
  TRPS_UP = 0x00,
  TRPS_DOWN,
  TRPS_SHUTDOWN
}TRPSSTATE;

typedef enum
{
  FW_INIT               = 0x0F, // Initializing
  FW_BUSY               = 0x00, // Busy. Performing a state transition
  FW_OFF                = 0x11,
  FW_ON                 = 0x12,
  FW_RX                 = 0x13,
  FW_TX                 = 0x14,
  FW_RSSI               = 0x05, // Performing CMD_GET_RSSI
  FW_SLEEP              = 0x06, // PHY_SLEEP
  FW_IR_CAL             = 0x07, // Performing CMD_IR_CAL
  FW_AES_INIT           = 0x08, // Performing CMD_AES_INIT
  FW_AES_DEC            = 0x09, // Performing CMD_AES_DEC
  FW_AES_ENC            = 0x0A  // Performing CMD_AES_ENC
#ifndef DISABLE_PATCH_802_15_4D
  ,
  FW_ON_15D4            = 0x18,
  FW_RX_CCA_15D4        = 0x1A,
  FW_TX_15D4            = 0x1C
#endif // DISABLE_PATCH_802_15_4D
}ADF_FwState;

typedef enum
{
  ISMBAND_863_870MHZ = 0,
  ISMBAND_896_901MHZ,
  ISMBAND_901_902MHZ,
  ISMBAND_902_928MHZ,
  ISMBAND_917_923MHZ,
  ISMBAND_928_960MHZ,
  ISMBAND_920_928MHZ,
  ISMBAND_950_958MHZ,
  ISMBAND_868MHZ,
  ISMBAND_433MHZ,
  ISMBAND_915MHZ,
  ISMBAND_2500MHZ
}ISMBAND;

typedef enum
{
    ADI_ADF7023_SUCCESS = 0,                       /*!< Successful completion of call */
    ADI_ADF7023_ERR_UNKNOWN,                        /*!< Something not covered by the rest of the error conditions */

    ADI_ADF7023_ERR_ALREADY_INITIALIZED,           /*!< UHF device already initialized */
    ADI_ADF7023_ERR_PHY_OFF,                       /*!< Error encountered while trying to turn 7023 PHY to OFF mode */
    ADI_ADF7023_ERR_PHY_ON,                        /*!< Error encountered while trying to turn 7023 PHY to ON mode */
    ADI_ADF7023_ERR_PHY_TX,                        /*!< Error encountered while trying to turn 7023 PHY to TX mode */
    ADI_ADF7023_ERR_PHY_RX,                        /*!< Error encountered while trying to turn 7023 PHY to RX mode */
    ADI_ADF7023_ERR_BAD_GPIO_CONFIG,               /*!< Error while initializing the GPIO service */
    ADI_ADF7023_ERR_BAD_SPI_CONFIG,                /*!< Error wihle initializing the SPI driver */
    ADI_ADF7023_ERR_BLOCKWRITE,                    /*!< Error while trying to write a block over SPI to the radio */
    ADI_ADF7023_ERR_INIT_FAILED,                   /*!< Initialization error */
    ADI_ADF7023_ERR_PREAMBLELEN,                   /*!< Error while trying to set preamble length */
    ADI_ADF7023_ERR_RESET_FAILURE,                 /*!< Error while issuing CMD_HW_RESET */
    ADI_ADF7023_ERR_UNSUPPORTED_RATE,              /*!< Invalid data rate */
    ADI_ADF7023_ERR_RSSI_READ_FAIL,                /*!< Error while trying to read current RSSI */
    ADI_ADF7023_ERR_INVALID_SYNC_WORD,             /*!< Invalid sync word from application */
    ADI_ADF7023_ERR_INVALID_PA_LEVEL,              /*!< Invalid power amplifier level */
    ADI_ADF7023_ERR_CHANNEL_FREQ,                  /*!< Unsupported radio frequency */
    ADI_ADF7023_ERR_INVALID_PA_RAMP_RATE,          /*!< Invalid power amplifier ramp rate */
    ADI_ADF7023_ERR_INVALID_IFBW,                  /*!< Invalid IF bandwidth */
    ADI_ADF7023_ERR_INVALID_MOD_SCHEME,            /*!< Attempt to set radio to unsupported modulation scheme */
    ADI_ADF7023_ERR_INVALID_DEMOD_SCHEME,          /*!<  Attempt to set radio to unsupported demodulation scheme  */
    ADI_ADF7023_ERR_INVALID_FREQ_DEV,              /*!< Invalid frequency deviation */
    ADI_ADF7023_ERR_DISCRIMBW,                     /*!< Invalid discriminator bandwidth */
    ADI_ADF7023_ERR_POSTDEMODBW,                   /*!< Invalid post demodulator bandwidth */
    ADI_ADF7023_ERR_BAD_CMD,                       /*!< Attempt to issue invalid command  */
    ADI_ADF7023_ERR_RADIO_BUSY,                    /*!< The driver already has a valid buffer that is being processed  */
    ADI_ADF7023_ERR_RADIO_SPORT_MODE,              /*!< The radio is in SPORT mode. So the ReceiveFrame, and Transmit
    Frame SPI APIs cannot be used in this case. Use SPORT APIs for transfers.  */
    ADI_ADF7023_ERR_SPI_BUSY,				       /*!< SPI already busy with data transfer */
    ADI_ADF7023_ERR_SPORT_INIT,					   /*!< Error while SPORT init */
    ADI_ADF7023_ERR_SPORT_DISABLE,				   /*!< Error when tried to use SPORT when disable 	*/
    ADI_ADF7023_ERR_SPORT_SUBMIT_BUFF_FAIL,        /*!< Error when Submitted buffer for sport failed*/
    ADI_ADF7023_ERR_SPORT_ENABLE_DATA_FAIL,		   /*!< Error for SPORT data enable failed*/
    ADI_ADF7023_ERR_SPORT_RX_ALREADY_ENABLED,	   /*!< Error for re-eanbling SPORT RX*/
    ADI_ADF7023_ERR_SPORT_RX_SPORT_NOT_ENABLED,	   /*!< Error for SPORT RX not enabled */
    ADI_ADF7023_ERR_SPORT_RX_BUF_IN_USE,		   /*!< Error for trying to use another RX buffer when one is already in use */
    ADI_ADF7023_ERR_RADIO_IR_CAL_BUSY,             /*!< Error as Radio is busy with IR calibration */
    ADI_ADF7023_ERR_RADIO_INVALID_TESTMODE,		   /*!< Invalid Variable test mode value */
    ADI_ADF7023_ERR_RADIO_INVALID_CCA_RSSI_THERSHOLD, /*!< Invalid RSSI thershold value for CCA */
    ADI_ADF7023_ERR_RADIO_INVALID_TIMER_DURATION,  /*!< Invalid timer duration for CCA evalution */
    ADI_ADF7023_ERR_IR_CAL_PRAM_DATA_READ,		   /*!< Error while verfication of IR firmware 	*/
    ADI_ADF7023_ERR_RADIO_INVALID_STATE_CHANGE	   /*!< Error while waiting to reach a PHY state 	*/
} ADI_ADF7023_RESULT_TYPE;


/*!
 *****************************************************************************
 * \enum ADI_ADF7023_PHY_STATE_TYPE
 *
 * UHF radio states
 *****************************************************************************/
typedef enum
{
    PHY_OFF = 0x11,                            /*!< Phy in OFF mode */
    PHY_ON = 0x12,                             /*!< Phy in ON mode */
    PHY_RX = 0x13,                             /*!< Phy in RX mode */
    PHY_TX = 0x14,                             /*!< Phy in TX mode */
#ifdef  PATCH_802_15_4D
	PHY_ON_15D4     = 0x18,					   /*!< Phy in 15D4_ON mode */
   	PHY_RX_CCA_15D4 = 0x1A,					   /*!< Phy in 15D4_RX mode */
   	PHY_TX_15D4     = 0x1C,					   /*!< PHy in 15D4_TX mode */
#endif
    PHY_SLEEP = 0x6,                           /*!< Phy in sleep mode */
    PHY_CMD_AES_INIT = 	0x08,                  /*!< Radio currently performing AES initialization */
    PHY_CMD_AES_DEC = 	0x09,                  /*!< Radio currently performing AES decryption */
    PHY_CMD_AES_ENC = 	0x0A,                  /*!< Radio currently performing AES encryption */
    PHY_CMD_GET_RSSI = 	0x05,                  /*!< Radio reading current received signal strength */
    PHY_CMD_IR_CAL = 	0x07,                  /*!< Radio performing image rejection calibration */
  	//PHY_BUSY = 0x00, 						   /*!< Phy busy */
    PHY_BUSY_TRX = 0x00,					   /*!< Phy busy used by stack,so continued to use */
    PHY_STATE_UNKNOWN,                         /*!< Invalid phy state */
} ADI_ADF7023_PHY_STATE_TYPE;


/*!
 *****************************************************************************
 * \enum ADI_ADF7023_STATE_TYPE
 *
 * UHF initialization state
 *****************************************************************************/
typedef enum
{
    ADI_ADF7023_STATE_UNKNOWN = 0,                  /*!< Radio currently uninitialized */
    ADI_ADF7023_STATE_INITIALIZED,                  /*!< Radio initialized */
} ADI_ADF7023_STATE_TYPE;


/*!
 *****************************************************************************
 * \enum ADI_ADF7023_MOD_TYPE
 *
 * UHF modulation types
 *****************************************************************************/
typedef enum
{
    MOD_2FSK = 0,                               /*!< FSK mode */
    MOD_2GFSK,                                  /*!< GFSK mode */
    MOD_OOK_RESERVED,                           /*!< OOK mode/RESERVED */
    MOD_CARRIER,                                /*!< Transmit carrier */
    MOD_UNKNOWN,                                /*!< Unknown modulation scheme */
} ADI_ADF7023_MOD_TYPE;


/*!
 *****************************************************************************
 * \enum ADI_ADF7023_DEMOD_TYPE
 *
 * UHF demodulation types
 *****************************************************************************/
typedef enum
{
    DEMOD_2FSK_GFSK_MSK_GMSK = 0,                /*!< 2FSK/GFSK/MSK/GMSK mode */
    DEMOD_UNKNOWN,                               /*!< Reserved/Unknown */
} ADI_ADF7023_DEMOD_TYPE;

/*!
 *****************************************************************************
 * \enum ADI_ADF7023_CMD_TYPE
 *
 * UHF command types
 *****************************************************************************/
typedef enum
{
    /* SPI Commands */
    ADI_ADF7023_CMD_NULL,
    ADI_ADF7023_CMD_SPI_MEM_WR       =  0x3,          /*!<  7023 SPI memory write command */
    ADI_ADF7023_CMD_SPI_MEM_RD       =  0x7,          /*!<  7023 SPI memory read command */
    ADI_ADF7023_CMD_SPI_MEMR_WR      =  0x1,          /*!<  7023 SPI memory random write command */
    ADI_ADF7023_CMD_SPI_MEMR_RD      =  0x5,          /*!<  7023 SPI memory random read command */
    ADI_ADF7023_CMD_SPI_NOP          =  0xFF,         /*!<  7023 NOP command  */

    /* Radio controller commands */
    ADI_ADF7023_CMD_SYNC             =  0xA2,         /*!<  Command sync command */
    ADI_ADF7023_CMD_PHY_OFF          =  0xB0,         /*!<  Command to turn PHY OFF */
    ADI_ADF7023_CMD_PHY_ON           =  0xB1,         /*!<  Command to turn PHY ON  */
    ADI_ADF7023_CMD_PHY_RX           =  0xB2,         /*!<  Command to turn PHY RX  */
    ADI_ADF7023_CMD_PHY_TX           =  0xB5,         /*!<  Command to turn PHY TX  */
    ADI_ADF7023_CMD_PHY_SLEEP        =  0xBA,         /*!<  Command to set PHY to sleep mode  */
    ADI_ADF7023_CMD_CONFIG_DEV       =  0xBB,         /*!<  Command to configure radio after updating BBRAM registers  */
    ADI_ADF7023_CMD_GET_RSSI         =  0xBC,         /*!<  Command to get current RSSI  */
    ADI_ADF7023_CMD_IR_CAL           =  0xBD,         /*!<  Command to perform IR calibration  */
    ADI_ADF7023_CMD_BB_CAL           =  0xBE,         /*!<  Command to perform BB calibration  */
    ADI_ADF7023_CMD_RAM_LOAD_INIT    =  0xBF,         /*!<  Command to load code into the program RAM area of the 7023 */
    ADI_ADF7023_CMD_RAM_LOAD_DONE    =  0xC7,         /*!<  Command to let 7023 know that code loading in program RAM is complete  */
    ADI_ADF7023_CMD_HW_RESET	     =  0xC8,         /*!<  Command to reset the 7023  */
    ADI_ADF7023_CMD_AES_ENCRYPT      =  0xD0,         /*!<  Command to perform AES encryption  */
    ADI_ADF7023_CMD_AES_DECRYPT      =  0xD2,         /*!<  Command to perform AES decryption */
    ADI_ADF7023_CMD_AES_DECRYPT_INIT =  0xD1,         /*!<  Command to initiate AES decryption mode */
#ifdef PATCH_802_15_4D
    ADI_ADF7023_CMD_ENTER_15D4_MODE  = 0xC1, 		  /*!<	Transions from PHY_ON to PHY_ON_15d4 */
    ADI_ADF7023_CMD_EXIT_15D4_MODE   = 0xB1, 		  /*!<  Transions from PHY_ON_15d4 to PHY_ON */
    ADI_ADF7023_CMD_PHY_RX_CCA_15D4  = 0xB2, 		  /*!<  Transions from XXXX_15d4 states to PHY_RX_CCA_15d4 */
    ADI_ADF7023_CMD_PHY_TX_15D4      = 0xB5, 		  /*!<  Transions from XXXX_15d4 states to PHY_TX_15d4 */
    ADI_ADF7023_CMD_PHY_ON_15D4      = 0xB1, 		  /*!<  Transions from PHY_TX_15d4,PHY_RX_CCA_15d4 states to PHY_ON_15d4 */
    ADI_ADF7023_CMD_SYNTH_CAL_15D4   = 0xEE, 		  /*!<  Carry out a full synth calibration, makes for fast RX,TX */
    ADI_ADF7023_CMD_CCA_START        = 0xB7,		  /*!<  Starts the CCA timer sets the CCA_flag  */
    ADI_ADF7023_CMD_CCA_STOP         = 0xB8, 		  /*!<  Stops the CCA timer clears the CCA_flag */
    ADI_ADF7023_CMD_CCA_FILTER       = 0xB9  		  /*!<  Sets the RXBB Filter to the value in BIT_CCA_FILTER_REG */
#endif
} ADI_ADF7023_CMD_TYPE;


typedef void (* ADI_ADF7023_CALLBACK)
(     /* Callback function pointer                */
    void    *pCBParam,                  /* Client supplied callback param           */
    u32     nEvent,                     /* Event ID                                 */
    void    *pArg);                     /* Pointer to the event specific argument   */

typedef struct
{
  unsigned char interrupt_mask_0_r;                   // 0x100
  unsigned char interrupt_mask_1_r;                   // 0x101
  unsigned char number_of_wakeups_0_r;                // 0x102
  unsigned char number_of_wakeups_1_r;                // 0x103
  unsigned char number_of_wakeups_irq_threshold_0_r;  // 0x104
  unsigned char number_of_wakeups_irq_threshold_1_r;  // 0x105
  unsigned char rx_dwell_time_r;                      // 0x106
  unsigned char parmtime_divider_r;                   // 0x107
  unsigned char swm_rssi_thresh_r;                    // 0x108
  unsigned char channel_freq_0_r;                     // 0x109
  unsigned char channel_freq_1_r;                     // 0x10A
  unsigned char channel_freq_2_r;                     // 0x10B
  unsigned char radio_cfg_0_r;                        // 0x10C
  unsigned char radio_cfg_1_r;                        // 0x10D
  unsigned char radio_cfg_2_r;                        // 0x10E
  unsigned char radio_cfg_3_r;                        // 0x10F
  unsigned char radio_cfg_4_r;                        // 0x110
  unsigned char radio_cfg_5_r;                        // 0x111
  unsigned char radio_cfg_6_r;                        // 0x112
  unsigned char radio_cfg_7_r;                        // 0x113
  unsigned char radio_cfg_8_r;                        // 0x114
  unsigned char radio_cfg_9_r;                        // 0x115
  unsigned char radio_cfg_10_r;                       // 0x116
  unsigned char radio_cfg_11_r;                       // 0x117
  unsigned char image_reject_cal_phase_r;             // 0x118
  unsigned char image_reject_cal_amplitude_r;         // 0x119
  unsigned char mode_control_r;                       // 0x11A
  unsigned char preamble_match_r;                     // 0x11B
  unsigned char symbol_mode_r;                        // 0x11C
  unsigned char preamble_len_r;                       // 0x11D
  unsigned char crc_poly_0_r;                         // 0x11E
  unsigned char crc_poly_1_r;                         // 0x11F
  unsigned char sync_control_r;                       // 0x120
  unsigned char sync_byte_0_r;                        // 0x121
  unsigned char sync_byte_1_r;                        // 0x122
  unsigned char sync_byte_2_r;                        // 0x123
  unsigned char tx_base_adr_r;                        // 0x124
  unsigned char rx_base_adr_r;                        // 0x125
  unsigned char packet_length_control_r;              // 0x126
  unsigned char packet_length_max_r;                  // 0x127
  unsigned char static_reg_fix_r;                     // 0x128
  unsigned char address_match_offset_r;               // 0x129
  unsigned char address_filtering_r[0x14];            // 0x12A - 0x13D
  unsigned char rx_synth_lock_time_r;                 // 0x13E
  unsigned char tx_synth_lock_time_r;                 // 0x13F
} TyBBRAM;

#ifndef DISABLE_PATCH_802_15_4D

typedef struct
{
  unsigned char bb_cca_cfg_0;                        // 0x103
  unsigned char bb_cca_cfg_1;                        // 0x104
  unsigned char bb_ca_threshold;                     // 0x105
} TyBBRAM15_4d_CCA;

#endif // DISABLE_PATCH_802_15_4D

/*!
 *****************************************************************************
 * \struct ADI_ADF7023_SHORT_ADDR_TYPE
 *
 * UHF short address type. Used to pass radio short address and short address mask to the driver.
 *****************************************************************************/
typedef struct
{
	u16 ShortAddr;                         /*!< Radio short address */
    u16 ShortAddrMask;                     /*!< Short address mask */
} ADI_ADF7023_SHORT_ADDR_TYPE;


/*!
 *****************************************************************************
 * \struct ADI_ADF7023_GENERIC_SETTINGS_TYPE
 *
 * UHF Device Generic Settings Structure. Set basic settings for the radio in a single call.
 *****************************************************************************/
typedef struct
{
    u16                         DataRate;        /*!< Set radio data rate */
    u32                         Frequency;       /*!< Set radio frequency */
    ADI_ADF7023_SHORT_ADDR_TYPE     ShortAddr;       /*!< Set radio short address */
    ADI_ADF7023_MOD_TYPE            ModScheme;       /*!< Set radio modulation scheme */
    ADI_ADF7023_DEMOD_TYPE          DemodScheme;     /*!< Set radio demodulation scheme */
} ADI_ADF7023_GENERIC_SETTINGS_TYPE;


/*!
 * ***************************************************************************
 * \struct ADI_ADF7023_BUFFER_TYPE
 *
 * UHF buffer type.
 * ***************************************************************************/
typedef struct {

    u32              ElementCount;
    u8*              pData;
    volatile bool    bBufferProcessed;
} ADI_ADF7023_BUFFER_TYPE;


/*!
 *****************************************************************************
 * \struct ADI_ADF7023_BBRAM_TYPE
 *
 * UHF device internal register names. For more details please refer to the ADuCRF101 User Guide.
 *****************************************************************************/
typedef struct
{
    u8 interrupt_mask_0_r;                   /*!< Register address 0x100 Interrupt mask */
    u8 interrupt_mask_1_r;                   /*!< Register address 0x101 Interrupt mask */
    u8 number_of_wakeups_0_r;                /*!< Register address 0x102 Wake up controller*/
    u8 number_of_wakeups_1_r;                /*!< Register address 0x103 Wake up controller */
    u8 number_of_wakeups_irq_threshold_0_r;  /*!< Register address 0x104 WUC irq */
    u8 number_of_wakeups_irq_threshold_1_r;  /*!< Register address 0x105 WUC irq */
    u8 max_wakeup_2_sync_time_r;             /*!< Register address 0x106 WUC sync */
    u8 parmtime_divider_r;                   /*!< Register address 0x107 WUC div */
    u8 listen_rssi_thresh_r;                 /*!< Register address 0x108 RSSI threshold register for wakeup*/
    u8 channel_freq_0_r;                     /*!< Register address 0x109 To set radio frequency (lower byte)*/
    u8 channel_freq_1_r;                     /*!< Register address 0x10A To set radio frequency (middle byte)*/
    u8 channel_freq_2_r;                     /*!< Register address 0x10B To set radio frequency (upper byte)*/
    u8 radio_cfg_0_r;                        /*!< Register address 0x10C Radio data rate */
    u8 radio_cfg_1_r;                        /*!< Register address 0x10D Data rate (3:0) +  freq_deviation (7:4) */
    u8 radio_cfg_2_r;                        /*!< Register address 0x10E Frequency deviation */
    u8 radio_cfg_3_r;                        /*!< Register address 0x10F Discriminator_bw */
    u8 radio_cfg_4_r;                        /*!< Register address 0x110 Post demodulation bandwidth */
    u8 radio_cfg_5_r;                        /*!< Register address 0x111 T4 phase adjust lsb */
    u8 radio_cfg_6_r;                        /*!< Register address 0x112 None */
    u8 radio_cfg_7_r;                        /*!< Register address 0x113 None */
    u8 radio_cfg_8_r;                        /*!< Register address 0x114 Power amplifier settings */
    u8 radio_cfg_9_r;                        /*!< Register address 0x115 Mod/demod settings */
    u8 radio_cfg_10_r;                       /*!< Register address 0x116 Afc settings 0 */
    u8 radio_cfg_11_r;                       /*!< Register address 0x117 Afc settings 1 */
   	u8 ir_cal_phase_r;                       /*!< Register address 0x118 IR calibration phase setting */
    u8 ir_cal_amplitude_r;                   /*!< Register address 0x119 IR calibration gain setting */
    u8 mode_control_r;                       /*!< Register address 0x11A Radio settings */
    u8 preamble_match_r;                     /*!< Register address 0x11B Preamble settings */
    u8 symbol_mode_r;                        /*!< Register address 0x11C Symbol config settings */
    u8 preamble_len_r;                       /*!< Register address 0x11D Preamble length register*/
    u8 CRC_poly_low_r;                       /*!< Register address 0x11E User defined CRC polynomial (low byte)*/
    u8 CRC_poly_high_r;                      /*!< Register address 0x11F User defined CRC polynomial (high byte)*/
    u8 sync_control_r;                       /*!< Register address 0x120 Syncword settings */
    u8 sync_byte_0_r;                        /*!< Register address 0x121 Sync word (low byte)*/
    u8 sync_byte_1_r;                        /*!< Register address 0x122 Sync word (mid byte)*/
    u8 sync_byte_2_r;                        /*!< Register address 0x123 Sync word (high byte)*/
    u8 tx_base_adr_r;                        /*!< Register address 0x124 Start address of a TX packet in packet RAM*/
    u8 rx_base_adr_r;                        /*!< Register address 0x125 Start address of a RX packet in packet RAM*/
    u8 packet_length_control_r;              /*!< Register address 0x126 Variable or fixed packet length*/
    u8 packet_length_max_r;                  /*!< Register address 0x127 Maximum packet length*/
    u8 Static_Reg_fix_r;                     /*!< Register address 0x128 None [7:0] static_reg_fix_addr RW 0 Pointer to the first address on BBRAM for the automatic register fix */
    u8 Address_Match_Offset_r;               /*!< Register address 0x129 None [7:0] address_match_offset RW 0 Location of first byte of address information in Packet RAM */
    u8 Address_Length_r;                     /*!< Register address 0x12A None [7:0] address_length RW 0 Number of bytes in each address field (NADR) */
    u8 Address_match_byte_0;                 /*!< Register address 0x12B First byte of radio address*/
    u8 Address_mask_byte_0 ;                 /*!< Register address 0x12C Address maske.g 0xFF if entire byte significant */
    u8 Address_match_byte_1;                 /*!< Register address 0x12D Support for more address bytes and masks */
    u8 Address_mask_byte_1 ;                 /*!< Register address 0x12E Support for more address bytes and masks */
    u8 Address_match_byte_2;                 /*!< Register address 0x12F Support for more address bytes and masks */
    u8 Address_mask_byte_2 ;                 /*!< Register address 0x130 Support for more address bytes and masks */
    u8 Address_match_byte_3;                 /*!< Register address 0x131 Support for more address bytes and masks */
    u8 Address_mask_byte_3 ;                 /*!< Register address 0x132 Support for more address bytes and masks */
    u8 Address_match_byte_4;                 /*!< Register address 0x133 Support for more address bytes and masks */
    u8 Address_mask_byte_4 ;                 /*!< Register address 0x134 Support for more address bytes and masks */
    u8 Address_match_byte_5;                 /*!< Register address 0x135 Support for more address bytes and masks */
    u8 Address_mask_byte_5 ;                 /*!< Register address 0x136 Support for more address bytes and masks */
    u8 Address_match_byte_6;                 /*!< Register address 0x137 Support for more address bytes and masks */
    u8 Address_mask_byte_6 ;                 /*!< Register address 0x138 Support for more address bytes and masks */
    u8 Address_match_byte_7;                 /*!< Register address 0x139 Support for more address bytes and masks */
    u8 Address_mask_byte_7 ;                 /*!< Register address 0x13A Support for more address bytes and masks */
    u8 Address_match_byte_8;                 /*!< Register address 0x13B Support for more address bytes and masks */
    u8 Address_mask_byte_8 ;                 /*!< Register address 0x13C Support for more address bytes and masks */
    u8 r_13D;                                /*!< Register address 0x13D  0x0 to end the address sequence */
    u8 RX_synth_lock_time;                   /*!< Register address 0x13E RX synthesizer lock time*/
    u8 TX_synth_lock_time;                   /*!< Register address 0x13F TX synthesizer lock time*/
} ADI_ADF7023_BBRAM_TYPE;


/*!
 *****************************************************************************
 * \enum ADI_ADF7023_BBRAM_ADDR_TYPE
 *
 * UHF device internal register address enumeration
 *****************************************************************************/
typedef enum
{
    BBRAM_BASE       = 0x100,                      /*!< Address 0x100 Base address of BBRAM */
    INTERRUPT_MASK_0 = BBRAM_BASE,                 /*!< Register address 0x100 Interrupt mask */
    INTERRUPT_MASK_1,                              /*!< Register address 0x101 Interrupt mask */
    NUMBER_OF_WAKEUPS_0,                           /*!< Register address 0x102 Wake up controller*/
    NUMBER_OF_WAKEUPS_1,                           /*!< Register address 0x103 Wake up controller */
    NUMBER_OF_WAKEUPS_IRQ_THRESHOLD_0,             /*!< Register address 0x104 WUC irq */
    NUMBER_OF_WAKEUPS_IRQ_THRESHOLD_1,             /*!< Register address 0x105 WUC irq */
    MAX_WAKEUP_2_SYNC_TIME,                        /*!< Register address 0x106 WUC sync */
    PARMTIME_DIVIDER,                              /*!< Register address 0x107 WUC div */
    LISTEN_RSSI_THRESH,                            /*!< Register address 0x108 RSSI threshold register for wakeup*/
    CHANNEL_FREQ_0,                                /*!< Register address 0x109 To set radio frequency (lower byte)*/
    CHANNEL_FREQ_1,                                /*!< Register address 0x10A To set radio frequency (middle byte)*/
    CHANNEL_FREQ_2,                                /*!< Register address 0x10B To set radio frequency (upper byte)*/
    RADIO_CFG_0,                                   /*!< Register address 0x10C Radio data rate */
    RADIO_CFG_1,                                   /*!< Register address 0x10D Data rate (3:0) +  freq_deviation (7:4) */
    RADIO_CFG_2,                                   /*!< Register address 0x10E Frequency deviation */
    RADIO_CFG_3,                                   /*!< Register address 0x10F Discriminator_bw */
    RADIO_CFG_4,                                   /*!< Register address 0x110 Post demodulation bandwidth */
    RADIO_CFG_5,                                   /*!< Register address 0x111 T4 phase adjust lsb */
    RADIO_CFG_6,                                   /*!< Register address 0x112 None */
    RADIO_CFG_7,                                   /*!< Register address 0x113 None */
    RADIO_CFG_8,                                   /*!< Register address 0x114 Power amplifier settings */
    RADIO_CFG_9,                                   /*!< Register address 0x115 Mod/demod settings */
    RADIO_CFG_10,                                  /*!< Register address 0x116 Afc settings 0 */
    RADIO_CFG_11,                                  /*!< Register address 0x117 Afc settings 1 */
    IR_CAL_PHASE,                                  /*!< Register address 0x118 IR calibration phase setting */
    IR_CAL_AMPLITUDE,                              /*!< Register address 0x119 IR calibration gain setting */
    MODE_CONTROL,                                  /*!< Register address 0x11A Radio settings */
    PREAMBLE_MATCH,                                /*!< Register address 0x11B Preamble settings */
    SYMBOL_MODE,                                   /*!< Register address 0x11C Symbol config settings */
    PREAMBLE_LEN_REG,                              /*!< Register address 0x11D Preamble length register*/
    CRC_POLY_LOW,                                  /*!< Register address 0x11E User defined CRC polynomial (low byte)*/
    CRC_POLY_HIGH,                                 /*!< Register address 0x11F User defined CRC polynomial (high byte)*/
    SYNC_CONTROL,                                  /*!< Register address 0x120 Syncword settings */
    SYNC_BYTE_0,                                   /*!< Register address 0x121 Sync word (low byte)*/
    SYNC_BYTE_1,                                   /*!< Register address 0x122 Sync word (mid byte)*/
    SYNC_BYTE_2,                                   /*!< Register address 0x123 Sync word (high byte)*/
    TX_BASE_ADR,                                   /*!< Register address 0x124 Start address of a TX packet in packet RAM*/
    RX_BASE_ADR,                                   /*!< Register address 0x125 Start address of a RX packet in packet RAM*/
    PACKET_LENGTH_CONTROL,                         /*!< Register address 0x126 Variable or fixed packet length*/
    PACKET_LENGTH_MAX,                             /*!< Register address 0x127 Maximum packet length*/
    STATIC_REG_FIX,                                /*!< Register address 0x128 None [7:0] static_reg_fix_addr RW 0 Pointer to the first address on BBRAM for the automatic register fix */
    ADDRESS_MATCH_OFFSET,                          /*!< Register address 0x129 None [7:0] address_match_offset RW 0 Location of first byte of address information in Packet RAM */
    ADDRESS_LENGTH,                                /*!< Register address 0x12A None [7:0] address_length RW 0 Number of bytes in each address field (NADR) */
    ADDRESS_MATCH_BYTE_0,                          /*!< Register address 0x12B First byte of radio address*/
    ADDRESS_MASK_BYTE_0 ,                          /*!< Register address 0x12C Address maske.g 0xFF if entire byte significant */
    ADDRESS_MATCH_BYTE_1,                          /*!< Register address 0x12D Support for more address bytes and masks */
    ADDRESS_MASK_BYTE_1 ,                          /*!< Register address 0x12E Support for more address bytes and masks */
    ADDRESS_MATCH_BYTE_2,                          /*!< Register address 0x12F Support for more address bytes and masks */
    ADDRESS_MASK_BYTE_2 ,                          /*!< Register address 0x130 Support for more address bytes and masks */
    ADDRESS_MATCH_BYTE_3,                          /*!< Register address 0x131 Support for more address bytes and masks */
    ADDRESS_MASK_BYTE_3 ,                          /*!< Register address 0x132 Support for more address bytes and masks */
    ADDRESS_MATCH_BYTE_4,                          /*!< Register address 0x133 Support for more address bytes and masks */
    ADDRESS_MASK_BYTE_4 ,                          /*!< Register address 0x134 Support for more address bytes and masks */
    ADDRESS_MATCH_BYTE_5,                          /*!< Register address 0x135 Support for more address bytes and masks */
    ADDRESS_MASK_BYTE_5 ,                          /*!< Register address 0x136 Support for more address bytes and masks */
    ADDRESS_MATCH_BYTE_6,                          /*!< Register address 0x137 Support for more address bytes and masks */
    ADDRESS_MASK_BYTE_6 ,                          /*!< Register address 0x138 Support for more address bytes and masks */
    ADDRESS_MATCH_BYTE_7,                          /*!< Register address 0x139 Support for more address bytes and masks */
    ADDRESS_MASK_BYTE_7 ,                          /*!< Register address 0x13A Support for more address bytes and masks */
    ADDRESS_MATCH_BYTE_8,                          /*!< Register address 0x13B Support for more address bytes and masks */
    ADDRESS_MASK_BYTE_8 ,                          /*!< Register address 0x13C Support for more address bytes and masks */
    ADDRESS_SEQUENCE_END,                          /*!< Register address 0x13D  0x0 to end the address sequence */
    RX_SYNTH_LOCK_TIME,                            /*!< Register address 0x13E RX synthesizer lock time*/
    TX_SYNTH_LOCK_TIME,                            /*!< Register address 0x13F TX synthesizer lock time*/
} ADI_ADF7023_BBRAM_ADDR_TYPE;


#ifdef PATCH_802_15_4D

#define BBRam154d_MMemmap_Start                    0x129

// The following BBRAM locations are used\defined in 15.4d mode
// These are not stored in our structure...is this ok

#define BB_cca_cfg_0                               0x103
#define BB_cca_cfg_1                               0x104
#define BB_cca_threshold                           0x105

#define PR_phy_rx_status                           0x01F  // for 4.01 version

#define BB_antenna_RX_diversity_cfg				   0x129

#define BB_TX_anetnna_cfg_r 					   0x12A
#define BB_antenna0_rssi                           0x12B
#define BB_antenna1_rssi                           0x12C
#define BB_threshold_diff_rssi                     0x12D


#define BB_nb_preamble_bytes_low                   0x12E // Longer pre-amble
#define BB_nb_preamble_bytes_high                  0x12F // Longer pre-amble
#define BB_sfd_low                                 0x130
#define BB_sfd_high                                0x131
#define BB_PHR_low                                 0x132  // tmp
#define BB_PHR_high                                0x133  // tmp
// The rolling buffer fills from the RX_base_addr with data.
// When the buffer fills a location = RX_base_addr +
// BB_RX_buff_signal with data an interrupt is generated
#define BB_rx_buff_signal                          0x134
// This is the max size of the buffer when the loaction= RX_base_addr +
// BB_RX_buff_size is filled with data the buffer loops back to loaction
// RX_base_addr and fills from here
#define BB_rx_buff_size                            0x135
// The rolling buffer Sends data from the TX_base_addr.
// When the buffer sends data from location = TX_base_addr +
// BB_TX_buff_signal an interrupt is generated
#define BB_tx_buff_signal                          0x136
// This is the max size of the buffer, when data is sent from the
// loaction= TX_base_addr + BB_TX_buff_size the buffer loops back
// to loaction TX_base_addr and send data from here
#define BB_tx_buff_size                            0x137
#define BB_reserved1                               0x138
#define BB_testmodes                               0x139
#define BB_reserved2                               0x13A

// Once the command COMMAND_SYNTH_CAL_15d4 has been issued the results are stored in these registers.
#define BB_vco_band_readback                       0x13B
#define BB_vco_ampl_readback                       0x13C


typedef struct
{
    unsigned char antenna_RX_diversity_cfg_r;             // 0x129
    unsigned char TX_anetnna_cfg_r;
    unsigned char antenna0_rssi_r;                     // 0x12B
    unsigned char antenna1_rssi_r;                     // 0x12C
    unsigned char threshold_diff_rssi_r;               // 0x12D
    unsigned char nb_preamble_bytes_low_r;             // 0x12E
    unsigned char nb_preamble_bytes_high_r;            // 0x12F
    unsigned char sfd_low_r;                           // 0x130
    unsigned char sfd_high_r;                          // 0x131
    unsigned char PHR_low_r;                           // 0x132
    unsigned char PHR_high_r;                          // 0x133
    unsigned char rx_buff_signal_r;                    // 0x134
    unsigned char rx_buff_size_r;                      // 0x135
    unsigned char tx_buff_signal_r;                    // 0x136
    unsigned char tx_buff_size_r;                      // 0x137
    unsigned char reserved1_r;                         // 0x138
    unsigned char testmodes_r;                         // 0x139
    unsigned char reserved2_r;                         // 0x13A
//    unsigned char vco_band_readback_r;                 // 0x13B
//    unsigned char vco_ampl_readback_r;                 // 0x13C
} TyBBRAM15_4d;

//We should not overwrite vco_band_readback_r and vco_ampl_readback_r registers,

typedef enum
{
	ADI_ADF7023J_15D4_ANTENNA0 = 0,
	ADI_ADF7023J_15D4_ANTENNA1,
} ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED;

typedef enum
{
    ADI_ADF7023J_15D4_VDD_DRV = 0,                 // VDD driver selected
    ADI_ADF7023J_15D4_1_8V_DRV,                    // 1.8V driver selected
} ADI_ADF7023J_15D4_TRX_ATB_LEVEL;

typedef enum
{
    ADI_ADF7023J_15D4_AD_DISABLE = 0,
    ADI_ADF7023J_15D4_AD_ENABLE,
} ADI_ADF7023J_15D4_RX_ANTENNA_DIVERSITY;

typedef enum
{
  ADI_ADF7023J_4BYTE_FCS = 0,
  ADI_ADF7023J_2BYTE_FCS,
} ADI_ADF7023_15D4_FCS_TYPE;
#endif

typedef enum
{
    CCA_FILTER_BW_100_KHZ = 0,
    CCA_FILTER_BW_150_KHZ,
    CCA_FILTER_BW_200_KHZ,
    CCA_FILTER_BW_300_KHZ,
} ADI_ADF7023J_CCA_FILTER_BW;

/*!
 *****************************************************************************
 * \enum ADI_ADF7023_BBRAM_ADDR_TYPE
 *
 * UHF Modem Configuration Memory registers
 *****************************************************************************/
typedef enum
{
	POWER_AMPLIFIER         =       0x307,
    MCR_CONFIG_HIGH         =     	0x30C,
    MCR_CONFIG_LOW          =       0x30D,
	RSSI_READBACK           =       0x312,
	SILICON_REV_0           =       0x329,
	SILICON_REV_1           =       0x32A,
	SILICON_REV_2           =       0x32B,
	SILICON_REV_3           =       0x32C,
	AGC_CLK_DIVIDE			=		0x32F,
	INTERRUPT_SOURCE_0      =       0x336,
	INTERRUPT_SOURCE_1      =       0x337,
	CALIBRATION_CONTROL     =       0x338,
	CALIBRATION_STATUS      =       0x339,
#ifdef PATCH_802_15_4D
	Rx_Filter_Bias          =       0x343,
#endif
	MAX_AFC_RANGE           =       0x315,
	GPIO_CONFIGURE			=		0x3FA,
	AGC_MODE				=		0x35D,
	AGC_GAIN_STATUS			=		0x360,
	ADC_READBACK			=		0x327,
	ADC_READBACK_HIGH		=		0x327,
	ADC_READBACK_LOW		=		0x328,
	POWERDOWN_RX			=		0x324,
	POWERDOWN_AUX			=		0x325,
	ADC_CONFIG_LOW			=		0x359,
	VCO_OVRW_EN				=		0x3CD,

} ADI_ADF7023_MCR_TYPE;

enum
{
    ADI_ADF7023_EVENT_CMD_READY,
    ADI_ADF7023_EVENT_CMD_FINISHED,
    ADI_ADF7023_EVENT_PACKET_TRANSMITTED,
    ADI_ADF7023_EVENT_PACKET_RECEIVED,
    ADI_ADF7023_EVENT_SYNC_WORD_DETECT,
    ADI_ADF7023_EVENT_SPORT_RX_DONE,
    ADI_ADF7023_EVENT_SPI_TX_DONE,
    ADI_ADF7023_EVENT_SPI_RX_DONE,
#ifdef PATCH_802_15_4D
    ADI_ADF7023_15D4_EVENT_PHR_DETECT,
    ADI_ADF7023_15D4_EVENT_CRC_CORRECT,
    ADI_ADF7023_15D4_EVENT_CCATIMER_EXPIRED,
#endif
	ADI_ADF7023_EVENT_ERR_PACKET_RECEIVED,
    ADI_ADF7023_UNSUPPORTED_EVENT,

};

typedef enum
{
    ADI_ADF7023_SPI_IDLE,
    ADI_ADF7023_SPI_TX,
    ADI_ADF7023_SPI_RX,
    ADI_ADF7023_SPI_CMD,
} ADI_ADF7023_SPI_STATE;



/*! ************************************
 * \struct ADI_ADF7023_DEV_DATA_TYPE       *
 * UHF Device instance data structure  *
 * *************************************/
typedef struct ADF7023
{
    ADI_ADF7023_STATE_TYPE      	InitState;      /*!< To track 7023 initialization state */
    ADI_ADF7023_PHY_STATE_TYPE  	PhyState;       /*!< Tracks the current state of the PHY */
    ADI_ADF7023_MOD_TYPE        	ModScheme;      /*!< Mode_mode = 0 (FSK), mode = 1 (GFSK) */
    ADI_ADF7023_DEMOD_TYPE      	DemodScheme;    /*!< Mode_mode = 0 (FSK), mode = 1 (GFSK) */
    u16                         DataRate;       /*!< Radio data rate */
    u32                         Frequency;      /*!< Radio channel frequency */
    volatile bool               	bIsCmdRdy;      /*!< Flag that indicates if the radio is ready to receive a command  */
    bool                        	bBlockingMode;  /*!< Driver in blocking, or non-blocking mode */
    volatile bool               	bTxDone;        /*!< flag to indicate if TX is complete  */
    volatile bool               	bIsRadioActive; /*!< Radio either transmitting or receiving a packet */
    volatile bool               	bRxDone;        /*!< flag to indicate if a packet has been successfully received */
    bool                        	bForceTransfer; /*!< Force driver to accept new buffer irrespective of current state */
    bool                        	bIsSportMode;   /*!< To enable and disable sport mode transfers */
    ADI_ADF7023_BUFFER_TYPE     	*pPacket;       /*!< Pointer to buffer to be transmitted or received */
    ADI_ADF7023_CMD_TYPE        	IssuedCmd;      /*!< Current command being issued/processed */
    u8                          TxPktBase;      /*!< Base address in packet RAM where TX packets are loaded */
    u8                          RxPktBase;      /*!< Base address in packet RAM where received packets are loaded */
    u8                          MaxPktLen;
    ADI_ADF7023_CALLBACK        	pcCallback;     /*!< Pointer to the application callback. If null, default is used */
    void                        	*pCBParam;      /*!< Callback parameter that is passed to the application during a callback */
    ADI_SPI_DEV_ID_TYPE           SpiDevNum;      /*!< SPI device number */
    ADI_SPI_HANDLE          	pSpi;          /*!< Handle to talk to the underlying SPI driver */
    ADI_SPI_TRANSCEIVE_TYPE     SpiBuffer;
    volatile ADI_ADF7023_SPI_STATE      SpiState;
    u8                          SpiPrologue[3];
    volatile bool 			bIsIRCALactive;	  /* Flag indicates IR calibration process is active or inactive */
    volatile int			IR_cal_cnt;
    bool				RSSI_CCA_MODE_AGC_SET;	/* flag indicates if the AGC has been set for the CCA RSSI mode */
#ifdef PATCH_802_15_4D
    u16 						iNextRadioOffset;
    u16 						iPacketOffset;
    volatile bool 				bCCA_Status;
    bool 						bTxRxBlockingMode;
    u8 							PHR_FCS_Type;
    volatile bool 				bCRC_Correct_Status;
    volatile ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED Antenna_Selected;
    u8							Antenna_Diversity_Enabled;
#endif
 	u8							mcrPALevel;
	u8							synth_cal_pending;
#ifdef FAST_TRANSITION_ENABLED
	u8							auto_tx_to_rx_enabled;
#endif
	u8							FastTransitionsEnabled;
	u8*              			pBuf;

} ADF7023;

/*! Opaque device handle type */
typedef struct ADF7023* ADI_ADF7023_DEV_HANDLE;

extern TyBBRAM         gBBRAM;
extern ADF_CmdCodes    mPHYState;

/*!
 *****************************************************************************
 * UHF Device External API function prototypes
 *****************************************************************************/

/*! Device Initialization and Uninitialization Interface **/
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_Init                    (ADI_ADF7023_DEV_HANDLE* phDevice,
								    ADI_SPI_HANDLE const SpiDev,
								    ADI_SPI_DEV_ID_TYPE const spidevnum,
								    IRQn_Type const IrqFlag,
								    ADI_ADF7023_CALLBACK pcCallback,
								    void *pCBParam);

extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_UnInit                  (ADI_ADF7023_DEV_HANDLE  const hDevice);


/******************************************************************
 * Eliminatable functions that may be optimized out by the linker *
 *****************************************************************/
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPreambleLen          (ADI_ADF7023_DEV_HANDLE const hDevice, u8 const prelen);

extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetRSSI                 (ADI_ADF7023_DEV_HANDLE const hDevice, u8* const RssiRead);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetRSSI_CCA_mode			(ADI_ADF7023_DEV_HANDLE const hDevice, int* const RssiRead);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_RSSI_CCA_mode_start		(ADI_ADF7023_DEV_HANDLE const hDevice);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_RSSI_CCA_mode_stop		(ADI_ADF7023_DEV_HANDLE const hDevice);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SPORTmode_GetRSSI	   (ADI_ADF7023_DEV_HANDLE const hDevice, int* const RssiRead);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetTemperatureReading	(ADI_ADF7023_DEV_HANDLE const hDevice, double *TempRead);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetImask                (ADI_ADF7023_DEV_HANDLE const hDevice, u16 const Imask);

extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_IssueCmd                (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_CMD_TYPE const cmd);

extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_BlockMemWrite           (ADI_ADF7023_DEV_HANDLE const hDevice,
                                                                    u8* const pDataTx,
                                                                    u8* const pStatus,
                                                                    u32 const pSize,
                                                                    u16 const AdfAddr);

extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_BlockMemRead            (ADI_ADF7023_DEV_HANDLE const hDevice,
                                                                    u8* const pDataRx,
                                                                    u32 const pSize,
                                                                    u16 const AdfAddr);

extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetDataRate             (ADI_ADF7023_DEV_HANDLE const hDevice, u16 const DataRate);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetShortAddr            (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_SHORT_ADDR_TYPE* const pShortAddr);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetAddressMatchOffset   (ADI_ADF7023_DEV_HANDLE const hDevice, u8 const AddrOffset);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetSyncWord             (ADI_ADF7023_DEV_HANDLE const hDevice, u8 const SyncLength, u32 SyncWord);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPacketEndian         (ADI_ADF7023_DEV_HANDLE const hDevice, bool const PacketEndian);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetClearAutoCRC         (ADI_ADF7023_DEV_HANDLE const hDevice, bool const CrcEn);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetClearFixedPktLen     (ADI_ADF7023_DEV_HANDLE const hDevice, bool const FixedPktLen);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyOn                (ADI_ADF7023_DEV_HANDLE const hDevice);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_InitBBRAM               (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BBRAM_TYPE* const BBRAM);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetMaxPacketLength      (ADI_ADF7023_DEV_HANDLE const hDevice, u8 const PktLength);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyTx                (ADI_ADF7023_DEV_HANDLE const hDevice);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_ReceiveFrame            (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const RxFrame);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_TransmitFrame           (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const TxFrame);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetTransmitterPowerLevel(ADI_ADF7023_DEV_HANDLE const hDevice, u8 const TxPower);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetTransmitterPowerLevelMCR(ADI_ADF7023_DEV_HANDLE const hDevice, u8 MCRLevel);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPARampRate           (ADI_ADF7023_DEV_HANDLE const hDevice, u8 const PARamp);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetChannelFrequency     (ADI_ADF7023_DEV_HANDLE const hDevice, u32 const ChFreq);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetExtendedAddress      (ADI_ADF7023_DEV_HANDLE const hDevice, u8* const ExAddr, u8* ExAddrMask);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetIFFilterBandwidth    (ADI_ADF7023_DEV_HANDLE const hDevice, u8 const ifbw);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetModulationScheme     (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_MOD_TYPE const mod);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetDemodulationScheme   (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_DEMOD_TYPE const demod);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyRx                (ADI_ADF7023_DEV_HANDLE const hDevice);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyOff               (ADI_ADF7023_DEV_HANDLE const hDevice);

extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetFrequencyDeviation   (ADI_ADF7023_DEV_HANDLE const hDevice, u16 const FreqDev);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetDiscrimBW            (ADI_ADF7023_DEV_HANDLE const hDevice, u8 const DiscrimBW);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPostDemodBW          (ADI_ADF7023_DEV_HANDLE const hDevice, u8 const post_demod_bw);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetPhyState             (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_PHY_STATE_TYPE* const PhyState);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetGenericSettings      (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_GENERIC_SETTINGS_TYPE* const pSettings);
extern u16                     adi_ADF7023_GetDataRate             (ADI_ADF7023_DEV_HANDLE const hDevice);
extern u32                     adi_ADF7023_GetChannelFrequency     (ADI_ADF7023_DEV_HANDLE const hDevice);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetGenericSettings      (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_GENERIC_SETTINGS_TYPE* const pSettings);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetShortAddr            (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_SHORT_ADDR_TYPE* const pShortAddr);
extern ADI_ADF7023_DEMOD_TYPE  adi_ADF7023_GetDemodulationScheme   (ADI_ADF7023_DEV_HANDLE const hDevice);
extern ADI_ADF7023_MOD_TYPE    adi_ADF7023_GetModulationScheme     (ADI_ADF7023_DEV_HANDLE const hDevice);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_ConfigureAFC            (ADI_ADF7023_DEV_HANDLE const hDevice,
                                                                    u8 const AfcMode,
                                                                    u8 const AfcLockMode,
                                                                    u8 const PullIn);

extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetRxBaseAddr           (ADI_ADF7023_DEV_HANDLE const hDevice, u8 const RxBase);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetTxBaseAddr           (ADI_ADF7023_DEV_HANDLE const hDevice, u8 const TxBase);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetBlockingMode         (ADI_ADF7023_DEV_HANDLE const hDevice, bool bBlockingMode);
extern bool                    adi_ADF7023_GetBlockingMode         (ADI_ADF7023_DEV_HANDLE const hDevice);
//extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SportInit               (ADI_ADF7023_DEV_HANDLE const hDevice,
//                                                                    ADI_DEV_MANAGER_HANDLE const hDevManager,
//                                                                    ADI_DMA_MANAGER_HANDLE const hDmaManager,
//                                                                    uint32_t const nSportDev);
//extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_EnableSportMode 		   (ADI_ADF7023_DEV_HANDLE const hDevice,bool RxContinuousMode);
extern ADI_ADF7023_SPI_STATE   adi_ADF7023_GetSpiState             (ADI_ADF7023_DEV_HANDLE const hDevice);

extern int Set_TX_SPORT_Continuous_TX(int enable);
extern int Disable_TX_SPORT(void);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetPhyState_PhyRx(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_PHY_STATE_TYPE* const PhyState);
extern void SetTestFlag(u32 flag, int value);

#ifdef PATCH_802_15_4D

#ifdef PATCH_802_DBGCOUNTS
	extern u16 RxBuffalmostFullCnt;
	extern u16 RxBuffFullCnt;
	extern u16 TxBuffalmostFullCnt;
	extern u16 TxBuffFullCnt;
#endif

extern void cbTxBufferAlmostFull(ADI_ADF7023_DEV_HANDLE hd);
extern void cbTxBufferFull(ADI_ADF7023_DEV_HANDLE hd);
extern ADI_ADF7023_RESULT_TYPE cbRxBufferAlmostFull(ADI_ADF7023_DEV_HANDLE hd);
extern void cbRxBufferFull(ADI_ADF7023_DEV_HANDLE hd);
extern ADI_ADF7023_RESULT_TYPE cb154dPacketReceived(ADI_ADF7023_DEV_HANDLE hd);

extern ADI_ADF7023_RESULT_TYPE	adi_ADF7023_154D_PatchRoutine(ADI_ADF7023_DEV_HANDLE hdevice);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyOn15d4	(ADI_ADF7023_DEV_HANDLE const hDevice);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetTxRxBlockingMode (ADI_ADF7023_DEV_HANDLE const hDevice, bool bTxRxBlockingMode);
extern bool	adi_ADF7023_GetTxRxBlockingMode      (ADI_ADF7023_DEV_HANDLE const hDevice);

extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_config_15d4_BBRAM_Regs(ADI_ADF7023_DEV_HANDLE uhf_handle);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_Transmit_15d4_Frame(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const TxFrame);
extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_Receive_15d4_Frame(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const RxFrame);
extern ADI_ADF7023_RESULT_TYPE ADF7023_ReadAntennaRSSI(ADI_ADF7023_DEV_HANDLE const hDevice, int *RSSI_r);
extern ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_Send_Pkt(ADI_ADF7023_DEV_HANDLE hDevice,u16 Phr_Header, u8 *PSDUdata,u8 CCA_Therhold, u8 TimerValue,bool CCA_EnableFlag);
extern ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_RxAntennaDivConfig(ADI_ADF7023_DEV_HANDLE const hDevice, u8 Antenna_Path_0, u8 Antenna_Path_1, ADI_ADF7023J_15D4_TRX_ATB_LEVEL ATB_Level, ADI_ADF7023J_15D4_RX_ANTENNA_DIVERSITY ADA_Enable, ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED Antenna_Select);
extern ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_TxAntennaConfig(ADI_ADF7023_DEV_HANDLE const hDevice, u8 Antenna_Path_0, u8 Antenna_Path_1, ADI_ADF7023J_15D4_TRX_ATB_LEVEL ATB_Level, ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED Antenna_Select);
extern ADI_ADF7023_RESULT_TYPE adi_adf7023_154d_Set_Preamble_Length(ADI_ADF7023_DEV_HANDLE const hDevice, u16 Preamble_length);
extern void Compute_4byte_CRC(u8 *databuff, u16 packet_length);
bool Verify_4byte_CRC(u8 *RxBuffer, u16 data_length, uint8_t start);

#endif // end of PATCH_802_15_4D

/******************************************************************
 * SPORT driver items                                            *
 *****************************************************************/

#if 0
#define MODE_802_15_4_B
#else
#define MODE_802_15_4_G
#endif



#define CONSTANT_DATA_PER_PACKET		0
#define INCREMENTING_DATA				1

#if 0
#define TEST_PATTERN	CONSTANT_DATA_PER_PACKET
#else
#define TEST_PATTERN	INCREMENTING_DATA
#endif

#define TEST_PKT_LEN_CONSTANT			0
#define TEST_PKT_LEN_INCREMENTING		1

#if 0
#define TEST_PKT_LEN	TEST_PKT_LEN_CONSTANT
#else
#define TEST_PKT_LEN	TEST_PKT_LEN_INCREMENTING
#endif

#define TX_PKT_PSDU_PAYLOAD_LEN 5		/* lenght of test packet psdu payload data for constant lenght*/

//range of test packet lenght for incrementing lenght pattern
#define TESTPKT_INCREMENTING_LEN_MIN	4
#define TESTPKT_INCREMENTING_LEN_MAX	2047

#define TX_TEST_PKT_COUNT	  100000
#define TX_TEST_PKT_SEND_DELAY	  1000


//#define SPORT_AUTOBUF
#define SPORT_TX_FSYNC_TX_DELAY 0
// SPORT_TX_FSYNC_TX_DELAY: internal tx frame sync -> number of SCLK cycles
// before we start sending data

#if defined(MODE_802_15_4_B)
#define HEADER_LEN_INCL_PHR 	2
#define SYNC_WORD_LEN	1 		/*sync word len in bytes */
#endif

#if defined(MODE_802_15_4_G)
#define HEADER_LEN_INCL_PHR 	3
#define SYNC_WORD_LEN	1 		/*sync word len in bytes */
/*for some reason, we only get 1 byte of the 2 byte 802.15.4g
sync word from the sport, so the total 1st header lenght becomes
3 instead of 4. And we label the sync word lenght as 1 since that
is all we need to skip over.*/
#endif

#define SYNC_WORD 0x7209      /*reverse of 0x904E */
#define PREAMBLE_BYTE 0x55
#define PREAMBLE_LEN 8
#define PHR_0_4_BITS	0
#define PHR_FRAME_LEN_MASK 0x07ff


//#define SPORT_RX_DETECT_SYNC

#define SYNC_DMA_LEN 4	/* number of bytes to dma during searching for sync phase*/
#define PKT_DMA_WINDOW_SIZE  8  /*length of dma "window" in the middle of the packet */
//PKT_DMA_WINDOW_SIZE must be >= SYNC_DMA_LEN as the same buffer is used for
//these 2 dma operations, and PKT_DMA_WINDOW_SIZE defines the buffer size.

#define PHR_BITLEN 16

typedef struct
{
	unsigned short sync_offset;	/*bit offset when sync is found, 0-15
	if sync_offset = 0, then the SFD was found in the previous 16 bits only,
	and none of the new bits. If sync_offset = 15, then sync was found in 1 of
	the previous bits and 15 of the new bits. So we should shift out sync_offset
	number of the new bits since those would be part of the SFD, once sync is found.
	*/
	unsigned short SFD_found;	//SFD type found, 1-4 if sync found. 0 for no sync.
	unsigned int State;	//lower 16 bits contain the previous 16 bits of data
	u16 *inp;
} Sync_t;

#define RX_FIFO_MAXLEN 8

typedef struct {
	u8 data_cnt;
	u8 *r_ptr;
	u8 data[RX_FIFO_MAXLEN];
} RX_FIFO_T;

#define SYNC_DEBUG_BUFF
/* we use this to capture the incoming sport RX data
to a buffer without any modifications for inspection. */

//#define SYNC_DEBUG_SIMULATE
/* we use this to simulate getting packet sport RX data
from the radio. */


extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SPORT_TX_packet              (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const buf);

//extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_IEEE802_15_4g_sport_init     (ADI_ADF7023_DEV_HANDLE const hDevice,
//                                                                         ADI_DEV_MANAGER_HANDLE const hDevManager,
//                                                                         ADI_DMA_MANAGER_HANDLE const hDmaManager,
//                                                                         uint32_t                    const nSportDev);
//
//extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SPORT_RX_802_15_4g_packet    (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const buf);
//
ADI_ADF7023_RESULT_TYPE adi_ADF7023_Start_SPORT_RX(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t* data, int len);
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SPORT_RX_set_next_databuf(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t* data, int len);
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SPORT_RX_stop_RX(ADI_ADF7023_DEV_HANDLE const hDevice);
void adi_ADF7023_SPORT_RX_802_15_4g_packet_cb(ADI_ADF7023_DEV_HANDLE uhf_handle);
void adi_ADF7023_SPORT_RX_Continuous_Data_cb(ADI_ADF7023_DEV_HANDLE uhf_handle);
extern void adi_ADF7023_SPORT_RX_Continuous_Data_App_cb(ADI_ADF7023_BUFFER_TYPE *current_buf);

extern void Reset_sport_generic_RX( void );
extern u8 Radio_read_reg(ADI_ADF7023_DEV_HANDLE const hdevice, int reg_addr);


extern ADI_ADF7023_RESULT_TYPE adi_ADF7023_SPORT_RX_Continuous_Data     (ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const buf1, ADI_ADF7023_BUFFER_TYPE* const buf2);
extern void SetReceiverToIdleState(void);

void FSK4g_Sync_Detection(Sync_t*);

extern ADI_ADF7023_BBRAM_TYPE BBRAM_test;
int LoadRX_fifo(RX_FIFO_T *fifo, u8* data, u8 len);
u16 *GetRx_fifo_short(RX_FIFO_T *fifo);
u8 *GetRx_fifo_byte(RX_FIFO_T *fifo);
void get_pktdata_from_fifo(RX_FIFO_T *fifo, int len);

void init_sync_debug(void);
void sync_debug_store_data(u8* data, int len);
void debug_halt(void);

void init_sync_simu_debug();
void sync_debug_get_simu_data(u8* data, int len);

#ifndef DISABLE_PATCH_802_15_4D

#if (I_SUPPORT_4BYTE_FCS == 1)
extern void Compute_4byte_CRC(uint8_t *databuff, uint16_t packet_length);
extern void swapbits(uint8_t *data);
extern unsigned int CRC32(unsigned char *c, int n);
#endif // (I_SUPPORT_4BYTE_FCS == 1)

#endif // DISABLE_PATCH_802_15_4D

unsigned char*  ADF_GetSiVer(void);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_PROM_BlockMemRead (ADI_ADF7023_DEV_HANDLE const hDevice,
       u8* const pDataRx, u32 const size);


ADI_ADF7023_RESULT_TYPE adi_ADF7023_do_IR_Calibration(ADI_ADF7023_DEV_HANDLE const hdevice);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_Set_Antenna_Path_TX0_RX1(ADI_ADF7023_DEV_HANDLE const hDevice);
ADI_ADF7023_RESULT_TYPE adi_ADF7023_Set_Antenna_Path_TX1_RX0(ADI_ADF7023_DEV_HANDLE const hDevice);

ADI_ADF7023_RESULT_TYPE DynamicIFFilterSet_CCATimerMode(
								ADI_ADF7023_DEV_HANDLE const hdevice,
								u8 enableIFBWAutoSwitch,
								u8 enableIFBWSwitchOnPreamble,
								u8 cca_filter_bw);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_15d4g_Enable_Disable_Auto_SynthCalibration(ADI_ADF7023_DEV_HANDLE const hDevice, u8 state);
void adi_ADF7023_EnableSynthCalPending(ADI_ADF7023_DEV_HANDLE const hDevice, u8 value);
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetTxToRxAutoTurnAround(ADI_ADF7023_DEV_HANDLE const hDevice, u8 value);
ADI_ADF7023_RESULT_TYPE adi_ADF7023_Set_15d4_Receive_Buffer(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const RxFrame);
ADI_ADF7023_RESULT_TYPE adi_ADF7023_Synthesizer_Calibration(ADI_ADF7023_DEV_HANDLE const hdevice);

#define TXPKTLEN	256
#define HEADERLENGTH   (PREAMBLE_LEN + 2 + 2)
#define NO_PREAMBLE_SYNCWORD
#define VAR_TX_MODE_REG     0x0D

#define AGC_MODE_FREE_RUN			0
#define AGC_MODE_LOCK_AFTER_PREAMBLE			(0x3 << 5)

//filter gain high, mixer gain low, LNA gain high.
//This setting used for -80dBm CCA threshold
#define AGC_MODE_CCA_NEG80DBM_THRESHOLD		0x32


#endif /* __UHF_H__ */

/*
** EOF
*/

/*@}*/

/*************************************************************************************************************************/