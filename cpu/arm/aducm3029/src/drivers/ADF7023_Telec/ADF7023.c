/*!
 *****************************************************************************
 * @file:    uhf.c
 * @brief:   UHF Device Implimentations for ADuCRFxxx
 * @version: $Revision: 13580 $
 * @date:    $Date: 2012-04-06 16:16:27 -0400 (Fri, 06 Apr 2012) $
 *----------------------------------------------------------------------------
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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "telec_common.h"
#include <limits.h>
#include "ADF7023.h"
#include "adf7023_private.h"
//#include "statistics.h"
#include "stack_support.h"

#include <gpio/adi_gpio.h>
#include "dev/aducm3029-radio.h"
#include <uart/adi_uart.h>
#include "utest_support.h"
#include <spi/adi_spi.h>
#include <crc/adi_crc.h>

#ifdef  DO_IR_CALIBRATION
#pragma location = "FLASH"
	u8 const IR_calbration_module_data[] =
	{
		#include  "rom_ram_7023_2_2_IRcal_filter_visu.dat"
	};
#endif

#ifdef PATCH_802_15_4D
#pragma location = "FLASH"
    u8 const patch_802_15_4_d[]  =
    {
    	#include  "rom_ram_7023_2_2_AD_15d4g_R4p3.dat"
    };
#endif

#pragma location = "FLASH"
static const uint32_t crctable[] =
{
	#include  "crc_lut.dat"
};

#define DEBUG_TRX_STATE_ERR
#define WISUN_FREQ_ADJ

#ifdef  DO_IR_CALIBRATION

u32 IRCalBusyErrCount = 0;

#endif //DO_IR_CALIBRATION

volatile u32 Antenna0_Counter = 0, Antenna1_Counter = 0;

#ifdef ADF7023_DEBUG

ADI_ADF7023_BBRAM_TYPE BBRAM_test;

u32 tranceiverState[255];

int BeforeCRCComputeBMTime, AfterCRCComputeBMTime;

#endif//#ifdef ADF7023_DEBUG

#pragma location="never_retained_ram"
uint8_t rx_buf[ ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD ];

int RadioDataIRQErr=0;

unsigned long remainder_1;

extern uint32_t value1, value2;
extern uint32_t value3;

#ifdef DEBUG_TRX_STATE_ERR
u8 testStatusIndexTx = 0, testStatusTx[10];
u8 testStatusIndexRx = 0, testStatusRx[10];
#endif

static ADF7023 UHF_DevData =
{
    ADI_ADF7023_STATE_UNKNOWN,          /* Not yet initialized */
    PHY_STATE_UNKNOWN,              /* not yet initialized */
    MOD_UNKNOWN,                    /* not yet initialized */
    DEMOD_UNKNOWN,                  /* not yet initialized */
    500,                           /* default data rate is 100 kbps */
    900000000,                      /* default frequency is 900 Mhz */
    true,                           /* by default assume radio is ready to receive cmd */
    true,                           /* blocking mode unless changed */
    false,                          /* no packets transmitted */
    false,                          /* radio is not active yet */
    false,                          /* no packets received */
    false,                          /* do not force transfer by default */
    false,                          /* not in sport mode by default */
    NULL,                           /* packet buffer unitialized */
    ADI_ADF7023_CMD_NULL,               /* unknown command */
    PKT_RAM_TXBASE,                 /* default packet ram base address for tx */
    PKT_RAM_RXBASE,                 /* default packet ram base address for rx */
    MAX_PKT_LEN,                    /* max packet length is 128 bytes */
    NULL,                           /* no application callback */
    NULL,                           /* no callback parameter */
//    ADI_FLAG_UNDEFINED,             /* Assigning PH1 to radio on SPI 1 */
//    ADI_FLAG_UNDEFINED,             /* default daughter card slave select */
    ADI_SPI_DEVID_0,                /* spi device port H */
    NULL,                           /* spi handle not yet initialized */
    {                               /* START SPI buffer */
        NULL,                       /* pPrologue */
        NULL,                       /* pointer to SPI tx data */
        NULL,                       /* Pointer to SPI RX data */
        0,                          /* prologue size */
        0,                          /* data size */
        false,                      /* tx pointer increment flag */
        false,                      /* rx pointer increment flag */

    },
    ADI_ADF7023_SPI_IDLE,           /* SPI defaults to idle */
    {                               /* clear out prologue buffer */
        0,
        0,
        0,
    },
//    NULL,                           /* SPORT TX handle */
//    NULL,                           /* SPORT RX handle */
    false,
	0,								/* IR cal cnt */
	false, 							/* RSSI_CCA_MODE_AGC_SET */
#ifdef PATCH_802_15_4D
   0,
   0,
   false,
   false,
   false,
   false,
   ADI_ADF7023J_15D4_ANTENNA0,
   0,
#endif
   0,
   0,
#ifdef FAST_TRANSITION_ENABLED
   0,
#endif
   0
};


/* TODO try to get rid of bit-fields */
typedef struct {
	ADI_ADF7023_PHY_STATE_TYPE fw_state 			   :5;
	u8 cmd_ready               :1;
	u8 irq_status              :1;
	u8 spi_ready               :1;

} ADI_ADF7023_STATUS_TYPE;


typedef unsigned char byte;

#ifdef BENCH_MARKING
	extern uint64_t ackTimeStamp1, TxToRxTimeStamp1, TxToRxTimeStamp2, ccaTimeStamp2;
	extern uint32_t time_diff_TX_to_RX[];
	extern uint8_t TxtoRxInd;
	extern uint8_t DeviceReady;
#endif

u16 AdfpreviousIntSource = 0;
u8 buff_allocated = 0;

#pragma data_alignment=4
static uint8_t gpioMemory[ADI_GPIO_MEMORY_SIZE];

#pragma data_alignment=4
uint8_t spidevicemem[ADI_SPI_MEMORY_SIZE];

/* CRC Device Handle */
static ADI_CRC_HANDLE   hCrcDev;

/* Memory to handle CRC Device */
static uint8_t          CrcDevMem[ADI_CRC_MEMORY_SIZE];

IRQn_Type eIrq;

bool continuousRx;

uint32_t rx_start_time, rx_duration;

/*  For 50kbps datarate to transmit 1 bit we need 20us (1/50000).
    So to receive 2048 byte packet we need:
    2048 * 8 * 20 = 327680 us (327.68 ms).

    This means that from PHR detect within 350ms entire packet reception should
    be over.
*/

#define PACKET_RCV_TIME     (352)

#define ADF_MISO_IN                                             // Macros for manual GPIO checking of Radio MISO pin P2.0 (SPI0)
#define ADF_CSN_DEASSERT    //(pADI_SPI2->CS_CTL |= ADI_SPI_CS3)  // Macros for manual GPIO control of P2.3 (Radio SPI CS) (SPI0)
#define ADF_CSN_ASSERT      //(pADI_SPI2->CS_CTL &= ADI_SPI_CS3)  // Macros for Sending\Receiving single bytes via SPI
#define SEND_SPI(x)         pADI_SPI2->TX = x
#define WAIT_SPI_RX         while((pADI_SPI2->STAT & SPISTA_RXFSTA_MSK) == 0x0);
#define READ_SPI            pADI_SPI2->RX

#define DEF_HW_WAIT_PERIOD                  100000
#define SECDLY                              500

#define FREQ_CNVRT_VAL                      0.00252061538
/* To reduce code memory, double multiplication is converted into integer
   multiplication and division. That is:
   10000000000 / 3967285163514 = 0.00252061538 */
#define FREQ_CNVRT_VAL_DIVIDEND             10000000000
#define FREQ_CNVRT_VAL_DIVISOR              3967285163514
// Packet RAM Memory
#define MMemMap_Adr_Mask                    0x7FF
#define PR_var_tx_mode_ADR                  0x00D
#define PR_var_params                       1
#define PR_var_tx_mode_ADR                  0x00D

#define BBRam_MMemmap_FOffset               9
#define MCR_MMemmap_Start                   0x300
#define PKT_MAX_PAYLOAD_LEN                 2048//128

#ifndef DISABLE_PATCH_802_15_4D
#define PKT_RAM_BASE_PTR                    0x20
#define PKT_RAM_BASE_RXPTR                  0x20
#define PKT_RAM_BASE_TXPTR                  0x20
#define BBRam154d_MMemmap_Start             0x129
#else
#define PKT_RAM_BASE_PTR                    0x10
#define PKT_RAM_BASE_RXPTR                  0x10
#define PKT_RAM_BASE_TXPTR                  0x10
#endif // DISABLE_PATCH_802_15_4D

//#define NULL 0

#if (I_SUPPORT_4BYTE_FCS == 1)
/*! API for generating CRC32  */
#define GenerateCRC32(inbuff,len)     CRC32(inbuff,len)
#endif // (I_SUPPORT_4BYTE_FCS == 1)

#ifdef DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

unsigned char GetPHYRX_RSSI(signed char* RSSIVal);

uint32_t rem;
uint32_t TickCounter = 0;
volatile uint32_t pkt_recv_err_timeout = 0;
volatile uint32_t pkt_transmit_err_timeout = 0xFFFFFFFF;
uint32_t pkt_Tx_start_time;
uint32_t pkt_Rx_start_time;
uint32_t pkt_cca_start_time;

volatile bool pkt_recv_err_timeout_set = false;
volatile bool pkt_transmit_err_timeout_set = false;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// The firmware state variable

TyBBRAM           gBBRAM;
unsigned char     mSiVer[4];
ADF_CmdCodes    mPHYState = (ADF_CmdCodes)0;
IRQn_Type eIrq;

#ifndef DISABLE_PATCH_802_15_4D
ADF_FwState mPHYState_15D4;
#endif // DISABLE_PATCH_802_15_4D

unsigned char* ADF_GetSiVer(void) { return mSiVer; }

volatile uint8_t bTxDone;

#ifndef DISABLE_PATCH_802_15_4D
ADI_ADF7023_BUFFER_TYPE TRxFrame = {0, 0, NULL};
#endif // DISABLE_PATCH_802_15_4D

ADI_SPI_HANDLE hSpi;
/* CRC Device Handle */
static ADI_CRC_HANDLE   hCrc;
#if (I_SUPPORT_4BYTE_FCS == 0x01)
//uint8_t Verify_4byte_CRC(uint8_t *RxBuffer, uint16_t data_length);
uint8_t bCRC_Correct_Status;
#endif // I_SUPPORT_4BYTE_FCS

/*********************************************************************************
 *                              LOCAL FUNCTIONS                                 *
 *********************************************************************************/

/* Used to clear 7023 interrupts in the initialization routine */
static ADI_ADF7023_RESULT_TYPE AdfClearInterrupts    (ADI_ADF7023_DEV_HANDLE const hDevice);
/* Polls for MISO high after pulling /CE low on the 7023 */
static ADI_ADF7023_RESULT_TYPE PhyWakeUp(ADI_ADF7023_DEV_HANDLE hDevice);
/* Reads and returns the current status register of the 7023 */
static ADI_ADF7023_RESULT_TYPE ReadStatusReg(ADI_ADF7023_DEV_HANDLE const , ADI_ADF7023_STATUS_TYPE* );
/* Used by the initialization routine to initialize 7023 BBRAM with default values */
static void SetBBRAMDefault(ADI_ADF7023_BBRAM_TYPE *pBBRAM) ;
/* Issues a CMD_HW_RESET to put the 7023 in a clean state upon initialization */
static ADI_ADF7023_RESULT_TYPE UHF_FirstConnect(ADI_ADF7023_DEV_HANDLE const hDevice);
static ADI_ADF7023_RESULT_TYPE SetupIrqFlag(IRQn_Type Flag);
static ADI_ADF7023_RESULT_TYPE ResetSpiBuffer(ADI_ADF7023_DEV_HANDLE const );
#ifdef PATCH_802_15_4D
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetAgcClockDivideMCR(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t clkDivideVal);
#endif
void spicallback(void *, uint32_t , void *);
void Adf7023_Callback(void *ClientCallback, uint32_t Event, void *pArg);
ADI_ADF7023_RESULT_TYPE adi_ADF7023_WaitforPhyStateReady(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_PHY_STATE_TYPE PhyState);
#ifdef CALIB_WORKS
static ADI_ADF7023_RESULT_TYPE ADFCalibrationRoutine           (ADI_ADF7023_DEV_HANDLE const hDevice);
#endif

#ifdef PATCH_802_15_4D
ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_SetCCARSSIThreshold(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t cca_threshold);
ADI_ADF7023_RESULT_TYPE  ADF7023_15d4g_EnableAutoTx(ADI_ADF7023_DEV_HANDLE const hDevice,bool AutoTxflag);
ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_SetCCATimerDuration(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t TimerDuration);
ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_StartCCATimer(ADI_ADF7023_DEV_HANDLE const hDevice);

#endif

#ifdef BENCH_MARKING

void benchMark_set_time(int i);
uint64_t getBenchMarkTime(void);

#endif

ADI_ADF7023_RESULT_TYPE adi_ADF7023_IssueCmd(ADI_ADF7023_DEV_HANDLE const , ADI_ADF7023_CMD_TYPE const );

ADI_ADF7023_RESULT_TYPE adi_ADF7023_Init(ADI_ADF7023_DEV_HANDLE* const ,
									ADI_SPI_HANDLE ,
									ADI_SPI_DEV_ID_TYPE const spidevnum,
									IRQn_Type const IrqFlag,
									ADI_ADF7023_CALLBACK ,
									void *);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_InitBBRAM(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BBRAM_TYPE* BBRAM);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_BlockMemWrite(ADI_ADF7023_DEV_HANDLE const hDevice,
                                                                    uint8_t* const pDataTx,
                                                                    uint8_t* const pStatus,
                                                                    uint32_t const pSize,
                                                                    uint16_t const AdfAddr);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_BlockMemRead(ADI_ADF7023_DEV_HANDLE const hDevice,
                                                                    uint8_t* const pDataRx,
                                                                    uint32_t const pSize,
                                                                    uint16_t const AdfAddr);

bool adi_ADF7023_GetBlockingMode(ADI_ADF7023_DEV_HANDLE const hDevice);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetBlockingMode(ADI_ADF7023_DEV_HANDLE const hDevice, bool bBlockingMode);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetImask(ADI_ADF7023_DEV_HANDLE const hDevice, uint16_t const Imask);

void cbRxBufferFull(ADI_ADF7023_DEV_HANDLE hDevice);

ADI_ADF7023_RESULT_TYPE cbRxBufferAlmostFull(ADI_ADF7023_DEV_HANDLE hDevice);

void cbTxBufferAlmostFull(ADI_ADF7023_DEV_HANDLE hDevice);

void cbTxBufferFull(ADI_ADF7023_DEV_HANDLE hDevice);

ADI_ADF7023_RESULT_TYPE cb154dPacketReceived(ADI_ADF7023_DEV_HANDLE hDevice);

bool Verify_4byte_CRC(uint8_t *RxBuffer, uint16_t data_length, uint8_t start);

unsigned int CRC32(unsigned char *c, int n);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetPhyState(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_PHY_STATE_TYPE* const PhyState);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyOff(ADI_ADF7023_DEV_HANDLE const hDevice);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetPhyState_PhyRx(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_PHY_STATE_TYPE* const PhyState);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_Synthesizer_Calibration(ADI_ADF7023_DEV_HANDLE const hdevice);

uint8_t Radio_read_reg(ADI_ADF7023_DEV_HANDLE const hdevice, int reg_addr);

ADI_ADF7023_RESULT_TYPE Radio_write_reg(ADI_ADF7023_DEV_HANDLE const hdevice, int reg_addr, uint8_t reg_value);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyOn15d4(ADI_ADF7023_DEV_HANDLE const hDevice);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_config_15d4_BBRAM_Regs(ADI_ADF7023_DEV_HANDLE hdevice);

static void Reset_Device_Handle_State(ADI_ADF7023_DEV_HANDLE const hDevice);

void Compute_4byte_CRC(uint8_t *databuff, uint16_t data_length);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetDataRate(ADI_ADF7023_DEV_HANDLE const hDevice, uint16_t const DataRate);

uint16_t adi_ADF7023_GetDataRate(ADI_ADF7023_DEV_HANDLE const hDevice);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetChannelFrequency(ADI_ADF7023_DEV_HANDLE const hDevice, uint32_t const ChFreq);

uint32_t adi_ADF7023_GetChannelFrequency(ADI_ADF7023_DEV_HANDLE const hDevice);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetShortAddr(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_SHORT_ADDR_TYPE* pShortAddr);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetShortAddr(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_SHORT_ADDR_TYPE* const pShortAddr);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetModulationScheme(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_MOD_TYPE const mod);

ADI_ADF7023_MOD_TYPE adi_ADF7023_GetModulationScheme(ADI_ADF7023_DEV_HANDLE const hDevice);

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetDemodulationScheme(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_DEMOD_TYPE const demod);

ADI_ADF7023_DEMOD_TYPE adi_ADF7023_GetDemodulationScheme(ADI_ADF7023_DEV_HANDLE const hDevice);

/***********************************************************************************
 *                             INLINE FUNCTIONS                                    *
 ***********************************************************************************/

/* Converts the 7023 channel_freq register to frequency in Hz */
static inline uint32_t MakeFreq(uint32_t const Channel);

/* Converts a frequency in Hz to channel_freq value */
static inline uint32_t MakeChannel(uint32_t const Freq);

static inline uint8_t CreateMemcmd(ADI_ADF7023_CMD_TYPE const cmd, uint16_t const addr);

/***********************************************************************************
 *                             EXTERN FUNCTIONS                                    *
 ***********************************************************************************/

extern ADI_SPI_RESULT adi_spi_RegisterCallback(ADI_SPI_HANDLE const ,ADI_CALLBACK const ,void *const );

/*---------------------------------------------*/
/* Interrupt servicing is done here */
/*---------------------------------------------*/
extern uint8_t bAckTxProgress;

/*---------------------------------------------------------------------------*/

void Enable_Transceiver_Interrupts( )
{
  NVIC_EnableIRQ(SYS_GPIO_INTA_IRQn);
}

/*---------------------------------------------------------------------------*/

void Disable_Transceiver_Interrupts( )
{
  NVIC_DisableIRQ(SYS_GPIO_INTA_IRQn);
}

#ifndef DISABLE_PATCH_802_15_4D

unsigned char reason_for_failure;

/*---------------------------------------------------------------------------*/
#if (I_SUPPORT_4BYTE_FCS == 1)

/*---------------------------------------------------------------------------*/
/**
* @brief Swaps the bits in the given byte.
*
* @param[in]   data	 Pointer to the data buffer of the unsigned char type
*
* @return swapped data in the buffer pointed by data.
* Swaps the bits for each byte of the given input buffer.
*/

void swapbits(uint8_t *data)
{
//  *data = (*data & 0x0F) << 4 | (*data & 0xF0) >> 4;
//  *data = (*data & 0x33) << 2 | (*data & 0xCC) >> 2;
//  *data = (*data & 0x55) << 1 | (*data & 0xAA) >> 1;

}
/*---------------------------------------------------------------------------*/

#endif // (I_SUPPORT_4BYTE_FCS == 1)

void SysTick_Handler(void)
{
//  TickCounter++;
//
//
//  if((pkt_recv_err_timeout_set) && (pkt_recv_err_timeout == TickCounter))
//  {
//    rf_stats.Rx_timeout_re_init_count++;
//    ADF7023_ReInit(NULL);
//    pkt_recv_err_timeout_set = false;
//  }
//
//  if((pkt_transmit_err_timeout_set) && (pkt_transmit_err_timeout == TickCounter))
//  {
//    bTxDone = 1;
//    rf_stats.Tx_timeout_re_init_count++;
//    ADF7023_ReInit(NULL);
//    pkt_transmit_err_timeout_set = false;
//  }
}

/*---------------------------------------------------------------------------*/
#endif // DISABLE_PATCH_802_15_4D


ADI_SPI_HANDLE get_spi_handle()
{
  return UHF_DevData.pSpi;
}

/**
 * @brief       Opens and initializes the underlying drivers and the ADF7023
 *
 * @param[in]   *phDevice       Pointer to the ADF7023 device handle
 * @param[in]   SpiDevNumber    The SPI device to be used to talk to the ADF7023
 * @param[in]   IrqFlag         The Blackfin GPIO that connects to the IRQ pin of the 7023
 * @param[in]   CS_Gpio         The Blackfin GPIO to be used as a chip select for the 7023
 * @param[in]   pcCallback      The application callback function for event notification
 * @param[in]   pCBParam        Application specified callback parameter that is passed back during a callback
 *
 * Initializes the ADF7023 and the underlying SPI devices. Does a startup
 * sequence that ensures the the ADF7023 PHY is in off state before
 * commencing initiliazation. After ensuring the phy is off, issues
 * CMD_SYNC, and then initializes the BBRAM registers to certain default values. In addition, sets
 * up ADF7023 interrupt mask and enables interrupts on the Blackfin GPIO.
 *
 * @sa      adi_ADF7023_UnInit(), adi_ADF7023_SportInit()
 *
 * @note If the SPORT mode operation is required. A separate call to the adi_ADF7023_SportInit
 * function is required. If using the EZKIT, ensure that the switch settings are properly
 * set for any CS and IRQ combination used.
*
 * @return
 */
extern ADI_SPI_HANDLE hDevice_SPI;
ADI_ADF7023_RESULT_TYPE adi_ADF7023_Init(ADI_ADF7023_DEV_HANDLE* const phDevice,
									ADI_SPI_HANDLE SpiDev,
									ADI_SPI_DEV_ID_TYPE const spidevnum,
									IRQn_Type const IrqFlag,
									ADI_ADF7023_CALLBACK pcCallback,
									void *pCBParam)
{
    ADI_ADF7023_DEV_HANDLE hDevice;  /* local device handle */
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    ADI_ADF7023_PHY_STATE_TYPE phy_state;

    //ADI_SPI_DEV_GENERIC_SETTINGS_TYPE settings;

    hDevice = &UHF_DevData;

    if (hDevice->InitState == ADI_ADF7023_STATE_INITIALIZED)
        return ADI_ADF7023_ERR_ALREADY_INITIALIZED;

    adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE);

    if(adi_gpio_SetHigh(ADI_GPIO_PORT2, ADI_GPIO_PIN_7) != ADI_GPIO_SUCCESS)
    {
        return ADI_ADF7023_ERR_UNKNOWN;
    }

    adi_spi_Open(spidevnum, spidevicemem, ADI_SPI_MEMORY_SIZE, &SpiDev);
    //Test Code
    hDevice_SPI = SpiDev;
    adi_spi_SetMasterMode(SpiDev, true);

    //adi_spi_SetTransmitUnderflow(SpiDev, true);

    /* throttle bitrate to something the controller can reach */
    adi_spi_SetBitrate(SpiDev, 4000000);

//    adi_spi_EnableDmaMode(SpiDev, false);

    adi_spi_RegisterCallback(SpiDev, spicallback, hDevice);

    adi_spi_SetChipSelect (SpiDev, ADI_SPI_CS3);

    /* Set the SPI clock phase */
	adi_spi_SetClockPhase(SpiDev, false);

    adi_spi_SetContinousMode(SpiDev, true);

    /* store a bad handle in case of failure */
    *phDevice = (ADI_ADF7023_DEV_HANDLE) NULL;

    hDevice->pSpi = SpiDev;
    hDevice->SpiDevNum = spidevnum;

    if(pcCallback != NULL)
        hDevice->pcCallback = pcCallback;

    hDevice->pCBParam = pCBParam;

    /* Open a CRC device instance */
    adi_crc_Open (0, &CrcDevMem[0], ADI_CRC_MEMORY_SIZE, &hCrcDev);

    /* Now initializing the radio*/
    if(!uhf_result)
        uhf_result = UHF_FirstConnect(hDevice);

    /* setup interrupts before issuing commands */
#ifdef PATCH_802_15_4D
    uint16_t adf_imask = INT_CMD_FINISHED |INT_BUFF_FULL | INT_TX_EOF | INT_RX_EOF | INT_BUFF_ALMOST_FULL| INT_PHR_DETECT|INT_CRC_CORRECT|INT_CCA;
#else
    uint16_t adf_imask = INT_CMD_FINISHED | INT_TX_EOF | INT_RX_ADDR_MATCH | INT_SYNC_DETECT;
#endif
    /* now we should start getting command ready interrupts */

    if(uhf_result == ADI_ADF7023_SUCCESS)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_SYNC);

    /* currently we are in blocking mode. so no need to check if command is done */

    /* Initialize BBRAM to a set of default values */
    ADI_ADF7023_BBRAM_TYPE BBRAM_t = {0};

    SetBBRAMDefault(&BBRAM_t);
    if(uhf_result == ADI_ADF7023_SUCCESS)
        uhf_result = adi_ADF7023_InitBBRAM(hDevice, &BBRAM_t);

    hDevice->mcrPALevel = DEFAULT_PA_MCR_REG_SETTING;

    /* first clear any pending spurious interrupts */
    if(uhf_result == ADI_ADF7023_SUCCESS)
        uhf_result = AdfClearInterrupts(hDevice);

    if(uhf_result == ADI_ADF7023_SUCCESS)
		uhf_result = SetupIrqFlag(IrqFlag);

//    hDevice->IrqFlag = IrqFlag;

    if(uhf_result == ADI_ADF7023_SUCCESS)
        uhf_result = adi_ADF7023_SetImask(hDevice, adf_imask);

    if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_OFF);

    if(uhf_result == ADI_ADF7023_SUCCESS)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    /* the timeout below is required to ensure the command completes */

    if(uhf_result == ADI_ADF7023_SUCCESS)
    	uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);

    if(uhf_result == ADI_ADF7023_SUCCESS)
    {
    	hDevice->InitState = ADI_ADF7023_STATE_INITIALIZED;
        hDevice->PhyState  = phy_state;
        *phDevice = hDevice;    /* finally assign to user passed handle */

        return ADI_ADF7023_SUCCESS;
    }
    else
    {
    	return uhf_result;
    }
}

/**
 * @brief       Opens and Reinitializes the underlying drivers and the ADF7023
 *
 * @param[in]   hDevice       Pointer to the ADF7023 device handle
 *
 * ReInitializes the ADF7023 drivers. Does a startup
 * sequence that ensures the the ADF7023 PHY is in off state before
 * commencing initiliazation. Issues CMD_SYNC.
 * After ensuring the phy is off,  and then initializes the BBRAM registers to backed up values.
 *
 * @return uhf_result returns the status of reinitialization.
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_ReInit(ADI_ADF7023_DEV_HANDLE const hDevice)
{
    ADI_ADF7023_RESULT_TYPE uhf_result;
    ADI_ADF7023_BBRAM_TYPE BBRAM_re_init;
    ADI_ADF7023_PHY_STATE_TYPE phy_state;


    //backup bbram
    uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&BBRAM_re_init, sizeof(ADI_ADF7023_BBRAM_TYPE), INTERRUPT_MASK_0);

    if(!uhf_result)
	hDevice->InitState = ADI_ADF7023_STATE_UNKNOWN;

    if(!uhf_result)
	uhf_result = UHF_FirstConnect(hDevice);

    /* now we should start getting command ready interrupts */

    if(!uhf_result)
	uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_SYNC);

    adi_ADF7023_WaitforPhyStateReady(hDevice, PHY_OFF);
    /* currently we are in blocking mode. so no need to check if command is done */

    /* Re-Initialize BBRAM  */

    if(!uhf_result)
	uhf_result = adi_ADF7023_InitBBRAM(hDevice, &BBRAM_re_init);

    if(!uhf_result)
	uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    adi_ADF7023_WaitforPhyStateReady(hDevice, PHY_OFF);

    if(!uhf_result)
	uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);

    if(!uhf_result) {

	hDevice->InitState = ADI_ADF7023_STATE_INITIALIZED;
	hDevice->PhyState  = phy_state;
    }

    return uhf_result;
}

/*!
 * @brief Uninitializes the ADF7023 and switches the phy on
 *
 * @param[in]   hDevice     handle to the ADF7023 device
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Uninitialize all device resources.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_UnInit(ADI_ADF7023_DEV_HANDLE const hDevice)
{
    ADI_ADF7023_RESULT_TYPE uhf_result;
    
    /* shut down the radio */
    uhf_result = adi_ADF7023_SetPhyOff(hDevice);
    if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_HW_RESET);

    if(!uhf_result){
      if(adi_spi_Close(hDevice->pSpi) != ADI_SPI_SUCCESS)
	return ADI_ADF7023_ERR_INIT_FAILED;

      /* set device data to uninitialized state */
      hDevice->InitState  = ADI_ADF7023_STATE_UNKNOWN;
    }
    return uhf_result;
}

#ifdef SPORT_MODE
 /**
 * @brief   Initializes the SPORT driver
 *
 * @param[in]   hDevice         Handle to ADF7023 device
 * @param[in]   hDevManager     Handle to device manager
 * @param[in]   hDmaManager     Handle to the DMA manager
 * @param[in]   nSportDev       The device number of the SPORT to be used
 *
 * @return  ADI_ADF7023_RESULT_TYPE
 *
 * This function initializes the underlying SPORT driver, and then configures
 * the SPORT peripheral for communication with the ADF7023.
 *
 * @note    Ensure that sufficient memory has been provided to the
 *          Device Manager, and the DMA manager, and that they have been
 *          properly initialized prior to calling this function. Also,
 *          this function ONLY initializes the SPORT driver. To use the
 *          7023 in SPORT mode, a subsequent call to the adi_ADF7023_EnableSportMode()
 *          function is required.
 *
 * @sa      adi_ADF7023_SportEnable(), adi_ADF7023_SportDisable()
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SportInit(ADI_ADF7023_DEV_HANDLE const hDevice,
                                 				ADI_DEV_MANAGER_HANDLE const hDevManager,
                                 				ADI_DMA_MANAGER_HANDLE const hDmaManager,
                                 				u32 const nSportDev)
{

	if(NULL == hDevManager || NULL == hDmaManager)
    	return ADI_ADF7023_ERR_SPORT_INIT;

    ADI_ADF7023_RESULT_TYPE result;

    result = adi_ADF7023_IEEE802_15_4g_sport_init(hDevice,
                                                    hDevManager,
                                                    hDmaManager,
                                                    nSportDev);

    if(!result)
        return result;
    else
        return ADI_ADF7023_ERR_SPORT_INIT;

}

/**
 * @brief Switches radio operation to SPORT mode
 *
 * @param[in]   hDevice     Handle to UHF device
 * @param[in]   bool 		Receive mode, continous or packet
 *
 * @return  ADI_ADF7023_RESULT_TYPE
 *
 * Sets the DATA_MODE setting of the PACKET LENGTH CONTROL register,
 * and the GPIO_CONFIGURE register to set it to SPORT mode. The
 * default mode of operation in DATA_MODE 2, which sets the 7023 to issue the first
 * frame sync synchronous with the last byte of the received
 * sync word, and the default GPIO_CONFIGURE is 0xA2, setting the
 * 7023 to issue byte-aligned frame syncs for the data.
 *
 * @note The adi_ADF7023_ReceiveFrame, and adi_ADF7023_TransmitFrame will no
 * longer work. All subsequent packets will need to be submitted over the
 * SPORT receive and transmit functions. For more information refer to the
 * SPORT driver documentation.
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_EnableSportMode (ADI_ADF7023_DEV_HANDLE const hDevice, bool ContinuousRxMode)
{

    ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;

    u8 pkt_len_control;     /* read the packet length control reg and set data mode */
    if(!result)
        result = adi_ADF7023_BlockMemRead(hDevice, &pkt_len_control, (u32)1, PACKET_LENGTH_CONTROL);
    pkt_len_control &= ~(packet_length_control_sport_mode_unused);

#ifdef SPORT_RX_DETECT_SYNC
    pkt_len_control |= (packet_length_control_sport_mode_sport_preamble);    /* data mode = 1
                                       SPORT receive FS on preable detect */
#else
    pkt_len_control |= (packet_length_control_sport_mode_sport_sync);    /* data mode = 2
                                       SPORT receive FS on sync detect */
#endif

    if(!result)
        result = adi_ADF7023_BlockMemWrite(hDevice, &pkt_len_control, NULL, (u32)1, PACKET_LENGTH_CONTROL);
    if(!result)
        result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    volatile u32 timeout = 100000;
    if(!result)
    {
    	while(!hDevice->bIsCmdRdy && timeout--);          /* sort of force blocking here. */
    }

    if(!timeout)
        return ADI_ADF7023_ERR_SPORT_INIT;

    /* repurpose pkt_len_control variable for the GPIO_CONFIGURE register now */

    if(ContinuousRxMode == true)
    	pkt_len_control = 0xA0; // No preamble sync detction and no frame sync signal from ADF7023
    else
    	pkt_len_control = 0xA2;         /* put this in mode A2. Receive frame syncs rather than interrupts */

    if(!result)
        result = adi_ADF7023_BlockMemWrite(hDevice, &pkt_len_control, NULL, (u32)1, GPIO_CONFIGURE);

    if(!result)
    {

    	hDevice->bIsSportMode = true;
        return ADI_ADF7023_SUCCESS;

    }
    else
    	return result;

}


/**
 * @brief   Disables sport mode and resets radio back to packet mode
 *
 * @param[in]   hdevice     UHF device handle
 *
 * @return  ADI_ADF7023_RESULT_TYPE
 *
 * The function resets the radio back to the default packet/SPI mode.
 * Any further packets transmitted or received will need to go through the
 * SPI, unless SPORT mode is re-enabled.
 *
 * @sa adi_ADF7023_EnableSportMode
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_DisableSportMode (ADI_ADF7023_DEV_HANDLE const hDevice)
{

	ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;

    u8 pkt_len_control;     /* read the packet length control reg and set data mode */

    if(!result)
    	result = adi_ADF7023_BlockMemRead(hDevice, &pkt_len_control, (u32)1, PACKET_LENGTH_CONTROL);

    pkt_len_control &= ~(packet_length_control_sport_mode_unused);
    if(!result)
        result = adi_ADF7023_BlockMemWrite(hDevice, &pkt_len_control, NULL, (u32)1, PACKET_LENGTH_CONTROL);

    if(!result)
        result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);
    volatile u32 timeout = 100000;
    do
    {
    	while(!hDevice->bIsCmdRdy);
    }while(timeout--);

	if(0 == timeout)
    result = ADI_ADF7023_ERR_SPORT_DISABLE;

    pkt_len_control = 0;
    if(!result)
    	result = adi_ADF7023_BlockMemWrite(hDevice, &pkt_len_control, NULL, (u32)1, GPIO_CONFIGURE);

    if(!result)
    {
    	hDevice->bIsSportMode = false;
        return result;
    }
    else
        return result;
}
#endif//SPORT_MODE
/*!
 * @brief Can be used to quickly set a number of basic radio configurations
 *
 * @param[in]   hDevice     Pointer to device handle
 * @param[in]   pSettings   Pointer to a list of generic settings
 *
 * @return      ADI_ADF7023_RESULT_TYPE
 *
 * Can be used during initialization to quickly configure a number of basic radio
 * parameters. Currently can set the data rate, the channel, the short address
 * of the radio, and the modulation, and demodulations schemes.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetGenericSettings(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_GENERIC_SETTINGS_TYPE* const pSettings)
{

	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uhf_result = adi_ADF7023_SetDataRate(hDevice, pSettings->DataRate);

    if(!uhf_result)
        uhf_result = adi_ADF7023_SetChannelFrequency(hDevice, pSettings->Frequency);

    if(!uhf_result)
        uhf_result = adi_ADF7023_SetShortAddr(hDevice, &pSettings->ShortAddr);

    if(!uhf_result)
        uhf_result = adi_ADF7023_SetModulationScheme(hDevice, pSettings->ModScheme);

    if(!uhf_result)
        uhf_result = adi_ADF7023_SetDemodulationScheme(hDevice, pSettings->DemodScheme);

    if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    /* TODO cleaner way to determine when config dev command is done */
    while(!hDevice->bIsCmdRdy);

    return uhf_result;

}
/**
 * @brief Used to read generic radio settings
 *
 * @param[in]       hDevice         Pointer to device handle
 * @param[out]      pSettings       Pointer to generic settings struct
 *
 * @return ADI_ADF7023_RESULT_TYPE
 *
 * This functions reads and returns a number of generic radio settings -
 *
 *      i)   Current data rate
 *      ii)  Current channel frequency
 *      iii) Radio short address
 *      iv)  Current modulation scheme
 *      v)   Current demodulation scheme
 *
 * @sa adi_ADF7023_SetGenericSettings().
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetGenericSettings(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_GENERIC_SETTINGS_TYPE* const pSettings)
{
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    pSettings->DataRate  = adi_ADF7023_GetDataRate(hDevice);

    pSettings->Frequency = adi_ADF7023_GetChannelFrequency(hDevice);

    uhf_result = adi_ADF7023_GetShortAddr(hDevice, &pSettings->ShortAddr);
    if(!uhf_result)
    {
        pSettings->ModScheme   = adi_ADF7023_GetModulationScheme(hDevice);
        pSettings->DemodScheme = adi_ADF7023_GetDemodulationScheme(hDevice);
    }

    return uhf_result;

}
/*!
 * @brief Turns ADF7023 phy TX mode on and transmits a frame
 *
 * @param[in]   hDevice    handle to the ADF7023 device
 * @param[in]   *TxFrame   pointer to the TX buffer.
 * @param[in]   BufLen     length of the frame to be transmitted.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Loads the packet into the Packet RAM of the ADF7023 and then turns
 * phy TX on. Currently only supports packet lengths <=240 bytes.
 *
 * TODO SPORT mode of this function
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_TransmitFrame(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const TxFrame)
{

	ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;
    uint16_t BufAddr = hDevice->TxPktBase;
    ADI_ADF7023_PHY_STATE_TYPE phy_state;
    ADI_ADF7023_STATUS_TYPE status;
    bool lBlockingMode;


    /* State machine -
     * If radio active, and force transfer = true, overwrite contents of packet RAM and continue
     *      a) if in rx or tx, replace packet pointer with new buffer, and issue tx command
     *      b) enter critical region and ignore if an interrupt comes in till phy_tx is issued.
     *
     * If radio active, and force transfer = false, warn that a packet is alredy waiting to be
     * sent or received
     *
     * */

    if(true == hDevice->bIsSportMode)
    	return ADI_ADF7023_ERR_RADIO_SPORT_MODE;

    if(255 < TxFrame->ElementCount)
    	return ADI_ADF7023_ERR_UNKNOWN;

    adi_int_EnterCriticalRegion();

    if(NULL != hDevice->pPacket && false == hDevice->bForceTransfer)
        return ADI_ADF7023_ERR_RADIO_BUSY;

    else if(NULL != hDevice->pPacket && hDevice->bForceTransfer)
    {
        /* disable interrupts,  and set phy to on state
         * we don't want an RX interrupt coming in and screwing with buffers here */

        if(PHY_ON != hDevice->PhyState)
            result = adi_ADF7023_SetPhyOn(hDevice);

        hDevice->bIsRadioActive = false;
    }

    /* set the packet address */
    hDevice->pPacket = TxFrame;

    adi_int_ExitCriticalRegion();
//    ADI_EXIT_CRITICAL_REGION();

    lBlockingMode = adi_ADF7023_GetBlockingMode(hDevice);

    /* force blocking till we at least right the packet out to
     * the radio so that it is safe to issue PHY_TX */
    adi_ADF7023_SetBlockingMode(hDevice, true);

    /* clear any ADF interrupts that came while we were doing this */
    AdfClearInterrupts(hDevice);

    /* This call should be blocking because we don't want the
     * SetPhyTx call to be issued before all the data is sent
     * to the radio */

    /* write packet out to packet RAM */
    if(!result)
        result = adi_ADF7023_BlockMemWrite(hDevice, TxFrame->pData, (uint8_t *)&status, TxFrame->ElementCount, BufAddr);

    /* restore original blocking mode */
    adi_ADF7023_SetBlockingMode(hDevice, lBlockingMode);

    hDevice->pPacket->bBufferProcessed = false;
    hDevice->bIsRadioActive = true;
    hDevice->bTxDone = false;

    if (!result)
        result = adi_ADF7023_SetPhyTx(hDevice);

    /* wait for interrupt to fire if in blocking mode */
    if(hDevice->bBlockingMode)
    {

		while(!hDevice->bTxDone);

        /* if we get here packet has been transmitted */
        hDevice->bTxDone = false;

        if(!result)
            result = adi_ADF7023_GetPhyState(hDevice, &phy_state);

       /* at this point the radio should be back in ON state */

        if(!result)
            hDevice->PhyState = phy_state;
    }

    /* if not in blocking mode, we can peace out now */
	return result;
}

/*!
 * @brief Turns ADF7023 phy RX mode on and waits for a packet
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[out]  *RxFrame    pointer to the RX buffer.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Waits for a RX address match interrupt and then reads the packet from
 * packet RAM. It reads the first byte of the packet to determine the packet
 * length and then schedules a second read to read the rest of the packet.
 *
 *
 * @sa adi_ADF7023_TransmitFrame().
 *
 * TODO SPORT mode of this function
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_ReceiveFrame(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const RxFrame)
{

    /* if packet length passed down is zero, and we are in SPI mode,
     * the first byte of the packet is read once a valid packet is received
     * and the size of the packet is determined from the header. If
     * RxLen is not set to zero, then RxLen bytes is read from the
     * packet RAM */

	ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;
    ADI_ADF7023_PHY_STATE_TYPE phy_state;

    if(true == hDevice->bIsSportMode)
        return ADI_ADF7023_ERR_RADIO_SPORT_MODE;

    /*TODO instead of turning off interrupts overall, just switch off
    	radio interrupts here. That way we can still issue SPI
    	transfers to read and clear SPI interrupts prior to re-enabling
    	radio interrupts. */

    adi_int_EnterCriticalRegion();

    if(NULL != hDevice->pPacket && false == hDevice->bForceTransfer)
        return ADI_ADF7023_ERR_RADIO_BUSY;

    else if(NULL != hDevice->pPacket && hDevice->bForceTransfer)
    {

        /* disable interrupts,  and set phy to on state
         * we don't want an RX interrupt coming in and screwing with buffers here */

        if(PHY_ON != hDevice->PhyState)
            result = adi_ADF7023_SetPhyOn(hDevice);

        hDevice->bIsRadioActive = false;
    }

    hDevice->pPacket = RxFrame;

    adi_int_ExitCriticalRegion();/* re-enable interrupts */

    AdfClearInterrupts(hDevice);

    RxFrame->bBufferProcessed = false;
    hDevice->bRxDone = false;
    hDevice->bIsRadioActive = true;

    if(!result)
        result = adi_ADF7023_SetPhyRx(hDevice);

    if(hDevice->bBlockingMode)
    {
        while(true != hDevice->bRxDone);

        /* at this point packet has been received and callbacks completed */
        hDevice->bRxDone = false; /* set variable to false and exit */
        hDevice->bIsRadioActive = false;
        hDevice->pPacket = NULL;

        if(!result)
            result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
        /* at this point the radio should be back in ON state */

        hDevice->PhyState = phy_state;
    }

    return result;

}

/*!
 * @brief Sets the ADF7023 phy to ON state
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Queries the current state of the radio and then turns the ADF7023 phy ON.
 * Modifies the PhyState paramenter in the device handle.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyOn(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    ADI_ADF7023_PHY_STATE_TYPE phy_state = PHY_BUSY_TRX;
    volatile ADI_ADF7023_STATUS_TYPE status;
    uint8_t bb_cal = 0;
    volatile uint16_t timeout;

    /* first query the 7023 to determine current phy state */
    uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
    if(uhf_result != ADI_ADF7023_SUCCESS)
       return uhf_result;

    if(phy_state == PHY_RX_CCA_15D4)
        rx_duration += get_time_difference(rx_start_time);

    switch(phy_state)
	{

        case PHY_SLEEP:
            uhf_result = PhyWakeUp(hDevice);

            if(!uhf_result)
            	uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_SYNC);
            //ADFDoubleNop(hDevice);
            if(!uhf_result)
                uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

            //set bit in mode control to perform IF filter calibration
            if(!uhf_result) {
                uhf_result = adi_ADF7023_BlockMemRead(hDevice, &bb_cal, (uint32_t)sizeof(bb_cal), MODE_CONTROL);
                bb_cal |= 0x40;
                if(!uhf_result)
                    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &bb_cal, (uint8_t *)&status, (uint32_t)sizeof(bb_cal), MODE_CONTROL);
            }

            if(!uhf_result)
                uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_ON);
            //now ensure that phy is ON before quitting
            //also timeout if it never gets into ON state
            timeout = 15000;     //TODO typically PHY_OFF -> PHY_ON takes 261us. right now arbit
                                //assuming 4Mhz CCLK, 1044 cycles. need to make this more precise
            if(!uhf_result)
            {
            	do
                {
                	if( 0 < timeout )
                    	uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
                    else
                    	uhf_result = ADI_ADF7023_ERR_PHY_ON;
                    timeout--;

                }while( !uhf_result && phy_state != PHY_ON );
            }
            if (phy_state == PHY_ON)
                hDevice->PhyState = PHY_ON;

            break;

        case PHY_OFF:
        /* FIXME : Is this step still required */
            //set bit in mode control to perform IF filter calibration
          /*  uhf_result = adi_ADF7023_BlockMemRead(hDevice, &bb_cal, (uint32_t)sizeof(bb_cal), MODE_CONTROL);
            bb_cal |= 0x40;
            if(!uhf_result)
                uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &bb_cal, (uint8_t *)&status, (uint32_t)sizeof(bb_cal), MODE_CONTROL);*/

            if(!uhf_result)
                uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_ON);
            //now ensure that phy is ON before quitting
            //also timeout if it never gets into ON state
            timeout = 15000;     //TODO typically PHY_OFF -> PHY_ON takes 261us. right now arbit
                                //assuming 4Mhz CCLK, 1044 cycles. need to make this more precise
            //time1 = get_current_time();
            if(!uhf_result)
            {
            	do
            	{
                	if( 0 < timeout )
                    	uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
                    else
                        uhf_result = ADI_ADF7023_ERR_PHY_ON;
                    timeout--;
                } while( !uhf_result && phy_state != PHY_ON );
            }
            //time2 = get_current_time();
            if (phy_state == PHY_ON)
                hDevice->PhyState = PHY_ON;

            break;

        case PHY_TX:
        case PHY_RX:
            uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_ON);
            //now ensure that phy is ON before quitting
            //also timeout if it never gets into ON state
            timeout = 4000; //TODO typically PHY_RX or PHY_TX -> PHY_ON takes 70us
                           //need to make timeout more precise
            if(!uhf_result)
            {
                do
                {
                    if( 0 < timeout )
                        uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
                    else
                        uhf_result = ADI_ADF7023_ERR_PHY_ON;
                    timeout--;
                } while( !uhf_result && phy_state != PHY_ON );
            }
            if (phy_state == PHY_ON)
                hDevice->PhyState = PHY_ON;

            break;
#ifdef PATCH_802_15_4D
        case PHY_RX_CCA_15D4:
        case PHY_TX_15D4 :

         if(!uhf_result)
             uhf_result = adi_ADF7023_SetPhyOn15d4(hDevice);

        case  PHY_ON_15D4 :

        if(!uhf_result)
                uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_EXIT_15D4_MODE);
            //now ensure that phy is in 15d4 ON before quitting
               //also timeout if it never gets into ON state
            timeout = 15000;     //TODO typically PHY_OFF -> PHY_ON takes 261us. right now arbit
                                //assuming 4Mhz CCLK, 1044 cycles. need to make this more precise
            if(!uhf_result)
            {
                do
                {
                    if( 0 < timeout )
                        uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
                    else
                        uhf_result = ADI_ADF7023_ERR_PHY_ON;
                    timeout--;
                } while( !uhf_result && phy_state != PHY_ON );
            }
            if (phy_state == PHY_ON)
                hDevice->PhyState = PHY_ON;

			break ;
#endif

        case PHY_ON:
            //great. less work
            hDevice->PhyState = PHY_ON;
            break;

        default:
            //we are in one of the states that we cannot issue PHY_ON in return error
            return ADI_ADF7023_ERR_PHY_ON;
    }

    return uhf_result;
}

/*!
 * @brief Sets the ADF7023 radio to OFF state
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Queries the current state of the radio and then turns the ADF7023 phy off.
 * Modifies the PhyState paramenter in the device handle.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyOff(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    volatile ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    ADI_ADF7023_PHY_STATE_TYPE phy_state = PHY_BUSY_TRX;
    volatile int timeout; //arbit

    uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
    if(uhf_result != ADI_ADF7023_SUCCESS)
       return uhf_result;

    switch(phy_state)
    {

        case PHY_SLEEP:
            uhf_result = PhyWakeUp(hDevice); //this should be good enough to put the part into PHY_OFF
                         //let rest of the chip init be handled by app??
            if(!uhf_result)
                uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_SYNC);
            timeout = 45000;
            //time1 = get_current_time();
            while(timeout--);
            //time2 = get_current_time();
            //ADFDoubleNop(hDevice);

            timeout = 15000; //arbit
            if(!uhf_result)
            {
                do
                {
                    if( 0 < timeout )
                        uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
                    else
                        uhf_result = ADI_ADF7023_ERR_PHY_OFF;
                    timeout--;
                } while( !uhf_result && phy_state != PHY_OFF );
            }

            if(!uhf_result)
                uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

            if (phy_state == PHY_OFF)
                hDevice->PhyState = PHY_OFF;

            break;


        case PHY_RX:
        case PHY_TX:

#ifdef PATCH_802_15_4D

        case PHY_RX_CCA_15D4:
        case PHY_TX_15D4 :
        case PHY_ON_15D4 :
#endif
        	uhf_result = adi_ADF7023_SetPhyOn(hDevice);

        case PHY_ON:
            if(!uhf_result)
                uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_OFF);
            //now ensure that phy is OFF before quitting
            //also timeout if it never gets into OFF state
            timeout = 15000;    //TODO typically PHY_ON -> PHY_OFF takes 226us
                               //need to make timeout more precise
            if(!uhf_result)
            {
                do
                {
                    if( 0 < timeout )
                        uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
                    else
                        uhf_result = ADI_ADF7023_ERR_PHY_OFF;
                    timeout--;
                }while( !uhf_result && phy_state != PHY_OFF );
            }
            if (phy_state == PHY_OFF)
                hDevice->PhyState = PHY_OFF;

            break;

       case PHY_OFF:
            //easy
            hDevice->PhyState = PHY_OFF;
            break;

       default:
            //one of the states we can't do anything in
            return ADI_ADF7023_ERR_PHY_OFF;
    }

    return uhf_result;
}

/*!
 * @brief Sets the ADF7023 radio to TX state
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Queries the current state of the radio and then turns the ADF7023 phy TX on.
 * Modifies the PhyState paramenter in the device handle.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyTx(ADI_ADF7023_DEV_HANDLE const hDevice)
{

	volatile ADI_ADF7023_RESULT_TYPE uhf_result;
	ADI_ADF7023_PHY_STATE_TYPE phy_state = PHY_BUSY_TRX;
	volatile int timeout;    //to timeout if we can't set the phy in TX mode


#ifdef FLAG_BENCHMARKING
	//    SetTestFlag(0,1);	//set ATB1 test flag
#endif

#ifdef BENCH_MARKING
	benchMark_set_time(23);
#endif

	uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);

#ifdef BENCH_MARKING
	benchMark_set_time(24);
#endif

	if(uhf_result != ADI_ADF7023_SUCCESS)
		return uhf_result;

	switch(phy_state)
	{

	case PHY_SLEEP:
		uhf_result = adi_ADF7023_SetPhyOff(hDevice);

	case PHY_OFF:
		if(!uhf_result)
			uhf_result = adi_ADF7023_SetPhyOn(hDevice);

	case PHY_ON:
	case PHY_TX: //tricky.. there is no change in state here, but the command can still be issued..
	case PHY_RX:

#ifdef  PATCH_802_15_4D
	case PHY_ON_15D4:
	case PHY_TX_15D4:
	case PHY_RX_CCA_15D4:
#endif

#ifdef BENCH_MARKING

		benchMark_set_time(25);
#endif
		while(hDevice->SpiState != ADI_ADF7023_SPI_IDLE);

#ifdef BENCH_MARKING

		benchMark_set_time(26);
#endif
#ifdef FLAG_BENCHMARKING
		//    		SetTestFlag(0,0);	//set ATB1 test flag
#endif

		if( hDevice->FastTransitionsEnabled )
		{
			if( (hDevice->synth_cal_pending) )
			{
				//				adi_ADF7023_15d4g_Enable_Disable_Auto_SynthCalibration(hDevice, 1);
				uhf_result = adi_ADF7023_Synthesizer_Calibration(hDevice);
			}
		}

		//SetTestFlag(0,0);
		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_TX);
#ifdef FLAG_BENCHMARKING
		//    		SetTestFlag(0,1);	//set ATB1 test flag
#endif
		timeout = 15000; //typical PHY_ON -> PHY_TX = 300us
		//TODO need to replace this with a more accurate counter
#ifdef BENCH_MARKING

		benchMark_set_time(27);
#endif
		if(!uhf_result)
		{
			do
			{
				if( 0 < timeout )
					uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
				else
					uhf_result = ADI_ADF7023_ERR_PHY_TX;
				timeout--;
			}
#ifdef  PATCH_802_15_4D
			while(!uhf_result && phy_state != PHY_TX_15D4);
#else
			while(!uhf_result && phy_state != PHY_TX );
#endif
		}

#ifdef BENCH_MARKING
		benchMark_set_time(28);
#endif

#ifdef  PATCH_802_15_4D
		if(phy_state == PHY_TX_15D4)
			hDevice->PhyState = PHY_TX_15D4;
#else

		if(phy_state == PHY_TX)
			hDevice->PhyState = PHY_TX;
#endif

		if( hDevice->FastTransitionsEnabled )
		{
			if(hDevice->synth_cal_pending)
			{
				//				adi_ADF7023_15d4g_Enable_Disable_Auto_SynthCalibration(hDevice, 0);

#ifdef UTEST_UHF
				uint8_t vco_band_read_back = 0, vco_ampl_read_back = 0;

				if(!uhf_result)
					uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&vco_band_read_back, 1, BB_vco_band_readback);

				if(!uhf_result)
					uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&vco_ampl_read_back, 1, BB_vco_ampl_readback);


				Set_UTU_TimeStamp(190,hDevice->Frequency);
				Set_UTU_TimeStamp(191,((vco_band_read_back << 8) | vco_ampl_read_back ));
#endif

#ifdef UHF_LOG_VCO_VALS_DRIVER
				uint8_t vco_band_read_back = 0, vco_ampl_read_back = 0;

				if(!uhf_result)
					uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&vco_band_read_back, 1, BB_vco_band_readback);

				if(!uhf_result)
					uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&vco_ampl_read_back, 1, BB_vco_ampl_readback);


				Log_Frequency_VCO_ReadBack(hDevice->Frequency, vco_band_read_back, vco_ampl_read_back);
#endif
#ifdef STATISTICS_ENABLED
				rf_statistics.radio_synth_cal_performed++;
#endif//#ifdef STATISTICS_ENABLED
				hDevice->synth_cal_pending = 0;
			}
		}

#ifdef BENCH_MARKING
		benchMark_set_time(29);
#endif

		break;

	default:
		//one of the states that we cannot switch to PHY_TX
		return ADI_ADF7023_ERR_PHY_TX;
	}

#ifdef FLAG_BENCHMARKING
	//    SetTestFlag(0,0);	//set ATB1 test flag
#endif
	return uhf_result;
}

/*!
 * @brief Sets the ADF7023 radio to RX state
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Queries the current state of the radio and then turns the ADF7023 phy RX on.
 * Modifies the PhyState paramenter in the device handle.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyRx(ADI_ADF7023_DEV_HANDLE const hDevice)
{

	volatile ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	ADI_ADF7023_PHY_STATE_TYPE phy_state = PHY_BUSY_TRX;
	volatile uint16_t timeout = 15000;    //to timeout if we can't set the phy in TX mode

	//    uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
	uhf_result = adi_ADF7023_GetPhyState_PhyRx(hDevice, &phy_state);
        //phy_state =  hDevice->PhyState;

//	if(uhf_result != ADI_ADF7023_SUCCESS)
//		return uhf_result;

     if(phy_state != PHY_RX_CCA_15D4)
        rx_start_time = get_current_time();

	switch(phy_state)
	{

	case PHY_SLEEP:
		uhf_result = adi_ADF7023_SetPhyOff(hDevice);

	case PHY_OFF:
		if(!uhf_result)
			uhf_result = adi_ADF7023_SetPhyOn(hDevice);



	case PHY_TX:
	case PHY_RX:
    case PHY_ON:
#ifdef  PATCH_802_15_4D
        if(!uhf_result)
			uhf_result = adi_ADF7023_SetPhyOn15d4(hDevice);
#endif

#ifdef  PATCH_802_15_4D
	case PHY_ON_15D4:
	case PHY_TX_15D4:
	//case PHY_RX_CCA_15D4:
#endif

		if( hDevice->FastTransitionsEnabled )
		{
			if( (hDevice->synth_cal_pending) )
			{
                            //ADI_ENABLE_INT(SYS_GPIO_INTA_IRQn);
				//					adi_ADF7023_15d4g_Enable_Disable_Auto_SynthCalibration(hDevice, 1);
				uhf_result = adi_ADF7023_Synthesizer_Calibration(hDevice);

			}
		}

		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_RX);
		timeout = 15000; //typical PHY_ON -> PHY_TX = 300us
		//TODO need to replace this with a more accurate counter
		if(!uhf_result)
		{
			do
			{
				if( 0 < timeout )
					uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
				else
					uhf_result = ADI_ADF7023_ERR_PHY_RX;
				timeout--;
			}
#ifdef  PATCH_802_15_4D
			while(!uhf_result && phy_state != PHY_RX_CCA_15D4);
#else
			while(!uhf_result && phy_state != PHY_RX );
#endif
		}

		if( hDevice->FastTransitionsEnabled )
		{
			if(hDevice->synth_cal_pending)
			{
				//				adi_ADF7023_15d4g_Enable_Disable_Auto_SynthCalibration(hDevice, 0);
#ifdef UTEST_UHF
				uint8_t vco_band_read_back = 0, vco_ampl_read_back = 0;

				if(!uhf_result)
					uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&vco_band_read_back, 1, BB_vco_band_readback);

				if(!uhf_result)
					uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&vco_ampl_read_back, 1, BB_vco_ampl_readback);


				Set_UTU_TimeStamp(190,hDevice->Frequency);
				Set_UTU_TimeStamp(191,((vco_band_read_back << 8) | vco_ampl_read_back ));
#endif

#ifdef UHF_LOG_VCO_VALS_DRIVER
				uint8_t vco_band_read_back = 0, vco_ampl_read_back = 0;

				if(!uhf_result)
					uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&vco_band_read_back, 1, BB_vco_band_readback);

				if(!uhf_result)
					uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&vco_ampl_read_back, 1, BB_vco_ampl_readback);


				Log_Frequency_VCO_ReadBack(hDevice->Frequency, vco_band_read_back, vco_ampl_read_back);
#endif
				hDevice->synth_cal_pending = 0;
#ifdef STATISTICS_ENABLED
				rf_statistics.radio_synth_cal_performed++;
#endif//#ifdef STATISTICS_ENABLED
			}
		}

#ifdef  PATCH_802_15_4D

		if(phy_state == PHY_RX_CCA_15D4)
			hDevice->PhyState = PHY_RX_CCA_15D4;
#else
		if(phy_state == PHY_RX)
			hDevice->PhyState = PHY_RX;

#endif
		break;
    case PHY_RX_CCA_15D4:
        break;

	default:
		//one of the states that we cannot switch to PHY_RX
		return ADI_ADF7023_ERR_PHY_RX;
	}

	return uhf_result;
}

/*!
 * @brief Returns the current state of the 7023 phy
 *
 * @param[in]   hDevice     Handle to the UHF device.
 * @param[out]  *PhyState   Pointer to variable where the current phy state is written.
 *
 * @return      ADI_ADF7023_RESULT_TYPE
 *
 * Returns the current state of the radio. Reads the status word from the radio by issuing
 * 2 SPI_NOP commands, and extracts and returns the current state of the radio from the
 * status word.
 *
 */
uint32_t busyState10Counter = 0, noOfState10Counter =0;
ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetPhyState(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_PHY_STATE_TYPE* const PhyState)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    ADI_ADF7023_STATUS_TYPE status;
    uint8_t onceVal = 0;

    do
    {
	uhf_result = ReadStatusReg(hDevice, &status);
	if(!uhf_result)
	    *PhyState = status.fw_state;
	if (*PhyState == 0x10)
	{
	    busyState10Counter++;
	    if (onceVal == 0)
	    {
		noOfState10Counter++;
		onceVal = 1;
	    }
	}

    } while((*PhyState == PHY_BUSY_TRX)	|| (*PhyState == 0x10));

    return uhf_result;
}

/*!
 * @brief This function will block and poll radio until it reaches PhyState and cmd_ready becomes true.
 *
 * @param[in]   hDevice     Handle to the UHF device.
 * @param[out]  PhyState    expected Radio state to which radio should be transited.
 *
 * @return      ADI_ADF7023_RESULT_TYPE
 *
 * This function will block and poll radio until it reaches expected(passed) radio state.
 *
 */
//
//uint32_t time1, time2;
ADI_ADF7023_RESULT_TYPE adi_ADF7023_WaitforPhyStateReady(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_PHY_STATE_TYPE PhyState)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    ADI_ADF7023_STATUS_TYPE status;
    ADI_ADF7023_PHY_STATE_TYPE CurrentPhyState = PHY_BUSY_TRX;
    int cmd_ready = 0;
    volatile uint32_t timeout = 13250; //35 uS to execute the do - while  once. For 0.5 Secs value is 13500.
    //time1 = get_current_time();
    do {

	    uhf_result = ReadStatusReg(hDevice, &status);

		if(!uhf_result)
		{
	        CurrentPhyState = status.fw_state;
	        cmd_ready = status.cmd_ready;
		}

	} while(((CurrentPhyState != PhyState) || !cmd_ready) && timeout--);
	//time2 = get_current_time();
	if(timeout == 0)
		return ADI_ADF7023_ERR_RADIO_INVALID_STATE_CHANGE;

	if( hDevice->FastTransitionsEnabled )
	{
		#ifdef BENCH_MARKING
			if(hDevice->auto_tx_to_rx_enabled)
			{
				TxToRxTimeStamp2 = getBenchMarkTime();

				time_diff_TX_to_RX[TxtoRxInd++] = TxToRxTimeStamp2 - TxToRxTimeStamp1;
			}
		#endif

		//STOP_CYCLE_COUNT(stop_count, start_count);
	}

    return uhf_result;
}

/*!
 * @brief Used to write any number of consecutive memory locations in the 7023 memory map
 *
 * @param[in]       hDevice     Pointer to device handle.
 * @param[in]       pDataTx     Pointer to the data buffer to be transmitted.
 * @param[out]      pStatus     Pointer to status word that is returned by the radio.
 * @param[in]       size        Size of the buffer to be written.
 * @param[in]       AdfAddr     Start address of the write on the 7023.
 *
 * @return          ADI_ADF7023_RESULT_TYPE
 *
 * This function can be used to write an arbitrary number of bytes to consecutive
 * memory locations of the 7023 memory map. This includes the BBRAM, the MCR and the
 * packet RAM regions of the 7023.
 *
 * @note    The function does not perform any bounds, or range checks on the address.
 *          So the user must ensure that they are not writing to invalid memory.
 *
 * @sa      adi_ADF7023_BlockMemRead().
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_BlockMemWrite (ADI_ADF7023_DEV_HANDLE const hDevice,
        uint8_t* const pDataTx, uint8_t* const pStatus,
        uint32_t const size, uint16_t const AdfAddr)
{
    volatile ADI_ADF7023_RESULT_TYPE spi_result = ADI_ADF7023_SUCCESS;

//    do
//    {
	adi_int_EnterCriticalRegion();

	while(ADI_ADF7023_SPI_IDLE != hDevice->SpiState)
	{
	    adi_int_ExitCriticalRegion();
	    if(true != hDevice->bBlockingMode)
		return ADI_ADF7023_ERR_RADIO_BUSY;
	    adi_int_EnterCriticalRegion();
	}

	if(ResetSpiBuffer(hDevice) != ADI_ADF7023_SUCCESS)
	{
	    adi_int_ExitCriticalRegion();
	    return ADI_ADF7023_ERR_SPI_BUSY;
	}

	/* first we need to make sure that the spi can take a buffer
	* poll on the spi result until there is one free buffer for
	* the spi driver to take */

	hDevice->SpiPrologue[0] = CreateMemcmd(ADI_ADF7023_CMD_SPI_MEM_WR, AdfAddr);
	hDevice->SpiPrologue[1] = (uint8_t)(AdfAddr & 0xff);

	hDevice->SpiBuffer.pPrologue    = hDevice->SpiPrologue;
	hDevice->SpiBuffer.DataSize    = size;
	hDevice->SpiBuffer.pTxData      = pDataTx;
	hDevice->SpiBuffer.pRxData      = pStatus;
	hDevice->SpiBuffer.PrologueSize = 2;
	hDevice->SpiBuffer.bTxIncrement = true;  // stationary transmit data pointer
	hDevice->SpiBuffer.bRxIncrement = false;   // dynamic receive data pointer
//	hDevice->SpiBuffer.bBufferProcessed = false;

    adi_spi_SetChipSelect(hDevice->pSpi, ADI_SPI_CS3);
    //value1 = get_current_time();

    //volatile int ii = 20;
    //while(ii--);

	spi_result = adi_spi_MasterRadioTx(hDevice->pSpi, &hDevice->SpiBuffer);
    //value2 = get_current_time();
	if(spi_result == ADI_ADF7023_SUCCESS)
	    hDevice->SpiState = ADI_ADF7023_SPI_TX;

    //value1 = get_current_time();
	adi_int_ExitCriticalRegion();
//    } while( ADI_SPI_RESULT_XFER_QUEUE_FULL == spi_result && true == hDevice->bBlockingMode);
    //value2 = get_current_time();
    if(hDevice->bBlockingMode)
    {

	if (spi_result != ADI_ADF7023_SUCCESS)
	    return ADI_ADF7023_ERR_BLOCKWRITE;
    //value1 = get_current_time();
	while(hDevice->SpiState != ADI_ADF7023_SPI_IDLE); /* wait till transfer is complete
	* hDevice->SpiBuffering, but put timeout here?? */
    //value2 = get_current_time();
    }

    else if(spi_result != ADI_ADF7023_SUCCESS && hDevice->bBlockingMode == false)
	return ADI_ADF7023_ERR_BLOCKWRITE;

    return ADI_ADF7023_SUCCESS;

}

/*!
 * @brief Used to read a number of bytes from the 7023.
 *
 * @param[in]       hDevice     Pointer to device handle.
 * @param[out]      pDataRx     Pointer to the receive data buffer.
 * @param[in]       size        Number of bytes to be read from the 7023.
 * @param[in]       AdfAddr     Start address of the read.
 *
 * @return          ADI_ADF7023_RESULT_TYPE
 *
 * This function can be used to read a specified number of bytes from
 * consecutive memory locations on the 7023. This function can access
 * all parts of the 7023 memory map including the BBRAM, the MCR, and
 * the packet RAM.
 *
 * @note    The function does not perform range, or bound checks on the
 *          address passed to it.
 *
 * @sa      adi_ADF7023_BlockMemWrite().
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_BlockMemRead (ADI_ADF7023_DEV_HANDLE const hDevice,
        uint8_t* const pDataRx, uint32_t const size,
        uint16_t AdfAddr)
{
    volatile uint32_t timeout = 3000000;
    volatile ADI_ADF7023_RESULT_TYPE spi_result = ADI_ADF7023_SUCCESS;

//    do
//    {
	adi_int_EnterCriticalRegion();

	while(ADI_ADF7023_SPI_IDLE != hDevice->SpiState)
	{
	    adi_int_ExitCriticalRegion();
	    if(true != hDevice->bBlockingMode)
		return ADI_ADF7023_ERR_RADIO_BUSY;
	    adi_int_EnterCriticalRegion();
	}

	if(ResetSpiBuffer(hDevice) != ADI_ADF7023_SUCCESS)
	{
	    adi_int_ExitCriticalRegion();
	    return ADI_ADF7023_ERR_SPI_BUSY;
	}
	hDevice->IssuedCmd = ADI_ADF7023_CMD_SPI_NOP;

	hDevice->SpiPrologue[0] = CreateMemcmd(ADI_ADF7023_CMD_SPI_MEM_RD, AdfAddr);
	hDevice->SpiPrologue[1] = (uint8_t)(AdfAddr & 0xff);
	hDevice->SpiPrologue[2] = (uint8_t)ADI_ADF7023_CMD_SPI_NOP;

	hDevice->SpiBuffer.pPrologue    = hDevice->SpiPrologue;
	hDevice->SpiBuffer.DataSize    = size;
	hDevice->SpiBuffer.pTxData      = (uint8_t *)&hDevice->IssuedCmd;
	hDevice->SpiBuffer.pRxData      = pDataRx;
	hDevice->SpiBuffer.PrologueSize = 3;
	hDevice->SpiBuffer.bTxIncrement = false;  // stationary transmit data pointer
	hDevice->SpiBuffer.bRxIncrement = true;   // dynamic receive data pointer
//	hDevice->SpiBuffer.bBufferProcessed = false;


	/* first we need to make sure that the spi can take a buffer
	* poll on the spi result until there is one free buffer for
	* the spi driver to take */
    adi_spi_SetChipSelect(hDevice->pSpi, ADI_SPI_CS3);

	spi_result = adi_spi_MasterRadioTx(hDevice->pSpi, &hDevice->SpiBuffer);

	if(spi_result == ADI_ADF7023_SUCCESS)
	    hDevice->SpiState = ADI_ADF7023_SPI_RX;
	adi_int_ExitCriticalRegion();
//    } while(ADI_SPI_RESULT_XFER_QUEUE_FULL == spi_result && true == hDevice->bBlockingMode);

    if(hDevice->bBlockingMode)
    {     /* fake non-blocking */

        if (spi_result != ADI_ADF7023_SUCCESS)
            return ADI_ADF7023_ERR_BLOCKWRITE;

        while ( (hDevice->SpiState != ADI_ADF7023_SPI_IDLE) && (0 < timeout))
        {
            timeout--;
            //hDevice->SpiState = ADI_ADF7023_SPI_IDLE;
        }

        hDevice->SpiState = ADI_ADF7023_SPI_IDLE;
        //while(hDevice->SpiState != ADI_ADF7023_SPI_IDLE);
        /* wait till transfer is complete
        * hDevice->SpiBuffering, but put timeout here?? */
        hDevice->IssuedCmd = ADI_ADF7023_CMD_NULL;
    }
    else if(spi_result != ADI_ADF7023_SUCCESS && hDevice->bBlockingMode == false)
		return ADI_ADF7023_ERR_BLOCKWRITE;

    return ADI_ADF7023_SUCCESS;
}

/*!
 * @brief
 *
 * @param[in]       hDevice     Pointer to device handle
 * @param[in]       cmd         The command to be issued to the radio
 *
 * @return          ADI_ADF7023_RESULT_TYPE
 *
 * Issues a command to the radio. Once the command is issued, the fucntion
 * waits till it receives a status word back from the radio indicating
 * that the 7023 is ready to receive more commands and then returns.
 *
 * @note    SPI read, or write commands cannot be issued using this function,
 *          and will result in an error being returned. However the function
 *          does not check for all possible illegal commands.
 *
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_IssueCmd(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_CMD_TYPE const cmd)
{
    volatile ADI_ADF7023_RESULT_TYPE spi_result = ADI_ADF7023_SUCCESS;
    volatile ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t rbuf;

    if(cmd < ADI_ADF7023_CMD_SYNC)
	return ADI_ADF7023_ERR_BAD_CMD;


    adi_int_EnterCriticalRegion();

    while(true != hDevice->bIsCmdRdy  || ADI_ADF7023_SPI_IDLE != hDevice->SpiState)
    {
        adi_int_ExitCriticalRegion();
        if(true != hDevice->bBlockingMode)
            return ADI_ADF7023_ERR_RADIO_BUSY;
        adi_int_EnterCriticalRegion();
    }

    if(ResetSpiBuffer(hDevice) != ADI_ADF7023_SUCCESS)
    {
        adi_int_ExitCriticalRegion();

        return ADI_ADF7023_ERR_SPI_BUSY;
    }

    hDevice->IssuedCmd = cmd;

    // prologue not used here, just send cmd as data
    hDevice->SpiBuffer.pPrologue    = NULL;
    hDevice->SpiBuffer.DataSize     = 1;
    hDevice->SpiBuffer.pTxData      = (uint8_t *)&hDevice->IssuedCmd;
    hDevice->SpiBuffer.pRxData      = &rbuf; //NULL; //(uint8_t *)&status; RK mod - in non-hDevice->SpiBuffering receiving status
    // will cause stack corruption
    // because a SPI interrupt comes in and messes things
    hDevice->SpiBuffer.PrologueSize = 0;
    hDevice->SpiBuffer.bTxIncrement = false;
    hDevice->SpiBuffer.bRxIncrement = false;
    //        hDevice->SpiBuffer.bBufferProcessed = false;


    /* next, wait till the spi is ready to accept another buffer
    * again, blocking might be a problem. can break out if needed */
    adi_spi_SetChipSelect(hDevice->pSpi, ADI_SPI_CS3);

    //volatile int ii = 20;
    //while(ii--);

    spi_result = adi_spi_MasterRadioTx(hDevice->pSpi, &hDevice->SpiBuffer);

    if(spi_result == ADI_ADF7023_SUCCESS)
    {
	hDevice->bIsCmdRdy = false;
	hDevice->SpiState = ADI_ADF7023_SPI_CMD; /* if buffer isn't submitted, do not
	change spi state, else will never go back
	to idle */
    }
    adi_int_ExitCriticalRegion();

    if(hDevice->bBlockingMode)
    {

        if (spi_result != ADI_ADF7023_SUCCESS)
            return ADI_ADF7023_ERR_BLOCKWRITE;

        while(hDevice->SpiState != ADI_ADF7023_SPI_IDLE); /* wait till transfer is complete
        * hDevice->SpiBuffering, but put timeout here?? */

        if(!hDevice->InitState)
        {  					/* because interrupts have not been initialized yet
            make sure the radio is ready to accept more commands
            before returning*/
            ADI_ADF7023_STATUS_TYPE status;
            do
            {
            ReadStatusReg(hDevice, &status);
            }while(!status.cmd_ready);

            hDevice->bIsCmdRdy = true;
        }

    }
    else if(spi_result != ADI_ADF7023_SUCCESS && hDevice->bBlockingMode == false)
        return ADI_ADF7023_ERR_BLOCKWRITE;

    if (cmd == ADI_ADF7023_CMD_CONFIG_DEV)
    {
#ifdef PATCH_802_15_4D
        uint8_t agcClkDivideVal = DEFAULT_AGC_CLK_DIVIDE_VAL;

        adi_ADF7023_SetAgcClockDivideMCR(hDevice, agcClkDivideVal);
#endif
	   	uhf_result = adi_ADF7023_SetTransmitterPowerLevelMCR(hDevice, hDevice->mcrPALevel);
    }

    return uhf_result;
}



/**
 * @brief   Sets driver blocking mode
 *
 * @param[in]   hDevice             UHF device handle
 * @param[in]   bBlockingMode       Blocking mode (true = blocking, false = non-blocking)
 *
 * @return      ADI_ADF7023_RESULT_TYPE
 *
 * Sets the driver blocking mode.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetBlockingMode(ADI_ADF7023_DEV_HANDLE const hDevice, bool bBlockingMode)
{
    hDevice->bBlockingMode = bBlockingMode;
        return ADI_ADF7023_SUCCESS;
}
/**
 * @brief   Gets driver blocking mode
 *
 * @param[in]   hDevice             UHF device handle
 *
 * @return      bool
 *
 * Gets the driver blocking mode.
 *
 */
bool adi_ADF7023_GetBlockingMode(ADI_ADF7023_DEV_HANDLE const hDevice)
{
    return hDevice->bBlockingMode;
}



/**
 * @brief   Allows the driver to stop the current transfer and schedule a new one
 *
 * @param[in]   hDevice             UHF Device handle
 * @param[in]   bForceTransfer      True to allow forced transfers. False to disallow.
 *
 * @return
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetForceTransfer(ADI_ADF7023_DEV_HANDLE const hDevice, bool bForceTransfer)
{
    hDevice->bForceTransfer = bForceTransfer;
    return ADI_ADF7023_SUCCESS;
}

/**
 * @brief   Returns the force transfer status
 *
 * @param hDevice
 *
 * @return  True - Forced transfers are allowd. False - They are not.
 */
bool adi_ADF7023_GetForceTransfer(ADI_ADF7023_DEV_HANDLE const hDevice)
{
    return hDevice->bForceTransfer;
}


/*!
 * @brief Initializes all of BBRAM
 *
 * @param[in]   hDevice     UHF device handle
 * @param[in]   *BBRAM      Pointer to the BBRAM type
 *
 * @return      ADI_ADF7023_RESULT_TYPE
 *
 * This function initializes all of BBRAM to values passed down in the pointer to
 * the pre-initialized BBRAM structure. Is useful when one wants to quickly populate
 * all of BBRAM with a set of custom register values.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_InitBBRAM(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BBRAM_TYPE* BBRAM)
{
    ADI_ADF7023_RESULT_TYPE result;
    uint8_t status;

    result = adi_ADF7023_BlockMemWrite(hDevice, (uint8_t *)BBRAM, &status, sizeof(ADI_ADF7023_BBRAM_TYPE), INTERRUPT_MASK_0);

#ifdef ADI_DEBUG
    ADI_ADF7023_BBRAM_TYPE BBRAM_test;
    if(!result)
        result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&BBRAM_test, sizeof(ADI_ADF7023_BBRAM_TYPE), INTERRUPT_MASK_0);
    if(BBRAM_test.interrupt_mask_0_r != BBRAM->interrupt_mask_0_r)
      	result = ADI_ADF7023_ERR_UNKNOWN;
#endif
    return result;
}

/*!
 * @brief Reads the current RSSI at the receiver using RSSI mode 2
 *
 * @param[in]       hDevice      handle to the ADF7023 device.
 * @param[out]      *RssiRead    pointer to RSSI value.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Reads the received input power in 2's complement dBm
 *
 * @note    This function can only be called in PHY_ON state
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetRSSI(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t* const RssiRead)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    if(hDevice->PhyState != PHY_ON)
        return ADI_ADF7023_ERR_RSSI_READ_FAIL;

    /* The GET_RSSI command turns the receiver on, reads the RSSI on the current */
    /* channel, stores the result in the rssi_readback register and returns the */
    /* phy to ON state. */
    uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_GET_RSSI);
	while(!hDevice->bIsCmdRdy);

    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, RssiRead, (uint32_t)0x1, RSSI_READBACK);

    *RssiRead -= 107;

    return uhf_result;
}
/*!
 * @brief Reads the current RSSI at the receiver using ADF7023 RSSI mode 3
 * with the AGC set manually in the range of the CCA threshold
 *
 * @param[in]       hDevice      handle to the ADF7023 device.
 * @param[out]      *RssiRead    pointer to RSSI value.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Reads the received input power in 2's complement dBm
 *
 * @note    This function can only be called in PHY_RX state
 *			The function adi_ADF7023_RSSI_CCA_mode_start() should be
 *			called first to set the AGC manually. Then this function
 *			can be called repeatedly to sample the RSSI level. The function
 *			adi_ADF7023_RSSI_CCA_mode_stop should be called after CCA
 *			sampling is done to set the AGC back to free run.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetRSSI_CCA_mode(ADI_ADF7023_DEV_HANDLE const hDevice, int* const RssiRead)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint16_t adc_readback, gain_correction;

    if(hDevice->PhyState != PHY_RX)
        return ADI_ADF7023_ERR_RSSI_READ_FAIL;

	//AGC should be set manually at this point
#if 0
    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t*) &adc_readback, (uint32_t)0x2, ADC_READBACK_HIGH);
#else

//sequence needed to get correct reading...
	if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t*) &adc_readback, (uint32_t)0x1, ADC_READBACK_HIGH);
	if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t*) &adc_readback+1, (uint32_t)0x1, ADC_READBACK_LOW);
	if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t*) &adc_readback, (uint32_t)0x1, ADC_READBACK_HIGH);
#endif

   	if(!uhf_result)
	{
		adc_readback = ((adc_readback>>8)|(adc_readback<<8)) >> 6;
#if 1
		gain_correction = 10; //AGC 0x12 level assumed
#else
		//option to read back the AGC level manually.
		if(!uhf_result)
        	uhf_result = adi_ADF7023_BlockMemRead(hDevice, &agc_gain, (uint32_t)0x1, AGC_GAIN_STATUS);

        switch(agc_gain)
		{
			case 0:
				gain_correction = 44;
				break;
			case 1:
				gain_correction = 35;
				break;
			case 2:
				gain_correction = 26;
				break;
			case 0xa:
				gain_correction = 17;
				break;
			case 0x12:
				gain_correction = 10;
				break;
			case 0x16:
				gain_correction = 0;
				break;
			default:
				return ADI_ADF7023_ERR_RSSI_READ_FAIL;
		}
#endif
		*RssiRead = (adc_readback & 0xff)/7 + gain_correction - 109;
	}

    return uhf_result;
}
/*!
 * @brief Sets the AGC manually within the range of the expected
 *		CCA threshold so the function adi_ADF7023_GetRSSI_CCA_mode
 *		can be called repeatedly without need to lock/unlock the AGC.
 *
 * @param[in]       hDevice      handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_RSSI_CCA_mode_start(ADI_ADF7023_DEV_HANDLE const hDevice)
{
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t status, reg_val;

    //Set the AGC in manual mode with high gain
    //filter gain high, mixer gain low, LNA gain high.
    //This setting used for -80dBm CCA threshold
    reg_val = AGC_MODE_CCA_NEG80DBM_THRESHOLD;
    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &reg_val, &status, 1, AGC_MODE);

    if(!uhf_result)
    	hDevice->RSSI_CCA_MODE_AGC_SET =  true;

    return uhf_result;
}
/*!
 * @brief Sets the AGC back to free run mode.
 *
 * @param[in]       hDevice      handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_RSSI_CCA_mode_stop(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t status, reg_val;

    //re-enable AGC
	reg_val = AGC_MODE_LOCK_AFTER_PREAMBLE;
    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &reg_val, &status, 1, AGC_MODE);

    if(!uhf_result)
    	hDevice->RSSI_CCA_MODE_AGC_SET =  false;

    return uhf_result;
}

#ifdef SPORT_MODE
/*!
 * @brief Reads the current RSSI at the receiver using ADF7023 RSSI mode 3
 *
 * @param[in]       hDevice      handle to the ADF7023 device.
 * @param[out]      *RssiRead    pointer to RSSI value.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Reads the received input power in 2's complement dBm
 *
 * @note    This function can only be called in PHY_RX state
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SPORTmode_GetRSSI(ADI_ADF7023_DEV_HANDLE const hDevice, int* const RssiRead)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    u8 status, reg_val, agc_gain;
    u16 adc_readback, gain_correction;

    if(hDevice->PhyState != PHY_RX)
        return ADI_ADF7023_ERR_RSSI_READ_FAIL;
    //lock the AGC
    reg_val = 0x40;
    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &reg_val, &status, 1, AGC_MODE);
    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, &agc_gain, (u32)0x1, AGC_GAIN_STATUS);
#if 0

    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (u8*) &adc_readback, (u32)0x2, ADC_READBACK_HIGH);
#else

//sequence needed to get correct reading...
	if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (u8*) &adc_readback, (u32)0x1, ADC_READBACK_HIGH);
	if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (u8*) &adc_readback+1, (u32)0x1, ADC_READBACK_LOW);
	if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (u8*) &adc_readback, (u32)0x1, ADC_READBACK_HIGH);
#endif

   	if(!uhf_result)
	{
		//re-enable AGC
		reg_val = AGC_MODE_LOCK_AFTER_PREAMBLE;
		uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &reg_val, &status, 1, AGC_MODE);

		adc_readback = ((adc_readback>>8)|(adc_readback<<8)) >> 6;

		switch(agc_gain)
		{
			case 0:
				gain_correction = 44;
				break;
			case 1:
				gain_correction = 35;
				break;
			case 2:
				gain_correction = 26;
				break;
			case 0xa:
				gain_correction = 17;
				break;
			case 0x12:
				gain_correction = 10;
				break;
			case 0x16:
				gain_correction = 0;
				break;
			default:
				return ADI_ADF7023_ERR_RSSI_READ_FAIL;
		}

		*RssiRead = (adc_readback & 0xff)/7 + gain_correction - 109;
	}

    return uhf_result;
}
#endif //SPORT_MODE
/*!
 * @brief Read the ADF7023 temperature sensor
 *
 * @param[in]       hDevice      handle to the ADF7023 device.
 * @param[out]      *TempRead    pointer to temperature value returned as a double containing
 *									the temperature in degrees Celcius.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Reads the ADF7023 temperature sensor and returns the result in degrees Celcius.
 *
 * @note    The constant TEMP_CORRECTION can be adjusted to calibrate the temperature reading.
 *			Please refer to the ADF7023-J.pdf manual section "TEMPERATURE SENSOR" for the details.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetTemperatureReading(ADI_ADF7023_DEV_HANDLE const hDevice, double *TempRead)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t status, pwdwn_rx, pwdwn_aux, adc_cfg_low;
    uint16_t adc_readback;
	double temp;
    ADI_ADF7023_STATUS_TYPE statusReg;

    uhf_result = ReadStatusReg(hDevice, &statusReg);

    if (uhf_result == ADI_ADF7023_SUCCESS)
    {
    	/* Radio state should be in one of the following state
    	to read the temperature. */
    	if (
#ifdef  PATCH_802_15_4D
			(statusReg.fw_state != PHY_ON_15D4) &&
			(statusReg.fw_state != PHY_TX_15D4) &&
#endif
    		(statusReg.fw_state != PHY_OFF) &&
    		(statusReg.fw_state != PHY_ON) &&
    		(statusReg.fw_state != PHY_TX))
    	{
    		return ADI_ADF7023_ERR_RADIO_BUSY;
    	}
    }

	//enable ADC on ADF7023 -- is done automatically in PHY_RX state
	uhf_result = adi_ADF7023_BlockMemRead(hDevice, &pwdwn_rx, 1, POWERDOWN_RX);
	pwdwn_rx |= ADC_PD_N;
	if(!uhf_result)
		uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &pwdwn_rx, &status, 1, POWERDOWN_RX);


	//enable temp monitor on ADF7023
        if(!uhf_result)
            uhf_result = adi_ADF7023_BlockMemRead(hDevice, &pwdwn_aux, 1, POWERDOWN_AUX);
    
	if(!uhf_result){
                pwdwn_aux |= TEMPMON_PD_EN;
		uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &pwdwn_aux, &status, 1, POWERDOWN_AUX);
        }


	//connect ADC to temp sensor on ADF7023
        if(!uhf_result){
          uhf_result = adi_ADF7023_BlockMemRead(hDevice, &adc_cfg_low, 1, ADC_CONFIG_LOW);
        }
        
	if(!uhf_result){
              adc_cfg_low &= ~ADC_REF_CHSEL;	//zero the bits
              adc_cfg_low |= ADC_TO_TEMP;
              uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &adc_cfg_low, &status, 1, ADC_CONFIG_LOW);
        }

	//read ADC
	//sequence needed to get correct reading...
	if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t*) &adc_readback, (uint32_t)0x1, ADC_READBACK_HIGH);
	if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t*) &adc_readback+1, (uint32_t)0x1, ADC_READBACK_LOW);
	if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t*) &adc_readback, (uint32_t)0x1, ADC_READBACK_HIGH);


    if(!uhf_result)
	{
		uhf_result = ReadStatusReg(hDevice, &statusReg);

	   	if (uhf_result == ADI_ADF7023_SUCCESS)
    	{
    		/* 	Radio state should be in one of the following state
    			to read the temperature. reverify the state after
    			reading the temprature to make sure the temperature
    			read is valid.*/
    		if(
#ifdef  PATCH_802_15_4D
			(statusReg.fw_state != PHY_ON_15D4) &&
			(statusReg.fw_state != PHY_TX_15D4) &&
#endif
    		(statusReg.fw_state != PHY_OFF) &&
    		(statusReg.fw_state != PHY_ON) &&
    		(statusReg.fw_state != PHY_TX))
    		{
    			return ADI_ADF7023_ERR_RADIO_BUSY;
    		}
    	}

		adc_readback = ((adc_readback>>8)|(adc_readback<<8)) >> 6;

		temp = (double) (adc_readback & 0xff);
		*TempRead = (temp - 42.197)/1.023 + TEMP_CORRECTION;
	}

	if(!uhf_result){
                	//restore ADC to connect to RSSI
            adc_cfg_low &= ~ADC_REF_CHSEL;	//zero the bits
            adc_cfg_low |= ADC_TO_RSSI;
            uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &adc_cfg_low, &status, 1, ADC_CONFIG_LOW);
        }

    return uhf_result;
}

/*!
 * @brief Sets the ADF7023 data rate
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   DataRate    the desired data rate in kbps * 10.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the ADF7023 data rate to the value passed in. Checks data rate to
 * ensure that the value is within the radio supported range or returns an
 * error.
 *
 * @note The value passed in must be the desired data rate in kbps times 10. For
 *       instance, for a data rate of 250 kbps, DataRate should be set to 2500.
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetDataRate(ADI_ADF7023_DEV_HANDLE const hDevice, uint16_t const DataRate)
{

    uint8_t DataRate_t[2], status;
    ADI_ADF7023_RESULT_TYPE result;

    if ((DataRate > 3000) || (DataRate < 10))
        return ADI_ADF7023_ERR_UNSUPPORTED_RATE;

    result = adi_ADF7023_BlockMemRead(hDevice, DataRate_t, 2, RADIO_CFG_0);
    if (result)
        return result;

    DataRate_t[0] = DataRate & 0xFF;
    DataRate_t[1] &= 0xF0; // Bottom nibble of radio_cfg_1_r is used for data_rate[11:8]
    DataRate_t[1] |= (DataRate >> 8) & 0xF;

    result = adi_ADF7023_BlockMemWrite(hDevice, &DataRate_t[0], &status, 2, RADIO_CFG_0);
    if (!result)
    {
        hDevice->DataRate = DataRate;
        return result;
    }

    return result;
}
/*!
 * @brief Returns the current data rate
 *
 * @param[in]   hDevice     Pointer to the device handle
 *
 * @return      Returns the current radio data rate.
 *
 * @sa adi_ADF7023_SetDataRate().
 */
uint16_t adi_ADF7023_GetDataRate(ADI_ADF7023_DEV_HANDLE const hDevice)
{
    uint8_t DataRate_t[2];
    uint16_t DataRate;

    adi_ADF7023_BlockMemRead(hDevice, DataRate_t, 2, RADIO_CFG_0);

    DataRate = (((uint16_t)DataRate_t[1] & 0xF) << 8) | DataRate_t[0];

    return DataRate;
}

/*!
 * @brief Sets the ADF7023 2FSK/GFSK frequency deviation
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   FreqDev    the desired frequency deviation in kHz x 10.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the desired 2FSK/GFSK frequency deviation. The value passed in should
 * be the desired frequency deviation in kHz * 10. For example for a 10 kHz
 * frequency deviation, pass in 100. The min and max possible frequency deviation
 * are 0.1 kHz and 409.5 khz respectively.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetFrequencyDeviation(ADI_ADF7023_DEV_HANDLE const hDevice, uint16_t const FreqDev)
{
    uint8_t FreqDev_t[2], status;
    ADI_ADF7023_RESULT_TYPE result;

    if ((FreqDev > 4095) || (FreqDev < 1))
        return ADI_ADF7023_ERR_INVALID_FREQ_DEV;

    result = adi_ADF7023_BlockMemRead(hDevice, FreqDev_t, 2, RADIO_CFG_1);
    if (result)
        return result;

    FreqDev_t[0] &= 0x0F;
    FreqDev_t[0] |= (uint8_t)((FreqDev >> 4) & 0xF0); // Bottom nibble of radio_cfg_1_r is used for data_rate[11:8]
    FreqDev_t[1] = (uint8_t)(FreqDev & 0xFF);

    result = adi_ADF7023_BlockMemWrite(hDevice, FreqDev_t, &status, 2, RADIO_CFG_1);

    return result;
}

/*!
 * @brief Sets the ADF7023 discriminator bandwidth coefficient
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   DiscrimBW  the desired discriminator bandwidth coefficient.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the ADF7023 discriminator bandwidth coefficient. This value is computed
 * as a function of the Modulation Index, the IF frequency, the datarate and
 * the expected maximum frequency error. Please refer to the datasheet for more
 * information.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetDiscrimBW(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const DiscrimBW)
{
    ADI_ADF7023_STATUS_TYPE status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t DiscrimBW_l;

    DiscrimBW_l = DiscrimBW;

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &DiscrimBW_l, (uint8_t *)&status, 1, RADIO_CFG_3);

    if (!uhf_result)
        return uhf_result;
    else
        return ADI_ADF7023_ERR_DISCRIMBW;
}

/*!
 * @brief Sets the ADF7023 post demodulation bandwidth
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   PostDemodBW   the post demodulation bandwidth.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * This value should be computed based on the formula from the data sheet and is
 * a function of the data rate.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPostDemodBW(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const PostDemodBW)
{
    ADI_ADF7023_STATUS_TYPE status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t PostDemodBW_l;

    PostDemodBW_l = PostDemodBW;

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &PostDemodBW_l, (uint8_t *)&status, 1, RADIO_CFG_4);

    if (!uhf_result)
        return uhf_result;
    else
        return ADI_ADF7023_ERR_POSTDEMODBW;
}

/*!
 * @brief Sets the ADF7023 packet preamble length
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   prelen      the desired preamble length in bytes.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the ADF7023 preamble length in bytes to the value specified in prelen.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPreambleLen(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const prelen)
{
    uint8_t prelen_l;
    ADI_ADF7023_STATUS_TYPE status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    prelen_l = prelen;

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &prelen_l, (uint8_t *)&status, 1, PREAMBLE_LEN_REG);

#ifdef ADI_DEBUG
    uint8_t prelen_test;
    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, &prelen_test, 1, PREAMBLE_LEN_REG);

    if (prelen_test == prelen)
        uhf_result = ADI_ADF7023_SUCCESS;
    else
        uhf_result = ADI_ADF7023_ERR_PREAMBLELEN;
#endif

    if (!uhf_result)
        return uhf_result;
    else
        return ADI_ADF7023_ERR_PREAMBLELEN;
}
/**
 * @brief Set the 7023 interrupt mask
 *
 * @param[in]   hDevice     Pointer to device handle.
 * @param[in]   Imask       Interrupt mask value
 *
 * @return      ADI_ADF7023_RESULT_TYPE
 *
 * This function is used to turn or off interrupts on the 7023.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetImask(ADI_ADF7023_DEV_HANDLE const hDevice, uint16_t const Imask)
{
    uint8_t uImask[2];
    ADI_ADF7023_STATUS_TYPE status;
    ADI_ADF7023_RESULT_TYPE result;

    //clear any pending interrupts
    AdfClearInterrupts(hDevice);
    uImask[0] = (uint8_t)(Imask & 0xFF);
    uImask[1] = (uint8_t)((Imask >> 8) & 0xFF);
    //write to imask and enable interrupts
    result = adi_ADF7023_BlockMemWrite(hDevice, uImask, (uint8_t *)&status, sizeof(uImask), INTERRUPT_MASK_0);

#ifdef ADI_DEBUG
    result = adi_ADF7023_BlockMemRead(hDevice, &uImask[0], 2, INTERRUPT_MASK_0);
    if ((uImask[0] == (Imask & 0xFF)) && (uImask[1] == ((Imask >> 8) & 0xFF)))
    {
        result = ADI_ADF7023_SUCCESS;
    }
    else
    {
        result = ADI_ADF7023_ERR_UNKNOWN;
    }
#endif

    return result;
}

/*!
 * @brief Sets the packet sync word length and format
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   SyncLength  the desired sync word length.
 * @param[in]   SyncWord    the sync word to be transmitted.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the sync word length and the sync word pattern. The packet handler supports sync words of up to
 * 24 bits in length. If the passed in sync word length is greater than 24, an error is returned.
 * The packet handler only supports sync words that are multiples of 8-bits in length. If a length that is
 * not a multiple of 8-bits is desired, the sync word should be padded with the preamble pattern (0101..)
 *
 */
//TODO handle the preamble padding in the function??
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetSyncWord(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const SyncLength, uint32_t const SyncWord)
{
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t SyncWord_l, SyncLength_l, status;
    uint32_t SyncWord_temp = SyncWord; //passed in values could be in registers. just make local copies

    if(SyncLength > 24 || SyncLength == 0)
        return ADI_ADF7023_ERR_INVALID_SYNC_WORD; //sync word is too long or too short

    SyncLength_l = SyncLength;
    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &SyncLength_l, &status, 1, SYNC_CONTROL);
#if 1
    if(!uhf_result && SyncLength > 16)
    {
        SyncWord_l = (uint8_t) (SyncWord_temp>>16) & 0xFF;
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &SyncWord_l, &status, 1, SYNC_BYTE_0);
    }
    if(!uhf_result && SyncLength > 8)
    {
        SyncWord_l = (uint8_t) (SyncWord_temp>>8) & 0xFF;
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &SyncWord_l, &status, 1, SYNC_BYTE_1);
    }
    if(!uhf_result)
    {
        SyncWord_l = (uint8_t) SyncWord_temp & 0xFF;
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &SyncWord_l, &status, 1, SYNC_BYTE_2);
    }
#else
    if(!uhf_result && SyncLength > 16)
    {
        SyncWord_l = (uint8_t) SyncWord_temp & 0xFF;
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &SyncWord_l, &status, 1, SYNC_BYTE_0);
        SyncWord_temp >>= 8;
    }
    if(!uhf_result && SyncLength > 8)
    {
        SyncWord_l = (uint8_t) SyncWord_temp & 0xFF;
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &SyncWord_l, &status, 1, SYNC_BYTE_1);
        SyncWord_temp >>= 8;
    }
    else if(!uhf_result)
    {
        SyncWord_l = (uint8_t) SyncWord_temp & 0xFF;
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &SyncWord_l, &status, 1, SYNC_BYTE_2);
    }
#endif

    return uhf_result;
}
/*!
 * @brief Sets the ADF7023 packet endian
 *
 * @param[in]   hDevice         handle to the ADF7023 device.
 * @param[in]   PacketEndian    to control the packet endian.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * PacketEndian = FALSE - LSB first
 * PacketEndian = TRUE  - MSB first.
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPacketEndian(ADI_ADF7023_DEV_HANDLE const hDevice, bool const PacketEndian)
{
    ADI_ADF7023_RESULT_TYPE result;
    uint8_t packet_length_control_reg, status;

    result = adi_ADF7023_BlockMemRead(hDevice, &packet_length_control_reg, 1, PACKET_LENGTH_CONTROL);
    if (PacketEndian)
        packet_length_control_reg |= PACKET_ENDIAN;  //set bit 7 of packet_length_control
    else
        packet_length_control_reg &= ~PACKET_ENDIAN;  //clear bit 7 of packet_length_control

    result = adi_ADF7023_BlockMemWrite(hDevice, &packet_length_control_reg, &status, 1, PACKET_LENGTH_CONTROL);

    return result;
}
/*!
 * @brief Used to turn automatic CRC checking, removal on or off
 *
 * @param[in]   hDevice     Pointer to device handle
 * @param[in]   CrcEn       Boolean parameter to turn the CRC engine on or off
 *
 * @return  ADI_ADF7023_RESULT_TYPE
 *
 * Used to turn the CRC engine on of off. When on, the radio automatically
 * adds a CRC to the end of a transmit packet, and automatically removes the
 * CRC field from a received packet.
 *
 * CrcEn = FALSE - Disables the CRC engine
 * CrcEn = TRUE  - Enable the CRC engine
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetClearAutoCRC(ADI_ADF7023_DEV_HANDLE const hDevice, bool const CrcEn)
{

    ADI_ADF7023_RESULT_TYPE result;
    uint8_t packet_length_control_reg, status;

    result = adi_ADF7023_BlockMemRead(hDevice, &packet_length_control_reg, 1, PACKET_LENGTH_CONTROL);
    if (CrcEn)
        packet_length_control_reg |= CRC_EN;  /* turn on CRC engine */
    else
        packet_length_control_reg &= ~CRC_EN;  /* turn off the CRC engine */

    result = adi_ADF7023_BlockMemWrite(hDevice, &packet_length_control_reg, &status, 1, PACKET_LENGTH_CONTROL);
    return result;
}
/*!
 * @brief Sets the radio to fixed or variable packet length mode
 *
 * @param[in]   hDevice         Pointer to device handle
 * @param[in]   FixedPktLen     Boolean value to set desired packet length mode
 *
 * @return      ADI_ADF7023_RESULT_TYPE
 *
 * Used to set whether the packet lengths are fixed or variable.
 *
 * FixedPktLen = FALSE - Radio operates in variable packet length mode with max packet length
 *                       defined by PACKET_LENGTH_MAX register
 *
 * FixedPktLen = TRUE  - Radio operates in fixed packet length mode with packet length
 *                       defined by PACKET_LENGTH_MAX register
 *
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetClearFixedPktLen(ADI_ADF7023_DEV_HANDLE const hDevice, bool const FixedPktLen)
{
    ADI_ADF7023_RESULT_TYPE result;
    uint8_t packet_length_control_reg, status;

    result = adi_ADF7023_BlockMemRead(hDevice, &packet_length_control_reg, 1, PACKET_LENGTH_CONTROL);
    if (FixedPktLen)
        packet_length_control_reg |= PACKET_LENGTH_FIXED;   /* set the radio to fixed packet length mode */
    else
        packet_length_control_reg &= ~PACKET_LENGTH_FIXED;  /* set the radio to variable packet length mode */

    result = adi_ADF7023_BlockMemWrite(hDevice, &packet_length_control_reg, &status, 1, PACKET_LENGTH_CONTROL);
    return result;
}
/*!
 * @brief Sets the maximum packet length of the radio
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   PktLength   desired maximum packet length .
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the maximum packet length for the radio. If variable packet length
 * mode is enabled, this sets the maximum length of a variable length packet.
 * If on the other hand, the radio is in fixed packet length mode, this
 * sets the size of a packet
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetMaxPacketLength(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const PktLength)
{
    ADI_ADF7023_RESULT_TYPE result;
    uint8_t status, PktLength_l = PktLength;

    result = adi_ADF7023_BlockMemWrite(hDevice, &PktLength_l, &status, 1, PACKET_LENGTH_MAX);

#ifdef ADI_DEBUG
    if (!result)
    {
        result = adi_ADF7023_BlockMemRead(hDevice, &status, 1, PACKET_LENGTH_MAX);
    }
    if (status != PktLength)
        result = ADI_ADF7023_ERR_UNKNOWN;
#endif

    return result;

}
/*!
 * @brief Sets the transmitter PA mode.
 *
 * @param[in]   hDevice    handle to the ADF7023 device.
 * @param[in]   PAmode     desired PAmode ( true or false)
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the desired PA mode. The value can be true for Differential PA
   and false for single ended PA.
 */


ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPAmode(ADI_ADF7023_DEV_HANDLE const hDevice, bool PAmode)
{

	uint8_t radio_cfg_8_reg, status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uhf_result = adi_ADF7023_BlockMemRead(hDevice, &radio_cfg_8_reg, 1, RADIO_CFG_8);
    // Clear PA mode bit Single-ended PA enabled
    radio_cfg_8_reg &= 0x7F;

    if(PAmode == true)
    //Differential PA enabled
    radio_cfg_8_reg |= (1<< 7);


    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &radio_cfg_8_reg, &status, 1, RADIO_CFG_8);

    return uhf_result;
}
/*!
 * @brief Sets the transmitter PA power level
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   TxPower    desired TX PA power level (value between 0 and 15 ).
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the desired PA power level. The value can range from 0 to 15, and corresponds
 * to one of 63 PA power levels. The precise power level is computed as
 * pa_level = 4 x TxPower + 3.
 *
 * @note For finer control over the power level, write the required value to the POWER_AMPLIFIER register
 * in the MCR (address 0x307).
 *
 * @sa adi_ADF7023_SetPARampRate().
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetTransmitterPowerLevel(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const TxPower)
{

    uint8_t radio_cfg_8_reg, status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    if(TxPower > 15)
        return ADI_ADF7023_ERR_INVALID_PA_LEVEL;

    uhf_result = adi_ADF7023_BlockMemRead(hDevice, &radio_cfg_8_reg, 1, RADIO_CFG_8);
    radio_cfg_8_reg &= ~PA_POWER; //clear the pa_power field in the register
    radio_cfg_8_reg |= (TxPower << 3);

    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &radio_cfg_8_reg, &status, 1, RADIO_CFG_8);

    return uhf_result;
}
/*!
 * @brief Sets the transmitter PA power level directly to MCR register
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   TxPower    desired TX PA power level (value between 0 and 63 ).
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the desired PA power level. The value can range from 0 to 63.
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetTransmitterPowerLevelMCR(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t MCRLevel)
{

    uint8_t status;

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &MCRLevel, &status, 1, POWER_AMPLIFIER);

	hDevice->mcrPALevel = MCRLevel;

    return uhf_result;
}
#ifdef PATCH_802_15_4D
/*!
 * @brief Sets the AGC Clock divide MCR register
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   TxPower    desired AGC Clock divide value to be set
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the AGC Clock divide MCR register.
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetAgcClockDivideMCR(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t clkDivideVal)
{
	uint8_t status;

	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &clkDivideVal, &status, 1, AGC_CLK_DIVIDE);

    return uhf_result;

}
#endif //PATCH_802_15_4D
/*!
 * @brief Sets the ramp rate for the power amplifier
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   PARamp     desired PA ramp rate (value between 1 and 7 ).
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the PA ramp rate. To ensure optimum performance, the PA ramp rate has
 * a minimum value based on the data rate and the transmitter PA level setting.
 * The formula to compute the minimum value is specified in the data sheet.
 *
 * @sa adi_ADF7023_SetTransmitterPowerLevel().
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPARampRate(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const PARamp)
{

    uint8_t radio_cfg_8_reg, status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    if(PARamp == 0 || PARamp > 7)
        return ADI_ADF7023_ERR_INVALID_PA_RAMP_RATE;

    uhf_result = adi_ADF7023_BlockMemRead(hDevice, &radio_cfg_8_reg, 1, RADIO_CFG_8);
    radio_cfg_8_reg &= ~PA_RAMP; //clear the PA Ramp field in the register
    radio_cfg_8_reg |= PARamp;

    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &radio_cfg_8_reg, &status, 1, RADIO_CFG_8);

    return uhf_result;
}

/*!
 * @brief Sets the current 7023 channel frequency
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   ChFreq     desired center frequency.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the frequency to the closest possible value.
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetChannelFrequency(ADI_ADF7023_DEV_HANDLE const hDevice, uint32_t const ChFreq)
{

    uint8_t  channel_freq_reg[3], status;
    uint32_t channel;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    int i = 0;

//    if(ChFreq > 960000000 || ChFreq < 431000000 || (ChFreq > 464000000 && ChFreq < 868000000)) /* Min-Max frequencies for the radio */
//        return ADI_ADF7023_ERR_CHANNEL_FREQ;


#ifdef WISUN_FREQ_ADJ
    channel = MakeChannel(ChFreq + 5000);
#else
    channel = MakeChannel(ChFreq);
#endif
    for(i = 0; i <= 2; i++)
        channel_freq_reg[i] = (uint8_t) (channel >> (8 * i) & 0xFF);

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &channel_freq_reg[0], &status, 3, CHANNEL_FREQ_0);

    if (!uhf_result)
    {
        hDevice->Frequency = MakeFreq(channel);
    }

	if( hDevice->FastTransitionsEnabled )
	{
		if(hDevice->synth_cal_pending != 0xFF)
		{
			hDevice->synth_cal_pending = 1;
		}
	}

    return uhf_result;
}
/*!
 * @brief Returns the current center frequency of the radio
 *
 * @param[in]   hDevice     Pointer to device handle
 *
 * @return      Returns the current carrier frequency of the radio
 *
 * @sa          adi_ADF7023_SetChannelFrequency().
 */
uint32_t adi_ADF7023_GetChannelFrequency(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    uint8_t ChFreq[3];
    uint32_t channel;

    adi_ADF7023_BlockMemRead(hDevice, ChFreq, 3, CHANNEL_FREQ_0);

    channel = ((uint32_t)ChFreq[2] << 16) | ((uint32_t)ChFreq[1] << 8) | (uint32_t)ChFreq[0];

    channel = MakeFreq(channel);

    return channel;

}
/*!
 * @brief Sets the radios address length
 *
 * @param[in]   hDevice         Pointer to device handle
 * @param[in]   AddrLength      Desired address length
 *
 * @return ADI_ADF7023_RESULT_TYPE
 *
 * Sets the address length of the radio. It is used by the radio
 * when performing address matching on an incoming packet.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetAddressLength(ADI_ADF7023_DEV_HANDLE hDevice, uint8_t const AddrLength)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t status, AddrLength_l = AddrLength;

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &AddrLength_l, &status, 1, ADDRESS_LENGTH);

    return uhf_result;
}
/*!
 * @brief Returns the current radio address length
 *
 * @param[in]   hDevice     Pointer to device handle
 *
 * @return uint8_t - The current radio address length
 *
 * Reads and returns the current address length of the radio
 *
 */
uint8_t adi_ADF7023_GetAddressLength(ADI_ADF7023_DEV_HANDLE hDevice)
{
    uint8_t AddrLength = 0;

    adi_ADF7023_BlockMemRead(hDevice, &AddrLength, 1, ADDRESS_LENGTH);

    return AddrLength;
}
/*!
 * @brief Sets radio short address (16-bits)
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   pShortAddr  pointer to short address and address mask.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the 16-bit short address in 7023 BBRAM. The address mask can be used to determine if
 * the entire address or parts of the address will be used during the address match on RX.
 * For instance, an address mask of 0xffff indicates that all 16-bits on the received packet
 * address have to match the short address stored in 7023 memory. A mask of 0xff00 will limit
 * address checking to only the first byte of the received packet address against the short addrress
 * in memory.
 *
 * @note This function can only set one address for the radio. The 7023 supports multiple address
 *       checking, with different associated address masks. To set other addresses and masks, use
 *       the adi_ADF7023_BlockMemWrite() function to directly write the addresses out to 7023 memory
 *
 * @sa adi_ADF7023_BlockMemWrite(), adi_ADF7023_SetExtendedAddress().
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetShortAddr(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_SHORT_ADDR_TYPE* pShortAddr)
{
    ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;
    uint8_t AddrLoad[4], status;

    result = adi_ADF7023_SetAddressLength(hDevice, 2);  /* short addresses are 16-bit, so set the address length in the radio */

    AddrLoad[0] = (uint8_t)(pShortAddr->ShortAddr >> 8) & 0xFF;
    AddrLoad[1] = (uint8_t)(pShortAddr->ShortAddrMask >> 8) & 0xFF;
    AddrLoad[2] = (uint8_t)(pShortAddr->ShortAddr & 0xFF);
    AddrLoad[3] = (uint8_t)(pShortAddr->ShortAddrMask & 0xFF);
    result = adi_ADF7023_BlockMemWrite(hDevice, &AddrLoad[0], &status, sizeof(AddrLoad), ADDRESS_MATCH_BYTE_0);

#ifdef ADI_DEBUG
    uint8_t debug_AddrLoad[4], i = 0;
    result = adi_ADF7023_BlockMemRead(hDevice, debug_AddrLoad, sizeof(debug_AddrLoad), ADDRESS_MATCH_BYTE_0);
    for(i = 0; i < 4; i++)
    {
        if (debug_AddrLoad[i] == AddrLoad[i])
            continue;
        else
            result = ADI_ADF7023_ERR_UNKNOWN;
    }
#endif

    return result;

}
/*!
 * @brief Enables/Disables auto tx to rx turnaround
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in] 	value 		Zero 						- Disable
 * 							Non_Zero (positive values) 	- Enable
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * To achieve better TX to RX turnaround time the auto TX to RX transition can be enabled.
 * This API enables or disables this feature based on the value passed.By enabling this
 * feature the ADF7023 transceiver will switch the PHY state to PHY_RX on receiving the TX_EOF
 * interrupt.
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetTxToRxAutoTurnAround(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t value)
{
    ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;
    uint8_t AutoTurnAround = 0, status;

    if(value)
    {
    	value = (1 << AUTO_TX_TO_RX_ENABLE);
    }
    else
    {
    	value = (0 << AUTO_TX_TO_RX_ENABLE);
    }


    result = adi_ADF7023_BlockMemRead(hDevice, &AutoTurnAround, 1, MODE_CONTROL);

    AutoTurnAround &= ~(1 << AUTO_TX_TO_RX_ENABLE);
	AutoTurnAround |= value;

    if(!result)
    	result = adi_ADF7023_BlockMemWrite(hDevice, &AutoTurnAround, &status, 1, MODE_CONTROL);

    return result;

}


/*!
 * @brief Reads the current short address of the radio
 *
 * @param[in]    hDevice        Pointer to device handle
 * @param[out]   pShortAddr     Pointer to short address type
 *
 * @return       ADI_ADF7023_RESULT_TYPE
 *
 * Reads and returns the current short address of the radio.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetShortAddr(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_SHORT_ADDR_TYPE* const pShortAddr)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t AddrRead[4];

    uhf_result = adi_ADF7023_BlockMemRead(hDevice, AddrRead, 4, ADDRESS_MATCH_BYTE_0);

    pShortAddr->ShortAddr = ((uint16_t)AddrRead[0] << 8) | (uint16_t)AddrRead[2];
    pShortAddr->ShortAddrMask = ((uint16_t)AddrRead[1] << 8) | (uint16_t)AddrRead[3];

    return uhf_result;

}
/*!
 * @brief Set the address match offset
 *
 * @param[in]   hDevice         Pointer to the device handle.
 * @param[in]   AddrOffset      Value of the address match offset
 *
 * @return ADI_ADF7023_RESULT_TYPE
 *
 * This function is used to set the address match offset. This is the offset
 * from the start of received packet that is checked for an address match.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetAddressMatchOffset(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const AddrOffset)
{

	ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;
    uint8_t status, AddrOffset_l = AddrOffset;

    result = adi_ADF7023_BlockMemWrite(hDevice, &AddrOffset_l, &status, 1, ADDRESS_MATCH_OFFSET);

#ifdef ADI_DEBUG
    result = adi_ADF7023_BlockMemRead(hDevice, &status, 1, ADDRESS_MATCH_OFFSET);
    if (status != AddrOffset)
        result = ADI_ADF7023_ERR_UNKNOWN;
#endif
    return result;
}
/*!
 * @brief Used to set the TX base address in packet RAM.
 *
 * @param[in]   hDevice     Pointer to device handle.
 * @param[in]   TxBase      The address from the start of packet RAM for a TX packet
 *
 * @return ADI_ADF7023_RESULT_TYPE
 *
 * This function sets the base address in packet RAM for a packet to be transmitted. When
 * the phy is set to PHY_TX mode, the radio starts transmitting from the TX base address.
 * When loading a TX packet into packet RAM, this is the address that the packet needs
 * to be written to.
 *
 * @sa adi_ADF7023_SetRxBaseAddr().
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetTxBaseAddr(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const TxBase)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t TxBase_l = TxBase, status;

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &TxBase_l, &status, 1, TX_BASE_ADR);

    if(!uhf_result)
        hDevice->TxPktBase = TxBase;

    return uhf_result;
}

/*!
 * @brief Used to set the TX base address in packet RAM.
 *
 * @param[in]   hDevice     Pointer to device handle.
 * @param[in]   RxBase      The address from the start of packet RAM for a RX packet
 *
 * @return ADI_ADF7023_RESULT_TYPE
 *
 * When a valid packet is received, it is written to the RX base address.
 *
 * @sa adi_ADF7023_SetTxBaseAddr().
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetRxBaseAddr(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const RxBase)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t RxBase_l = RxBase, status;

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &RxBase_l, &status, 1, RX_BASE_ADR);

    if(!uhf_result)
        hDevice->RxPktBase = RxBase;

    return uhf_result;
}

/*!
 * @brief Sets the 64-bit radio address
 *
 * @param[in]       hDevice         Pointer to the device handle.
 * @param[in]       ExAddr          Pointer to the extended address.
 * @param[in]       ExAddrMask      Pointer to the address mask.
 *
 * @return          ADI_ADF7023_RESULT_TYPE
 *
 * Set the 64-bit address of the radio. This address is used during address
 * checking on incoming packets. This function also sets the ADDRESS_LENGTH
 * register on the 7023.
 *
 * @note - The ADF7023 supports multiple addresses. This functions only sets the
 *         address at the first address match offset. For additional addresses
 *         for using in address checking, use the adi_ADF7023_BlockMemWrite function.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetExtendedAddress(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t* const ExAddr, uint8_t* const ExAddrMask)
{

    ADI_ADF7023_RESULT_TYPE uhf_result;
    uint8_t ExAddr_buffer[17], status;
    uint8_t *ExAddr_l = ExAddr, *ExAddrMask_l = ExAddrMask;
    int i = 0;

    ExAddr_buffer[0] = 8;  /* address length */

    for(i = 1; i <= 16; i++)
    {
        if(i & 0x1)
            ExAddr_buffer[i] = *ExAddr_l++;
        else
            ExAddr_buffer[i] = *ExAddrMask_l++;
    }

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, ExAddr_buffer, &status, 17, ADDRESS_LENGTH);

    return uhf_result;
}


/* RADIO_CFG_9 Functions */

/*!
 * @brief Sets the 7023 IF filter bandwidth
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   ifbw        desired IF filter bandwidth.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the IF filter bandwidth. The valid values are 0x0 (100 kHz),
 * 0x1 (150 kHz), 0x2 (200 kHz), 0x3 (300 kHz).
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetIFFilterBandwidth(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const ifbw)
{

    uint8_t tfsize = 1, radio_cfg_9_reg, status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    if(ifbw > 3)
        return ADI_ADF7023_ERR_INVALID_IFBW;

    uhf_result = adi_ADF7023_BlockMemRead(hDevice, &radio_cfg_9_reg, tfsize, RADIO_CFG_9);

    radio_cfg_9_reg &= ~IFBW_FIELD;
    radio_cfg_9_reg |= (ifbw << 6);

    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &radio_cfg_9_reg, &status, tfsize, RADIO_CFG_9);

    return uhf_result;

}
/*!
 * @brief This routine sets the modulation scheme
 *
 * @param[in]   hDevice     Pointer to the device handle.
 * @param[in]   mod         desired modulation scheme.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Input argument mod value: 0 (for 2 level FSK), 1 (for 2 level GFSK), 2 (for OOK) and 3 (carrier only)
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetModulationScheme(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_MOD_TYPE const mod)
{

    uint8_t tfsize = 1, radio_cfg_9_reg, status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    if(mod > MOD_CARRIER)
        return ADI_ADF7023_ERR_INVALID_MOD_SCHEME;

    uhf_result = adi_ADF7023_BlockMemRead(hDevice, &radio_cfg_9_reg, tfsize, RADIO_CFG_9);

    radio_cfg_9_reg &= ~MOD_FIELD;
    radio_cfg_9_reg |= (mod << 3);

    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &radio_cfg_9_reg, &status, tfsize, RADIO_CFG_9);

    if (!uhf_result) {
        hDevice->ModScheme = mod;
    }

    return uhf_result;

}




/**
 * @brief Returns the current radio modulation scheme
 *
 * @param[in]   hDevice     Pointer to the device handle.
 *
 * @return      ADI_ADF7023_MOD_TYPE
 *
 * Reads and returns the current modulation scheme of the radio.
 *
 */
ADI_ADF7023_MOD_TYPE adi_ADF7023_GetModulationScheme(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    ADI_ADF7023_MOD_TYPE mod_scheme;
    uint8_t radio_cfg_9_reg;

    adi_ADF7023_BlockMemRead(hDevice, &radio_cfg_9_reg, 1, RADIO_CFG_9);
    radio_cfg_9_reg &= MOD_FIELD;

    mod_scheme = (ADI_ADF7023_MOD_TYPE)(radio_cfg_9_reg >> 3);

    return mod_scheme;

}
/* this routine sets the demodulation scheme */
/* input argument mod value: 0 (for FSK), 1 (for GFSK), 2 (OOK) and 3 (reserved) */

/*!
 * @brief This routine sets the demodulation scheme
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[in]   demod       desired demodulation scheme.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Input argument demod value: 0 (for 2 level FSK), 1 (for 2 level GFSK), 2 (for OOK) and 3 (carrier only)
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetDemodulationScheme(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_DEMOD_TYPE const demod)
{

    uint8_t tfsize = 1, radio_cfg_9_reg, status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    if(demod > DEMOD_2FSK_GFSK_MSK_GMSK)
        return ADI_ADF7023_ERR_INVALID_DEMOD_SCHEME;

    uhf_result = adi_ADF7023_BlockMemRead(hDevice, &radio_cfg_9_reg, tfsize, RADIO_CFG_9);

    radio_cfg_9_reg &= ~DEMOD_FIELD;
    radio_cfg_9_reg |= demod;

    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &radio_cfg_9_reg, &status, tfsize, RADIO_CFG_9);

    if(!uhf_result)
        hDevice->DemodScheme = demod;

    return uhf_result;

}
/*!
 * @brief Returns the current demodulation scheme
 *
 * @param[in]    hDevice    Pointer to the device handle.
 *
 * @return       ADI_ADF7023_DEMOD_TYPE
 *
 * Reads and returns the current demodulation scheme.
 *
 */
ADI_ADF7023_DEMOD_TYPE adi_ADF7023_GetDemodulationScheme(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    uint8_t radio_cfg_9_reg;
    ADI_ADF7023_DEMOD_TYPE demod_scheme;

    adi_ADF7023_BlockMemRead(hDevice, &radio_cfg_9_reg, 1, RADIO_CFG_9);

    demod_scheme = (ADI_ADF7023_DEMOD_TYPE)(radio_cfg_9_reg & DEMOD_FIELD);

    return demod_scheme;
}

ADI_ADF7023_SPI_STATE adi_ADF7023_GetSpiState(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    return hDevice->SpiState;
}
/*!
 * @brief
 *
 * @param hDevice
 * @param AfcMode
 * @param AfcLockMode
 * @param PullIn
 *
 * @return
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_ConfigureAFC(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t const AfcMode, uint8_t const AfcLockMode, uint8_t const PullIn)
{

    ADI_ADF7023_RESULT_TYPE uhf_result;
    uint8_t AfcConfig_r, status, PullIn_l = PullIn;

    AfcConfig_r = 0x0 | (1<<4) | (AfcMode<<2) | (AfcLockMode);
    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &AfcConfig_r, &status, 1, RADIO_CFG_10);

#ifdef ADI_DEBUG
    uhf_result = adi_ADF7023_BlockMemRead(hDevice, &status, 1, RADIO_CFG_10);
    if (status != AfcConfig_r)
        while(1);
#endif
    if (!uhf_result)
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &PullIn_l, &status, 1, MAX_AFC_RANGE);

#ifdef ADI_DEBUG
    uhf_result = adi_ADF7023_BlockMemRead(hDevice, &status, 1, MAX_AFC_RANGE);
    if (status != PullIn)
        while(1);
#endif
    return uhf_result;

}
//this function handles the current state of the radio and makes sure
//that it is in PHY_OFF
static ADI_ADF7023_RESULT_TYPE UHF_FirstConnect(ADI_ADF7023_DEV_HANDLE const hDevice)
{
    volatile ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    //volatile uint64_t timeout = 2000;
    ADI_ADF7023_STATUS_TYPE status;


    //hopefully it's in PHY_SLEEP at this point
    uhf_result = PhyWakeUp(hDevice);

//    do
//    {
//        timeout--;
//    }while(0 < timeout);

    //issue a hard reset
    uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_HW_RESET);
    //hard reset -> PHY_SLEEP timing is still TBD
    //TODO replace in future with a more accurate value
    //No point as it is optimized.
//    timeout = 2000;
//    do
//    {
//        timeout--;
//    }while(0 < timeout);

    do
    {
        uhf_result = ReadStatusReg(hDevice, &status);
    }while(status.fw_state != PHY_OFF && !uhf_result);             //wait for PHY_OFF state


    if(!uhf_result)
        uhf_result = adi_ADF7023_SetPhyOff(hDevice);

    return uhf_result;
}
//initializes the BBRAM with a set of default values
static void SetBBRAMDefault(ADI_ADF7023_BBRAM_TYPE *pBBRAM)
{

    uint32_t ulChannelFreq     = MakeChannel(UHF_DEFAULT_RADIO_FREQ);
    uint32_t ulDataRate        = UHF_DEFAULT_DATA_RATE * 100;
    uint32_t ulFreqDev         = UHF_FREQ_DEVIATION;

    // Configure which events will be flagged to the Cortex via interrupt
    pBBRAM->interrupt_mask_0_r                   = 0x0;//interrupt_mask_0_tx_eof          | // Packet transmitted              (PHY_RX -> PHY_ON)
    // interrupt_mask_0_address_match   | // Valid Packet for us received!   (PHY_RX -> PHY_ON)
    // interrupt_mask_0_crc_correct     | // Invalid packet received!        (PHY_RX          )
    // interrupt_mask_0_syncbyte_detect | // Sync byte detected              (PHY_RX          )
    // interrupt_mask_0_premable_detect ; // Preamble detecte d              (PHY_RX          )

    pBBRAM->interrupt_mask_1_r                   = 0x0;

    // These can be initialised to zero
    // Internal 16-bit count of the number of wake ups (wuc timeouts) the device has gone through
    // We don't have the wake-up timer connected so not used.
    pBBRAM->number_of_wakeups_0_r                = 0x0;
    pBBRAM->number_of_wakeups_1_r                = 0x0;

    // This is the threshold for the number of wakeups
    // (wuc timeouts). It is a 16-bit count threshold that is compared
    // against the number_of_wakeups. When this threshold is exceeded
    // the device wakes up into the state PHY_OFF and optionally
    // generates interrupt_num_wakeups.
    // We don't have the wake-up timer connected so not used.
    pBBRAM->number_of_wakeups_irq_threshold_0_r  = 0x0;
    pBBRAM->number_of_wakeups_irq_threshold_1_r  = 0x0;

    // When in Smart Wake mode this is the time period that the receiver
    // is awake for. Units of the timer period are determined by
    // parmtime_divider.
    // We don't have the wake-up timer connected so not used.
    pBBRAM->max_wakeup_2_sync_time_r             = 0x0;

    // Units of time used to define max_wakeup_2_sync_time time
    // period. A value of 0x33h will give clock of 995.7Hz or a period of
    // 1.004ms
    // We don't have the wake-up timer connected so not used.
    pBBRAM->parmtime_divider_r                   = 0x33;  // 995.7Hz

    // This sets the RSSI threshold when in Smart Wake Mode with RSSI
    // detection enabled.
    // Threshold (dBm) = listen_rssi_thresh - 119
    pBBRAM->listen_rssi_thresh_r                 = 0x31; // -70dBm

    // The RF channel frequency bits [7:0] in Hz is set according to:
    // Frequency(Hz) = FPFD x channel_Freq[23:0] /2^16
    // where FPFD is the PFD frequency and is equal to 26MHz
    //

    //ulChannelFreq = (uint32_t)(ulChannelFreqHz * FREQ_CNVRT_VAL * dCrystalPE);
    pBBRAM->channel_freq_0_r                     = (ulChannelFreq >> 0) & 0xFF;
    pBBRAM->channel_freq_1_r                     = (ulChannelFreq >> 8) & 0xFF;
    pBBRAM->channel_freq_2_r                     = (ulChannelFreq >> 16)& 0xFF;

    // The datarate in bps is set according to:
    // DataRate (bps) = data_rate[11:0] x 100
    pBBRAM->radio_cfg_0_r = (uint8_t)((ulDataRate / 100) & 0xFF);
    pBBRAM->radio_cfg_1_r &= 0xF0; // Bottom nibble of radio_cfg_1_r is used for data_rate[11:8]
    pBBRAM->radio_cfg_1_r |= (uint8_t)(((ulDataRate / 100) & 0xF00) >> 8);;

    // The binary level FSK frequency deviation in Hz (defined as
    // frequency difference between carrier frequency and 1/0 tones) is
    // set according to:
    // Frequency Deviation (Hz) = Freq_Deviation[11:0] x100

    // Frequency Deviation (50 kHz)  ((0x1F4 = 500d) * 100)
    pBBRAM->radio_cfg_2_r = (uint8_t)((ulFreqDev / 100) & 0xFF);
    pBBRAM->radio_cfg_1_r &= 0x0F; // Upper nibble of radio_cfg_1_r is used for Frequency Deviation[11:8]
    pBBRAM->radio_cfg_1_r |= (uint8_t)(((ulFreqDev / 100) & 0xF00) >> 4);;


    // Set the disscriminator bandwidth and post-demodulator bandwidth based on data rate
    // if 100kbps data rate
    if(ulDataRate == 100000)
    {

        pBBRAM->radio_cfg_3_r =0x20;
        pBBRAM->radio_cfg_4_r =0x26;
    }
    // Setting for 50kbps data rate
    else
    {
        pBBRAM->radio_cfg_3_r =0x82;
        pBBRAM->radio_cfg_4_r =0x13;
    }

    // When AFC is enabled, the discriminator bandwidth will limit the AFC pull
    //  in range to the bandwidth of the discriminator. In this case,
    // the discriminator bandwidth can be increased to allow a wider AFC pull in range.
    // TO_DO!!! 0x20 value is good for 50 kHz Deviation, see new doc on the formula
    //pBBRAM->radio_cfg_3_r = 0x20; // 0x41 old discriminator_bw (dependent

    // The bandwidth of the post-demodulator bandwidth is given by:
    // For optimum receiver sensitivity the post demodulator bandwidth
    // should be set at 0.8x times the data rate.
    //pBBRAM->radio_cfg_4_r = (uint32_t)((0.75 * ulDataRate)/2000); // post_demod_bw , 0.8 earlier
    // if (pBBRAM->radio_cfg_4_r == 0x0)
    //pBBRAM->radio_cfg_4_r = 0x1;

    // The discrim_phase parameter allows the centre frequency
    // of the discriminator to be adjusted. It should be set according to:
    // discrim_phase[9:0]=remainder[(discrim_bw×IFfrequency)/(13×?10?^6 )  ]×1024
    // TO_DO!!! 0x20 value is good for 50 kHz Deviation, see new doc on the formula
    // Latest data sheet (rev 0) says set this to 0.. so....
    pBBRAM->radio_cfg_5_r = 0x0; // t4_phase_adjust

    // Disriminator phase is described in the doc
    // Synth_LUT_Config_0 used with custom LUT tables

    pBBRAM->radio_cfg_6_r = 0x0; //t4_phase_adjust_msb +Synth_LUT_Config_0

    // agc_mode, Synth_Lut_control, Synth_LUT_config_1
    pBBRAM->radio_cfg_7_r =  radio_cfg_7_agc_lock_mode_lock_after_preamble;

    // PA settings
    // TODO unsure about the Ramp Rate setting here. Rev 0 datasheet has a new formula
    pBBRAM->radio_cfg_8_r = radio_cfg_8_pa_single_diff_sel_single_ended |
	radio_cfg_8_pa_power_setting_63             |
	    radio_cfg_8_pa_ramp_32;

    // Modulation/Demodulation Scheme
    // FSK/FSK/150 Hz receiver filter bandwidth
    pBBRAM->radio_cfg_9_r = radio_cfg_9_demod_scheme_2FSK_GFSK_MSK_GMSK    |
	radio_cfg_9_mod_scheme_2_level_FSK  |
	    radio_cfg_9_ifbw_150kHz;
    // AFC
    // TODO rev 0 of data sheet says set upper bits to 0, and set bits 3:2 to 2.
    // THIS DID NOT WORK!
    // this should be 0x9. Afc disabled and scheme set to to 2.
    pBBRAM->radio_cfg_10_r = 0x0               |  // TO_DO Why inverted
	(0x2<<2)                 |
	    0x3; //enable AFC

    // Sets the AFC PI controller proportional gain. Recommended value is 0x4
    // Sets the AFC PI controller integral gain. Recommended value is 0x7 // TO_DO Why 10 here
    pBBRAM->radio_cfg_11_r   = radio_cfg_11_afc_kp_2_power_3 | radio_cfg_11_afc_ki_2_power_7;

    // image_reject_cal_phase
    pBBRAM->ir_cal_phase_r   = 0x0;
    // image_reject_cal_amplitude
    pBBRAM->ir_cal_amplitude_r   = 0x0;

    pBBRAM->mode_control_r = mode_control_swm_en_disabled   		 |
			     mode_control_bb_cal_enabled    			         |   // Gets cleared???
	    		 mode_control_swm_rssi_qual_disabled        	     |
			     mode_control_tx_auto_turnaround_disabled   	     |
		    	 mode_control_rx_auto_turnaround_disabled   	     |
			     mode_control_custom_trx_synth_lock_time_en_disabled |
			     mode_control_ext_lna_en_disabled                    |
			     mode_control_ext_pa_en_disabled                     ;

    // Number of preamble bit errors in 24 bit window (RX)
    // value of <4 bit errors is recommended to prevent false preamble detections
    pBBRAM->preamble_match_r = preamble_match_0_in_24_win;

    // Symbol Paramters
    pBBRAM->symbol_mode_r = symbol_mode_symbol_length_8_bit          |
                            symbol_mode_data_whitening_disabled      |
                            symbol_mode_eight_ten_encoding_disabled  |
                            symbol_mode_prog_crc_en_disabled         | // Default CRC selected (x16 + x15 + x2 + 1)
                            symbol_mode_manchester_enc_disabled ;

    // Length of (TX) preamble in bytes. Example a value of decimal 3
    // results in a preamble of 24 bits.
    pBBRAM->preamble_len_r = 0x8;

    // crc_poly[15:0], which sets the CRC polynomial. ( we have selected the default one (x16 + x15 + x2 + 1) with symbol_mode_prog_crc_en_disabled aove )
    pBBRAM->CRC_poly_low_r   = 0x80;
    pBBRAM->CRC_poly_high_r  = 0x05;

    // Sets the sync word error tolerance in bits.
    // Sets the sync word length in bits. 24 bits is the maximum.
    // Note that the sync word matching length can be any value up to 24 bits
    // , but the transmitted sync word pattern is a multiple of 8 bits.
    // Hence, for non-byte-length sync words, the transmitted sync pattern
    // should be filled out with the preamble pattern.
    pBBRAM->sync_control_r = sync_control_sync_error_tol_0_errors_allowed |
	sync_control_sync_word_length_16; //sync_control_sync_word_length_8;
    // The sync word pattern is transmitted most significant bit first
    // starting with sync_byte[7:0].
    // For non-byte length sync words the reminder of the least
    // significant byte should be stuffed with preamble.
    // If sync_word_length length is >16 bits then sync_byte_0,
    // sync_byte_1 and sync_byte_2 are all transmitted for a total of 24 bits.
    // If sync_word_length is between 8 and 15 then sync_byte_1 and sync_byte_2
    // are transmitted.
    // If sync_word_length is between 1 and 7 then sync_byte_2 is
    // transmitted for a total of 8 bits.
    // If the sync word length is 0 then no sync bytes are transmitted.
    pBBRAM->sync_byte_0_r  = 0x0;
    pBBRAM->sync_byte_1_r  = 0x0;
    pBBRAM->sync_byte_2_r  = 0xA7; //802.15.4 sync word


    // Address in Packet RAM of transmit packet. This address
    // indicates to the comms processor the location of the
    // first byte of the transmit packet
    pBBRAM->tx_base_adr_r            = PKT_RAM_TXBASE;    // 36   TX Base Address

    // Address in Packet RAM of receive packet. The communications
    // processor will write any qualified received packet to Packet RAM,
    // starting at this memory location.
    pBBRAM->rx_base_adr_r            = PKT_RAM_RXBASE;

    // Various packet options
    pBBRAM->packet_length_control_r  =  packet_length_control_data_byte_lsb         | // LSB
                                        packet_length_control_packet_len_fixed      | // Fixed packet length
                                        packet_length_control_crc_en_yes            | // CRC Enabled
                                        packet_length_control_sport_mode_disabled   | // No sport
                                        packet_length_control_length_offset_minus0;   // For variable length packets where the first byte (length) needs to be adjusted

    // If variable packet length mode is used (packet_len_control = 0),
    // then packet_length_max sets the maximum packet length in bytes.
    // If fixed packet length mode is used (packet_len_control = 1),
    // then packet_length_max sets the length of the fixed packet in bytes.
    // Note that the packet length is defined as the number of bytes from
    // the end of the sync word to the start of the CRC. It also does not
    // include the length_offset value
    pBBRAM->packet_length_max_r      = MAX_PKT_LEN;

    // Set to 0x00
    pBBRAM->Static_Reg_fix_r         = 0x0;

    // Location of first byte of address information in packet RAM (relative to rx_base)
    pBBRAM->Address_Match_Offset_r   = 1;
    // Number of bytes in each address field (NADR)
    pBBRAM->Address_Length_r         = 0x2;               // Using a two byte address (0 for addr matching disabled)
    // Address match bytes and masks
    // Two byte addess - 0x01FF
    pBBRAM->Address_match_byte_0     = 0x01;
    pBBRAM->Address_mask_byte_0      = 0xFF;
    pBBRAM->Address_match_byte_1     = 0xFF;
    pBBRAM->Address_mask_byte_1      = 0xFF;

    // Two byte addess - 0x0F0F
    pBBRAM->Address_match_byte_2     = 0x0F;
    pBBRAM->Address_mask_byte_2      = 0xFF;
    pBBRAM->Address_match_byte_3     = 0x0F;
    pBBRAM->Address_mask_byte_3      = 0xFF;

    // Two byte addess - 0x5061
    pBBRAM->Address_match_byte_4     = 0x50;
    pBBRAM->Address_mask_byte_4      = 0xFF;
    pBBRAM->Address_match_byte_5     = 0x61;
    pBBRAM->Address_mask_byte_5      = 0xFF;

    pBBRAM->Address_match_byte_6     = 0x0;
    pBBRAM->Address_mask_byte_6      = 0xa7;	//0x0 new RSSI setting for new silicon.
    pBBRAM->Address_match_byte_7     = 0x0;
    pBBRAM->Address_mask_byte_7      = 0x0;
    pBBRAM->Address_match_byte_8     = 0x0;
    pBBRAM->Address_mask_byte_8      = 0x0;
    pBBRAM->r_13D                    = 0x0;                                // 0x0 to end the address sequence

    // Allows the use of a custom synthesizer lock time counter in receive
    // mode in conjunction with the custom_trx_synth_lock_time_en setting
    // in the mode_control register. Applies after VCO calibration is
    // complete.  2us step.
    pBBRAM->RX_synth_lock_time       = 0x0;
    // Allows the use of a custom synthesizer lock time counter in transmit
    // mode in conjunction with the custom_trx_synth_lock_time_en setting
    // in the mode_control register. Applies after VCO calibration is complete.
    // 2us step.
    pBBRAM->TX_synth_lock_time       = 0x0;
}
static ADI_ADF7023_RESULT_TYPE PhyWakeUp(ADI_ADF7023_DEV_HANDLE hDevice)
{

    volatile uint32_t i = 0;
    uint16_t value = 0;

    disable_CS3();
    /* FIXME use Murali's ADC delay function... not int counts!
    do we still need second try with new silicon?
    perhaps just wiggle select twice and then watch for miso? */
    /* assert ADF7023 select and watch for wakeup on MISO */
    if(adi_gpio_SetLow (ADI_GPIO_PORT2, ADI_GPIO_PIN_7) != ADI_GPIO_SUCCESS)
    {
        adi_initpinmux();
        return ADI_ADF7023_ERR_UNKNOWN;
    }
    do
    {
        adi_gpio_GetData(ADI_GPIO_PORT1, ADI_GPIO_PIN_4, &value);
        if (value)
            break;
        i++;
    } while(i < 5000);

    if(adi_gpio_SetHigh(ADI_GPIO_PORT2, ADI_GPIO_PIN_7) != ADI_GPIO_SUCCESS)
    {
        adi_initpinmux();
        return ADI_ADF7023_ERR_UNKNOWN;
    }

    /* if at first you don't succeed... */
    if (i > 4999)
	{
        i = 0;
		if(adi_gpio_SetLow (ADI_GPIO_PORT2, ADI_GPIO_PIN_7) != ADI_GPIO_SUCCESS)
        {
            adi_initpinmux();
			return ADI_ADF7023_ERR_UNKNOWN;
        }
		do
		{
			adi_gpio_GetData(ADI_GPIO_PORT1, ADI_GPIO_PIN_4, &value);
            if (value)
                break;
			i++;
		} while(i < 5000);

		if(adi_gpio_SetHigh(ADI_GPIO_PORT2, ADI_GPIO_PIN_7) != ADI_GPIO_SUCCESS)
        {
            adi_initpinmux();
			return ADI_ADF7023_ERR_UNKNOWN;
        }
	}

    // if second try fails, return error
    if (i > 4999)
    {
        adi_initpinmux();
        return ADI_ADF7023_ERR_UNKNOWN;
    }
    adi_initpinmux();
    return ADI_ADF7023_SUCCESS;
}

static ADI_ADF7023_RESULT_TYPE ReadStatusReg(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_STATUS_TYPE* pStatus)
{

    uint32_t tfsize = 2 * sizeof(uint8_t);
    volatile ADI_ADF7023_RESULT_TYPE spi_result;

    //will just send SPI_NOPs out twice WITHOUT DEASSERTING CS. This is important
    pStatus->spi_ready = 0;
    pStatus->cmd_ready = 0;

    adi_int_EnterCriticalRegion();

    while(ADI_ADF7023_SPI_IDLE != hDevice->SpiState)
    {
        adi_int_ExitCriticalRegion();

        /* we have to block in this function, no non blocking mode */
        adi_int_EnterCriticalRegion();
    }

    if(ResetSpiBuffer(hDevice) != ADI_ADF7023_SUCCESS)
    {
        adi_int_ExitCriticalRegion();
        return ADI_ADF7023_ERR_SPI_BUSY;
    }
    hDevice->IssuedCmd = ADI_ADF7023_CMD_SPI_NOP;

    hDevice->SpiBuffer.pPrologue    = NULL;
    hDevice->SpiBuffer.DataSize     = tfsize;
    hDevice->SpiBuffer.pTxData      = (uint8_t *)&hDevice->IssuedCmd;
    hDevice->SpiBuffer.pRxData      = (uint8_t *)pStatus;
    hDevice->SpiBuffer.PrologueSize = 0;
    hDevice->SpiBuffer.bTxIncrement = false;   // stationary transmit data pointer
    hDevice->SpiBuffer.bRxIncrement = false;  // stationary receive data pointer

    adi_spi_SetChipSelect(hDevice->pSpi, ADI_SPI_CS3);

    spi_result = adi_spi_MasterRadioTx(hDevice->pSpi, &hDevice->SpiBuffer);

    if(ADI_ADF7023_SUCCESS == spi_result)
    {
        hDevice->SpiState = ADI_ADF7023_SPI_CMD; /* if buffer isn't submitted, do not
        change spi state, else will never go back
        to idle */
    }

    adi_int_ExitCriticalRegion();

    if(ADI_ADF7023_SUCCESS != spi_result)
    {
        hDevice->IssuedCmd = ADI_ADF7023_CMD_NULL;
        return ADI_ADF7023_ERR_BLOCKWRITE;
    }
    else
    {
        while(ADI_ADF7023_SPI_IDLE != hDevice->SpiState);
        hDevice->IssuedCmd = ADI_ADF7023_CMD_NULL;
        return ADI_ADF7023_SUCCESS;
    }
}
/**
 * @brief   Internal function to setup GPIO pins
 *
 * @param   hDevice
 *
 * @return  ADI_ADF7023_RESULT_TYPE */

static ADI_ADF7023_RESULT_TYPE SetupIrqFlag(IRQn_Type Flag)
{
    /* init the GPIO service */
#ifdef ADCM3029_EZKIT_USED
    #define GPIO_PORT   ADI_GPIO_PORT2
    #define GPIO_PIN    ADI_GPIO_PIN_3
#else
    #define GPIO_PORT   ADI_GPIO_PORT0
    #define GPIO_PIN    ADI_GPIO_PIN_15
#endif

    adi_gpio_InputEnable(GPIO_PORT, GPIO_PIN, true);

    adi_gpio_SetGroupInterruptPolarity(GPIO_PORT, GPIO_PIN);

    /* Enable pin interrupt on group interrupt A */
    adi_gpio_SetGroupInterruptPins(GPIO_PORT, SYS_GPIO_INTA_IRQn, GPIO_PIN);

    eIrq = SYS_GPIO_INTA_IRQn;
    /* Register the callback */
    adi_gpio_RegisterCallback (SYS_GPIO_INTA_IRQn, Adf7023_Callback, (void*)&eIrq);

    ADI_ENABLE_INT(SYS_GPIO_INTA_IRQn);

    return ADI_ADF7023_SUCCESS;
}
#if 0
//this routine performs a calibration of the VCO on startup
//TODO does this function apply anymore? did not work on previous Si
static ADI_ADF7023_RESULT_TYPE ADFCalibrationRoutine(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    ADI_ADF7023_RESULT_TYPE result;
    uint8_t CalibStatus = 0, CalibEn = 0x3, status;

    if (hDevice->PhyState != PHY_ON)
        result = adi_ADF7023_SetPhyOn(hDevice);

    if ((!result) && (hDevice->PhyState == PHY_ON))
    {
        result = adi_ADF7023_BlockMemWrite(hDevice, &CalibEn, &status, 1, CALIBRATION_CONTROL);

        if (!result)
            result = adi_ADF7023_SetPhyRx(hDevice);

        do
        {
            if (!result)
                result = adi_ADF7023_BlockMemRead(hDevice, &CalibStatus, 1, CALIBRATION_STATUS);
        } while(!(CalibStatus & 0x3));

        result = adi_ADF7023_SetPhyOn(hDevice);
        return result;
    }
    else
    {
        result = adi_ADF7023_SetPhyOn(hDevice);
        return result;
    }
}
#endif
static ADI_ADF7023_RESULT_TYPE AdfClearInterrupts(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    ADI_ADF7023_RESULT_TYPE uhf_result;
    uint8_t ClrInts[2], status;
    bool lBlockingMode;

    lBlockingMode = adi_ADF7023_GetBlockingMode(hDevice);

    adi_ADF7023_SetBlockingMode(hDevice, true);

    uhf_result = adi_ADF7023_BlockMemRead(hDevice, ClrInts, 2, INTERRUPT_SOURCE_0);
    //adi_GPIO_Toggle(ADI_GPIO_PORT_0, ADI_GPIO_PIN_0);
    if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemWrite(hDevice, ClrInts, &status, 2, INTERRUPT_SOURCE_0);
    //adi_GPIO_Toggle(ADI_GPIO_PORT_0, ADI_GPIO_PIN_0);
    adi_ADF7023_SetBlockingMode(hDevice, lBlockingMode);

    return uhf_result;
}
/* this function frees up the driver's internal SPI buffer. If the SpiCallback hasn't kicked
 * in and the SPI state is not set to idle, this function returns an error. If the state is
 * idle, the buffer is freed. Should this be based on buffer complete or SPI idle? */
static ADI_ADF7023_RESULT_TYPE ResetSpiBuffer(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;
    if(hDevice->SpiState != ADI_ADF7023_SPI_IDLE)   /* this will be set in the SPI callback */
        return ADI_ADF7023_ERR_SPI_BUSY;

    hDevice->SpiBuffer.pPrologue = NULL;
    hDevice->SpiBuffer.pTxData = NULL;
    hDevice->SpiBuffer.pRxData = NULL;
    hDevice->SpiBuffer.PrologueSize = 0;
    hDevice->SpiBuffer.DataSize = 0;
//    hDevice->SpiBuffer.bBufferProcessed = false;

    hDevice->SpiPrologue[0] = 0;
    hDevice->SpiPrologue[1] = 0;
    hDevice->SpiPrologue[2] = 0;

    return result;
}
static inline uint32_t MakeChannel(uint32_t const Freq)
{

    /* This function gets the 24 bit channel value from 32 bit frequency.
    Suppose the frequency is 950000000(950 MHz), the 24 bit channel calculated as
    950000000 * 65536 = 62,259,200,000,000
    62,259,200,000,000 / 26,000,000 = 2,394,584.62
    2,394,584.62 converted to integer which is  2,394,584
    */
    return ((((Freq / 1000) << 12) / 26000) << 4);

   // return (((uint64_t)Freq << 16) / 26000000);
}

static inline uint32_t MakeFreq(uint32_t const Channel)
{
    /* This function gets the 32 bit frequency value from 24 bit channel
    Suppose the channel value is 2,394,584 as calculated in MakeChannel function,
    the frequency is
    2,394,584 * 26,000,000 = 62,259,184,000,000
    62,259,184,000,000 / 65536 = 949,999,755.86
    949,999,755.86 converted to int is  949,999,755
    */
    //return ((((uint64_t)Channel * 26000000 ) >> 16) + 1);

    return (((((Channel >> 4) * 26000) >> 12) * 1000) + 1);
    /*Note: If this frequency (949,999,755) is used to calculate the channel,
    then channel value is 949,999,755 * 65536 = 62,259,183,943,680
    62,259,183,943,680 / 26,000,000 = 2,394,583.98
    2,394,583.98 converted to int is  2,394,583. which is not the value expected
    and frequency goes on drifting.
    To solve this issue, add 1Hz to frequency calculated.
    then channel value is 949,999,756 * 65536 = 62,259,184,009,216
    62,259,184,009,216 / 26,000,000 = 2,394,584.003
    2,394,584.003 converted to int is  2,394,584 which retains its value and
    there is no frequency drifting.
    */

}
static inline uint8_t CreateMemcmd(ADI_ADF7023_CMD_TYPE const cmd, uint16_t const addr)
{
    return ((cmd << 3) | (addr >> 8));
}
/*!
 * @brief UHF default interrupt handler.
 *
 * This takes a callback from the flag service that indicates
 * an interrupt on the GPIO connected to IRQ_GP3 on the 7023.
 * Currently clears interrupts, handles TX, RX address match,
 * CMD_FINISHED, and CMD_READY interrupts, and passes events
 * back to an application callback if one exists.
 *
 */
//extern uint32_t value[];
//extern uint8_t index_cal;
//uint32_t value5, value6;
void Adf7023_Callback(void *ClientCallback, uint32_t Event, void *pArg)
{

    /* Currently only one radio is supported. At a later time, we can add
    * support for callbacks from multiple radios and different flags based
    * on pArg */

    uint16_t AdfIntSource = 0;
    ADI_ADF7023_RESULT_TYPE uhf_result;
    uint8_t status;
    bool KnownEvent = false;
    ADI_ADF7023_DEV_HANDLE lHandle;
    bool lBlockingMode = true;
    uint8_t cca_config_value;
    static uint8_t receptionErr = 0;

    volatile uint16_t timeout = 15000;

    lHandle = &UHF_DevData;

    /* save current blocking mode to local var */
    lBlockingMode = lHandle->bBlockingMode;

    /* within the interrupt handler, force blocking mode, otherwise major headaches */
    uhf_result = adi_ADF7023_SetBlockingMode(lHandle, true);

    //volatile int ii = 20;
    //while(ii--);
    while(1)
    {
        if(!uhf_result)
            uhf_result = adi_ADF7023_BlockMemRead(lHandle, (uint8_t *)&AdfIntSource, 2, INTERRUPT_SOURCE_0);
        if(!AdfIntSource)
            break;

        /* Write to the clear interrupts on the 7023 */
        if (!uhf_result)
            uhf_result = adi_ADF7023_BlockMemWrite(lHandle, (uint8_t *)&AdfIntSource, &status, 2, INTERRUPT_SOURCE_0);


        /* Are multiple interrupts possible? Let's avoid switch statements for now */

        /* Command finished is a harder requirement than CMD_READY. CMD_FINISHED implies CMD_READY. The 7023
        * can accept one additional command before it is done processing the previous command due to the
        * 1 deep command fifo. Either of these interrupts can be used to determine when to issue an additional
        * command, but in cases like the AES engine we *need* to know that a command is done. So use
        * appropriately. */
        if(AdfIntSource & INT_CMD_FINISHED)
        {

            KnownEvent = true;
            lHandle->bIsCmdRdy = true;
            if(lHandle->pcCallback != NULL)    /* if a valid client callback is available, call it now */
                (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_EVENT_CMD_FINISHED, (void *)lHandle->IssuedCmd);
            lHandle->IssuedCmd = ADI_ADF7023_CMD_NULL;
        }

        /* The ADF7023 has a 1 deep command FIFO and the cmd_ready interrupt allows the
        * application know when it is safe to send the next command over to the radio.
        * Either this interrupt or the CMD_FINISHED interrupt can be used to indicate when
        * to send the next command or to indicate when the command is done. Use ONLY ONE of
        * these interrupts at any time. Callbacks could have the wrong command information otherwise. */
        else if(AdfIntSource &  INT_CMD_READY)
        {
            KnownEvent = true;
            lHandle->bIsCmdRdy = true;
            if(lHandle->pcCallback != NULL)    /* if a valid client callback is available, call it now */
                (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_EVENT_CMD_READY, (ADI_ADF7023_CMD_TYPE *)&lHandle->IssuedCmd);
            lHandle->IssuedCmd = ADI_ADF7023_CMD_NULL;
        }

    #ifdef PATCH_802_15_4D
        /* When the data tx with CCA is triggered, once the CCA is completed this
        interrupt is triggered.*/
        if(AdfIntSource &  INT_CCA)
        {
          //value6 = get_current_time();

    #ifdef UTEST_TRXSM
            utu_timestamp(UTUL_CCA_INT, 0/*mlist_size( s->direct_queue )*/);
    #endif
            KnownEvent = true;
            uhf_result = adi_ADF7023_BlockMemRead(lHandle, &cca_config_value, 1, BB_cca_cfg_0);
            // Check if CCA status is busy or clear bit 7
            cca_config_value &= 0x80;
            if(cca_config_value ==0x80)
            {
                // Channel busy even though timer expired will remain in RX state
                lHandle->PhyState = PHY_RX_CCA_15D4;
                lHandle->bCCA_Status = true;

            }
            else if(cca_config_value ==0)
            {
                // Channel is clear, so auto TX will happen automatically.
                // So change the phystate to TX to use TX related functions.
                // uhf_result = adi_ADF7023_IssueCmd(lHandle, ADI_ADF7023_CMD_CCA_STOP);
                //		      SetTestFlag(0,1);			//CCA clear -> goto TX state and start TX
                lHandle->PhyState = PHY_TX_15D4;
                lHandle->bCCA_Status = false;

                rx_duration += get_time_difference(rx_start_time);

            }
            if(lHandle->pcCallback != NULL)
                (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_15D4_EVENT_CCATIMER_EXPIRED, lHandle);

        }
        /* This interrupt triggers when ALMOST_FULL configured bytes has
        been transmitted or received. In the current code it is configured to 10.

        */

        if(AdfIntSource &  INT_BUFF_ALMOST_FULL)
        {
            KnownEvent = true;
            if(lHandle->pPacket == NULL)
            {
                RadioDataIRQErr++;
            }
            else
            {
                if(lHandle->PhyState == PHY_TX_15D4)
                {
                    /* almost full configured bytes transmitted. */
                    cbTxBufferAlmostFull(lHandle);
                }
                else if(lHandle->PhyState == PHY_RX_CCA_15D4)
                {
                    if(!receptionErr)
                    {
                        /* almost full configured bytes received. */
                        if(ADI_ADF7023_ERR_UNKNOWN == cbRxBufferAlmostFull(lHandle))
                        {
                            receptionErr = 1;
                        }
                    }
                }
            }
        }
        /* This interrupt triggers when BUFF_FULL configured bytes has
        been transmitted or received. In the current code it is
        configured to 20.To transmit(or receive) more bytes again data
        is written(or received) from base address and INT_BUFF_ALMOST_FULL
        and INT_BUFF_FULL triggering continues.*/
        else if(AdfIntSource &  INT_BUFF_FULL)
        {
            KnownEvent = true;
            if(lHandle->pPacket == NULL)
            {
                RadioDataIRQErr++;
            }
            else
            {

                if(lHandle->PhyState ==PHY_TX_15D4)
                    cbTxBufferFull(lHandle);
                else if(lHandle->PhyState == PHY_RX_CCA_15D4)
                {
                    if(!receptionErr)
                    {
                        cbRxBufferFull(lHandle);
                    }
                }
            }
        }
        if(AdfIntSource &  INT_PHR_DETECT)
        {
            //	    SetTestFlag(1,1);	//RX of packet at PHR
            AdfpreviousIntSource = (AdfIntSource &  INT_PHR_DETECT);
            KnownEvent = true;
            if(lHandle->pcCallback != NULL)
                (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_15D4_EVENT_PHR_DETECT, lHandle);
            }
            else if(AdfIntSource &  INT_PREAMBLE_DETECT){ }

            /* When the packet is received with correct CRC, this interrupt is triggerd. */
            if(AdfIntSource &  INT_CRC_CORRECT)
            {
            KnownEvent = true;
            lHandle->bCRC_Correct_Status = true;
            if(lHandle->pcCallback != NULL)
                (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_15D4_EVENT_CRC_CORRECT, lHandle);
        }
    #else  // PATCH_802_15_4D
        if (AdfIntSource & INT_SYNC_DETECT)
        {
            KnownEvent = true;
            if(lHandle->pcCallback != NULL)
                (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_EVENT_SYNC_WORD_DETECT, lHandle);
        }
    #endif
        /* This interrupt triggered, when data tx is completed */
        if (AdfIntSource & INT_TX_EOF)
        {
    //      if(index_cal > 19)
    //        index_cal = 0;
    //        value[index_cal++] = get_current_time();
            //        SetTestFlag(0,0);	//TX finished.
    #ifdef BENCH_MARKING
            //gpio_set();
            TxToRxTimeStamp1 = getBenchMarkTime();
    #endif
            //START_CYCLE_COUNT(start_count);

//            timeout = 15000;
//            while(timeout--);

            KnownEvent = true;
            if(true == lBlockingMode)
                lHandle->bTxDone = true;
            if(lHandle->pPacket == NULL)
            {
                RadioDataIRQErr++;
            }
            else
            {
                lHandle->pPacket->bBufferProcessed = true;
            }

    #ifdef FAST_TRANSITION_ENABLED
    #ifdef BENCH_MARKING
            ccaTimeStamp2 = getBenchMarkTime();
    #endif

            if(lHandle->auto_tx_to_rx_enabled)
            {
    #ifdef PATCH_802_15_4D
                lHandle->PhyState  = PHY_RX_CCA_15D4;

                adi_ADF7023_WaitforPhyStateReady(lHandle, PHY_RX_CCA_15D4);

    #else
                lHandle->PhyState  = PHY_RX_ON;

                adi_ADF7023_WaitforPhyStateReady(lHandle, PHY_RX_ON);
    #endif

    #ifdef FAST_TRANSITION_ENABLED
    #ifdef BENCH_MARKING
                //gpio_clear();
                //		if( (lHandle->auto_tx_to_rx_enabled) && DeviceReady )
                {
                    TxToRxTimeStamp2 = getBenchMarkTime();
                }
    #endif

                //STOP_CYCLE_COUNT(stop_count, start_count);

    #endif
            }
            else
            {
    #ifdef PATCH_802_15_4D
                lHandle->PhyState  = PHY_ON_15D4;
    #else
                lHandle->PhyState  = PHY_ON;
    #endif
            }

    #else

    /*********************************************/
    #ifdef PATCH_802_15_4D
        lHandle->PhyState  = PHY_ON_15D4;
    #else
        lHandle->PhyState  = PHY_ON;
    #endif
    /*********************************************/

    #endif
            if(lHandle->pPacket != NULL)
            {
                if(lHandle->pcCallback != NULL)
                (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_EVENT_PACKET_TRANSMITTED, lHandle->pPacket);

                /* clear out internal buffer. Now ready to receive next buffer */
                lHandle->pPacket = NULL;
            }
            lHandle->bIsRadioActive = false;
        }

        /* When packet is received, this interrupt is triggered. */
    #ifdef PATCH_802_15_4D
        if(AdfIntSource & INT_RX_EOF)
        {

            if(AdfpreviousIntSource)
            {
                //SetTestFlag(1,0);	//RX finished
    #ifdef BENCH_MARKING
                ackTimeStamp1 = getBenchMarkTime();
    #endif
                KnownEvent = true;



                if(lHandle->pPacket == NULL)
                {
                RadioDataIRQErr++;
                }
                else
                {
                    if(!receptionErr)
                    {
                        if(ADI_ADF7023_ERR_UNKNOWN == cb154dPacketReceived(lHandle))
                        {
                            receptionErr = 1;
                        }
                    }
                }

                if(lBlockingMode == true)     /* indicate to the driver that the packet is here */
                    lHandle->bRxDone = true;

                if (continuousRx)
                {
                    lHandle->PhyState  = PHY_RX_CCA_15D4;
                }
                else
                {
                    if(lHandle->PhyState == PHY_RX_CCA_15D4)
                        rx_duration += get_time_difference(rx_start_time);

#ifdef PATCH_802_15_4D
                    lHandle->PhyState  = PHY_ON_15D4;
#else
                    lHandle->PhyState  = PHY_ON;
#endif
                }
                lHandle->bIsRadioActive = false;   /* radio should be back to ON state now */
                if(lHandle->pPacket != NULL)
                {
                    lHandle->pPacket->bBufferProcessed = true; /* indicate that buffer is done */
                    if(!receptionErr)
                    {
                        if(lHandle->pcCallback != NULL)    /* Now callback the application.*/
                        (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_EVENT_PACKET_RECEIVED, lHandle);
                    }
                    else
                    {
                        if(lHandle->pcCallback != NULL)    /* Now callback the application.*/
                        (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_EVENT_ERR_PACKET_RECEIVED, lHandle);
                    }

                    /* this should ensure that the application doesn't submit another buffer in the application
                    * callback before we get out of interrupt land. No support for deferred callbacks yet */
                    lHandle->pPacket = NULL;           /* clear out the internal buffer */
                }
                receptionErr = 0;
            }
            else
            {
                KnownEvent = true;

    #ifdef PATCH_802_15_4D
                lHandle->PhyState  = PHY_ON_15D4;
    #else
                lHandle->PhyState  = PHY_ON;
    #endif
                lHandle->bIsRadioActive = false;

                if(lHandle->pPacket != NULL)
                {
                    lHandle->pPacket->bBufferProcessed = true; /* indicate that buffer is done */

                    if(lHandle->pcCallback != NULL)    /* Now callback the application.*/
                        (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_EVENT_ERR_PACKET_RECEIVED, lHandle);

                    /* this should ensure that the application doesn't submit another buffer in the application
                    * callback before we get out of interrupt land. No support for deferred callbacks yet */
                lHandle->pPacket = NULL;           /* clear out the internal buffer */
                }
            }
            AdfpreviousIntSource = 0;
            buff_allocated = 0;
        }
    #else  //END of RX for PATCH_802_15_4D
        if(AdfIntSource & INT_RX_ADDR_MATCH)
        {
            uint8_t rxpktlen;
            uint8_t rxpktaddr;

            KnownEvent = true;
            rxpktaddr = lHandle->RxPktBase;
            rxpktlen = lHandle->pPacket->ElementCount;

            if(0 == rxpktlen)
            {
                /* no packet length provided, read the first byte and then schedule the rest of the
                transfer. In SPI mode, this has to be the length of the packet. */
                uhf_result = adi_ADF7023_BlockMemRead(lHandle, &rxpktlen, 1, rxpktaddr);
                rxpktaddr += 1;

                lHandle->pPacket->pData[0] = rxpktlen;
                rxpktlen--;
                uhf_result = adi_ADF7023_BlockMemRead(lHandle, &lHandle->pPacket->pData[1], (uint32_t)rxpktlen, rxpktaddr);
            }
            else
            {
                /* packet length provided and we just read RxPktLen bytes from the radio */
                uhf_result = adi_ADF7023_BlockMemRead(lHandle, lHandle->pPacket->pData, (uint32_t)rxpktlen, rxpktaddr);

            }
            if(lBlockingMode == true)     /* indicate to the driver that the packet is here */
                lHandle->bRxDone = true;
            else
                uhf_result = adi_ADF7023_SetPhyOn(lHandle);

            lHandle->bIsRadioActive = false;   /* radio should be back to ON state now */
            lHandle->pPacket->bBufferProcessed = true; /* indicate that buffer is done */

            if(lHandle->pcCallback != NULL)    /* Now callback the application.*/
                (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_EVENT_PACKET_RECEIVED, lHandle->pPacket);

            /* this should ensure that the application doesn't submit another buffer in the application
            * callback before we get out of interrupt land. No support for deferred callbacks yet */
            lHandle->pPacket = NULL;           /* clear out the internal buffer */

        }
    #endif
        else if(false == KnownEvent)
        {
            /* not one of the supported events. will check here if there is a callback, otherwise just quit */
            if(lHandle->pcCallback != NULL)    /* Now callback the application.*/
                (lHandle->pcCallback)(lHandle->pCBParam, ADI_ADF7023_UNSUPPORTED_EVENT, NULL);
            uhf_result = adi_ADF7023_SetBlockingMode(lHandle, lBlockingMode);
        }
    }

    uhf_result = adi_ADF7023_SetBlockingMode(lHandle, lBlockingMode);
}
//uint32_t curr_time2,curr_time1 = 0 ;
void spicallback(void *pcbparam, uint32_t spievent, void *parg)
{
    ADI_ADF7023_DEV_HANDLE hDevice = (ADI_ADF7023_DEV_HANDLE) pcbparam;

    if(spievent == ADI_SPI_EVENT_BUFFER_PROCESSED)
    {
	if(hDevice->SpiState == ADI_ADF7023_SPI_TX)
	{
//        if(curr_time1)
//        {
//            curr_time2 = get_current_time();
//            curr_time1 = 0;
//        }
	    hDevice->SpiState = ADI_ADF7023_SPI_IDLE;
	    if(hDevice->pcCallback != NULL)
		(hDevice->pcCallback)(hDevice->pCBParam, ADI_ADF7023_EVENT_SPI_TX_DONE, hDevice->SpiBuffer.pTxData);
	}

	if(hDevice->SpiState == ADI_ADF7023_SPI_RX)
	{
	    hDevice->SpiState = ADI_ADF7023_SPI_IDLE;
	    if(hDevice->pcCallback != NULL)
		(hDevice->pcCallback)(hDevice->pCBParam, ADI_ADF7023_EVENT_SPI_RX_DONE, hDevice->SpiBuffer.pRxData);
	    hDevice->IssuedCmd = ADI_ADF7023_CMD_NULL; /* IssuedCmd is used for BlockMemRead function
	    needs to be cleared if non-block here */
	}

	if(hDevice->SpiState == ADI_ADF7023_SPI_CMD)
	    hDevice->SpiState = ADI_ADF7023_SPI_IDLE;
}
}
#ifdef DO_IR_CALIBRATION

/**
 * @brief Performs the ADF7023 IR calibration routine
 *
 * @param[in]   hDevice         Pointer to device handle
 *
 * @return ADI_ADF7023_RESULT_TYPE
 *
 * Downloads the IR calibration firmware module and do IR calibration.
 * New values of IR phase and amplitude are used to configure radio.
 *
 * @note The IR calibration firmware module is only downloaded the first time
 * this function is called.
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_do_IR_Calibration(ADI_ADF7023_DEV_HANDLE const hdevice)
{

   	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	uint8_t status;
   	uint8_t ucVal;
   	volatile int timeout;
   	ADI_ADF7023_BBRAM_TYPE BBRAM_backup;
  	uint8_t IRcalResult[2];		//IR cal phase/amplitude

   	//if(hdevice->IR_cal_cnt == 0)		//use to only load IR cal patch 1 time
   	{
	    // We need to be in PHY OFF state
	   	uhf_result =  adi_ADF7023_SetPhyOff(hdevice); // Set radio in PHY_OFF mode

	   	adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

	   	//we must reset the adf7023 before loading a new firmware patch
	   	//for the adf7023 to remain stable
	   	if(!uhf_result)
	   		uhf_result =  adi_ADF7023_ReInit(hdevice);

	   	// We need to be in PHY OFF state
	   	if(!uhf_result)
	   		uhf_result =  adi_ADF7023_SetPhyOff(hdevice); // Set radio in PHY_OFF mode

	   	adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);


		if(!uhf_result)
	      	uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_RAM_LOAD_INIT);

		hdevice->bIsCmdRdy = true;  // Explictly changing the status of the flag since the interrupt will not come

		if(!uhf_result)
			uhf_result = adi_ADF7023_BlockMemWrite(hdevice, (unsigned char *)IR_calbration_module_data, &status, sizeof(IR_calbration_module_data), 0x600);

	   	if(!uhf_result)
	   		uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_RAM_LOAD_DONE);

	   	adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

	   	if(!uhf_result)
	   		uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_SYNC);

	   	adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

		hdevice->PhyState  = PHY_OFF;//hans phy_state;
   }
   //firmware has been loaded!

   #if 0
   	uhf_result = readback_PRAM(hdevice);
    if (uhf_result != ADI_ADF7023_SUCCESS)
   	while(1);
   #endif

   //Mode control register bit 6 should be set at this point
   //before going to PHY_ON, BB_Cal bit. We do this by default.

   // Before issuing command for IR calibration make sure that
   // BB_CAL 6th bit (IF filter calibration) enabled in the mode control register 0x11A
   // If not enabled then use following code, otherwise ignore
   // 0100 0000 = 0x40
   //  if(!uhf_result)
   // uhf_result = adi_ADF7023_BlockMemWrite(hdevice, 0x40, &status,1,MODE_CONTROL);


	//backup bbram before doing IR cal
    uhf_result = adi_ADF7023_BlockMemRead(hdevice, (uint8_t *)&BBRAM_backup, sizeof(ADI_ADF7023_BBRAM_TYPE), INTERRUPT_MASK_0);

   	hdevice->IR_cal_cnt++;

   	//IR cal firmware download complete change the state to On
   	if(!uhf_result)
        uhf_result = adi_ADF7023_SetPhyOn(hdevice);

   	adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_ON);

   	if(!uhf_result) {
    	ucVal = 0xC0;
    	uhf_result = adi_ADF7023_BlockMemWrite(hdevice, &ucVal, &status,1,0x10);
   	}

   	if(!uhf_result) {
   		ucVal = 0x4;
   		uhf_result = adi_ADF7023_BlockMemWrite(hdevice, &ucVal, &status,1,0x11);
   	}

   	// Enable flag to indicate that IR calibration activity is going on.
   	hdevice->bIsIRCALactive = true;

   	//start IR cal routine
   	if(!uhf_result)
   		uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_IR_CAL);

   	adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_ON);

   	// disable flag to indicate that IR activity is complete.
   	hdevice->bIsIRCALactive = false;

   	//readback IR cal phase/amplitude values
   	uhf_result = adi_ADF7023_BlockMemRead(hdevice, (uint8_t *)&IRcalResult, sizeof(IRcalResult), IR_CAL_PHASE);

   	//update BBRAM backup with new IR cal values
   	BBRAM_backup.ir_cal_phase_r = IRcalResult[0];
    BBRAM_backup.ir_cal_amplitude_r = IRcalResult[1];
   	//restore bbram
   	if(!uhf_result)
        uhf_result = adi_ADF7023_InitBBRAM(hdevice, &BBRAM_backup);

    if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hdevice, ADI_ADF7023_CMD_CONFIG_DEV);

    adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_ON);
   	/* the timeout below is required to ensure the command completes */
    timeout = 20000;
   	do {
      timeout--;
    } while(timeout > 0);

   	return uhf_result;
}
#endif

/**
 * @brief Sets the ADF7023 transmit test mode.
 *
 * @param[in]   hDevice         Pointer to device handle
 * @param[in]   ModeValue       Test mode setting
 *
 * @return ADI_ADF7023_RESULT_TYPE
 *
 * This functions sets various test mode for the Transmit by writing to
 * packet RAM register 0x0D.
 * Test mode  value = 0 :  Disable Transmit Test Mode
 * Test mode  value = 1 :  Transmit PN9 test pattern
 * Test mode  value = 2 :  Transmit preamble continously
 * Test mode  value = 3 :  Transmit carrier continously
 * For any other Test mode value function returns the error as invalid test mode,
 *
 * @note This function should be called before setting radio to PHY_TX state.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetVarTxTestMode(ADI_ADF7023_DEV_HANDLE const hdevice, uint8_t ModeValue)
{
	uint8_t status;
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
 	uint8_t TxModeValue;

    if (ModeValue > 3)
 		return ADI_ADF7023_ERR_RADIO_INVALID_TESTMODE;

	uhf_result = adi_ADF7023_BlockMemWrite(hdevice, &ModeValue, &status,(uint32_t)0x1,VAR_TX_MODE_REG);
 	if(!uhf_result) {
          uhf_result = adi_ADF7023_BlockMemRead(hdevice, &TxModeValue, (uint32_t)0x1, VAR_TX_MODE_REG);
          if(!uhf_result){
            if(TxModeValue != ModeValue)
              return ADI_ADF7023_ERR_RADIO_INVALID_TESTMODE;
          }
        }

    return uhf_result;

}
/**
 * @brief       Reads the value from the given register address
 *
 * @param[in]   hdevice       Pointer to device handle
 * @param[in]   reg_addr	  Register address
 *
 * Reads the register with given address from the ADF7023 memory. The register read is from BBRAM,
 * MCR or Packet RAM memory
 *
 * @sa      Radio_write_reg(),
 *
 *
 * @return - The register value for given address.
 */

uint8_t Radio_read_reg(ADI_ADF7023_DEV_HANDLE const hdevice, int reg_addr)
{
    uint8_t val;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uhf_result = adi_ADF7023_BlockMemRead(hdevice, (uint8_t *)&val, 1, reg_addr);
    if(uhf_result != ADI_ADF7023_SUCCESS)
		return ADI_ADF7023_ERR_BLOCKWRITE;

	return val;
}
/**
 * @brief       Writes register value to the given register address
 *
 * @param[in]   hdevice       Pointer to device handle
 * @param[in]   reg_addr	  Register address
 * @param[in]   reg_value	  Register value to write
 *
 * Reads the register with given address from the ADF7023 memory. The register read is from BBRAM,
 * MCR or Packet RAM memory
 *
 * @sa      Radio_read_reg()
 *
 *
 * @return ADI_ADF7023_RESULT_TYPE
 */

ADI_ADF7023_RESULT_TYPE Radio_write_reg(ADI_ADF7023_DEV_HANDLE const hdevice, int reg_addr, uint8_t reg_value)
{
	uint8_t status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uhf_result = adi_ADF7023_BlockMemWrite(hdevice, &reg_value, &status,(uint32_t)0x1,reg_addr);

   	return uhf_result;
}


// New function to reset the parameters used for TX/RX
static void Reset_Device_Handle_State(ADI_ADF7023_DEV_HANDLE const hDevice)
{
	hDevice->bIsRadioActive = false;
	hDevice->pPacket = NULL;
}


#ifdef PATCH_802_15_4D
// All new functions and code modified for 15d4 patch


/**
 * @brief Sets the ADF7023J phy to 15D4 ON state
 *
 * @param[in]   hDevice     handle to the ADF7023J  device.
 *
 * Queries the current state of the radio and then turns the ADF7023-J to 15d4 phyON state .
 * Modifies the PhyState parameter in the device handle.
 * @sa      adi_ADF7023_SetPhyOn(), adi_ADF7023_SetPhyOff(),
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 *
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyOn15d4(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    ADI_ADF7023_PHY_STATE_TYPE phy_state = PHY_BUSY_TRX;
    volatile ADI_ADF7023_STATUS_TYPE status;
    uint8_t bb_cal = 0;
    volatile uint16_t timeout;

    /* first query the 7023 to determine current phy state */
    uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
    if(uhf_result != ADI_ADF7023_SUCCESS)
	return uhf_result;

    switch(phy_state)
	{
	case PHY_SLEEP:
		uhf_result = PhyWakeUp(hDevice);
		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_SYNC);
		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

		//set bit in mode control to perform IF filter calibration
		if(!uhf_result)
		{
			uhf_result = adi_ADF7023_BlockMemRead(hDevice, &bb_cal, (uint32_t)sizeof(bb_cal), MODE_CONTROL);
			bb_cal |= 0x40;
			if(!uhf_result)
				uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &bb_cal, (uint8_t *)&status, (uint32_t)sizeof(bb_cal), MODE_CONTROL);
		}

		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_ON);
		//now ensure that phy is ON before quitting
		//also timeout if it never gets into ON state
		timeout = 15000;     //TODO typically PHY_OFF -> PHY_ON takes 261us. right now arbit
		//assuming 4Mhz CCLK, 1044 cycles. need to make this more precise
		if(!uhf_result)
		{
			do
			{
				if( 0 < timeout )
					uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
				else
					uhf_result = ADI_ADF7023_ERR_PHY_ON;
				timeout--;
			}while( !uhf_result && phy_state != PHY_ON );
		}

		// After device is in phy on mode now issue command to go to 15d4 mode
		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_ENTER_15D4_MODE);
		//now ensure that phy is in 15d4 ON before quitting
		//also timeout if it never gets into ON state
		timeout = 15000;     //TODO typically PHY_OFF -> PHY_ON takes 261us. right now arbit
		//assuming 4Mhz CCLK, 1044 cycles. need to make this more precise
		if(!uhf_result)
		{
			do
			{
				if( 0 < timeout )
					uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
				else
					uhf_result = ADI_ADF7023_ERR_PHY_ON;
				timeout--;
			} while( !uhf_result && phy_state != PHY_ON_15D4 );
		}
		if (phy_state == PHY_ON_15D4)
			hDevice->PhyState = PHY_ON_15D4;

		break;

	case PHY_OFF:
		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_ON);
		//now ensure that phy is ON before quitting
		//also timeout if it never gets into ON state
		timeout = 15000;     //TODO typically PHY_OFF -> PHY_ON takes 261us. right now arbit
		//assuming 4Mhz CCLK, 1044 cycles. need to make this more precise
		if(!uhf_result)
		{
			do
			{
				if( 0 < timeout )
					uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
				else
					uhf_result = ADI_ADF7023_ERR_PHY_ON;
				timeout--;
			} while( !uhf_result && phy_state != PHY_ON );
		}

		// After device is in phy on mode now issue command to go to 15d4 mode
		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_ENTER_15D4_MODE);
		//now ensure that phy is in 15d4 ON before quitting
		//also timeout if it never gets into ON state
		timeout = 15000;     //TODO typically PHY_OFF -> PHY_ON takes 261us. right now arbit
		//assuming 4Mhz CCLK, 1044 cycles. need to make this more precise
		if(!uhf_result)
		{
			do
			{
				if( 0 < timeout )
					uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
				else
					uhf_result = ADI_ADF7023_ERR_PHY_ON;
				timeout--;
			} while( !uhf_result && phy_state != PHY_ON_15D4 );
		}
		if (phy_state == PHY_ON_15D4)
			hDevice->PhyState = PHY_ON_15D4;

		break;

	case PHY_TX:
	case PHY_RX:
		uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_ON);
		//now ensure that phy is ON before quitting
		//also timeout if it never gets into ON state
		timeout = 4000; //TODO typically PHY_RX or PHY_TX -> PHY_ON takes 70us
		//need to make timeout more precise
		if(!uhf_result)
		{
			do
			{
				if( 0 < timeout )
					uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
				else
					uhf_result = ADI_ADF7023_ERR_PHY_ON;
				timeout--;
			} while( !uhf_result && phy_state != PHY_ON );
		}
		if (phy_state == PHY_ON)
			hDevice->PhyState = PHY_ON;

		// After device is in phy on mode now issue command to go to 15d4 mode
		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_ENTER_15D4_MODE);
		//now ensure that phy is in 15d4 ON before quitting
		//also timeout if it never gets into ON state
		timeout = 15000;     //TODO typically PHY_OFF -> PHY_ON takes 261us. right now arbit
		//assuming 4Mhz CCLK, 1044 cycles. need to make this more precise
		if(!uhf_result)
		{
			do
			{
				if( 0 < timeout )
					uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
				else
					uhf_result = ADI_ADF7023_ERR_PHY_ON;
				timeout--;
			} while( !uhf_result && phy_state != PHY_ON_15D4 );
		}
		if (phy_state == PHY_ON_15D4)
			hDevice->PhyState = PHY_ON_15D4;

		break;

	case PHY_TX_15D4:
	case PHY_RX_CCA_15D4:

		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_ON_15D4);
		//now ensure that phy is in 15d4 ON before quitting
		//also timeout if it never gets into ON state
		timeout = 15000;     //TODO typically PHY_OFF -> PHY_ON takes 261us. right now arbit
		//assuming 4Mhz CCLK, 1044 cycles. need to make this more precise
		if(!uhf_result)
		{
			do
			{
				if( 0 < timeout )
					uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
				else
					uhf_result = ADI_ADF7023_ERR_PHY_ON;
				timeout--;
			} while( !uhf_result && phy_state != PHY_ON_15D4 );
		}
		if (phy_state == PHY_ON_15D4)
			hDevice->PhyState = PHY_ON_15D4;

		Reset_Device_Handle_State(hDevice);  // Reset device state

		break;

	case PHY_ON:

		//Device is in phy on mode now issue command to go to 15d4 mode
		if(!uhf_result)
			uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_ENTER_15D4_MODE);
		//now ensure that phy is in 15d4 ON before quitting
		//also timeout if it never gets into ON state
		timeout = 15000;     //TODO typically PHY_OFF -> PHY_ON takes 261us. right now arbit
		//assuming 4Mhz CCLK, 1044 cycles. need to make this more precise
		if(!uhf_result)
		{
			do
			{
				if( 0 < timeout )
					uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
				else
					uhf_result = ADI_ADF7023_ERR_PHY_ON;
				timeout--;
			} while( !uhf_result && phy_state != PHY_ON_15D4 );
		}
		if (phy_state == PHY_ON_15D4)
			hDevice->PhyState = PHY_ON_15D4;
		break;

	case PHY_ON_15D4:

		// Nothing to do update the device handle with the phy state
		hDevice->PhyState = PHY_ON_15D4;

	default:
		//we are in one of the states that we cannot issue PHY_ON in return error
		return ADI_ADF7023_ERR_PHY_ON;
	}

    return uhf_result;
}

/**
 * @brief Loads 15d4g firmware module to program RAM of ADF7023J.
 *
 * @param[in]   hDevice         Pointer to device handle
 *
 * @return ADI_ADF7023_RESULT_TYPE
 *
 * This routine make sure that radio state is PHY_OFF. Then initiates the sequence
 * for the 15d4 firmware download to the Progaram memory of ADF7023.
 *
 * @sa    adi_ADF7023_do_IR_Calibration()
 * @note  Both The IR calibration & 15d4g firmware modules uses program RAM.
 * At a time only one module can be active.
 *
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_154D_PatchRoutine(ADI_ADF7023_DEV_HANDLE hdevice)
{
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	uint8_t status;

    // We need to be in PHY OFF state
    uhf_result =  adi_ADF7023_SetPhyOff(hdevice); // Set radio in PHY_OFF mode

    adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

    if(!uhf_result)
	uhf_result =  adi_ADF7023_ReInit(hdevice);

    // We need to be in PHY OFF state
    if(!uhf_result)
	uhf_result =  adi_ADF7023_SetPhyOff(hdevice); // Set radio in PHY_OFF mode

    adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

    if(!uhf_result)
	uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_RAM_LOAD_INIT);


    hdevice->bIsCmdRdy = true;  // Explictly changing the status of the flag

    if(!uhf_result)
	uhf_result = adi_ADF7023_BlockMemWrite(hdevice, (unsigned char *)patch_802_15_4_d, &status, sizeof(patch_802_15_4_d), 0x600);


    if(!uhf_result)
	uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_RAM_LOAD_DONE);

    adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

    if(!uhf_result)
	uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_SYNC);

    adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

    hdevice->PhyState  = PHY_OFF;

    return uhf_result;
}



/**
 * @brief Turns ADF7023 phy 15d4 TX mode on and transmits a frame.
 *
 * @param[in]   hDevice    handle to the ADF7023 device
 * @param[in]   *TxFrame   pointer to the TX buffer.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Loads the packet data to the Packet RAM of the ADF7023 and then turns
 * phy 15d4 TX on. With the use of rolling buffer mecahnsim packet size
 * upto 2047 bytes maximum can be used for transmit.
 *
 * @sa adi_ADF7023_Receive_15d4_Frame()
 *
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_Transmit_15d4_Frame(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const TxFrame)
{

    ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;
    uint16_t BufAddr = hDevice->TxPktBase;
    ADI_ADF7023_PHY_STATE_TYPE phy_state;
    ADI_ADF7023_STATUS_TYPE status;
    bool lBlockingMode;


    /* State machine -
     * If radio active, and force transfer = true, overwrite contents of packet RAM and continue
     *      a) if in rx or tx, replace packet pointer with new buffer, and issue tx command
     *      b) enter critical region and ignore if an interrupt comes in till phy_tx is issued.
     *
     * If radio active, and force transfer = false, warn that a packet is alredy waiting to be
     * sent or received
     *
     * */

    if(true == hDevice->bIsSportMode)
    	return ADI_ADF7023_ERR_RADIO_SPORT_MODE;

    if(USE_PDSU_15_4D < (TxFrame->ElementCount - 2))  // Max data size handled by 15d4
    	return ADI_ADF7023_ERR_UNKNOWN;

    adi_int_EnterCriticalRegion();

    if(NULL != hDevice->pPacket && false == hDevice->bForceTransfer)
    {
        adi_int_ExitCriticalRegion();
        return ADI_ADF7023_ERR_RADIO_BUSY;
    }
    else if(NULL != hDevice->pPacket && hDevice->bForceTransfer)
    {
    /* disable interrupts,  and set phy to on state
     * we don't want an RX interrupt coming in and screwing with buffers here */

        if(PHY_ON != hDevice->PhyState)
            result = adi_ADF7023_SetPhyOn(hDevice);

        adi_ADF7023_config_15d4_BBRAM_Regs(hDevice);
    	hDevice->bIsRadioActive = false;
    }
    /* set the packet address */
    hDevice->pPacket = TxFrame;

    adi_int_ExitCriticalRegion();

    lBlockingMode = adi_ADF7023_GetBlockingMode(hDevice);

    /* force blocking till we at least right the packet out to
     * the radio so that it is safe to issue PHY_TX */
    adi_ADF7023_SetBlockingMode(hDevice, true);

    /* clear any ADF interrupts that came while we were doing this */
    AdfClearInterrupts(hDevice);

    /* This call should be blocking because we don't want the
     * SetPhyTx call to be issued before all the data is sent
     * to the radio */
    //curr_time1 = get_current_time();
    /* Check the packet length and write the packet RAM accordingly */
 	if(TxFrame->ElementCount < TX_SZ_15_4D)
    {
        //curr_time1 = get_current_time();
    	if(!result)
    	    result = adi_ADF7023_BlockMemWrite(hDevice, TxFrame->pData, (uint8_t *)&status, TxFrame->ElementCount, BufAddr);
        hDevice->iPacketOffset = TxFrame->ElementCount;
    }
	else
    {
    	if(!result)
        	result = adi_ADF7023_BlockMemWrite(hDevice, TxFrame->pData, (uint8_t *)&status, TX_SZ_15_4D, BufAddr);
        hDevice->iPacketOffset = TX_SZ_15_4D;
    }
    hDevice->iNextRadioOffset = TX_BASE_154D;
    /* restore original blocking mode */
    adi_ADF7023_SetBlockingMode(hDevice, lBlockingMode);

    hDevice->pPacket->bBufferProcessed = false;
    hDevice->bIsRadioActive = true;
    hDevice->bTxDone = false;

    // For Debug purpose only
#ifdef PATCH_802_DBGCOUNTS
    TxBuffalmostFullCnt = 0;
    TxBuffFullCnt = 0;
#endif
    if (!result)
    {

		result = adi_ADF7023_SetPhyTx(hDevice);
    }
#ifdef DEBUG_TRX_STATE_ERR
	if (result)
    {
    	if(testStatusIndexTx >= 10)
    	{
    		testStatusIndexTx = 0;
    	}
    	testStatusTx[testStatusIndexTx] = result;
    	testStatusIndexTx++;
    	hDevice->pPacket = NULL;
    }
#endif

     /* wait for interrupt to fire if in blocking mode */
    if(hDevice->bTxRxBlockingMode)
    {

    	while(!hDevice->bTxDone);
        /* if we get here packet has been transmitted */
        hDevice->bTxDone = false;

        if(!result)
            result = adi_ADF7023_GetPhyState(hDevice, &phy_state);

       /* at this point the radio should be back in ON state */

        if(!result)
            hDevice->PhyState = phy_state;
    }

    /* if not in blocking mode, we can peace out now */

    return result;
}

/**
 * @brief Turns ADF7023 phy 15d4 RX mode on and waits for a packet
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[out]  *RxFrame    pointer to the RX buffer.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Waits for a RX End Of interrupt and then reads the packet from
 * packet RAM. Length of packet information extacted from PHR field of 15d4 packet.
 * Using rolling buffer mechanism, oacket length of 2047 bytes can be received.
 * @sa adi_ADF7023_Transmit_15d4_Frame()
 */


ADI_ADF7023_RESULT_TYPE adi_ADF7023_Receive_15d4_Frame(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const RxFrame)
{

    /* Size of the packet is determined from PHR header of the 15d4 packet.
    * This informatin used to extract length & subsequent read from packet RAM happens
    * using rolling buffer mechanism
    */

    ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;
    ADI_ADF7023_PHY_STATE_TYPE phy_state;

    if(true == hDevice->bIsSportMode)
	return ADI_ADF7023_ERR_RADIO_SPORT_MODE;

    adi_int_EnterCriticalRegion();

    if(NULL != hDevice->pPacket && false == hDevice->bForceTransfer)
    {
        adi_int_ExitCriticalRegion();
        return ADI_ADF7023_ERR_RADIO_BUSY;
    }

    else if(NULL != hDevice->pPacket && hDevice->bForceTransfer)
    {

        /* disable interrupts,  and set phy to on state
        * we don't want an RX interrupt coming in and screwing with buffers here */
        adi_int_ExitCriticalRegion();
        if(PHY_ON != hDevice->PhyState)
            result = adi_ADF7023_SetPhyOn(hDevice);

        // set to 15d4 ON mode & Config the bbram regsisters
        adi_ADF7023_config_15d4_BBRAM_Regs(hDevice);

        adi_int_EnterCriticalRegion();

        hDevice->bIsRadioActive = false;
    }

    hDevice->pPacket = RxFrame;

    adi_int_ExitCriticalRegion();

    AdfClearInterrupts(hDevice);

#ifdef PATCH_802_DBGCOUNTS
    RxBuffalmostFullCnt = 0;  // Debug purpose
    RxBuffFullCnt = 0;
#endif
    // Reset the parameters of the device handle
    hDevice->iPacketOffset = 0;         // Offsets used in buffer almost full/full functions
    hDevice->iNextRadioOffset = RX_BASE_154D;

    RxFrame->bBufferProcessed = false;
    hDevice->bRxDone = false;
    hDevice->bIsRadioActive = true;
    hDevice->bCRC_Correct_Status = false;
    hDevice->PHR_FCS_Type = ADI_ADF7023J_4BYTE_FCS; // Default fcs type

    if(!result)
    {
		result = adi_ADF7023_SetPhyRx(hDevice);
    }

#ifdef DEBUG_TRX_STATE_ERR
    if (result)
    {
        if(testStatusIndexRx >= 10)
        {
            testStatusIndexRx = 0;
        }
        testStatusRx[testStatusIndexRx] = result;
        testStatusIndexRx++;
        hDevice->pPacket = NULL;
    }
#endif
    if(hDevice->bTxRxBlockingMode)
    {

        while(true != hDevice->bRxDone);

        /* at this point packet has been received and callbacks completed */
        hDevice->bRxDone = false; /* set variable to false and exit */
        hDevice->bIsRadioActive = false;
        hDevice->pPacket = NULL;

        if(!result)
            result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
        /* at this point the radio should be back in ON state */

        hDevice->PhyState = phy_state;
    }

    return result;

}

/**
 * @brief Submits a buffer for reception
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 * @param[out]  *RxFrame    pointer to the RX buffer.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Submits a buffer for reception, without turning the PHY state to PHY_RX. Length
 * of packet information extacted from PHR field of 15d4 packet.
 * Using rolling buffer mechanism, oacket length of 2047 bytes can be received.
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_Set_15d4_Receive_Buffer(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const RxFrame)
{

    /* Size of the packet is determined from PHR header of the 15d4 packet.
    * This informatin used to extract length & subsequent read from packet RAM happens
    * using rolling buffer mechanism
    */

    ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;
    ADI_ADF7023_PHY_STATE_TYPE phy_state;

    if(true == hDevice->bIsSportMode)
        return ADI_ADF7023_ERR_RADIO_SPORT_MODE;

    adi_int_EnterCriticalRegion();

    if(NULL != hDevice->pPacket && false == hDevice->bForceTransfer)
    {
        adi_int_ExitCriticalRegion();
        return ADI_ADF7023_ERR_RADIO_BUSY;
    }

    else if(NULL != hDevice->pPacket && hDevice->bForceTransfer)
    {

        /* disable interrupts,  and set phy to on state
        * we don't want an RX interrupt coming in and screwing with buffers here */

        if(PHY_ON != hDevice->PhyState)
            result = adi_ADF7023_SetPhyOn(hDevice);
        // set to 15d4 ON mode & Config the bbram regsisters
        adi_ADF7023_config_15d4_BBRAM_Regs(hDevice);

        hDevice->bIsRadioActive = false;
    }

    hDevice->pPacket = RxFrame;

    adi_int_ExitCriticalRegion(); /* re-enable interrupts */
//    ADI_EXIT_CRITICAL_REGION();   /* re-enable interrupts */

    AdfClearInterrupts(hDevice);

#ifdef PATCH_802_DBGCOUNTS
    RxBuffalmostFullCnt = 0;  // Debug purpose
    RxBuffFullCnt = 0;
#endif
    // Reset the parameters of the device handle
    hDevice->iPacketOffset = 0;         // Offsets used in buffer almost full/full functions
    hDevice->iNextRadioOffset = RX_BASE_154D;

    RxFrame->bBufferProcessed = false;
    hDevice->bRxDone = false;
    hDevice->bIsRadioActive = true;
    hDevice->bCRC_Correct_Status = false;
    hDevice->PHR_FCS_Type = ADI_ADF7023J_4BYTE_FCS; // Default fcs type

#ifdef DEBUG_TRX_STATE_ERR
    if (result)
    {
        if(testStatusIndexRx >= 10)
        {
            testStatusIndexRx = 0;
        }
        testStatusRx[testStatusIndexRx] = result;
        testStatusIndexRx++;
        hDevice->pPacket = NULL;
    }
#endif
    if(hDevice->bTxRxBlockingMode)
    {

        while(true != hDevice->bRxDone);

        /* at this point packet has been received and callbacks completed */
        hDevice->bRxDone = false; /* set variable to false and exit */
        hDevice->bIsRadioActive = false;
        hDevice->pPacket = NULL;

        if(!result)
            result = adi_ADF7023_GetPhyState(hDevice, &phy_state);
        /* at this point the radio should be back in ON state */

        hDevice->PhyState = phy_state;
    }

    return result;

}

static void adi_adf7023_154d_bbram_setting(TyBBRAM15_4d *pBBRAM)
{
   // Clear the BBRAM register specific to 15d4 firmware

   memset(pBBRAM,0x0,sizeof(TyBBRAM15_4d));

   // Configure each BBRAM register as per need

   // Antenna diversity disabled
   pBBRAM->antenna_RX_diversity_cfg_r  = 0x0;   // 0x129

   // Set to 0x0
   pBBRAM->TX_anetnna_cfg_r 		  = 0x0;   // 0x12A

   // Readback measured values -
   pBBRAM->antenna0_rssi_r          = 0x0;   // 0x12B
   pBBRAM->antenna1_rssi_r          = 0x0;   // 0x12C
   pBBRAM->threshold_diff_rssi_r    = 0x0;   // 0x12D

   // Number of pre-amble bytes
   pBBRAM->nb_preamble_bytes_low_r  = 0x0A;   // 0x12E
   pBBRAM->nb_preamble_bytes_high_r = 0x00;   // 0x12F

   // Sync
   pBBRAM->sfd_low_r                = 0x09;   // 0x130
   pBBRAM->sfd_high_r               = 0x72;   // 0x131

   // I think these are copied fromt the packet we issue?
   pBBRAM->PHR_low_r                = 0x0;   // 0x132
   pBBRAM->PHR_high_r               = 0x0;   // 0x133

   // Rolling Receive buffer config
   pBBRAM->rx_buff_signal_r         = RX_SZ_15_4D/2; // 0x134  Almost rx full
   pBBRAM->rx_buff_size_r           = RX_SZ_15_4D-1; // 0x135  Rx full
   // Rolling transmit buffer config
   pBBRAM->tx_buff_signal_r         = TX_SZ_15_4D/2; // 0x136  Almost tx full
   pBBRAM->tx_buff_size_r           = TX_SZ_15_4D;   // 0x137  Tx full

   // Set to 0x0
   pBBRAM->reserved1_r              = 0x0;   // 0x138
   // No test modes required
   pBBRAM->testmodes_r              = 0x0;   // 0x139

   // Reserved value
   pBBRAM->reserved2_r              = 0x1;   // 0x13A

   // Readback values
//   pBBRAM->vco_band_readback_r      = 0x0;   // 0x13B
//   pBBRAM->vco_ampl_readback_r      = 0x0;   // 0x13C
//We should not overwrite vco_band_readback_r and vco_ampl_readback_r registers,
}

/**
 * @brief Configures BBRAM registers specific to 15d4g firmware
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * First change the radio state to PHY_15D4_ON. Then initialize the BBRAM
 * registers specific to 15d4g firmware module.
 */


ADI_ADF7023_RESULT_TYPE adi_ADF7023_config_15d4_BBRAM_Regs(ADI_ADF7023_DEV_HANDLE hdevice)
{

	ADI_ADF7023_RESULT_TYPE  uhf_result = ADI_ADF7023_SUCCESS;
	TyBBRAM15_4d BBRAM154d;
	uint8_t status;

	//Ensure radio device is PHY_ON_15D4 state

	uhf_result = adi_ADF7023_SetPhyOn15d4(hdevice);

	// Set 15d4 specific BBRAM registers

	adi_adf7023_154d_bbram_setting(&BBRAM154d);

	// Initilize 15d4 specific BBRAM register with the set configuration.
	if(!uhf_result)
		uhf_result = adi_ADF7023_BlockMemWrite(hdevice,(unsigned char *)&BBRAM154d, &status, sizeof(TyBBRAM15_4d), BBRam154d_MMemmap_Start);

	return  uhf_result;

}

/**
 * @brief Handles the TX interrupt for almost full buffer case.
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 *
 * @return None
 *
 * When number of  bytes transmitted >= TX_SZ_15_4D/2, then this Almost full buffer
 * interrupt is active and this function gets invoked. First half of the buffer
 * can be over-written to packet RAM with new data. TX_BASE + TX_SZ_15_4D/2 forms
 * the first half & TX_BASE + TX_SZ_15_4D/2 to   TX_BASE + TX_SZ_15_4D
 * forms the second half in the packet RAM.
 *
 * @sa 	cbTxBufferFull()
 *
 */


/*************************************************************************
void cbTxBufferAlmostFull(void)

This is internal ISR function.


*************************************************************************/
void cbTxBufferAlmostFull(ADI_ADF7023_DEV_HANDLE hDevice)
{
	uint8_t status;
	// The first half of the buffer has been transmitted. Update it now based on packet length
	int BytestoSend = (hDevice->pPacket->ElementCount) - hDevice->iPacketOffset;

	if ((BytestoSend > 0) && (BytestoSend < (TX_SZ_15_4D/2)))
		// Write remaing data bytes in Packet RAM to be Transmitted.
		adi_ADF7023_BlockMemWrite (hDevice,((hDevice->pPacket->pData)+(hDevice->iPacketOffset)),&status, BytestoSend,hDevice->iNextRadioOffset);
	else if (BytestoSend >= (TX_SZ_15_4D/2))
	{
		adi_ADF7023_BlockMemWrite (hDevice,((hDevice->pPacket->pData)+(hDevice->iPacketOffset)),&status,(TX_SZ_15_4D/2),hDevice->iNextRadioOffset);
		hDevice->iPacketOffset += (TX_SZ_15_4D/2);
		hDevice->iNextRadioOffset = TX_BASE_154D + (TX_SZ_15_4D/2);
	}
#ifdef PATCH_802_DBGCOUNTS
	TxBuffalmostFullCnt++;
#endif
}

/**
 * @brief Handles the TX interrupt for buffer full  case.
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 *
 * @return None
 *
 * When number of  bytes transmitted  >= TX_SZ_15_4D, then this full buffer
 * interrupt is active and this function gets invoked. Second half of the buffer
 * can be over-written to packet RAM with new data .TX_BASE + TX_SZ_15_4D/2 forms
 * the first half  & TX_BASE + TX_SZ_15_4D/2 to  TX_BASE + TX_SZ_15_4D forms the
 * second half in the packet RAM.
 *
 * @sa 	cbTxBufferAlmostFull()
 *
 */
void cbTxBufferFull(ADI_ADF7023_DEV_HANDLE hDevice)
{
	uint8_t status;
	// The second half of the buffer has been transmitted. can update it now based on packet length
	int BytestoSend = (hDevice->pPacket->ElementCount) - hDevice->iPacketOffset;

	if ((BytestoSend > 0) &&(BytestoSend < (TX_SZ_15_4D/2)))
		// Write remaing data bytes in Packet RAM to be Transmitted.
		adi_ADF7023_BlockMemWrite (hDevice,((hDevice->pPacket->pData)+(hDevice->iPacketOffset)),&status, BytestoSend,hDevice->iNextRadioOffset);

	else if (BytestoSend >=(TX_SZ_15_4D/2))
	{
		adi_ADF7023_BlockMemWrite (hDevice,((hDevice->pPacket->pData)+(hDevice->iPacketOffset)),&status,(TX_SZ_15_4D/2),hDevice->iNextRadioOffset);
		hDevice->iPacketOffset += (TX_SZ_15_4D/2);
		hDevice->iNextRadioOffset = TX_BASE_154D;
	}
#ifdef PATCH_802_DBGCOUNTS
	TxBuffFullCnt++;
#endif
}

/**
 * @brief Handles the RX interrupt for almost full buffer case.
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 *
 * @return None
 *
 * When number of  bytes received >= RX_SZ_15_4D/2, then this Almost full buffer
 * interrupt is active and this function gets invoked. First half of the buffer
 * can be over-written to packet RAM with new data. RX_BASE + RX_SZ_15_4D/2 forms
 * the first half & RX_BASE + RX_SZ_15_4D/2 to   RX_BASE + RX_SZ_15_4D -1
 * forms the second half in the packet RAM.
 *
 * @sa 	cbRxBufferFull()
 *
 */
ADI_ADF7023_RESULT_TYPE cbRxBufferAlmostFull(ADI_ADF7023_DEV_HANDLE hDevice)
{
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uint16_t RecvPktLength=0;

	uint8_t *loc;

    /* todo: HANDLE 6lowpan error */

    if(hDevice->iPacketOffset==0)
    {
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (hDevice->pPacket->pData), (RX_SZ_15_4D/2),hDevice->iNextRadioOffset);
        // Read the length of packet in first two bytes
        // Take only 3 bits of length information from first byte read
        //hDevice->pPacket->pData[0] &=0x7;
        RecvPktLength =  MAKE_UINT16(hDevice->pPacket->pData[1],(hDevice->pPacket->pData[0] & PHR_LENMSBMASK));

        if( RecvPktLength > USE_PDSU_15_4D)
        {
            // Pkt length exceeds the limit
            uhf_result = ADI_ADF7023_ERR_UNKNOWN;
            return uhf_result;
        }
        else
            hDevice->pPacket->ElementCount= RecvPktLength + 2; // include PHR length of 2 bytes

        hDevice->pBuf = allocate_rx_buffer(hDevice->pPacket->pData);

        if(hDevice->pBuf == NULL)
        {
            return ADI_ADF7023_ERR_UNKNOWN;
        }
        buff_allocated = 1;
        loc = Get_Reception_location(hDevice->pBuf, hDevice->pPacket->pData);

        memcpy(loc, hDevice->pPacket->pData, RX_SZ_15_4D/2);

        hDevice->pPacket->pData = loc;

        if(hDevice->pPacket->pData[0] & PHR_FCSMASK)
        {
            hDevice->PHR_FCS_Type = ADI_ADF7023J_2BYTE_FCS;
        }
        else
        {
            hDevice->PHR_FCS_Type = ADI_ADF7023J_4BYTE_FCS;
            hDevice->bCRC_Correct_Status = Verify_4byte_CRC(hDevice->pPacket->pData, (RX_SZ_15_4D/2), true);
        }

    }
    else if (((hDevice->iPacketOffset) < (hDevice->pPacket->ElementCount + 1)) && (((hDevice->iPacketOffset) + (RX_SZ_15_4D/2)) <= (hDevice->pPacket->ElementCount + 1)))
    {
        if (!buff_allocated)
		{
			uhf_result = ADI_ADF7023_ERR_UNKNOWN;
			return uhf_result;
		}
        // Only read if fit in the buffer size
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, ((hDevice->pPacket->pData)+(hDevice->iPacketOffset)), RX_SZ_15_4D/2,hDevice->iNextRadioOffset);
        if (hDevice->PHR_FCS_Type == ADI_ADF7023J_4BYTE_FCS)
        {
            hDevice->bCRC_Correct_Status = Verify_4byte_CRC(((hDevice->pPacket->pData)+(hDevice->iPacketOffset)), (RX_SZ_15_4D/2), false);
        }
    }
    else
    {
        // Some thing wrong
        uhf_result = ADI_ADF7023_ERR_UNKNOWN;
    }
    if(!uhf_result)
    {
        hDevice->iPacketOffset += (RX_SZ_15_4D/2);
        hDevice->iNextRadioOffset = RX_BASE_154D + (RX_SZ_15_4D/2);

#ifdef PATCH_802_DBGCOUNTS
        RxBuffalmostFullCnt++;
#endif
    }
    return uhf_result;
}

/**
 * @brief Handles the RX interrupt for buffer full  case.
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 *
 * @return None
 *
 * When number of  bytes received >= RX_SZ_15_4D, then this full buffer
 * interrupt is active and this function gets invoked. Second half of the buffer
 * can be over-written to packet RAM with new data .RX_BASE + TX_SZ_15_4D/2 forms
 * the first half  & RX_BASE + RX_SZ_15_4D/2 to  RX_BASE + RX_SZ_15_4D -1 forms the
 * second half in the packet RAM.
 *
 * @sa 	cbRxBufferAlmostFull()
 *
 */
void cbRxBufferFull(ADI_ADF7023_DEV_HANDLE hDevice)
{

    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

	if (!buff_allocated)
	{
		return;
	}
    // The second half of the buffer has been received. Read it out now
    if (((hDevice->iPacketOffset) < (hDevice->pPacket->ElementCount + 1)) && (((hDevice->iPacketOffset) + (RX_SZ_15_4D/2)) <= (hDevice->pPacket->ElementCount + 1)))
    {
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, ((hDevice->pPacket->pData)+(hDevice->iPacketOffset)), (RX_SZ_15_4D/2), hDevice->iNextRadioOffset);
        if (hDevice->PHR_FCS_Type == ADI_ADF7023J_4BYTE_FCS)
        {
            hDevice->bCRC_Correct_Status = Verify_4byte_CRC(((hDevice->pPacket->pData)+(hDevice->iPacketOffset)), (RX_SZ_15_4D/2) - 1, false);
        }
    }
    else
    {
        // Some thing wrong
        uhf_result = ADI_ADF7023_ERR_UNKNOWN;
    }
    if(!uhf_result)
    {
        hDevice->iPacketOffset += (RX_SZ_15_4D/2)-1;
        hDevice->iNextRadioOffset = RX_BASE_154D;

#ifdef PATCH_802_DBGCOUNTS
        RxBuffFullCnt++;
#endif
    }
}

/**
 * @brief Receives remaining bytes in the packets .
 *
 * @param[in]   hDevice     handle to the ADF7023 device.
 *
 * @return None
 *
 * This is internal function invoked when Receive End (RxEOF) of interrupt is active
 * By now received an entire packet with few less bytes left to read.
 *
 */
ADI_ADF7023_RESULT_TYPE cb154dPacketReceived(ADI_ADF7023_DEV_HANDLE hDevice)
{
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    int RecvPktLen =0;
    int iLeft  = 0x0;
	uint8_t *loc;
    if((hDevice->iPacketOffset) == 0)
    {
        // This will be the case when pkt length is less than half the RX_SZ_15_4D
        // Do not know lenth of packet yet
        // so get the length of packet  by reading first two bytes and then read remianing bytes
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, (hDevice->pPacket->pData),2,(hDevice->iNextRadioOffset));
        RecvPktLen =  MAKE_UINT16(hDevice->pPacket->pData[1],(hDevice->pPacket->pData[0] & PHR_LENMSBMASK));
        //Increment pkt & radio offset by 2 as first two bytes already read
        hDevice->iPacketOffset +=2;
        hDevice->iNextRadioOffset +=2;
        if( RecvPktLen > (RX_SZ_15_4D/2))
        {
            // Pkt length exceeds the limit
            uhf_result = ADI_ADF7023_ERR_UNKNOWN;
            return uhf_result;
        }
        else
            hDevice->pPacket->ElementCount = RecvPktLen;

        if(hDevice->pPacket->pData[0] & PHR_FCSMASK)
        {
            hDevice->PHR_FCS_Type = ADI_ADF7023J_2BYTE_FCS;
        }
        else
        {
            hDevice->PHR_FCS_Type = ADI_ADF7023J_4BYTE_FCS;
        }

        uhf_result = adi_ADF7023_BlockMemRead(hDevice, ((hDevice->pPacket->pData)+(hDevice->iPacketOffset)), RecvPktLen,(hDevice->iNextRadioOffset));

        hDevice->pBuf = allocate_rx_buffer(hDevice->pPacket->pData);

        if(hDevice->pBuf == NULL)
        {
            return ADI_ADF7023_ERR_UNKNOWN;
        }

        buff_allocated = 1;
        loc = Get_Reception_location(hDevice->pBuf, hDevice->pPacket->pData);

        memcpy(loc, hDevice->pPacket->pData, RecvPktLen + 2);

        hDevice->pPacket->pData = loc;

        if (hDevice->PHR_FCS_Type == ADI_ADF7023J_4BYTE_FCS)
        {
            hDevice->bCRC_Correct_Status = Verify_4byte_CRC((hDevice->pPacket->pData), RecvPktLen + 2, true);
        }

    }
    else if ((hDevice->iPacketOffset) > 2)
    {
		if (!buff_allocated)
		{
			uhf_result = ADI_ADF7023_ERR_UNKNOWN;
			return uhf_result;
		}
        // We must have received something already
        iLeft = ( hDevice->pPacket->ElementCount - (hDevice->iPacketOffset));
        if( hDevice->pPacket->ElementCount < (hDevice->iPacketOffset))
        {
            // Pkt length exceeds the limit
            uhf_result = ADI_ADF7023_ERR_UNKNOWN;
            return uhf_result;
        }
        // We might have to read out remaining data
        if (iLeft)
        {
          if (((hDevice->iPacketOffset) < (hDevice->pPacket->ElementCount + 1)) && (((hDevice->iPacketOffset) + iLeft) <= (hDevice->pPacket->ElementCount + 1))){
            uhf_result = adi_ADF7023_BlockMemRead(hDevice, ((hDevice->pPacket->pData)+(hDevice->iPacketOffset)), iLeft,(hDevice->iNextRadioOffset));
          }

            if (hDevice->PHR_FCS_Type == ADI_ADF7023J_4BYTE_FCS)
            {
            hDevice->bCRC_Correct_Status = Verify_4byte_CRC(((hDevice->pPacket->pData)+(hDevice->iPacketOffset)), iLeft, false);
            }
        }
    }
    else
    {
        uhf_result = ADI_ADF7023_ERR_UNKNOWN;
    }
    return uhf_result;
}

/**
 * @brief Sends packet with or without clear channel assesment (CCA)
 *
 * @param[in]   hDevice     	Handle to the ADF7023 device.
 * @param[in]   PHR_Header  	PHR of 16 bit
 * @param[in]   PSDUdata   	 	Pointer to payload data of unsigned char type (uint8_t)
 * @param[in]   CCA_Therhold    CCA RSSI thershold value
 * @param[in]   TimerValue      CCA timer duration value
 * @param[in]   CCA_EnableFlag  Enable or disable Clear Channel Assesment (CCA)
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * For CCA enabled with Auto TX case this function initilzes the parameter required for CCA,
 * also writes the data to be sent in the packet RAM and  last command issued is CCA timer
 * start.This function has to take care of possible packet reception when valid packet
 * detected during CCA evalution period.
 * With CCA disabled tansmission of packet happens without checking the channel status.
 *
 */

ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_Send_Pkt(ADI_ADF7023_DEV_HANDLE hDevice,uint16_t Phr_Header, uint8_t *PSDUdata,uint8_t CCA_Therhold, uint8_t TimerValue,bool CCA_EnableFlag)
{

	static ADI_ADF7023_BUFFER_TYPE TxFrame;
  	volatile uint16_t TxPacketLength;
  	uint8_t BufAddr = 0x20, status;
  	volatile ADI_ADF7023_RESULT_TYPE	result = ADI_ADF7023_SUCCESS;
  	volatile uint8_t phy_rx_status = 0;
  	uint8_t *pPSDUdata = PSDUdata;

  	// Make sure that 2 byte fcs is not used
  	if((Phr_Header & PHR_FCSMASK) != PHR_FCSMASK)
  	{
	   	//Get packet length
  		TxPacketLength = (Phr_Header >> 8) & 0xFF  | (Phr_Header & 0x07) <<8;
#ifdef BENCH_MARKING
  		//BeforeCRCComputeBMTime = getBenchMarkTime();
#endif
//        if(index_cal > 19)
//           index_cal = 0;
        //value1 = get_current_time();
//        value[index_cal++] = get_current_time();
  		// Compute 4 byte FCS for given payload
  		Compute_4byte_CRC(PSDUdata,TxPacketLength);
//        if(index_cal > 19)
//           index_cal = 0;
//        value[index_cal++] = get_current_time();

        //value2 = get_current_time();

#ifdef BENCH_MARKING
  		//AfterCRCComputeBMTime = getBenchMarkTime();
#endif
  	}

  	if(CCA_EnableFlag == true)
  	{
  		// Send the packet with CCA option
  		// configure CCA parameters and transmit the data in the Auto TX mode.
  		result = ADF7023_15d4g_SetCCARSSIThreshold(hDevice, CCA_Therhold);  // Thershold value can be 11 to 81
  	    // Timer duration e.g 6 means to 9.98 ms timer duration
  		// Careful using value of 7 as it set to infinite mode of timer.
  		if(!result)
	  		result = ADF7023_15d4g_SetCCATimerDuration(hDevice, TimerValue);  // Timer duration can be 0 to 7 maximum.

        // Eanble Auto TX mode, clears the bit 4 of CCA_Config_Reg 0
  		if(!result)
  			result = ADF7023_15d4g_EnableAutoTx(hDevice, true);

		if (! result)
  		{
  			// Extract Header information and use it as first 2 bytes in the Tx data buffer
			pPSDUdata--;
  			*pPSDUdata = (Phr_Header >>8);
  			pPSDUdata--;
  			*pPSDUdata = (Phr_Header & 0xFF);
			TxPacketLength = ((pPSDUdata[0] & 7) << 8) | pPSDUdata[1];  // Get packet length

			//
     	    if (TxPacketLength > USE_PDSU_15_4D)
  			{
		  	 	return ADI_ADF7023_ERR_UNKNOWN;
  			}

			// Initilize the frame
  			TxFrame.ElementCount = TxPacketLength + sizeof(Phr_Header); // Extra bytes to care of PHR size.
  			TxFrame.pData = pPSDUdata;
			TxFrame.bBufferProcessed = false;
			hDevice->bTxDone = false;

			// Make packet RAM ready for Tx

			if( TxFrame.ElementCount >= (TX_SZ_15_4D))
     		{
     			if(!result)
        			result = adi_ADF7023_BlockMemWrite(hDevice, TxFrame.pData, (uint8_t *)&status, TX_SZ_15_4D, BufAddr);
            }
   			else
   			{
                //value1 = get_current_time();
   				if(!result)
			        result = adi_ADF7023_BlockMemWrite(hDevice, TxFrame.pData, (uint8_t *)&status, TxFrame.ElementCount, BufAddr);
   			}
            //value2 = get_current_time();
            //value3 = get_current_time();
            //value1 = get_current_time();
    		// Enter in RX state
    		// Not to be confused with TX frame used with the Receive.
    		// as we do not know if channel is busy may receive valid data before TX
            //value5 = get_current_time();
		    if(!result)
    			result = adi_ADF7023_Receive_15d4_Frame(hDevice, &TxFrame);
            //value5 = get_current_time();
    		if( TxFrame.ElementCount >= (TX_SZ_15_4D))
     			hDevice->iPacketOffset = TX_SZ_15_4D;
    		else
    			hDevice->iPacketOffset = TxFrame.ElementCount;

    		// Set the CCA status in clear mode
    		hDevice->bCCA_Status = false;
    	    // Before issuing timer start command check if preamble & SFD is detected
    		if(!result)
    		{
    			phy_rx_status = Radio_read_reg(hDevice,PR_phy_rx_status);

    	    	if(phy_rx_status != PREAMBLE_SFD_DETECT)
    			{

    				if(hDevice->Antenna_Diversity_Enabled)
    				{

	    				uint8_t reg_val_zero = 0, status;

	    				if(!result)
	    					result = adi_ADF7023_BlockMemWrite(hDevice, &reg_val_zero, &status, 1, BB_antenna0_rssi);

	    				if(!result)
	    					result = adi_ADF7023_BlockMemWrite(hDevice, &reg_val_zero, &status, 1, BB_antenna1_rssi);
    				}
                    //value5 = get_current_time();
	    			// Call API to start timer
    				if(!result)
    					result = ADF7023_15d4g_StartCCATimer(hDevice);
   				}
   				else
   					// mark channel as busy and to receive data from other device
	  				hDevice->bCCA_Status = true;

    		}

      	}// if(!result)
    }// if(CCA_EnableFlag == true)
	else
	{
  		// Non-CCA  TX
  		// Create the packet data for transmission
  	    // Extract Header information and use it as first 2 bytes in the Tx data buffer
	    pPSDUdata--;
  		*pPSDUdata = (Phr_Header >>8);
  		pPSDUdata--;
  		*pPSDUdata = (Phr_Header & 0xFF);
		TxPacketLength = ((pPSDUdata[0] & 7) << 8) | pPSDUdata[1];  // Get packet length


	    // Initilize the frame
	    TxFrame.ElementCount = TxPacketLength + sizeof(Phr_Header); // Extra bytes to care of PHR size.

	    TxFrame.pData = pPSDUdata;
	  	TxFrame.bBufferProcessed = false;

	  	if (!result)
	  		result = adi_ADF7023_Transmit_15d4_Frame(hDevice, &TxFrame);
	}
  return result;
}

/**
 * @brief   	Sets TX & RX function for 15d4 in the blocking or non blocking mode
 *
 * @param[in]   hDevice             UHF device handle
 * @param[in]   bTxRxBlockingMode   Blocking mode (true = blocking, false = non-blocking)
 *
 * @return      ADI_ADF7023_RESULT_TYPE
 *
 * Sets the TXRX blocking mode.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetTxRxBlockingMode(ADI_ADF7023_DEV_HANDLE const hDevice, bool bTxRxBlockingMode)
{

    hDevice->bTxRxBlockingMode = bTxRxBlockingMode;
    return ADI_ADF7023_SUCCESS;
}
/**
 * @brief   Gets blocking mode status of TX & RX function for 15d4.
 *
 * @param[in]   hDevice             UHF device handle
 *
 * @return      boolean type true or false
 *
 * Gets the TXRX blocking mode.
 *
 */
bool adi_ADF7023_GetTxRxBlockingMode(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    return hDevice->bTxRxBlockingMode;
}

/**
 * @brief Sets the CCA thershold value
 *
 * @param[in]   hDevice     	Handle to the ADF7023 device.
 * @param[in]   cca_therhold    CCA RSSI thershold
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the CCA RSSI Thershold value to use in PHY_RX_CCA_15D4 state for CCA check.
 * The received signal strength is compared with this thershold value.If first one
 * is more than thershold then channel is busy else channel is clear.
 *
 * @note This function should be used before using any of the CCA feature.
 */

ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_SetCCARSSIThreshold(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t cca_threshold)
{

	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	uint8_t status;

	// ADF7023J datasheet mentions RSSI in the range of -96 to -26  in dbm
	// The equation CCA_THRESHOLD = RSSI(dbm) + 107
	// So Maximum and minimum values for thershold are 11 & 81.
	if((cca_threshold < 11) || ( cca_threshold > 81))
		return ADI_ADF7023_ERR_RADIO_INVALID_CCA_RSSI_THERSHOLD;

	uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &cca_threshold, &status, 1, BB_cca_threshold);

#if 0
	// Readback the thershold value & verify
	if(!uhf_result)
        uhf_result = adi_ADF7023_BlockMemRead(hDevice, &cca_th_readback, (uint32_t)1, BB_cca_threshold);

	if (cca_th_readback == cca_threshold)
    	uhf_result = ADI_ADF7023_SUCCESS;
	else
    	uhf_result = ADI_ADF7023_ERR_RADIO_INVALID_CCA_RSSI_THERSHOLD;
#endif

	return uhf_result;

}

/**
 * @brief Get the CCA Live status.
 *
 * @param[in]   hDevice		Handle to the ADF7023 device.
 *
 * @return unsigned char (uint8_t) type
 *
 * Get the CCA live status value.If status is "1" then channel is busy
 * else channel is clear.
 *
 */
uint8_t ADF7023_15d4g_GetCCALiveStatus(ADI_ADF7023_DEV_HANDLE const hDevice)
{
 	uint8_t CCALiveStatus;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

 	uhf_result = adi_ADF7023_BlockMemRead(hDevice, &CCALiveStatus, (uint32_t)1, BB_cca_cfg_1);

	// MSB bit indicates the CCA live busy or clear status
 	// Get the value "0" for clear or "1" for busy.
	if(uhf_result != ADI_ADF7023_SUCCESS)
	{
		CCALiveStatus =  (CCALiveStatus & 0x80) >> 7;
		 return CCALiveStatus;
	}
	else
		return ADI_ADF7023_ERR_BLOCKWRITE;
}

/**
 * @brief Enable or disable Auto Tx mode.
 *
 * @param[in]   hDevice     Handle to the ADF7023 device.
 * @param[in]   AutoTxflag  boolean type (true or false)
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Enable or disable Auto TX mode used with timer mode of CCA .
 *
 * @sa ADF7023_15d4g_SetCCARSSIThreshold(),ADF7023_15d4g_SetCCATimerDuration()
 *
 * @note This function should be used to before using any of the CCA feature.
 *
 */

ADI_ADF7023_RESULT_TYPE  ADF7023_15d4g_EnableAutoTx(ADI_ADF7023_DEV_HANDLE const hDevice,bool AutoTxflag)
{

	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

 	uint8_t status, cca_cfg0_reg_value;
 	uhf_result = adi_ADF7023_BlockMemRead(hDevice, &cca_cfg0_reg_value, (uint32_t)1, BB_cca_cfg_0);

 	if (AutoTxflag == true)
 	{
 	 // For Auto TX in CCA mode, bit 4 in this register should be cleared (zero ).
 	 // Make sure to set the timer duration as this mode works along with timer mode.
 		cca_cfg0_reg_value &= 0xEF;
 	}
 	else
 	{
 	 // Auto TX disabled, set bit 4 to 1
 	 	cca_cfg0_reg_value |= 0x10;
  	}
  	if(!uhf_result)
 		uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &cca_cfg0_reg_value, &status, 1, BB_cca_cfg_0);

 	return uhf_result;
}

/**
 * @brief Sets timer duration for the CCA evaluation.
 *
 * @param[in]   hDevice     Handle to the ADF7023 device.
 * @param[in]   TimeValue   Timer value in the range 0-7 (uint8_t)
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * This function sets the timer duration when used in timer mode for CCA evaluation.
 * Following values are used for Timevalue & corrsponding timer duration setting
 *
 *  TimeValue 	Timer Duration
 *  ==========	==============
 *  	0 			 160 microsecond
 *  	1			 320 microsecond
 *      2 	 		 640 microsecond
 *   	3			1280 microsecond
 *   	4  		    1920 microsecond
 *   	5 			2560 microsecond
 *  	6			9960 microsecond
 *   	7           Infinite Mode
 * For any value above 7 will return without setting any timer value.
 * Caution - While using Timevalue of 7, timer is set in infinite mode.
 *
 * @sa ADF7023_15d4g_SetCCARSSIThreshold(),ADF7023_15d4g_EnableAutoTx()
 *
 * @note This function should be used before using any of the CCA feature.
 *
 */

ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_SetCCATimerDuration(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t TimeValue)
{

	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
 	uint8_t status, cca_cfg0_Timer_value;

 	if (TimeValue > 7)
 	{
 	 	return ADI_ADF7023_ERR_RADIO_INVALID_TIMER_DURATION;
 	}

 	uhf_result = adi_ADF7023_BlockMemRead(hDevice, &cca_cfg0_Timer_value, (uint32_t)1, BB_cca_cfg_0);

    // Note bit 0 of cca_cfg_0 is for bypass cca update. for normal use
    // this bit set to "0" and  for debug purpose only this bit is set to "1".
 	// Set the timer duration value for bit position 1,2 & 3
  	// Clear the timer duration value read and set required value;

  	cca_cfg0_Timer_value = (cca_cfg0_Timer_value & 0xF1)| (TimeValue << 1);

   	if(!uhf_result)
 		uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &cca_cfg0_Timer_value, &status, 1, BB_cca_cfg_0);
#if 0
 	// Readback the timer value set
 	uhf_result = adi_ADF7023_BlockMemRead(hDevice, &Timer_Value_Rdbk, (uint32_t)1, BB_cca_cfg_0);

    if(!uhf_result)
    {

     	Timer_Value_Rdbk = (Timer_Value_Rdbk & 0xE) >> 1;
     	if(Timer_Value_Rdbk != TimeValue)
     	uhf_result = ADI_ADF7023_ERR_RADIO_INVALID_TIMER_DURATION;
    }
#endif
    return uhf_result;
}

/**
 * @brief Starts the CCA timer.
 *
 * @param[in]   hDevice     Handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Starts CCA timer by issuing command ADI_ADF7023_CMD_CCA_START.
 *
 * @sa ADF7023_15d4g_StopCCATimer()
 *
 * @note This function should be called when radio device state is
 * PHY_RX_CCA_15D4
 */

ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_StartCCATimer(ADI_ADF7023_DEV_HANDLE const hDevice)
{
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CCA_START);

    while(!hDevice->bIsCmdRdy);
    return uhf_result;
}

/**
 * @brief Stops the CCA timer.
 *
 * @param[in]   hDevice     Handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Stops CCA timer by issuing command ADI_ADF7023_CMD_CCA_STOP.
 *
 * @sa ADF7023_15d4g_StartCCATimer()
 *
 * @note This function should be called when radio device state is
 * PHY_RX_CCA_15D4
 */

ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_StopCCATimer(ADI_ADF7023_DEV_HANDLE const hDevice)
{
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CCA_STOP);

    return uhf_result;
}

/**
 * @brief Enable or disable continuous RX mode using test mode register.
 *
 * @param[in]   hDevice     Handle to the ADF7023 device.
 * @param[in] 	state 		Boolean type true or false
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Enable or disable the continuous RX using BBRAM test mode register
 * bit "1" of this register is used for this purpose. After packet reception the radio
 * device state remains as PHY_RX_CCA_15D4 when this bit set to 1.
 *
 * @note This function used only as test mode only
 */

ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_SetBitContinousRX(ADI_ADF7023_DEV_HANDLE const hDevice, bool state)
{

    ADI_ADF7023_RESULT_TYPE uhf_result;

    uint8_t status,value;

    uhf_result = adi_ADF7023_BlockMemRead(hDevice, &value, 1, BB_testmodes);

	if (state == 1)
    	value |= (1 << BIT_CONTINOUS_RX);
    else
    	value &= ~(1 << BIT_CONTINOUS_RX);

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &value, &status, 1, BB_testmodes);

    if(!uhf_result)
        continuousRx = state;

    return uhf_result;
}

/**
 * @brief Reads the RSSI value from the current active antenna.
 *
 * @param[in]   hDevice     Handle to the ADF7023 device.
 * @param[in] 	RSSI_r 		Pointer to integer type variable.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Checks the current active antenna and read the RSSI value from that antenna.
 * The read value converted to dBm by subtracting 107.
 *
 */

ADI_ADF7023_RESULT_TYPE ADF7023_ReadAntennaRSSI(ADI_ADF7023_DEV_HANDLE const hDevice, int *RSSI_r)
{
	ADI_ADF7023_RESULT_TYPE uhf_result;
	uint8_t value;

	uhf_result = adi_ADF7023_BlockMemRead(hDevice, &value, 1, BB_antenna_RX_diversity_cfg);
	// Read the the RSSI value from antenna selected either manually or
	// by antenna diversity algorithm (if enabled)
	if(value & ANTENNA_1)
	{
		uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)RSSI_r, 1, BB_antenna1_rssi);

		Antenna1_Counter++;

		hDevice->Antenna_Selected = ADI_ADF7023J_15D4_ANTENNA1;

	}
	else
	{
		uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)RSSI_r, 1, BB_antenna0_rssi);

		Antenna0_Counter++;

		hDevice->Antenna_Selected = ADI_ADF7023J_15D4_ANTENNA0;
	}
	*RSSI_r -= 107;
	return uhf_result;
}

/**
 * @brief Configures the receive antenna path & enable/disable Antenna diversity.
 *
 * @param[in]   hDevice     		Handle to the ADF7023 device.
 * @param[in] 	Antenna_Path_0 		ATB Control bit setting for path 0 (uint8_t)
 * @param[in] 	Antenna_Path_1		ATB Control bit setting for path 1 (uint8_t)
 * @param[in] 	ATB_Level           ADI_ADF7023J_15D4_TRX_ATB_LEVEL type
 * @param[in] 	ADA_Enable			ADI_ADF7023J_15D4_RX_ANTENNA_DIVERSITY type
 * @param[in] 	Antenna_Select      ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED type
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Configures the "BB_antenna_RX_diversity_cfg" (0x129) register for antenna 0 & 1 ATB bit
 * control path.Sets the ATB driver to be used along Antenna Diversity (AD) enable or disable.
 * If AD is enabled, then Antenna_Select parameter is of no significance because Antenna is
 * selected by Antenna Diversity Alogorithm.
 * When AD is disabled, then Antenna_Select parameter is used to select antenna to be used
 * for the packet reception
 *
 * @sa  ADF7023_15d4g_TxAntennaConfig()
 *
 * @note Active bit control setting for antenna 0 or 1 are 01 or 10.
 * Please see the detail description of Antenna Signal Path Control in the document
 * Antenna Diversity and 15d4g Firmware Module Specification_Rev4.01.docx.
 */

ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_RxAntennaDivConfig(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t Antenna_Path_0, uint8_t Antenna_Path_1, ADI_ADF7023J_15D4_TRX_ATB_LEVEL ATB_Level, ADI_ADF7023J_15D4_RX_ANTENNA_DIVERSITY ADA_Enable, ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED Antenna_Select)
{

	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	uint8_t status,value;

	value = 0; //Start from scratch

	// Set antenna 0 & 1 path configuration
	value = ((Antenna_Path_0 << 6) | (Antenna_Path_1 << 4));

	// set the driver level to use 3.3 or 1.8 volt
	if( ATB_Level == ADI_ADF7023J_15D4_1_8V_DRV)
		value |= (1<<3); // 1.8v
	else
		value &= 0xF7; // for 3.3v

	// Enable antenna diversity
	if (ADA_Enable == ADI_ADF7023J_15D4_AD_ENABLE)
		value |= 1;
	else
	{
		// otherwise set Anetnaa from the antenna select value
		if( Antenna_Select == ADI_ADF7023J_15D4_ANTENNA1 )
			value |= 2;
	  	else
	  		value &= 0xFC;
	}
	// Write the configuration to the  RX Antenna diversity BBRAM register.
	uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &value, &status, 1, BB_antenna_RX_diversity_cfg);

	hDevice->Antenna_Diversity_Enabled = ADA_Enable;

	return uhf_result;
}

/**
 * @brief Configures the antenna 0 & 1 path & select an antenna for transmission.
 *
 * @param[in]   hDevice     		Handle to the ADF7023 device.
 * @param[in] 	Antenna_Path_0 		ATB Control bit setting for path 0 (uint8_t)
 * @param[in] 	Antenna_Path_1		ATB Control bit setting for path 1 (uint8_t)
 * @param[in] 	ATB_Level           ADI_ADF7023J_15D4_TRX_ATB_LEVEL type
 * @param[in] 	Antenna_Select      ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED type
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Configures the "BB_antenna_TX_cfg_r" register (0x12A) for antenna 0 & 1 ATB bit control
 * path.Sets the ATB driver to be used Antenna_Select parameter is used to select antenna
 * to be used for the packet transmission
 *
 * @sa  ADF7023_15d4g_RxAntennaDivConfig()
 *
 * @note Active bit control setting for antenna 0 or 1 are 01 or 10.
 * Please see the detail description of Antenna Signal Path Control in the document
 * Antenna Diversity and 15d4g Firmware Module Specification_Rev4.01.docx.
 */

ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_TxAntennaConfig(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t Antenna_Path_0, uint8_t Antenna_Path_1, ADI_ADF7023J_15D4_TRX_ATB_LEVEL ATB_Level,ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED Antenna_Select)
{


	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	uint8_t status,value;

	value = 0; //Start from scratch

	// Set antenna 0 & 1 path configuration
	value = ((Antenna_Path_0 << 6) | (Antenna_Path_1 << 4));

	// set the driver level to use 3.3 or 1.8 volt
	if( ATB_Level == ADI_ADF7023J_15D4_1_8V_DRV)
		value |= (1<<3); // 1.8v
	else
		value &= 0xF7; // for 3.3v

	// Set Antenna from the antenna select value
	if( Antenna_Select == ADI_ADF7023J_15D4_ANTENNA1 )
		value |= 2;
	else
		value &= 0xFC;
	// Write the configuration to the  RX Antenna diversity BBRAM register.
	uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &value, &status, 1, BB_TX_anetnna_cfg_r);

	return uhf_result;
}

/**
 * @brief Sets preamble length (in bytes).
 *
 * @param[in]   hDevice     		Handle to the ADF7023 device.
 * @param[in] 	Preamble_length 	Preamble length of unsigned short type (uint16_t)
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Sets the register "BB_nb_preamble_bytes_low" & "BB_nb_preamble_bytes_high" specific
 * to 15d4 firmware module for the length of preamble (in bytes).Preamble is sequence of 1010
 * added at the start of the packet during transmission.
 */

ADI_ADF7023_RESULT_TYPE adi_adf7023_154d_Set_Preamble_Length(ADI_ADF7023_DEV_HANDLE const hDevice, uint16_t Preamble_length)
{

	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	ADI_ADF7023_PHY_STATE_TYPE phy_state = PHY_BUSY_TRX;
    uint8_t status;
    uint8_t Preamble_Len[2];

    // Preamble length can be set from 4 to 1000, retrun error if not within limit.
    if((Preamble_length < 4) || (Preamble_length > 1000))
    	return ADI_ADF7023_ERR_UNKNOWN;

    /* first determine current phy state */
    uhf_result = adi_ADF7023_GetPhyState(hDevice, &phy_state);

    if(uhf_result != ADI_ADF7023_SUCCESS)
    	return uhf_result;

    // Check if radio state is PHY_ON_15D4 state
    if(PHY_ON_15D4 != phy_state)
    	return ADI_ADF7023_ERR_UNKNOWN;

    Preamble_Len[0] = (Preamble_length & 0xFF);
    Preamble_Len[1] = (Preamble_length & 0xFF00) >> 8;

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &Preamble_Len[0], &status, 2, BB_nb_preamble_bytes_low);

// Only for cross verification
#if 0
    uint8_t Test_Preamble_length[2], i = 0;

    uhf_result = adi_ADF7023_BlockMemRead(hDevice, Test_Preamble_length, sizeof(Test_Preamble_length), BB_nb_preamble_bytes_low);
    for(i = 0; i < 2; i++) {
        if (Test_Preamble_length[i] == Preamble_Len[i])
            continue;
        else
            uhf_result = ADI_ADF7023_ERR_UNKNOWN;
    }
#endif
    return uhf_result;
}

//uint32_t bit_reverse(uint32_t crc_val)
//{
//    crc_val = (crc_val & 0x0000FFFF) << 16 | (crc_val & 0xFFFF0000) >> 16;
//    crc_val = (crc_val & 0x00FF00FF) << 8 | (crc_val & 0xFF00FF00) >> 8;
//	crc_val = (crc_val & 0x0F0F0F0F) << 4 | (crc_val & 0xF0F0F0F0) >> 4;
//	crc_val = (crc_val & 0x33333333) << 2 | (crc_val & 0xCCCCCCCC) >> 2;
//	crc_val = (crc_val & 0x55555555) << 1 | (crc_val & 0xAAAAAAAA) >> 1;
//
//    return crc_val;
//}

/**
 * @brief Computes 4 byte CRC for the given payload data buffer.
 *
 * @param[in]   databuff	 Pointer to the data buffer of the unsigned char type
 * @param[in] 	data_length  length of unsigned short type
 *
 * @return 4 byte CRC appended to data buffer pointed by databuff.
 *
 * The 4 byte CRC is calculated using following standard generator polynomial of degree 32:
 * G32(x) = x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
 * The initial remainder of the division is preset to all ones and then modified via division
 * of the calculation field (payload data) by the generator polynomial G32(x).
 *
 * The library  function used to do this polynomial division is GenerateCRC32 ().
 * It uses three parameters namely pointer to payload buffer, length of payload buffer
 * and remainder which is output of this function. The 	one's complement of this remainder
 * is the 4-byte (octet) FCS field.The FCS field is transmitted commencing with the coefficient
 * of the highest 	order term.
 * If the length of the payload is less than 4 octets, the FCS computation shall assume
 * padding the payload field by appending zero value octets to the most significant bits
 * to make sure the ayload field length exactly 4 octets; however,these padded bits are not
 * transmitted.

 *
 * @sa  Verify_4byte_CRC()
 *
 */

void Compute_4byte_CRC(uint8_t *databuff, uint16_t data_length)
{
	unsigned int crc;

	data_length -= 4;

	if(data_length < 4)
	{
		switch(data_length)
		{
			case 1: databuff[1] = 0x0;
					databuff[2] = 0x0;
					databuff[3] = 0x0;
					break;

			case 2: databuff[2] = 0x0;
					databuff[3] = 0x0;
					break;

			case 3: databuff[3] = 0x0;
					break;

			default: break;

		}
	}

	//value1 = get_current_time();
    adi_crc_SetCrcSeedVal(hCrcDev, 0xFFFFFFFF);
    if(data_length < 4)
        adi_crc_Compute (hCrcDev, (uint32_t *)databuff, 4);
    else
        adi_crc_Compute (hCrcDev, (uint32_t *)databuff, data_length);

    bool bCRCProgress=true;

    while(bCRCProgress == true)
    {
        adi_crc_IsCrcInProgress(hCrcDev, &bCRCProgress);
    }

    /* Get the final result */
    adi_crc_GetFinalCrcVal(hCrcDev, &remainder_1);
    crc = remainder_1 ^ 0xFFFFFFFF;

	// Append CRC to payload data
	databuff[data_length]   = (uint8_t) ((crc >> 0) & 0xff);
	databuff[data_length+1] = (uint8_t) ((crc >> 8 ) & 0xff);
	databuff[data_length+2] = (uint8_t) ((crc >> 16) & 0xff);
	databuff[data_length+3] = (uint8_t) ((crc >> 24) & 0xff);
}

/**
 * @brief Verify the 4 byte CRC in the received packet.
 *
 * @param[in ]   RxBuffer	 	Pointer to the receive data buffer.
 * @param[out]	 true or false  Boolean type true if received payload & crc is validated .
 *								false if not validated
 *
 * @return true or false
 *
 *
 * The initial remainder preset to all ones.The incoming bits of the received payload data and FCS,
 * when divided by G32(x) in the absence of transmission errors result in a unique nonzero
 * remainder value.
 * The unique remainder value is the polynomial shown:
 * x31 + x30 + x26 + x25 + x24 + x18 + x15 + x14 + x12 + x11 + x10 + x8 + x6 + x5 + x4+ x3 + x + 1
 * This is equivalent to 0xC704DD7B as unique value. The library function used to do this
 * polynomial division is GenerateCRC32 (). It uses three parameters namely pointer to received
 * payload buffer,length of payload buffer and remainder which is output of this function.
 * If the remainder of	division results in the unique number then this function returns value as
 * true else false.

 * If the length of the payload field is less than 4 bytes (octets), the received payload field is
 * appended with zero value octets to the most significant bits to make the payload field length
 * exactly 4 octets prior to computing the FCS for validation.
 *
 * @sa  Compute_4byte_CRC()
 */
//uint32_t value1, value2;
bool Verify_4byte_CRC(uint8_t *RxBuffer, uint16_t data_length, uint8_t start)
{
    uint32_t crc_calculated = 0;
    uint8_t *dataPtr;
    int i;
    dataPtr = RxBuffer;
    //value1 = get_current_time();
    if (start)
	{
		remainder_1 = 0xFFFFFFFFUL;

		//Get packet length from received frame.
		//data_length = ((dataPtr[0] & 0x7) << 8) | (dataPtr[1]);
		data_length-=6;

		// Handle specific case for packet length less than 4
		if(data_length < 4)
		{

			for( i = 4; i > 0; i--)
			{
				dataPtr[i+5] = dataPtr[data_length + i + 1];
			}

			switch(data_length)
			{
			case 1:	 dataPtr[3] = 0x0;
			dataPtr[4] = 0x0;
			dataPtr[5] = 0x0;
			break;

			case 2:
				dataPtr[4] = 0x0;
				dataPtr[5] = 0x0;
				break;

			case 3:  dataPtr[5] = 0x0;
			break;

			default : break;

			}
			data_length = 4;
		}
		data_length += 4;
		dataPtr += 2;
	}

    // Verify the CRC calculated
    crc_calculated = GenerateCRC32(dataPtr, data_length);

    /* The unique number specified in the specification (0xC704DD7B),
    can achieved by reversing the bits and complementing them from the crc calculated(0x2144DF1C) */
    //value2 = get_current_time();
    if(crc_calculated != 0x2144DF1C)
	return false;
    else
	return true;
}

//end of all function used for PATCH_802_15_4D
#endif

/*!
 * @brief Calculates four byte CRC using LUT method.
 *
 * @param[in]   inbuf[] Input Data bytes.
 * @param[in]   n     	Number of data bytes for which crc to be calculated
 *
 * @sa makecrctable2()
 *
 * Calculates the four byte crc for the input data passed in little endian order
 * using lookup table method.
 *
 * @return      CRC calculated.
 */
unsigned int CRC32(unsigned char *c, int n)
{
    while (--n >= 0)
	remainder_1 = (remainder_1 >> CHAR_BIT) ^ crctable[(byte)remainder_1 ^ *c++];

    return remainder_1 ^ 0xFFFFFFFFUL;
}

/**
 * @brief Reads PHY status till gets valid PHY status.
 *
 * @param[in]   hDevice    	Handle to the ADF7023 device.
 * @param[in] 	PhyState 	ADI_ADF7023_PHY_STATE_TYPE
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Reads PHY status and continue to read till gets valid PHY status.
 *
 * @sa adi_ADF7023_GetPhyState()
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_GetPhyState_PhyRx(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_PHY_STATE_TYPE* const PhyState)
{
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	ADI_ADF7023_STATUS_TYPE status;

	do
	{
	    uhf_result = ReadStatusReg(hDevice, &status);
		if(!uhf_result)
		{
        	*PhyState = status.fw_state;
#ifdef ADF7023_DEBUG
	        tranceiverState[*PhyState]++;
#endif
		}
	}while(!((*PhyState == PHY_SLEEP) ||
		  (*PhyState == PHY_OFF)||
		  (*PhyState == PHY_ON) ||
		  (*PhyState == PHY_TX) ||
		  (*PhyState == PHY_RX)

#ifdef 	PATCH_802_15_4D
		  ||(*PhyState == PHY_ON_15D4) ||
		  (*PhyState == PHY_TX_15D4) ||
		  (*PhyState == PHY_RX_CCA_15D4)
#endif
		  ));

	return uhf_result;

}

/**
 * @brief Configures MCR registers to set antenna path 0 for TX and 1 for RX.
 *
 * @param[in]   hDevice    	Handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Configure the MCR registers to set an antenna path 0 for TX and 1 for RX
 * when 15d4 firmware module is not used.
 * It mainly use three MCR registers 0x3F3, 0x3F4 and 0x3F8.
 * 0x3F8 MCR register is set to 0 to use Vdd  driver.
 * 0x3F4 MCR register is set to 0x20 to use ATB1 & ATB2
 * 0x3F3 MCR register is set to 0x03 to use antenna path 0 for TX
 * and antenna path 1 for RX.

 * @note This API is useful when 15d4 firmware is not loaded (e.g.  in case of SPORT mode)
 * and still need to select the antenna for TX or RX when ADF7023-JDB3Z eval. board
 * having two antenna support is used.
 *
 * @sa adi_ADF7023_Set_Antenna_Path_TX1_RX0()
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_Set_Antenna_Path_TX0_RX1(ADI_ADF7023_DEV_HANDLE const hDevice)
{
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	volatile uint8_t MCR_3f8_read,MCR_3f4_read,MCR_3f3_read;

	// writes to MCR registers
    // Clear 0x3F8 MCR register to  use Vdd driver
    uhf_result = Radio_write_reg(hDevice, 0x3F8, 0x0);

    // Set 0x3F4 MCR register to 0x20 to use ATB1 and ATB2
    if(!uhf_result)
    	uhf_result = Radio_write_reg(hDevice, 0x3F4, 0x20);

    // Set 0x3F3 MCR register to 0x03 to use antenna 1 for RX and antenna 0 for TX
    if(!uhf_result)
    	uhf_result = Radio_write_reg(hDevice, 0x3F3, 0x03);

#if 0
	//  Read from MCR registers for cross verification
    // Read 0x3F8 MCR register

    if(!uhf_result)
    	MCR_3f8_read = Radio_read_reg(hDevice, 0x3F8);
    if(MCR_3f8_read != 0x0)
    	uhf_result = ADI_ADF7023_ERR_UNKNOWN;


    // Read 0x3F4 MCR register
    if(!uhf_result)
    	MCR_3f4_read = Radio_read_reg(hDevice, 0x3F4);
    if(MCR_3f4_read != 0x20)
    	uhf_result = ADI_ADF7023_ERR_UNKNOWN;

    // Read 0x3F3 MCR register
    if(!uhf_result)
    	MCR_3f3_read = Radio_read_reg(hDevice, 0x3F3);
    if(MCR_3f3_read != 0x03)
    	uhf_result = ADI_ADF7023_ERR_UNKNOWN;
#endif

	return uhf_result;

}

/**
 * @brief Configures MCR registers to set antenna path 1 for TX and 0 for RX.
 *
 * @param[in]   hDevice    	Handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Configure the MCR registers to set an antenna path 1 for TX and 0 for RX
 * when 15d4 firmware module is not used.
 * It mainly use three MCR registers 0x3F3, 0x3F4 and 0x3F8.
 * 0x3F8 MCR register is set to 0 to use Vdd  driver.
 * 0x3F4 MCR register is set to 0x20 to use ATB1 & ATB2
 * 0x3F3 MCR register is set to 0x06 to use antenna path 0 for TX
 * and antenna path 1 for RX.

 * @note This API is useful when 15d4 firmware is not loaded (e.g.  in case of SPORT mode)
 * and still need to select the antenna for TX or RX when ADF7023-JDB3Z eval. board
 * having two antenna support is used.
 *
 * @sa adi_ADF7023_Set_Antenna_Path_TX0_RX1()
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_Set_Antenna_Path_TX1_RX0(ADI_ADF7023_DEV_HANDLE const hDevice)
{

	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	volatile uint8_t MCR_3f8_read,MCR_3f4_read,MCR_3f3_read;


		// writes to MCR registers
    // Clear 0x3F8 MCR register to  use Vdd driver
    uhf_result = Radio_write_reg(hDevice, 0x3F8, 0x0);

    // Set 0x3F4 MCR register to 0x20 to use ATB1 and ATB2
    if(!uhf_result)
    	uhf_result = Radio_write_reg(hDevice, 0x3F4, 0x20);

    // Set 0x3F3 MCR register to 0x06 to use antenna 0 for RX and antenna 1 for TX
    if(!uhf_result)
    	uhf_result = Radio_write_reg(hDevice, 0x3F3, 0x06);

#if 0
	//  Read from MCR registers for cross verification
    // Read 0x3F8 MCR register

    if(!uhf_result)
    	MCR_3f8_read = Radio_read_reg(hDevice, 0x3F8);
    if(MCR_3f8_read != 0x0)
    	uhf_result = ADI_ADF7023_ERR_UNKNOWN;


    // Read 0x3F4 MCR register
    if(!uhf_result)
    	MCR_3f4_read = Radio_read_reg(hDevice, 0x3F4);
    if(MCR_3f4_read != 0x20)
    	uhf_result = ADI_ADF7023_ERR_UNKNOWN;

    // Read 0x3F3 MCR register
    if(!uhf_result)
    	MCR_3f3_read = Radio_read_reg(hDevice, 0x3F3);
    if(MCR_3f3_read != 0x06)
    	uhf_result = ADI_ADF7023_ERR_UNKNOWN;
#endif

    return uhf_result;

}



#if 0
// Internal functions used for debug purpose

/**
 * @brief Change radio device to PHY OFF & reads the Program RAM memory.
 *
 * @param[in]   hDevice    	Handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Configure the radio device to PHY_OFF state and then issue command to read the Program RAM.
 *
 */
ADI_ADF7023_RESULT_TYPE readback_PRAM(ADI_ADF7023_DEV_HANDLE const hdevice)
{

    uint8_t ucVal;
    int timeout;
    uint8_t status;
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t pram_readback[2048];
    int i;

    uhf_result =  adi_ADF7023_SetPhyOff(hdevice); // Set radio in PHY_OFF mode
    while(!hdevice->bIsCmdRdy);
    //flip ROM/PRAM addressing on ADF7023
    if(!uhf_result)
    	uhf_result = Radio_write_reg(hdevice, 0x309, 1);

    timeout = 200000;
    while(timeout--);

    if(!uhf_result)
		uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_RAM_LOAD_DONE);

	timeout = 200000;
    while(timeout--);
    while(!hdevice->bIsCmdRdy);

    if(!uhf_result)
    	uhf_result = adi_ADF7023_PROM_BlockMemRead(hdevice, pram_readback, sizeof(IR_calbration_module_data));

    //restore ROM/PRAM addressing on ADF7023
    if(!uhf_result)
    	uhf_result = Radio_write_reg(hdevice, 0x309, 0);
    timeout = 200000;
    while(timeout--);

    if(!uhf_result)
 	uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_RAM_LOAD_DONE);

 	timeout = 200000;
	while(timeout--);
    while(!hdevice->bIsCmdRdy);

    //Check the PRAM read data with the IR calibration firmware data
    for(i=0;i<sizeof(IR_calbration_module_data); i++)
    {
    	if(pram_readback[i] != IR_calbration_module_data[i])
    		return ADI_ADF7023_ERR_IR_CAL_PRAM_DATA_READ;
    }
    return uhf_result;

}

// Internal functions used for debug purpose

/**
 * @brief Reads the Program RAM memory.
 *
 * @param[in]   hDevice    	Handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * This function makes sure that memory read command is specific to read
 * Program RAM. Configure the SPI buffer accordingly and then issue SPI read
 * command to read data from the Program RAM.
 *  *
 */


ADI_ADF7023_RESULT_TYPE adi_ADF7023_PROM_BlockMemRead (ADI_ADF7023_DEV_HANDLE const hDevice,
        uint8_t* const pDataRx, uint32_t const size)
{

	volatile uint32_t spi_result = ADI_SPI_RESULT_SUCCESS;

    do
    {
        adi_int_EnterCriticalRegion();
    	while(ADI_ADF7023_SPI_IDLE != hDevice->SpiState)
    	{
            adi_int_ExitCriticalRegion();
    	    if(true != hDevice->bBlockingMode)
                return ADI_ADF7023_ERR_RADIO_BUSY;

            adi_int_EnterCriticalRegion();
    	}
        if(ResetSpiBuffer(hDevice) != ADI_ADF7023_SUCCESS)
        {
            adi_int_ExitCriticalRegion();
            return ADI_ADF7023_ERR_SPI_BUSY;
        }

        hDevice->IssuedCmd = ADI_ADF7023_CMD_SPI_NOP;
        //we can only read PROM from addr 0.
        hDevice->SpiPrologue[0] = 0x3A;  //CreateMemcmd(ADI_ADF7023_CMD_SPI_MEM_RD, AdfAddr);
        hDevice->SpiPrologue[1] = 0xFF;  //(uint8_t)(AdfAddr & 0xff);
        hDevice->SpiPrologue[2] = (uint8_t)ADI_ADF7023_CMD_SPI_NOP;

        hDevice->SpiBuffer.pPrologue    = hDevice->SpiPrologue;
        hDevice->SpiBuffer.nDataSize    = size;
        hDevice->SpiBuffer.pTxData      = (uint8_t *)&hDevice->IssuedCmd;
        hDevice->SpiBuffer.pRxData      = pDataRx;
        hDevice->SpiBuffer.nPrologueSize = 3;
        hDevice->SpiBuffer.bTxIncrement = false;  // stationary transmit data pointer
        hDevice->SpiBuffer.bRxIncrement = true;   // dynamic receive data pointer
        hDevice->SpiBuffer.bBufferProcessed = false;


        /* first we need to make sure that the spi can take a buffer
         * poll on the spi result until there is one free buffer for
         * the spi driver to take */
        spi_result = adi_SPI_int_Transceiver(hDevice->pSpi, &hDevice->SpiBuffer);

        if(!spi_result)
        	hDevice->SpiState = ADI_ADF7023_SPI_RX;
        adi_int_ExitCriticalRegion();
    } while(ADI_SPI_RESULT_XFER_QUEUE_FULL == spi_result && true == hDevice->bBlockingMode);

    if(hDevice->bBlockingMode)
    {     /* fake non-blocking */

        if (spi_result != ADI_SPI_RESULT_SUCCESS)
        	return ADI_ADF7023_ERR_BLOCKWRITE;

        while(hDevice->SpiState != ADI_ADF7023_SPI_IDLE); /* wait till transfer is complete
                                         * hDevice->SpiBuffering, but put timeout here?? */
        hDevice->IssuedCmd = ADI_ADF7023_CMD_NULL;
    }
    else if(spi_result != ADI_SPI_RESULT_SUCCESS && hDevice->bBlockingMode == false)
    	return ADI_ADF7023_ERR_BLOCKWRITE;

    return ADI_ADF7023_SUCCESS;
}

#endif

#ifdef PATCH_802_15_4D
/**
 * @brief Configures MCR registers to enable or disable
 * Dynamic IF Filter in CCA Timer Mode
 *
 * @param[in] hDevice    			Handle to the ADF7023 device.
 *
 * @param[in] enableIFBWAutoSwitch 	Enable or Disable Dynamic IF Filter.
 *
 * @param[in] enableIFBWSwitchOnPreamble Enable or Disable IFBW switch
 * on preamble to decide the frequency to which device has to switch back.
 *
 * @param[in] cca_filter_bw   	 	CCA Filter Bandwidth value to be
 *									 switched dynamically
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Configures MCR registers to enable or disable
 * Dynamic IF Filter in CCA Timer Mode. It helps to vary the
 * frequency for CCA dynamically.
 *
 * It mainly use two MCR registers BB_cca_cfg_0 and BB_cca_cfg_1
 *
 * BB_cca_cfg_1 MCR register bit 5 for IFBWAutoSwitch and
 * bit 6 for IFBWSwitchOnPreamble.
 *
 * BB_cca_cfg_0 MCR register is set cca_filter_bw using bit 5 & 6.
 *
 */
ADI_ADF7023_RESULT_TYPE DynamicIFFilterSet_CCATimerMode(
								ADI_ADF7023_DEV_HANDLE const hdevice,
								uint8_t enableIFBWAutoSwitch,
								uint8_t enableIFBWSwitchOnPreamble,
								uint8_t cca_filter_bw)
{
	uint8_t regVal;
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

	cca_filter_bw = ((cca_filter_bw & 0x03) << CCA_FILTER_BIT_POS);

	/* Read the existing register value to retain its
	   existing contents. */
	regVal = Radio_read_reg(hdevice, BB_cca_cfg_0);

	/* update cca filter bw bits and write into register
	   BB_cca_cfg_0 */
	regVal |= cca_filter_bw;
	uhf_result = Radio_write_reg(hdevice, BB_cca_cfg_0, regVal);

	/* Read the existing register BB_cca_cfg_1 to retain its
	   existing contents. */
	regVal = Radio_read_reg(hdevice, BB_cca_cfg_1);

	if (!uhf_result)
	{
		/* Update the BB_cca_cfg_1 register bits to
		   enable/disable ENABLE_IFBW_AUTO_SWITCH */
		regVal |= (enableIFBWAutoSwitch <<
						ENABLE_IFBW_AUTO_SWITCH_BIT_POS);

		/* Update the BB_cca_cfg_1 register bits to
		   enable/disable ENABLE_IFBW_SWITCH_ON_PREAMBLE */
		regVal |= (enableIFBWSwitchOnPreamble <<
						ENABLE_IFBW_SWITCH_ON_PREAMBLE_BIT_POS);

		uhf_result = Radio_write_reg(hdevice, BB_cca_cfg_1, regVal);
	}
	return uhf_result;
}


/**
 * @brief Does the synthesizer calibration to perform
 *        fast tx/rx transitions
 *
 * @param[in] hDevice    			Handle to the ADF7023 device.
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Synthesizer calibration performed only during center frequency change and at regular intervals,
 * so that TRX state change from PHY_ON_15d4 to PHY_TX_15d4 and PHY_RX_CCA_15d4 can be performed faster.
 *
 */
ADI_ADF7023_RESULT_TYPE adi_ADF7023_Synthesizer_Calibration(ADI_ADF7023_DEV_HANDLE const hdevice)
{
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    uint8_t status, reg_val = MCR_SYNTH_CAL_OVERWRITE;

	uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_SYNTH_CAL_15D4);

    if(!uhf_result)
    {
    	while(!hdevice->bIsCmdRdy);

    	uhf_result = adi_ADF7023_BlockMemWrite(hdevice, &reg_val, &status, 1, VCO_OVRW_EN);
    }
    return uhf_result;
}
#endif

/**
 * @brief Enables/Disables a flag to indicate that Synthesizer
 *        Calibration is Pending.
 *
 * @param[in] hDevice    			Handle to the ADF7023 device.
 *
 * @param[in] value    				Zero 						- Disable
 * 									Non_Zero (positive values) 	- Enable
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * Synthesizer calibration has to be performed at regular intervals, this flag is used to notify driver that
 * synthesizer calibration is pending and needs to be performed.
 *
 */

void adi_ADF7023_EnableSynthCalPending(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t value)
{
	if( hDevice->FastTransitionsEnabled )
	{
		hDevice->synth_cal_pending = value;
	}
}

/**
 * @brief Enables/Disables Auto synthesizer calibration
 *
 * @param[in] hDevice    			Handle to the ADF7023 device.
 *
 * @param[in] state    				Zero 						- Disable auto synthesizer calibration
 * 									Non_Zero (positive values) 	- Enable auto synthesizer calibration
 *
 * @return ADI_ADF7023_RESULT_TYPE.
 *
 * When channel center frequency is changed or the transceiver state is changed (PHY_TX/PHY_RX) synthesizer
 * calibration is performed, if auto synthesizer calibration is enabled in VCO_OVRW_EN register (Address 0x3CD).
 * This function can be used to enable or disable Auto Synthesizer Calibration feature.
 */

ADI_ADF7023_RESULT_TYPE adi_ADF7023_15d4g_Enable_Disable_Auto_SynthCalibration(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t state)
{
	uint8_t vco_reg_val, status;
	ADI_ADF7023_RESULT_TYPE uhf_result;

	uhf_result = adi_ADF7023_BlockMemRead(hDevice, (uint8_t *)&vco_reg_val, 1, VCO_OVRW_EN);

	if(state)
	{
		vco_reg_val = 0;
	}
	else
	{
		vco_reg_val = 0x03;
	}

	uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &vco_reg_val, &status, 1, VCO_OVRW_EN);

	return uhf_result;
}
void adi_ADF7023_15d4g_Reset_vars(void)
{
    AdfpreviousIntSource = 0;
    buff_allocated = 0;
}

ADI_ADF7023_RESULT_TYPE adi_ADF7023_154D_Patch_load(ADI_ADF7023_DEV_HANDLE hdevice)
{
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	uint8_t status;

    // We need to be in PHY OFF state
    uhf_result =  adi_ADF7023_SetPhyOff(hdevice); // Set radio in PHY_OFF mode

    adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

    //if(!uhf_result)
	//uhf_result =  adi_ADF7023_ReInit(hdevice, 0);

    // We need to be in PHY OFF state
    //if(!uhf_result)
	//uhf_result =  adi_ADF7023_SetPhyOff(hdevice); // Set radio in PHY_OFF mode

    //adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

    if(!uhf_result)
	uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_RAM_LOAD_INIT);


    hdevice->bIsCmdRdy = true;  // Explictly changing the status of the flag

    if(!uhf_result)
	uhf_result = adi_ADF7023_BlockMemWrite(hdevice, (unsigned char *)patch_802_15_4_d, &status, sizeof(patch_802_15_4_d), 0x600);


    if(!uhf_result)
	uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_RAM_LOAD_DONE);

    adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

    if(!uhf_result)
	uhf_result = adi_ADF7023_IssueCmd(hdevice,ADI_ADF7023_CMD_SYNC);

    adi_ADF7023_WaitforPhyStateReady(hdevice, PHY_OFF);

    hdevice->PhyState  = PHY_OFF;

    return uhf_result;
}

ADI_ADF7023_RESULT_TYPE adi_ADF7023_Wakeup(ADI_ADF7023_DEV_HANDLE const hDevice)
{

    volatile ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    volatile uint64_t timeout = 2000;
    ADI_ADF7023_STATUS_TYPE status;


    //hopefully it's in PHY_SLEEP at this point
    uhf_result = PhyWakeUp(hDevice);

    do
    {
        timeout--;
    }while(0 < timeout);

    do
    {
        uhf_result = ReadStatusReg(hDevice, &status);
    }while(status.fw_state != PHY_OFF && !uhf_result);             //wait for PHY_OFF state


    if(!uhf_result)
        uhf_result = adi_ADF7023_SetPhyOff(hDevice);

    if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    if(!uhf_result)
        uhf_result = adi_ADF7023_SetPhyOn(hDevice);

    //adi_ADF7023_154D_PatchRoutine( hDevice);
    if(!uhf_result)
        uhf_result = adi_ADF7023_154D_Patch_load(hDevice);

    if(!uhf_result)
        uhf_result = adi_ADF7023_config_15d4_BBRAM_Regs(hDevice);

#ifdef ANTENNA_DIVERSITY
    if(!uhf_result)
        uhf_result = ADF7023_15d4g_TxAntennaConfig(2,1,ADI_ADF7023J_15D4_VDD_DRV,ADI_ADF7023J_15D4_ANTENNA0);
    if(!uhf_result)
        uhf_result = ADF7023_15d4g_RxAntennaDivConfig(2,1,ADI_ADF7023J_15D4_VDD_DRV,
                                     ADI_ADF7023J_15D4_AD_DISABLE,ADI_ADF7023J_15D4_ANTENNA0);
#endif //ANTENNA_DIVERSITY

    if(!uhf_result)
        hDevice->InitState = ADI_ADF7023_STATE_INITIALIZED;

    return uhf_result;

}

ADI_ADF7023_RESULT_TYPE adi_ADF7023_sequencial_Wakeup(ADI_ADF7023_DEV_HANDLE const hDevice, uint8_t sequence)
{

    volatile ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
    volatile uint64_t timeout = 2000;
    ADI_ADF7023_STATUS_TYPE status;

    if (sequence == 1)
    {
        //hopefully it's in PHY_SLEEP at this point
        uhf_result = PhyWakeUp(hDevice);

        do
        {
            timeout--;
        }while(0 < timeout);

        do
        {
            uhf_result = ReadStatusReg(hDevice, &status);
        }while(status.fw_state != PHY_OFF && !uhf_result);             //wait for PHY_OFF state


        if(!uhf_result)
            uhf_result = adi_ADF7023_SetPhyOff(hDevice);

        if(!uhf_result)
            uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);
    }

    if(sequence == 2)
    {
        uhf_result = adi_ADF7023_SetPhyOn(hDevice);
    }


    if(sequence == 3)
    {
        if(!uhf_result)
            uhf_result = adi_ADF7023_154D_Patch_load(hDevice);

        if(!uhf_result)
            uhf_result = adi_ADF7023_config_15d4_BBRAM_Regs(hDevice);

#ifdef ANTENNA_DIVERSITY
        if(!uhf_result)
            uhf_result = ADF7023_15d4g_TxAntennaConfig(2,1,ADI_ADF7023J_15D4_VDD_DRV,ADI_ADF7023J_15D4_ANTENNA0);
        if(!uhf_result)
            uhf_result = ADF7023_15d4g_RxAntennaDivConfig(2,1,ADI_ADF7023J_15D4_VDD_DRV,
                                         ADI_ADF7023J_15D4_AD_DISABLE,ADI_ADF7023J_15D4_ANTENNA0);
#endif //ANTENNA_DIVERSITY

        if(!uhf_result)
            hDevice->InitState = ADI_ADF7023_STATE_INITIALIZED;
    }

    return uhf_result;

}

void adi_ADF7023_Sleep(ADI_ADF7023_DEV_HANDLE const hDevice)
{
    unsigned char piVal, status = 0x08;
    ADI_ADF7023_RESULT_TYPE uhf_result;

    uhf_result = adi_ADF7023_SetPhyOn(hDevice);

    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &piVal, &status, 1, MCR_CONFIG_LOW);

    piVal = 0x00;
    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, &piVal, &status, 1, MCR_CONFIG_HIGH);

    hDevice->InitState = ADI_ADF7023_STATE_UNKNOWN;

    uhf_result =adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_PHY_SLEEP);
}
bool continuousRx_status(void)
{
    return continuousRx;
}

uint32_t get_rx_duration(void)
{
    return rx_duration;
}

/*
 ** EOF
 */