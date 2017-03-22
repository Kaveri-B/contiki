/***********************************************************************************************************************

Copyright (c) 2013 - 2015, Analog Devices, Inc.  All rights reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.

***********************************************************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "common.h"
#include "ADF7023.h"
#include <gpio/adi_gpio.h>
#include "dev/aducm3029-radio.h"
#include <uart/adi_uart.h>
#include "utest_support.h"
#include "limits.h"
#include <spi/adi_spi.h>
#include "ssdd_common/common_def.h"
#include "adi_spi_def_v1.h"
#include <crc/adi_crc.h>

#ifndef DISABLE_PATCH_802_15_4D
#include "rom_ram_7023_2_2_AD_15d4g_R4p3.dat"
#endif // DISABLE_PATCH_802_15_4D

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

#define SPI_CLK_FREQ                        4000000                           // 4MHz SPI CLK
#define UCLK                                16000000                          // 16MHz UCLK
#define DEF_HW_WAIT_PERIOD                  100000
#define DEF_INIT_HW_WAIT_1MS                4000                              // A little over 1ms
#define DEF_INIT_HW_WAIT_2_5MS              10000
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
#define BBRam_MMemmap_Start                 0x100
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

#define PKT_RAM_SIVER                       0x01
#define ANODE_NO_OF_PROFILES                6

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

void GetPHYRX_RSSI(signed char* RSSIVal);

#ifdef DISABLE_PATCH_802_15_4D
const U8 ADF_PHYProfiles[ANODE_NO_OF_PROFILES][64] =
{
  { 0x16,0x00,0x00,0x00,0xFF,0xFF,0x05,0x33,0x31,0x3B,0xB1,0x23,0x0A,0x00,0x64,0x41,  // 1kbps
  0x01,0x00,0x5A,0xC3,0xFD,0x00,0x0B,0x37,0x16,0x07,0x40,0x09,0x00,0x0C,0x00,0x00,
  0x10,0x89,0x1A,0x2B,0x10,0x10,0x24,0x83,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  { 0x16,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x31,0x76,0x62,0x21,0x6C,0x00,0xD8,0x82, // 10kbps, gfsk
  0x04,0x00,0x03,0xC0,0x9D,0x08,0x0B,0x37,0x16,0x07,0x40,0x09,0x08,0x08,0x00,0x00,
  0x18,0x89,0x1A,0x2B,0x10,0x10,0x24,0x83,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  { 0x16,0x00,0x00,0x00,0xFF,0xFF,0x05,0x33,0x31,0x3B,0xB1,0x23,0x80,0x01,0xC0,0x20,  // 38.4kbps
  0x0E,0x00,0x58,0xC3,0xFD,0x00,0x0B,0x37,0x16,0x07,0x40,0x09,0x00,0x0C,0x00,0x00,
  0x10,0x89,0x1A,0x2B,0x10,0x10,0x24,0x83,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  {0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x31, 0x89, 0x9d, 0x22, 0xf4, 0x11, 0xf4, 0x20,
  0x14, 0x20, 0x58, 0xc3, 0xff, 0x40, 0x07, 0x4a, 0x00, 0x00, 0x40, 0x0c, 0x20, 0x04, 0x80, 0x05,
  0x08, 0x00, 0x00, 0xa7, 0x10, 0x10, 0x24, 0x87, 0x00, 0x01, 0x02, 0x01, 0xff, 0xff, 0xff, 0x0f,
  0xff, 0x0f, 0xff, 0x50, 0xff, 0x61, 0xff, 0x00, 0xa7, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00},
  { 0x16,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x31,0x76,0x62,0x21,0xE8,0x03,0xFA,0x31,  // 100kbps, gfsk
  0x26,0x00,0x01,0xC0,0x9D,0x08,0x0B,0x37,0x16,0x07,0x40,0x09,0x08,0x08,0x00,0x00,
  0x18,0x89,0x1A,0x2B,0x10,0x10,0x24,0x83,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  { 0x16,0x00,0x00,0x00,0xFF,0xFF,0x05,0x33,0x31,0x3B,0xB1,0x23,0xB8,0x2B,0xEE,0x16,  // 300kbps
  0x72,0x0A,0x58,0xC3,0xFD,0xC0,0x0B,0x37,0x16,0x07,0x40,0x09,0x00,0x0C,0x00,0x00,
  0x10,0x89,0x1A,0x2B,0x10,0x10,0x24,0x83,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }
};
#else
const U8 ADF_PHYProfiles[ANODE_NO_OF_PROFILES][64] =
{
  { 0xfe,0x00,0x00,0x00,0xFF,0xFF,0x05,0x33,0x31,0x3B,0xB1,0x23,0x0A,0x00,0x64,0x41,  // 1kbps
  0x01,0x00,0x5A,0xC3,0xFD,0x00,0x0B,0x37,0x16,0x07,0x40,0x09,0x00,0x0C,0x00,0x00,
  0x10,0x89,0x1A,0x2B,0x10,0x10,0x24,0x83,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  { 0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x31,0x76,0x62,0x21,0x6C,0x00,0xD8,0x82, // 10kbps, gfsk
  0x04,0x00,0x03,0xC0,0x9D,0x08,0x0B,0x37,0x16,0x07,0x40,0x09,0x08,0x08,0x00,0x00,
  0x18,0x89,0x1A,0x2B,0x10,0x10,0x24,0x83,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  { 0xfe,0x00,0x00,0x00,0xFF,0xFF,0x05,0x33,0x31,0x3B,0xB1,0x23,0x80,0x01,0xC0,0x20,  // 38.4kbps
  0x0E,0x00,0x58,0xC3,0xFD,0x00,0x0B,0x37,0x16,0x07,0x40,0x09,0x00,0x0C,0x00,0x00,
  0x10,0x89,0x1A,0x2B,0x10,0x10,0x24,0x83,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  {0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x31, 0x89, 0x9d, 0x22, 0xf4, 0x11, 0xf4, 0x20, // 50kbps, gfsk
  0x14, 0x20, 0x00, 0xC0, 0xFF, 0x08, 0x0b, 0x37, 0x00, 0x00, 0x40, 0x0c, 0x28, 0x0A, 0x80, 0x05,
  0x10, 0x00, 0x00, 0xa7, 0x20, 0x20, 0x24, 0x87, 0x00, 0x01, 0x02, 0x01, 0xff, 0xff, 0xff, 0x0f,
  0xff, 0x0f, 0xff, 0x50, 0xff, 0x61, 0xff, 0x00, 0xa7, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00},
  { 0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x31,0x76,0x62,0x21,0xE8,0x03,0xFA,0x31,  // 100kbps, gfsk
  0x26,0x00,0x01,0xC0,0x9D,0x08,0x0B,0x37,0x16,0x07,0x40,0x09,0x08,0x08,0x00,0x00,
  0x18,0x89,0x1A,0x2B,0x10,0x10,0x24,0x83,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  { 0xfe,0x00,0x00,0x00,0xFF,0xFF,0x05,0x33,0x31,0x3B,0xB1,0x23,0xB8,0x2B,0xEE,0x16,  // 300kbps
  0x72,0x0A,0x58,0xC3,0xFD,0xC0,0x0B,0x37,0x16,0x07,0x40,0x09,0x00,0x0C,0x00,0x00,
  0x10,0x89,0x1A,0x2B,0x10,0x10,0x24,0x83,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }
};
#endif // DISABLE_PATCH_802_15_4D

extern uint8_t transmit_pwr;
extern uint16_t phy_current_channel;
extern uint32_t system_hibernate_duration;
extern uint32_t system_flexi_mode_sleep_duration;

const U16 ADF_PHY_PATO[ANODE_NO_OF_PROFILES] = { 1500, 200, 75, 60, 60, 60 };
const U16 ADF_PHY_PackTO[ANODE_NO_OF_PROFILES] = { 12850, 1330, 380, 310, 180, 100};

uint32_t rem;
uint32_t TickCounter = 0;
volatile uint32_t pkt_recv_err_timeout = 0;
volatile uint32_t pkt_transmit_err_timeout = 0xFFFFFFFF;
uint32_t pkt_Tx_start_time;
uint32_t pkt_Rx_start_time;
uint32_t pkt_cca_start_time;

volatile bool pkt_recv_err_timeout_set = false;
volatile bool pkt_transmit_err_timeout_set = false;

const agc_gain_map agc_gain_map_table[] =
{
  {0x00 , 44},
  {0x01 , 35},
  {0x02 , 26},
  {0x0A , 17},
  {0x12 , 10},
  {0x16 , 0}
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

typedef struct
{
  VU32 AdrH     :  3;   // 11 bit address
  VU32 Cmd       : 5;   // 5 bit command
  VU32 AdrL     :  8;   // 11 bit address
}ADF_MEMCMD;

typedef union
{
  unsigned char ucBytes[0x4];
  VU32          Reg;
  ADF_MEMCMD    Bits;
}ADFSTA_MemCmd;


// The firmware state variable

ADFSTA_Reg        mADF7023Status;
U8                mProfileNo = 0;
TyBBRAM           gBBRAM;
unsigned char     mIntState;
unsigned char     mSiVer[4];
ADF_CmdCodes    mPHYState = (ADF_CmdCodes)0;
IRQn_Type eIrq;

#ifndef DISABLE_PATCH_802_15_4D
ADF_FwState mPHYState_15D4;
#endif // DISABLE_PATCH_802_15_4D

unsigned char 	ADF_WaitFWState(ADF_FwState FWState);
unsigned char   ADF_WaitSpiRdy(void);
unsigned char   ADF_MMapWrite(unsigned long ulAdr,unsigned int ulLen,unsigned char *pData);
unsigned char   ADF_GoToOnState(void);
unsigned char   ADF_SyncComms       (void);
unsigned char   ADF_XMit            (unsigned char ucByte,unsigned char *pData);
unsigned char   ADF_SPI_XMIT(uint8_t *TxData, uint8_t *RxData, uint16_t DataLen, uint8_t *Prologue, uint16_t PrologueSize);
unsigned char   ADF_IssueCommandNW  (unsigned char Cmd);
unsigned char   ADF_IssueCommand(unsigned char Cmd);
unsigned char   ADF_WaitCmdLdr      (void);
void 		ADF_SetChannelFreq(TyBBRAM *pBBRAM,unsigned long ulChannelFreq);
void 		ADF_SetDataRate(TyBBRAM *pBBRAM, unsigned long ulDataRate);
boolean         ADF_ReadRSSI(unsigned char SPICmd, unsigned char *RSSIVal);
static unsigned char WaitForPowerUp(void);
boolean         ADF_IssueCommandNWDataback(unsigned char Cmd, unsigned char *pData);
void            InitADF7023Ports(void);
unsigned char   ADF_ReadStatus(ADFSTA_Reg *pStatus);
U16             ADF_GetPATTO(void) { return ADF_PHY_PATO[mProfileNo]; }
U16             ADF_GetPackTO(void) { return ADF_PHY_PackTO[mProfileNo]; }
static void     ADF7023_ReInit(void *ptr);
unsigned char   ADF7023_15d4g_ReInit(void);


unsigned char* ADF_GetSiVer(void) { return mSiVer; }
unsigned char ADF_GetMSBSiVer(void) { return mSiVer[2]; }
unsigned char ADF_GetNoPHYProfiles(void) { return ANODE_NO_OF_PROFILES; }
rf_statistics_t rf_stats;
void          Adf7023_Callback(void *ClientCallback, uint32_t Event, void *pArg);
static unsigned char SetupIrqFlag(IRQn_Type Flag);

volatile uint8_t bTxDone;

#ifndef DISABLE_PATCH_802_15_4D
ADI_ADF7023_BUFFER_TYPE TRxFrame = {0, 0, NULL};
#endif // DISABLE_PATCH_802_15_4D

uint8_t spidevicemem[ADI_SPI_MEMORY_SIZE];

static uint8_t gpioMemory[ADI_GPIO_MEMORY_SIZE];

/* Memory to handle CRC Device */
static uint8_t          CrcDevMem[ADI_CRC_MEMORY_SIZE];

ADI_SPI_HANDLE hSpi;
/* CRC Device Handle */
static ADI_CRC_HANDLE   hCrc;
#if (I_SUPPORT_4BYTE_FCS == 0x01)
uint8_t Verify_4byte_CRC(uint8_t *RxBuffer, uint16_t data_length);
uint8_t bCRC_Correct_Status;
#endif // I_SUPPORT_4BYTE_FCS

int adi_ADF7023_IssueCmd(unsigned char const cmd);
/*----------------------------------------------------------------------------------------*/
/* This procedure is called in the main thread. It monitors the state of the SRD link and */
/* processes any packages received.							  */
/*----------------------------------------------------------------------------------------*/
#ifdef DISABLE_PATCH_802_15_4D
PHYEVENTS GetPHYEventStatus(unsigned char *PackBuffer)
{
  U8 len;

  if(mIntState & interrupt_mask_0_interrupt_crc_correct)
  {
    mIntState = PHY_NOEVENT;
    pADI_IDU->EI2CFG = 0x00;
    ADF_MMapRead(PKT_RAM_BASE_RXPTR, 1, &len);

    if(len > PKT_MAX_PAYLOAD_LEN)
      len = PKT_MAX_PAYLOAD_LEN - 1;

    //        packetbuf_set_datalen (len - SIZE_LEN_FIELD - SIZE_CRC_FIELD );
    //        ADF_MMapRead((PKT_RAM_BASE_RXPTR + 1), (len - SIZE_LEN_FIELD - SIZE_CRC_FIELD), PackBuffer);
    //        packetbuf_set_datalen (len - SIZE_LEN_FIELD);

    ADF_MMapRead( PKT_RAM_BASE_RXPTR, len, PackBuffer );

    pADI_IDU->EI2CFG = EI2CFG_IRQ8MDE_HIGHLEVEL | EI2CFG_IRQ8EN_EN;
    return PHY_RX_CRCOK;
  }

  if(mIntState & interrupt_mask_0_interrupt_sync_detect)
  {
    mIntState = PHY_NOEVENT;
    return PHY_SYNCHDET;
  }

  if(mIntState & interrupt_mask_0_interrupt_tx_eof)
  {
    mIntState = PHY_NOEVENT;

    bTxDone = 1;
    return PHY_TX_END;
  }

  return PHY_NOEVENT;
}

uint8_t adi_ADF7023_ReceiveFrame(void)
{
  if(ADF_IssueCommand(CMD_PHY_RX)) return TRE_TO_RX_STATE;
}

#else

PHYEVENTS GetPHYEventStatus_15D4(unsigned char *PackBuffer)
{
  U8 len;

  if(mIntState & interrupt_mask_0_interrupt_buffer_almost_full)
  {
    if( mPHYState_15D4 == FW_TX_15D4 )
      cbTxBufferAlmostFull( );
    else if( mPHYState_15D4 == FW_RX_CCA_15D4 )
      cbRxBufferAlmostFull(PackBuffer);
  }

  if(mIntState & interrupt_mask_0_interrupt_buffer_full)
  {
    if( mPHYState_15D4 == FW_TX_15D4 )
      cbTxBufferFull( );
    else if( mPHYState_15D4 == FW_RX_CCA_15D4 )
      cbRxBufferFull(PackBuffer);
  }

  if(mIntState & interrupt_mask_0_phr_detect)
  {
    mIntState = PHY_NOEVENT;
    /* start timer for time required to receive maximum packet size (2048 bytes) */
    pkt_recv_err_timeout = PACKET_RCV_TIME + TickCounter;
    pkt_recv_err_timeout_set = true;
    rf_stats.PHR_detect++;
    ADF7023_ReadAntennaRSSI(&rf_stats.RSSI_val);
    return PHY_PHR_DETECT;
  }

  if(mIntState & interrupt_mask_0_interrupt_premable_detect)
  {
    mIntState = PHY_NOEVENT;
    return PHY_PREAMBLE_DETECT;
  }

  if(mIntState & interrupt_mask_0_interrupt_cca)
  {
    uint8_t  cca_config_value;

    rf_stats.cca_time += ((TickCounter - pkt_cca_start_time));

    ADF_MMapRead(BB_CCA_CFG_0, 1, &cca_config_value);

    // Check if CCA status is busy or clear bit 7
    cca_config_value &= 0x80;
    if(cca_config_value == 0x80)
    {
      utu_timestamp(ADF7023_CCA_BUSY,0);
      // Channel busy even though timer expired will remain in RX state
      mPHYState_15D4 = FW_RX_CCA_15D4;
      bTxDone = 1;
      rf_stats.CCA_Failed++;

    }
    else if(cca_config_value == 0)
    {
      // Channel is clear, so auto TX will happen automatically.
      // So change the phystate to TX to use TX related functions.
      mPHYState_15D4 = FW_TX_15D4;
      pkt_Tx_start_time = TickCounter;
    }

    //      mIntState = PHY_NOEVENT;
    //      return PHY_CCA;
  }

  if(mIntState & interrupt_mask_0_interrupt_tx_eof)
  {
    utu_timestamp(ADF7023_TX_EOF,0);
    mIntState = PHY_NOEVENT;
    mPHYState_15D4 = FW_ON_15D4;
    rf_stats.Tx_EOF++;

    bTxDone = 1;

    TRxFrame.Remaining.Number_Of_Bytes_To_Send = TRxFrame.Accomplished.Number_Of_Bytes_Sent = 0;

    //Each tick is 4ms and we took (TickCounter - pkt_Tx_start_time) ticks to transmit.
    rf_stats.pkt_Tx_time += ((TickCounter - pkt_Tx_start_time));
    pkt_Tx_start_time = 0;

    return PHY_TX_EOF;
  }

  if(mIntState & interrupt_mask_0_interrupt_rx_eof)
  {
    utu_timestamp(ADF7023_RX_EOF,0);
    pkt_recv_err_timeout_set = false;
    rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
    pkt_Rx_start_time = 0;

    if (cb154dPacketReceived(PackBuffer) == SUCCESS)
    {
      TRxFrame.Remaining.Number_Of_Bytes_To_Receive = TRxFrame.Accomplished.Number_Of_Bytes_Received = 0;

      mPHYState_15D4 = FW_ON_15D4;

      mIntState = PHY_NOEVENT;
      rf_stats.Rx_EOF++;
      return PHY_RX_EOF;
    } else {
      TRxFrame.Remaining.Number_Of_Bytes_To_Receive = TRxFrame.Accomplished.Number_Of_Bytes_Received = 0;
      mPHYState_15D4 = FW_ON_15D4;
      mIntState = PHY_NOEVENT;
      adi_ADF7023_Receive_15d4_Frame();
    }
  }

  return PHY_NOEVENT;
}

#endif // DISABLE_PATCH_802_15_4D
/*------------------------------------------------------------------------*/
/* Change the transceiver settings here. Takes 5.5ms with W1=0 and W2=500 */
/*------------------------------------------------------------------------*/
unsigned char ADF_RadioInit(unsigned char ProfileNo, ISMBAND ISMBand, U32 Freq, U8 InitParm)
{
  unsigned int n;
  unsigned char ucInt0 = 0xFF;

  NVIC_DisableIRQ(SYS_GPIO_INTA_IRQn);

  if(ProfileNo >= ANODE_NO_OF_PROFILES) ProfileNo = ANODE_DEFAULT_PROFILE;
  mProfileNo = ProfileNo;
  if(InitParm & INIT_TRSC_PORTS)
  {
    InitADF7023Ports();

    UHF_FirstConnect();

    for(n=0; n<DEF_INIT_HW_WAIT_1MS; n++);

    if(ADF_WaitCmdLdr())
      return TRE_WAIT_CMD;

    ADF_MMapRead(PKT_RAM_SIVER, 4, mSiVer);
    if(ADF_IssueCommand(CMD_PHY_OFF))
      return TRE_TO_OFF_STATE;

    for(n=0; n<DEF_INIT_HW_WAIT_1MS; n++);

  }
  if(InitParm & INIT_TRSC_CNGPHY)
  {
    if(ADF_IssueCommand(CMD_PHY_ON))
      return TRE_TO_ON_STATE;
    if(ADF_WaitCmdLdr())
      return TRE_WAIT_CMD;
  }
  memcpy((void *)&gBBRAM, (void *)ADF_PHYProfiles[ProfileNo], sizeof(gBBRAM));

  if(ISMBand == ISMBAND_433MHZ)
  {
    gBBRAM.image_reject_cal_phase_r = 0x08;
    gBBRAM.image_reject_cal_amplitude_r = 0x03;
  }
  ADF_SetChannelFreq(&gBBRAM, Freq);
  if(ADF_MMapWrite(BBRam_MMemmap_Start, sizeof(gBBRAM), (unsigned char *)&gBBRAM))
    return TRE_MAPWR_CMD;

  SetupIrqFlag(SYS_GPIO_INTA_IRQn);
#ifndef DISABLE_PATCH_802_15_4D
  adi_ADF7023_154D_PatchRoutine( );

  adi_ADF7023_config_15d4_BBRAM_Regs( );

#ifdef ANTENNA_DIVERSITY
  ADF7023_15d4g_TxAntennaConfig(2,1,ADI_ADF7023J_15D4_VDD_DRV,ADI_ADF7023J_15D4_ANTENNA0);

  ADF7023_15d4g_RxAntennaDivConfig(2,1,ADI_ADF7023J_15D4_VDD_DRV,
                                   ADI_ADF7023J_15D4_AD_DISABLE,ADI_ADF7023J_15D4_ANTENNA0);
#endif //ANTENNA_DIVERSITY

#endif // DISABLE_PATCH_802_15_4D

  if(ADF_IssueCommand(CMD_CONFIG_DEV)) return TRE_CFG_CMD;

#ifdef DISABLE_PATCH_802_15_4D
  if(ADF_GoToOnState()) return TRE_TO_ON_STATE;
#endif // DISABLE_PATCH_802_15_4D

  NVIC_ClearPendingIRQ(SYS_GPIO_INTA_IRQn);

  ADF_MMapWrite(MCR_interrupt_source_0_Adr, 0x1, &ucInt0);

  NVIC_EnableIRQ(SYS_GPIO_INTA_IRQn);

  adi_crc_Open (0, &CrcDevMem[0], ADI_CRC_MEMORY_SIZE, &hCrc);

#if ((DEV_TYPE == DEV_TYPE_HOST) || (DEV_TYPE == DEV_TYPE_EDGE_ROUTER) || (DEV_TYPE == DEV_TYPE_ROUTER))
#ifdef DISABLE_PATCH_802_15_4D
  if(InitParm & INIT_TRSC_RX) if(ADF_IssueCommandNW(CMD_PHY_RX)) return TRE_TO_RX_STATE;
#else
  adi_ADF7023_Receive_15d4_Frame( );
#endif // DISABLE_PATCH_802_15_4D
#endif //((DEV_TYPE == DEV_TYPE_HOST) || (DEV_TYPE == DEV_TYPE_EDGE_ROUTER) || (DEV_TYPE == DEV_TYPE_ROUTER))

  SysTick_Config(MAX_SYS_TICK_COUNT);

  return TRE_NO_ERR;
}

/*----------------------------------------------*/
/* Power down the transceiver to various states */
/*----------------------------------------------*/
unsigned char ADF_PwrMgt(TRPSSTATE PDState)
{
  unsigned char piVal = 0x08;
  unsigned int n;
  if(PDState == TRPS_SHUTDOWN)
  {
    if(ADF_IssueCommand(CMD_HW_RESET)) return TRE_TO_SLEEP_STATE;

    if(pkt_Rx_start_time)
    {
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
      pkt_Rx_start_time = 0;
    }
  }

  if(PDState == TRPS_DOWN)
  {
    if(pkt_Rx_start_time)
    {
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
      pkt_Rx_start_time = 0;
    }

#ifndef DISABLE_PATCH_802_15_4D
    if(ADF_GoTo15D4PHYOnMode()) return TRE_TO_15D4_ON_STATE;
    if(ADF_Exit15D4()) return TRE_TO_ON_FROM_15D4_ON_STATE;
#else
    if(ADF_IssueCommand(CMD_PHY_ON)) while(1)return TRE_TO_ON_STATE;
    if(ADF_WaitFWState(FW_ON)) return TRE_WAIT_A_STATE;
#endif

    if(ADF_MMapWrite(MCR_CONFIG_LOW, 1, (unsigned char *)&piVal)) return TRE_MAPWR_CMD;
    piVal = 0x00;
    if(ADF_MMapWrite(MCR_CONFIG_HIGH, 1, (unsigned char *)&piVal)) return TRE_MAPWR_CMD;
    if(ADF_IssueCommand(CMD_PHY_SLEEP)) return TRE_TO_SLEEP_STATE;

    mPHYState_15D4 = FW_SLEEP;
  }
  if(PDState == TRPS_UP)
  {
    if(WaitForPowerUp()) return TRE_PWRUP;                                // Now in PHY off
    if(ADF_WaitFWState(FW_OFF)) return TRE_PWRUP;
    for(n=0; n<SECDLY; n++);                                                 // To get around a bug in the ADF7023 (ver 2)
    if(ADF_WaitCmdLdr()) return TRE_WAIT_CMD;
    if(ADF_IssueCommand(CMD_CONFIG_DEV)) return TRE_CFG_CMD;
    if(ADF_WaitCmdLdr()) return TRE_WAIT_CMD;
    if(ADF_GoToOnState()) return TRE_TO_ON_STATE;

#ifndef DISABLE_PATCH_802_15_4D
    adi_ADF7023_154D_PatchRoutine( );
    adi_ADF7023_config_15d4_BBRAM_Regs();

#ifdef ANTENNA_DIVERSITY
    ADF7023_15d4g_TxAntennaConfig(2,1,ADI_ADF7023J_15D4_VDD_DRV,ADI_ADF7023J_15D4_ANTENNA0);
    ADF7023_15d4g_RxAntennaDivConfig(2,1,ADI_ADF7023J_15D4_VDD_DRV,
                                     ADI_ADF7023J_15D4_AD_DISABLE,ADI_ADF7023J_15D4_ANTENNA0);
#endif //ANTENNA_DIVERSITY
#endif // DISABLE_PATCH_802_15_4D

    if(mPHYState_15D4 != FW_ON_15D4)
      return TRE_PWRUP;

  }
  return TRE_NO_ERR;
}

/*---------------------------------------------------------*/
/* Do an RSSI measurement when the PHY is in the on state. */
/*---------------------------------------------------------*/
unsigned char GetPHYON_RSSI(signed char* RSSIVal)
{
  if(ADF_GoToOnState()) return TRE_TO_ON_STATE;
  if(ADF_IssueCommand(CMD_GET_RSSI)) return TRE_GET_RSSI;
  if(ADF_WaitFWState(FW_RSSI)) return TRE_WAIT_CMD;
  if(ADF_IssueCommand(CMD_SYNC)) return TRE_SYNC_CMD;
  if(ADF_WaitCmdLdr()) return TRE_WAIT_CMD;
  *RSSIVal = GetRSSIReadBackReg();
  return TRE_NO_ERR;
}

/*---------------------------------------------*/
/* Get the RSSI value in the readback register */
/*---------------------------------------------*/
signed char GetRSSIReadBackReg(void)
{
  signed char RegVal;
  ADF_MMapRead(0x0312, 0x01, (unsigned char *)&RegVal);    // Read the RSSI readback reg
  return (signed char)((int)RegVal - 107);               // and convert to dBm
}

unsigned char Map_agc_gain_correction(unsigned char agc_gain_settings)
{
  unsigned char i;
  for( i = 0; i < (sizeof(agc_gain_map_table) / sizeof (agc_gain_map)); i++)
  {
    if( agc_gain_map_table[i].gain_status == agc_gain_settings)
      break;
  }

  if(i == (sizeof(agc_gain_map_table) / sizeof (agc_gain_map)))
    return TRE_CONFIG_ERR;

  return agc_gain_map_table[i].gain_correction;
}
/*---------------------------------------------------------*/
/* Do an RSSI measurement using Method3 */
/*---------------------------------------------------------*/
void GetPHYRX_RSSI(signed char* RSSIVal)
{
  unsigned char agc_mode_value = HOLD_AGC;
  unsigned char agc_gain_settings = 0x00;
  unsigned char adc_readback[2];
  unsigned char adc_readback_value = 0x00;
  unsigned char gain_correction = 0x00;

  /* check whether the Radio is in PHY_RX state :TBD*/

  /* write 0x40 into the AGC_MODE_REG */
  ADF_MMapWrite(AGC_MODE_REG, 0x01, &agc_mode_value);

  /* Read back the AGC gain settings */
  ADF_MMapRead(AGC_GAIN_STATUS_REG, 0x01, &agc_gain_settings);

  gain_correction = Map_agc_gain_correction(agc_gain_settings);

  ADF_MMapRead(ADC_READBACK_HIGH_REG, 0x02, &adc_readback[0]);

  adc_readback_value = (((adc_readback[0] & 0x3f) << 2) | ((adc_readback[1] & 0xC0) >> 6));

  /* write 0x00 into the AGC_MODE_REG . Re-enable AGC*/
  agc_mode_value = FREE_RUNNING_AGC;
  ADF_MMapWrite(AGC_MODE_REG, 0x01, &agc_mode_value);

  *RSSIVal = (signed char)((int)((adc_readback_value/7) + gain_correction) - 109);               // and convert to dBm

}
/*---------------------------------------------*/
/* Interrupt servicing is done here */
/*---------------------------------------------*/
extern uint8_t bAckTxProgress;
void Service_Interrupts ( void )
{
  static uint8_t rxbuf[ ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD ];

  if(!bAckTxProgress)
  {
    /* To clear the packetbuf and reset all internal state pointers */
//    packetbuf_clear();
  }

#ifdef DISABLE_PATCH_802_15_4D
  /* Checks which interrupts are triggered and services them */
  switch ( GetPHYEventStatus( (unsigned char *) rxbuf ) )
  {
    /* This is the case when packet reception is complete.
    passes the control to the upper layers */
  case PHY_RX_CRCOK:
    adi_ADF7023_ReceiveIsrCallback ( rxbuf );
    break;

  case PHY_TX_END:
    mPHYState = CMD_PHY_ON;
    adi_ADF7023_AsyncRx( );

  case PHY_NOEVENT:
  case PHY_SYNCHDET:
    /* Fall Through */

  default:
    break;
  }

#else

  /* Checks which interrupts are triggered and services them */
  switch ( GetPHYEventStatus_15D4( (unsigned char *) rxbuf ) )
  {
  case PHY_RX_EOF:

#if (I_SUPPORT_4BYTE_FCS == 0x01)
    uint16_t Rcv_Packet_Length = MAKE_UINT16(rxbuf[1],(rxbuf[0] & PHR_LENMSBMASK));
    bCRC_Correct_Status = Verify_4byte_CRC(rxbuf, Rcv_Packet_Length+2);	// PHR len of 2 is added to Rcv_Packet_Length
    if(!bCRC_Correct_Status)
    {
      utu_timestamp(ADF7023_CRC_FAILURE,0);
      rf_stats.CRC_failure++;
      adi_ADF7023_Receive_15d4_Frame( );
      return;
    }
#endif // I_SUPPORT_4BYTE_FCS

    if(!bAckTxProgress)
    {
      /* To clear the packetbuf and reset all internal state pointers */
//      packetbuf_clear();
    }
//    rf_stats.CRC_correct++;
    adi_ADF7023_ReceiveIsrCallback ( rxbuf );

    break;

  default:
    break;
  }
#endif // DISABLE_PATCH_802_15_4D
}
/*----------------------------------------------------------*/
/* Interrupt Int8 raised when an interrupt from the ADF7023 */
/*----------------------------------------------------------*/
void Adf7023_Callback(void *ClientCallback, uint32_t Event, void *pArg)
{
  //    unsigned char ucInt0;
  
  while (1){
    mIntState = 0;
    ADF_MMapRead(MCR_interrupt_source_0_Adr, 0x1, &mIntState);
    
    if(!mIntState){
      break;
    }
  
  //    ucInt0 = 0xFF;                                                            // Clear all sources
  
    ADF_MMapWrite(MCR_interrupt_source_0_Adr, 0x1, &mIntState);
    Service_Interrupts( );
  }
}

/*---------------------------------------------------------------------------*/
/* Issue an SPI command, just need to ensure the MSB of the status word is 1 */
/* and the communication processor is not initialisating. This can also be   */
/* be used for some CMD commands e.g. reset, synch.                          */
/*---------------------------------------------------------------------------*/
unsigned char ADF_IssueCommandNW(unsigned char Cmd)
{
  int i = 1;
  ADF_CSN_DEASSERT;  							      // De-assert SPI chip select
  //ADF_CSN_ASSERT;
  mPHYState = (ADF_CmdCodes)Cmd;
  // Assert SPI chip select
  ADF_XMit(Cmd, NULL); 					              // Send Command
  //ADF_CSN_DEASSERT;
  return TRE_NO_ERR;
}

/*-------------------------------------------------------*/
/* Change the channel freq and the baud rate if required */
/*-------------------------------------------------------*/
uint8_t channel[3];
unsigned char ADF_ChangeChans(unsigned long ChFreq)
{
  ADF_SetChannelFreq(&gBBRAM, ChFreq);
  if(ADF_MMapWrite(BBRam_MMemmap_Start + BBRam_MMemmap_FOffset, 3, (unsigned char*)&(gBBRAM.channel_freq_0_r))) return TRE_MAPWR_CMD;
  ADF_MMapRead(BBRam_MMemmap_Start + BBRam_MMemmap_FOffset, 3, channel);
  return TRE_NO_ERR;
}

/*----------------------------------------------------------------------------------------*/
/* After issuing a synch CMD, the host should wait for the CMD_READY status, this ensures */
/* that the next command is processed by the communications procxessor.		          */
/*----------------------------------------------------------------------------------------*/
unsigned char ADF_SyncComms(void)
{
  unsigned char bOk = TRE_NO_ERR;
  bOk = ADF_IssueCommandNW(CMD_SYNC);                                         // will synchronise processors, recommended seq at power up
  if(bOk) return TRE_SYNC_CMD;
  bOk = ADF_WaitCmdLdr();
  return TRE_NO_ERR;
}

/*---------------------------------------------------------------*/
/* Read the status byte of the ADF7023 by sending 2 NOP commands */
/*---------------------------------------------------------------*/
unsigned char ADF_ReadStatus(ADFSTA_Reg *pStatus)
{
  unsigned char RetVal = 0;
  uint8_t nop[2] = {0xFF, 0xFF};
  uint8_t read_data[2] = {0, 0};

  ADF_SPI_XMIT(nop, read_data, 2, NULL, 0);
  pStatus->Reg = (uint32_t)read_data[1];

  return RetVal;
}

/******************************************************************************/
/* Function    : ADF_WaitFWState                                                */
/* Description : Wait until firmware state is reached                          */
/******************************************************************************/
unsigned char ADF_WaitFWState(ADF_FwState FWState)
{
  unsigned char RetVal = TRE_NO_ERR;
  unsigned int i = 1;
  do  {
    RetVal = ADF_ReadStatus(&mADF7023Status); i++;
  } while((mADF7023Status.Bits.fw_state != FWState) && (i < DEF_HW_WAIT_PERIOD));
  if(DEF_HW_WAIT_PERIOD == i) return TRE_WAIT_A_STATE;
#ifndef DISABLE_PATCH_802_15_4D
  mPHYState_15D4 = FWState;
#endif // DISABLE_PATCH_802_15_4D

  return RetVal;
}

/******************************************************************************/
/* Function    : ADF_IssueCommand                                             */
/* Description : Issue specified command                                      */
/******************************************************************************/
unsigned char ADF_IssueCommand(unsigned char Cmd)
{
  unsigned char RetVal = TRE_NO_ERR;
  mPHYState = (ADF_CmdCodes)Cmd;
  if(ADF_WaitCmdLdr()) return TRE_WAIT_CMD;
  RetVal = ADF_IssueCommandNW(Cmd);
  return RetVal;
}

/*-------------------------------------------------------------------------------*/
/* Continuously read the status word of the ADF7023 until the CMD_READY flag = 1 */
/* this then indicates that the communications processor can accept another cmd. */
/*-------------------------------------------------------------------------------*/
unsigned char ADF_WaitCmdLdr(void)
{
  int i = 1;
  unsigned char RetVal = TRE_NO_ERR;
  do {
    RetVal = ADF_ReadStatus(&mADF7023Status); i++;
  }while((mADF7023Status.Bits.cmd_loader_empty == 0x00) && (i < DEF_HW_WAIT_PERIOD));
  if(DEF_HW_WAIT_PERIOD == i) return TRE_WAIT_CMD;                                            // ADF7023 didn't respond
  return RetVal;
}


/*-------------------------------------------------------------------------------*/
/* Continuously read the status word of the ADF7023 until the SPI_READY flag = 1 */
/* this then indicates that the communications processor can accept another cmd. */
/*-------------------------------------------------------------------------------*/
unsigned char ADF_WaitSpiRdy(void)
{
  int i = 1;
  unsigned char RetVal = TRE_NO_ERR;
  do {
    RetVal = ADF_ReadStatus(&mADF7023Status); i++;
  }while((mADF7023Status.Bits.spi_ready == 0x00) && (i < DEF_HW_WAIT_PERIOD));
  if(DEF_HW_WAIT_PERIOD == i) return TRE_WAIT_CMD;                                            // ADF7023 didn't respond
  return RetVal;
}

/*-------------------------------------------------------------------------*/
/* Check the state of the MISO input, if high then the SPI processor ready */
/*-------------------------------------------------------------------------*/
static unsigned char WaitForPowerUp(void)
{
  unsigned int i = 1;
  //  ADF_CSN_ASSERT;
  //
  //  ADF_CSN_DEASSERT;
  //  if(i == DEF_HW_WAIT_PERIOD) return TRE_PWRUP;                                          // Timed out waiting for MISO high?
  //  return TRE_NO_ERR;

  if(PhyWakeUp() == -1)
  {
    return TRE_PWRUP;
  }

  return TRE_NO_ERR;
}

/*--------------------------------------------------------------------------------------*/
/* Send a byte to the ADF7023. Wait for the MSB to go high, indicating the SPI is ready */
/*--------------------------------------------------------------------------------------*/
unsigned char ADF_XMit(unsigned char ucByte,unsigned char *pData)
{
  unsigned int n = 0;
  return (ADF_SPI_XMIT(&ucByte, pData, 1, NULL, 0));
}

unsigned char ADF_SPI_XMIT(uint8_t *TxData, uint8_t *RxData, uint16_t DataLen, uint8_t *Prologue, uint16_t PrologueSize)
{
  /* SPI transceiver instance */
  ADI_SPI_TRANSCEIVE_TYPE Transceiver;
  uint8_t nTemp = 0xFF;
  uint8_t size = 1;

  /* Initialize the transceiver */
  if(RxData)
  {
    Transceiver.pRxData           =   RxData;
    Transceiver.bRxIncrement      =   1;
  }
  else
  {
    Transceiver.pRxData           =   &nTemp;
    Transceiver.bRxIncrement      =   0;
  }

  if(TxData)
  {
    Transceiver.pTxData           =   TxData;
    Transceiver.bTxIncrement      =   1;
  }
  else
  {
    Transceiver.pTxData           =   &nTemp;
    Transceiver.bTxIncrement      =   0;
  }

  Transceiver.pPrologue             =   Prologue;
  Transceiver.PrologueSize          =   PrologueSize;
  Transceiver.DataSize              =   DataLen;

  adi_spi_SetChipSelect (hSpi, ADI_SPI_CS3);

  /* Transmit the first sequence */
  hSpi->bBlockingMode = true;
  if(adi_spi_MasterRadioTx(hSpi, &Transceiver) != ADI_SPI_SUCCESS)
  {
    hSpi->bBlockingMode = false;
    return TRE_XMIT_CMD;
  }

  hSpi->bBlockingMode = false;
  return TRE_NO_ERR;
}

/*************************************************************************/
/* void ADF_TransmitPacket(void)                                        */
/* Parameters:                                                           */
/*************************************************************************/
unsigned char ADF_TransmitPacket(unsigned char* pucTXData, unsigned char TxPwr)
{

  TxPwr &= 0x3F;                                                            // Only the bottom 6 LSBs are significant
  if(ADF_MMapWrite(MCR_PA_Level_Adr, 1, (unsigned char *)&TxPwr)) return TRE_SET_TX_PWR;
  if(ADF_MMapWrite(PKT_RAM_BASE_TXPTR, *pucTXData, pucTXData)) return TRE_TX_PACK_1;
  if(ADF_IssueCommand(CMD_PHY_TX)) return TRE_TX_PACK_4;


  /* start timer for time required to transmit maximum packet size (2048 bytes) */
  pkt_transmit_err_timeout = PACKET_RCV_TIME + TickCounter;
  pkt_transmit_err_timeout_set = true;

  while(!bTxDone && pkt_transmit_err_timeout_set);
  bTxDone = 0;

  pkt_transmit_err_timeout_set = false;
  return TRE_NO_ERR;
}

/*----------------------------------------------------------------------------------*/
/* Initialise the ports and the interrupts controlling the interface to the ADF7023 */
/*----------------------------------------------------------------------------------*/
void InitADF7023Ports(void)
{
  adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE);

  adi_spi_Open(ADI_SPI_DEVID_2, spidevicemem, ADI_SPI_MEMORY_SIZE, &hSpi);

#if (ADI_SPI_CFG_ENABLE_STATIC_CONFIG_SUPPORT != 1)
  adi_spi_SetMasterMode(hSpi, true);

  adi_spi_SetBitrate(hSpi, 4000000);

  adi_spi_SetClockPhase(hSpi, false);

  adi_spi_SetContinousMode(hSpi, true);
#endif

  adi_spi_SetChipSelect (hSpi, ADI_SPI_CS3);
}

/******************************/
/* void ADF_GoToOnState(void) */
/* Parameters:                */
/******************************/
unsigned char ADF_GoToOnState(void)
{
  if(ADF_IssueCommand(CMD_PHY_ON))
    return TRE_TO_ON_STATE;			      // Enter PHY_ON mode

  if(ADF_WaitFWState(FW_ON))
    return TRE_WAIT_CMD;

  return TRE_NO_ERR;
}

/******************************/
/* void ADF_GoToOffState(void) */
/* Parameters:                */
/******************************/
unsigned char ADF_GoToOffState(void)
{
  utu_timestamp(ADF7023_PHY_OFF,0);
  if(ADF_IssueCommand(CMD_PHY_OFF)) return TRE_TO_OFF_STATE;			      // Enter PHY_OFF mode
  if(ADF_WaitFWState(FW_OFF)) return TRE_WAIT_CMD;
  return TRE_NO_ERR;
}

/******************************/
/* void ADF_GoToRXState(void) */
/* Parameters:                */
/******************************/

unsigned char ADF_GoToRXState(void)
{
  if(ADF_IssueCommandNW(CMD_PHY_RX)) return TRE_TO_RX_STATE;			      // Enter PHY_RX mode
  return TRE_NO_ERR;
}

#ifndef DISABLE_PATCH_802_15_4D

/******************************/
/* void ADF_GoTo15D4OnMode(void) */
/* Parameters:                */
/******************************/

unsigned char ADF_GoTo15D4OnMode(void)
{
  utu_timestamp(ADF7023_STATE_PHY_ON_15D4,0);
  if(ADF_IssueCommand(CMD_ENTER_15d4_MODE)) return TRE_TO_15D4_ON_STATE;
  if(ADF_WaitFWState(FW_ON_15D4)) return TRE_WAIT_CMD;
  return TRE_NO_ERR;
}

unsigned char ADF_GoTo15D4PHYOnMode(void)
{
  utu_timestamp(ADF7023_STATE_PHY_ON_15D4,0);
  if(ADF_IssueCommand(CMD_PHY_ON_15d4)) return TRE_TO_15D4_ON_STATE;
  if(ADF_WaitFWState(FW_ON_15D4)) return TRE_WAIT_CMD;
  return TRE_NO_ERR;
}

unsigned char ADF_Exit15D4(void)
{
  if(ADF_IssueCommand(CMD_EXIT_15d4_MODE)) return TRE_EXIT_15D4;
  if(ADF_WaitFWState(FW_ON)) return TRE_WAIT_CMD;
  return TRE_NO_ERR;
}

/******************************/
/* void ADF_GoTo15D4TxMode(void) */
/* Parameters:                */
/******************************/

unsigned char ADF_GoTo15D4TxMode(void)
{
  utu_timestamp(ADF7023_STATE_PHY_TX_15D4,0);
  if(ADF_IssueCommand(CMD_PHY_TX_15d4)) return TRE_TO_15D4_TX_STATE;
  if(ADF_WaitFWState(FW_TX_15D4)) return TRE_WAIT_CMD;
  return TRE_NO_ERR;
}

/******************************/
/* void ADF_GoTo15D4RxCCAMode(void) */
/* Parameters:                */
/******************************/

unsigned char ADF_GoTo15D4RxCCAMode(void)
{
  utu_timestamp(ADF7023_STATE_PHY_RX_15D4,0);
  if(ADF_IssueCommand(CMD_PHY_RX_CCA_15d4)) return TRE_TO_15D4_RX_CCA_STATE;
  if(ADF_WaitFWState(FW_RX_CCA_15D4)) return TRE_WAIT_CMD;

  if(pkt_Rx_start_time)
  {
    //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
    rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
  }

  pkt_Rx_start_time = TickCounter;

  return TRE_NO_ERR;
}
#endif // DISABLE_PATCH_802_15_4D


/*************************************************************************/
/* void ADF_SetChannelFreq(TyBBRAM *pBBRAM,unsigned long ulChannelFreq)  */
/*************************************************************************/
// The RF channel frequency bits [7:0] in Hz is set according to:
// Frequency(Hz) = FPFD x channel_Freq[23:0] /2^16
// where FPFD is the PFD frequency and is equal to 26MHz
void ADF_SetChannelFreq(TyBBRAM *pBBRAM,unsigned long ulChannelFreq)
{
  if (pBBRAM == NULL) pBBRAM = &gBBRAM;
  ulChannelFreq = (unsigned long)(((uint64_t)ulChannelFreq *
                                   FREQ_CNVRT_VAL_DIVIDEND) /
                                  FREQ_CNVRT_VAL_DIVISOR);
  pBBRAM->channel_freq_0_r = (ulChannelFreq >> 0) & 0xFF;
  pBBRAM->channel_freq_1_r = (ulChannelFreq >> 8) & 0xFF;
  pBBRAM->channel_freq_2_r = (ulChannelFreq >> 16)& 0xFF;
}

/******************************************************************************/
/* Function    : MMapWrite                                                    */
/* Description : Write Byte to specified memory map address                   */
/******************************************************************************/
unsigned char ADF_MMapWrite(unsigned long ulAdr, unsigned int ulLen, unsigned char *pData)
{
  unsigned int iTmp = 0, i = 1;
  ADFSTA_MemCmd Cmd;

  if((ulLen == 0) || (ulLen > PKT_MAX_PAYLOAD_LEN)) { return TRE_MAPWR_CMD; }

  Cmd.Bits.Cmd  = SPI_MEM_WR >> 3;
  Cmd.Bits.AdrH = (ulAdr & 0x700) >> 8;
  Cmd.Bits.AdrL = ulAdr & 0xFF;

  ADF_SPI_XMIT(pData, NULL, ulLen, &Cmd.ucBytes[0], 2);

  return TRE_NO_ERR;
}

/******************************************************************************/
/* Function    : MMapReadByte                                                 */
/* Description : Read Byte from specified memory map address                  */
/******************************************************************************/
unsigned char ADF_MMapRead(unsigned long ulAdr, unsigned long ulLen, unsigned char *pData)
{
  unsigned int iTmp = 0, i = 1;

  if((ulLen == 0) || (ulLen > PKT_MAX_PAYLOAD_LEN)) return FALSE;

  ADFSTA_MemCmd Cmd;

  Cmd.Bits.Cmd  = SPI_MEM_RD >> 3;
  Cmd.Bits.AdrH = (ulAdr & 0x700) >> 8;
  Cmd.Bits.AdrL = ulAdr & 0xFF;
  //ADF_CSN_DEASSERT;
  //ADF_CSN_ASSERT;				  	                      // Assert SPI chip select
  ADF_SPI_XMIT(NULL, pData, ulLen, &Cmd.ucBytes[0], 3);

  return TRE_NO_ERR;
}

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

/*---------------------------------------------------------------------------*/

#ifndef DISABLE_PATCH_802_15_4D
unsigned char ADF7023_15d4g_Send_Pkt(uint16_t Phr_Header, uint8_t *PSDUdata,uint8_t CCA_Therhold, uint8_t TimerValue,bool_t CCA_EnableFlag,unsigned char TxPwr)
{
  uint8_t phy_rx_status = 0;
  unsigned char result = 0;

#if (I_SUPPORT_4BYTE_FCS == 1)
  // Make sure that 2 byte fcs is not used
  if((Phr_Header & PHR_FCSMASK) != PHR_FCSMASK)
  {
    //Get packet length
    TRxFrame.Remaining.Number_Of_Bytes_To_Send = (Phr_Header >> 8) & 0xFF  | (Phr_Header & 0x07) <<8;
    // Compute 4 byte FCS for given payload
    Compute_4byte_CRC((PSDUdata + 2),TRxFrame.Remaining.Number_Of_Bytes_To_Send);
  }
#endif /* (I_SUPPORT_4BYTE_FCS == 1) */

  // Extract Header information and use it as first 2 bytes in the Tx data buffer

  PSDUdata[1] = (Phr_Header >>8);                                  // PHR - High
  PSDUdata[0] = (Phr_Header & 0xFF); 	                      // PHR - Low
  TRxFrame.Remaining.Number_Of_Bytes_To_Send = ((PSDUdata[0] & 7) << 8) | PSDUdata[1];  // Get packet length
  TRxFrame.pPacket = PSDUdata;

  if (TRxFrame.Remaining.Number_Of_Bytes_To_Send > USE_PDSU_15_4D)
  {
    return TRE_PACKET_SIZE_OVERFLOW;
  }

  // Update the Packet length by considering the PHR bytes.
  TRxFrame.Remaining.Number_Of_Bytes_To_Send += sizeof(Phr_Header); // Extra bytes to care of PHR size.

  if(ADF_MMapWrite(MCR_PA_Level_Adr, 1, (unsigned char *)&TxPwr)) return TRE_SET_TX_PWR;

  if(CCA_EnableFlag == true)
  {
    utu_timestamp(ADF7023_CCA_TX,0);

    // Send the packet with CCA option
    // configure CCA parameters and transmit the data in the Auto TX mode.
    // Threshold value can be 11 to 81
    ADF7023_15d4g_SetCCARSSIThreshold(CCA_Therhold);

    // Timer duration e.g 6 means to 9.98 ms timer duration
    // Careful using value of 7 as it set to infinite mode of timer.
    // Timer duration can be 0 to 7 maximum.
    ADF7023_15d4g_SetCCATimerDuration(TimerValue);

    // Eanble Auto TX mode, clears the bit 4 of CCA_Config_Reg 0
    if ( ADF7023_15d4g_EnableAutoTx(true) == TRE_NO_ERR )
    {
      // Make packet RAM ready for Tx
      if( TRxFrame.Remaining.Number_Of_Bytes_To_Send >= (TX_SZ_15_4D))
      {
        ADF_MMapWrite(PKT_RAM_BASE_PTR, TX_SZ_15_4D, TRxFrame.pPacket);

        TRxFrame.Remaining.Number_Of_Bytes_To_Send -= TX_SZ_15_4D;
        TRxFrame.Accomplished.Number_Of_Bytes_Sent = TX_SZ_15_4D;
      }
      else
      {
        ADF_MMapWrite(PKT_RAM_BASE_PTR, TRxFrame.Remaining.Number_Of_Bytes_To_Send, TRxFrame.pPacket);

        TRxFrame.Accomplished.Number_Of_Bytes_Sent = TRxFrame.Remaining.Number_Of_Bytes_To_Send;
        TRxFrame.Remaining.Number_Of_Bytes_To_Send = 0;
      }

      // Enter in RX state
      // Not to be confused with TX frame used with the Receive.
      // as we do not know if channel is busy may receive valid data before TX

      adi_ADF7023_Receive_15d4_Frame( );

      // Before issuing timer start command check if preamble & SFD is detected

      ADF_MMapRead(PR_phy_rx_status, 1, (unsigned char*)&phy_rx_status);

      if(phy_rx_status != PREAMBLE_SFD_DETECT)
      {
        pkt_cca_start_time = TickCounter;
        if(ADF7023_15d4g_StartCCATimer( ) == TRE_NO_ERR)
        {

          /* start timer for time required to transmit maximum packet size (2048 bytes) */
          pkt_transmit_err_timeout = PACKET_RCV_TIME + TickCounter;
          pkt_transmit_err_timeout_set = true;

          while(!bTxDone && pkt_transmit_err_timeout_set);
          bTxDone = 0;

          pkt_transmit_err_timeout_set = false;

          //if(rf_stats.Tx_EOF == 1)
          //LEDControl(LED_RED,TURN_OFF);                                      // LED P12 Off
          /* at this point the radio should be back in ON state */
          mPHYState_15D4 = mADF7023Status.Bits.fw_state;

          return ADF_GoTo15D4RxCCAMode( );
        }
      }
      else
      {
        // mark channel as busy and to receive data from other device
      }
    } else {
      PRINTF(" Setting Auto TX mode failed\n ");
    }

  }// if(CCA_EnableFlag == true)
  else
  {
    // Non-CCA  TX
    utu_timestamp(ADF7023_NON_CCA_TX,0);
    return adi_ADF7023_Transmit_15d4_Frame( );
  }
  return TRE_DEBUG;
}
/*---------------------------------------------------------------------------*/

/**
* @brief Sets the CCA threshold value
*
* @param[in]   cca_therhold    CCA RSSI thershold
*
* @return unsigned char.
*
* Sets the CCA RSSI Thershold value to use in PHY_RX_CCA_15D4 state for CCA check.
* The received signal strength is compared with this thershold value.If first one
* is more than thershold then channel is busy else channel is clear.
*
* @note This function should be used before using any of the CCA feature.
*/

unsigned char ADF7023_15d4g_SetCCARSSIThreshold( uint8_t cca_threshold )
{
  // ADF7023J datasheet mentions RSSI in the range of -96 to -26  in dbm
  // The equation CCA_THRESHOLD = RSSI(dbm) + 107
  // So Maximum and minimum values for thershold are 11 & 81.
  if((cca_threshold < 11) || ( cca_threshold > 81)) return TRE_INVALID_CCA_RSSI_THERSHOLD;
  return ADF_MMapWrite(BB_CCA_THRESHOLD, 1, &cca_threshold);
}

/*---------------------------------------------------------------------------*/
/**
* @brief Sets timer duration for the CCA evaluation.
*
* @param[in]   TimeValue   Timer value in the range 0-7 (uint8_t)
*
* @return unsigned char.
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
*   	4       		1920 microsecond
*   	5 			2560 microsecond
*  	6			9960 microsecond
*   	7           Infinite Mode
* For any value above 7 will return without setting any timer value.
* Caution - While using Timevalue of 7, timer is set in infinite mode.
*
* @sa ADF7023_15d4g_SetCCARSSIThreshold()
*
* @note This function should be used before using any of the CCA feature.
*
*/

unsigned char ADF7023_15d4g_SetCCATimerDuration( uint8_t TimeValue )
{
  uint8_t cca_cfg0_Timer_value;

  if (TimeValue > 7) return TRE_INVALID_TIMER_DURATION;

  ADF_MMapRead(BB_CCA_CFG_0, 1, &cca_cfg0_Timer_value);

  // Note bit 0 of cca_cfg_0 is for bypass cca update. for normal use
  // this bit set to "0" and  for debug purpose only this bit is set to "1".
  // Set the timer duration value for bit position 1,2 & 3
  // Clear the timer duration value read and set required value;

  cca_cfg0_Timer_value = (cca_cfg0_Timer_value & 0xF1)| (TimeValue << 1);

  return ADF_MMapWrite(BB_CCA_CFG_0, 1, &cca_cfg0_Timer_value);
}

/*---------------------------------------------------------------------------*/

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

unsigned char ADF7023_15d4g_EnableAutoTx( bool_t AutoTxflag )
{
  uint8_t cca_cfg0_reg_value;

  ADF_MMapRead(BB_CCA_CFG_0, 1, &cca_cfg0_reg_value);

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

  return ADF_MMapWrite(BB_CCA_CFG_0, 1, &cca_cfg0_reg_value);
}

/*---------------------------------------------------------------------------*/
/**
* @brief Turns ADF7023 phy 15d4 TX mode on and transmits a frame.
*
* @param[in]   None
*
* @return unsigned char.
*
* Loads the packet data to the Packet RAM of the ADF7023 and then turns
* phy 15d4 TX on. With the use of rolling buffer mecahnsim packet size
* upto 2047 bytes maximum can be used for transmit.
*0
* @sa adi_ADF7023_Receive_15d4_Frame()
*
*/

unsigned char adi_ADF7023_Transmit_15d4_Frame(void)
{
  if(USE_PDSU_15_4D < TRxFrame.Remaining.Number_Of_Bytes_To_Send)  // Max data size handled by 15d4
    return TRE_PACKET_SIZE_OVERFLOW;

  /* clear any ADF interrupts that have come while we were doing this */
  AdfClearInterrupts( );

  /* Check the packet length and write the packet RAM accordingly */

  // Case 1 : Packet length less than TX_SZ_15_4D.
  if(TRxFrame.Remaining.Number_Of_Bytes_To_Send <= (TX_SZ_15_4D))
  {
    ADF_MMapWrite(PKT_RAM_BASE_TXPTR, TRxFrame.Remaining.Number_Of_Bytes_To_Send, TRxFrame.pPacket);
    // This needs to be done because if we get the buffer almost_full or full interrupt, we check the numbers in the
    // Bytes_To_Send and Number_Of_Bytes_Sent and start writing into the buffer again.
    TRxFrame.Accomplished.Number_Of_Bytes_Sent = TRxFrame.Remaining.Number_Of_Bytes_To_Send;
    TRxFrame.Remaining.Number_Of_Bytes_To_Send = 0;
  }

  // Case 2 : Packet length >= TX_SZ_15_4D.
  else if( TRxFrame.Remaining.Number_Of_Bytes_To_Send > (TX_SZ_15_4D))
  {
    ADF_MMapWrite(PKT_RAM_BASE_TXPTR, TX_SZ_15_4D, TRxFrame.pPacket);

    TRxFrame.Remaining.Number_Of_Bytes_To_Send -= TX_SZ_15_4D;
    TRxFrame.Accomplished.Number_Of_Bytes_Sent = TX_SZ_15_4D;
  }

//  pADI_IDU->EI2CFG = EI2CFG_IRQ8MDE_HIGHLEVEL | EI2CFG_IRQ8EN_EN;
  if (adi_ADF7023_SetPhyTx15D4( ) == TRE_NO_ERR)
  {

    /* start timer for time required to transmit maximum packet size (2048 bytes) */
    pkt_transmit_err_timeout = PACKET_RCV_TIME + TickCounter;
    pkt_transmit_err_timeout_set = true;

    while(!bTxDone && pkt_transmit_err_timeout_set);
    bTxDone = 0;

    pkt_transmit_err_timeout_set = false;

    /* at this point the radio should be back in ON state */
    mPHYState_15D4 = mADF7023Status.Bits.fw_state;

    return ADF_GoTo15D4RxCCAMode( );
  }
  else {
    /* TRX is not able to change to Tx state. Suspecting TRX went into a bad
    state. Hence re-initializing the TRX */
    rf_stats.TxStateFail_re_init_count++;
    ADF7023_ReInit(NULL);
  }
  return TRE_DEBUG;
}
/*---------------------------------------------------------------------------*/
/**
* @brief Handles the TX interrupt for almost full buffer case.
*
* @param[in]   None.
*
* @return None
*
* When number of  bytes transmitted >= TX_SZ_15_4D/2, then this Almost full buffer
* interrupt is active and this function gets invoked. First half of the buffer
* can be over-written to packet RAM with new data. TX_BASE + TX_SZ_15_4D/2 forms
* the first half & TX_BASE + TX_SZ_15_4D/2 to   TX_BASE + TX_SZ_15_4D
* forms the second half in the packet RAM.
*
* @sa 	cbTxBufferAlmostFull()
*
*/
/*************************************************************************
void cbTxBufferAlmostFull(void)

This is internal ISR function.
*************************************************************************/

void cbTxBufferAlmostFull (void)
{
  // The first half of the buffer has been transmitted. Update it now based on packet length
  if ((TRxFrame.Remaining.Number_Of_Bytes_To_Send >0) && (TRxFrame.Remaining.Number_Of_Bytes_To_Send <(TX_SZ_15_4D/2)))
  {
    // Write remaining data bytes in Packet RAM to be Transmitted.
    ADF_MMapWrite(PKT_RAM_BASE_PTR, TRxFrame.Remaining.Number_Of_Bytes_To_Send, (TRxFrame.pPacket + TRxFrame.Accomplished.Number_Of_Bytes_Sent));

    TRxFrame.Accomplished.Number_Of_Bytes_Sent += TRxFrame.Remaining.Number_Of_Bytes_To_Send;
    TRxFrame.Remaining.Number_Of_Bytes_To_Send = 0;
  } else if (TRxFrame.Remaining.Number_Of_Bytes_To_Send >=(TX_SZ_15_4D/2)) {
    // Write data bytes into Packet RAM(for half of its size) to be Transmitted.
    ADF_MMapWrite(PKT_RAM_BASE_PTR, (TX_SZ_15_4D/2), (TRxFrame.pPacket + TRxFrame.Accomplished.Number_Of_Bytes_Sent));

    TRxFrame.Remaining.Number_Of_Bytes_To_Send -= (TX_SZ_15_4D/2);
    TRxFrame.Accomplished.Number_Of_Bytes_Sent += (TX_SZ_15_4D/2);
  }
}
/*---------------------------------------------------------------------------*/
/**
* @brief Handles the TX interrupt for buffer full case.
*
* @param[in]   None
*
* @return None
*
* When number of  bytes trasnmitted  >= TX_SZ_15_4D, then this full buffer
* interrupt is active and this function gets invoked. Second half of the buffer
* can be over-written to packet RAM with new data .TX_BASE + TX_SZ_15_4D/2 forms
* the first half  & TX_BASE + TX_SZ_15_4D/2 to  TX_BASE + TX_SZ_15_4D forms the
* second half in the packet RAM.
*
* @sa 	cbTxBufferFull()
*
*/
/*************************************************************************
void cbTxBufferFull(void)

This is internal ISR function.
*************************************************************************/
void cbTxBufferFull (void)
{
  // The Second half of the buffer has been transmitted. Update it now based on packet length
  if ((TRxFrame.Remaining.Number_Of_Bytes_To_Send >0) && (TRxFrame.Remaining.Number_Of_Bytes_To_Send <(TX_SZ_15_4D/2)))
  {
    // Write remaining data bytes in Packet RAM to be Transmitted.
    ADF_MMapWrite((PKT_RAM_BASE_PTR + (TX_SZ_15_4D/2)), TRxFrame.Remaining.Number_Of_Bytes_To_Send, (TRxFrame.pPacket + TRxFrame.Accomplished.Number_Of_Bytes_Sent));

    TRxFrame.Accomplished.Number_Of_Bytes_Sent += TRxFrame.Remaining.Number_Of_Bytes_To_Send;
    TRxFrame.Remaining.Number_Of_Bytes_To_Send = 0;
  } else if (TRxFrame.Remaining.Number_Of_Bytes_To_Send >=(TX_SZ_15_4D/2)) {
    // Write data bytes into Packet RAM(for half of its size) to be Transmitted.
    ADF_MMapWrite((PKT_RAM_BASE_PTR + (TX_SZ_15_4D/2)), (TX_SZ_15_4D/2), (TRxFrame.pPacket + TRxFrame.Accomplished.Number_Of_Bytes_Sent));

    TRxFrame.Remaining.Number_Of_Bytes_To_Send -= (TX_SZ_15_4D/2);
    TRxFrame.Accomplished.Number_Of_Bytes_Sent += (TX_SZ_15_4D/2);
  }
}
/*---------------------------------------------------------------------------*/
/**
* @brief Handles the RX interrupt for almost full buffer case.
*
* @param[in]   pBuffer     Buffer to fill in the received bytes of data.
*
* @return None
*
* When number of  bytes received >= RX_SZ_15_4D/2, then this Almost full buffer
* interrupt is active and this function gets invoked. First half of the buffer
* can be over-written to packet RAM with new data. RX_BASE + RX_SZ_15_4D/2 forms
* the first half & RX_BASE + RX_SZ_15_4D/2 to   RX_BASE + RX_SZ_15_4D -1
* forms the second half in the packet RAM.
*
* @sa 	cbRxBufferAlmostFull(unsigned char *pBuffer)
*
*/
/*************************************************************************
void cbRxBufferAlmostFull(unsigned char *pBuffer)

This is internal ISR function.
*************************************************************************/
void cbRxBufferAlmostFull(unsigned char *pBuffer)
{
  if(TRxFrame.Accomplished.Number_Of_Bytes_Received == 0)
  {
    ADF_MMapRead(PKT_RAM_BASE_PTR, (RX_SZ_15_4D/2), pBuffer);

    // Read the length of packet in first two bytes
    // Take only 3 bits of length information from first byte read
    //hDevice->pPacket->pData[0] &=0x7;
    TRxFrame.Remaining.Number_Of_Bytes_To_Receive = MAKE_UINT16(pBuffer[1],(pBuffer[0] & PHR_LENMSBMASK));

    /* Including the number of bytes for phr_header */
    TRxFrame.Remaining.Number_Of_Bytes_To_Receive += 2;

    if( TRxFrame.Remaining.Number_Of_Bytes_To_Receive > USE_PDSU_15_4D)
    {
      // Pkt length exceeds the limit
      PRINTF(" Packet Length exceeds the Limit\n ");

      TRxFrame.Remaining.Number_Of_Bytes_To_Receive = TRxFrame.Accomplished.Number_Of_Bytes_Received = 0;
      return;
    }

    if(TRxFrame.Remaining.Number_Of_Bytes_To_Receive < (RX_SZ_15_4D/2))
    {
      // If we receive buffer almost full interrupt with number of bytes to receive less than
      // the rx_buff_signal it is invalid, as this should trigger RX_EOF interrupt and not
      // buffer_almost_full interrupt.
      PRINTF(" Received a invalid interrupt\n ");

      TRxFrame.Remaining.Number_Of_Bytes_To_Receive = TRxFrame.Accomplished.Number_Of_Bytes_Received = 0;
      return;
    }

    TRxFrame.Remaining.Number_Of_Bytes_To_Receive -= (RX_SZ_15_4D/2);
    TRxFrame.Accomplished.Number_Of_Bytes_Received = (RX_SZ_15_4D/2);

  }
  else if (TRxFrame.Remaining.Number_Of_Bytes_To_Receive > (RX_SZ_15_4D/2))
  {
    // Only read if it will fit in buffer.
    if((TRxFrame.Accomplished.Number_Of_Bytes_Received + (RX_SZ_15_4D/2)) < (ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD)){
      ADF_MMapRead(PKT_RAM_BASE_PTR, (RX_SZ_15_4D/2), (pBuffer + TRxFrame.Accomplished.Number_Of_Bytes_Received));
    }
    
    TRxFrame.Remaining.Number_Of_Bytes_To_Receive -= (RX_SZ_15_4D/2);
    TRxFrame.Accomplished.Number_Of_Bytes_Received += (RX_SZ_15_4D/2);
  }
  else
  {
    // If Some thing is wrong
    PRINTF(" Unknown Error\n ");
  }
}
/*---------------------------------------------------------------------------*/

/**
* @brief Handles the RX interrupt for buffer full  case.
*
* @param[in]   pBuffer     Buffer to fill in the received bytes of data.
*
* @return None
*
* When number of  bytes received >= RX_SZ_15_4D, then this full buffer
* interrupt is active and this function gets invoked. Second half of the buffer
* can be over-written to packet RAM with new data .RX_BASE + TX_SZ_15_4D/2 forms
* the first half  & RX_BASE + RX_SZ_15_4D/2 to  RX_BASE + RX_SZ_15_4D -1 forms the
* second half in the packet RAM.
*
* @sa 	cbRxBufferFull(unsigned char *pBuffer)
*
*/

/*************************************************************************
void cbRxBufferFull(unsigned char *pBuffer)

This is internal ISR function.
*************************************************************************/
void cbRxBufferFull(unsigned char *pBuffer)
{
  // The second half of the buffer has been received. Read it out now
  if (TRxFrame.Remaining.Number_Of_Bytes_To_Receive > (RX_SZ_15_4D/2))
  {
    // Only read if it will fit in buffer.
    if((TRxFrame.Accomplished.Number_Of_Bytes_Received + (RX_SZ_15_4D/2)) < (ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD)){
      ADF_MMapRead((PKT_RAM_BASE_PTR + (RX_SZ_15_4D/2)), (RX_SZ_15_4D/2), (pBuffer + TRxFrame.Accomplished.Number_Of_Bytes_Received));
    }
    
    TRxFrame.Remaining.Number_Of_Bytes_To_Receive -= (RX_SZ_15_4D/2);
    TRxFrame.Accomplished.Number_Of_Bytes_Received += (RX_SZ_15_4D/2);
  }
  else
  {
    // Some thing wrong
    PRINTF(" Unknown Error\n ");
  }
}

/*---------------------------------------------------------------------------*/
/**
* @brief Receives remaining bytes in the packets .
*
* @param[in]   pBuffer     Buffer to fill in the received bytes of data.
*
* @return None
*
* This is internal function invoked when Receive End (RxEOF) of interrupt is active
* By now received an entire packet with few less bytes left to read.
*
* @sa 	cb154dPacketReceived (unsigned char *pBuffer)
*
*/
/*************************************************************************
unsigned char cb154dPacketReceived    (unsigned char *pBuffer)

This is internal ISR function.
*************************************************************************/

unsigned char cb154dPacketReceived (unsigned char *pBuffer)
{
  if(TRxFrame.Accomplished.Number_Of_Bytes_Received == 0)
  {
    // This will be the case when pkt length is less than half the RX_SZ_15_4D
    // Do not know lenth of packet yet
    // so get the length of packet  by reading first two bytes and then read remianing bytes
    ADF_MMapRead(PKT_RAM_BASE_PTR, 2, pBuffer);

    // Take only 3 bits of length information from first byte read
    TRxFrame.Remaining.Number_Of_Bytes_To_Receive = MAKE_UINT16(pBuffer[1],(pBuffer[0] & PHR_LENMSBMASK));

    if( TRxFrame.Remaining.Number_Of_Bytes_To_Receive > USE_PDSU_15_4D)
    {
      // Pkt length exceeds the limit
      PRINTF(" Packet Length exceeds the Limit\n ");

      TRxFrame.Remaining.Number_Of_Bytes_To_Receive = TRxFrame.Accomplished.Number_Of_Bytes_Received = 0;
      return FAILURE;
    }

    /* Update the Received Packet attributes */
    //      TRxFrame.Remaining.Number_Of_Bytes_To_Receive -= 2;
    TRxFrame.Accomplished.Number_Of_Bytes_Received = 2;

    /* Copy the reamining number of bytes into the buffer */
    if((TRxFrame.Accomplished.Number_Of_Bytes_Received + TRxFrame.Remaining.Number_Of_Bytes_To_Receive) < (ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD)){
      ADF_MMapRead((PKT_RAM_BASE_PTR + TRxFrame.Accomplished.Number_Of_Bytes_Received) , TRxFrame.Remaining.Number_Of_Bytes_To_Receive, (pBuffer + TRxFrame.Accomplished.Number_Of_Bytes_Received));
    }
    else {
      return FAILURE;
    }
  
    return SUCCESS;   
  }
  else
  {
#if ( I_SUPPORT_4BYTE_FCS == 0x01)
    uint16_t Bytes_To_Receive_Considering_CRC = TRxFrame.Remaining.Number_Of_Bytes_To_Receive;
#else
    uint16_t Bytes_To_Receive_Considering_CRC = TRxFrame.Remaining.Number_Of_Bytes_To_Receive - SIZE_CRC_FIELD;
#endif // #if ( I_SUPPORT_4BYTE_FCS == 0x01)

    if ( Bytes_To_Receive_Considering_CRC <= (RX_SZ_15_4D/2))
    {
      /* This is done to determine the Base pointer from which to start copying the received payload.
      As we doesn't know this callback is being called after the processing of BUF_ALMOST_FULL interrupt or
      BUF_FULL interrupt, this check is mandatory */
      if ( ((Bytes_To_Receive_Considering_CRC + TRxFrame.Accomplished.Number_Of_Bytes_Received) % RX_SZ_15_4D) <= (RX_SZ_15_4D/2) )
      {
        if (((Bytes_To_Receive_Considering_CRC + TRxFrame.Accomplished.Number_Of_Bytes_Received) % RX_SZ_15_4D) == 0)
        {
          if((TRxFrame.Accomplished.Number_Of_Bytes_Received + TRxFrame.Remaining.Number_Of_Bytes_To_Receive) < (ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD)){
            ADF_MMapRead((PKT_RAM_BASE_PTR + (RX_SZ_15_4D/2)), TRxFrame.Remaining.Number_Of_Bytes_To_Receive, (pBuffer + TRxFrame.Accomplished.Number_Of_Bytes_Received));
          }
          else{
            return FAILURE;
          }
        } 
        else
        {
          /* If the value of the above condition check satisfies, it does mean that the remaining bytes are in the upper half of the buffer and
          need to be copied */
          if((TRxFrame.Accomplished.Number_Of_Bytes_Received + TRxFrame.Remaining.Number_Of_Bytes_To_Receive) < (ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD)){
            ADF_MMapRead(PKT_RAM_BASE_PTR, TRxFrame.Remaining.Number_Of_Bytes_To_Receive, (pBuffer + TRxFrame.Accomplished.Number_Of_Bytes_Received));
          }
          else {
            return FAILURE;
          }
        }    
      }
      else
      {
        /* If the above condition check fails,it does mean that the remaining bytes are stored in the lower half of the packet RAM and needs to be copied */
        if((TRxFrame.Accomplished.Number_Of_Bytes_Received + TRxFrame.Remaining.Number_Of_Bytes_To_Receive) < (ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD)) {
        ADF_MMapRead((PKT_RAM_BASE_PTR + (RX_SZ_15_4D/2)), TRxFrame.Remaining.Number_Of_Bytes_To_Receive, (pBuffer + TRxFrame.Accomplished.Number_Of_Bytes_Received));
        }
        else {
          return FAILURE;
        }
      }

      return SUCCESS;
    }
  }
  return FAILURE;
}
/*---------------------------------------------------------------------------*/
/**
* @brief      Clears the interrupts.
*
* @param[in]   None.
*
* @return     unsigned char.
*
* This function is invoked to clear the inerrupt source register
*
* @sa 	AdfClearInterrupts(void)
*
*/
/*************************************************************************
static unsigned char AdfClearInterrupts(void)
*************************************************************************/
static unsigned char AdfClearInterrupts(void)
{
  unsigned char ucInt0;

  ADF_MMapRead(MCR_interrupt_source_0_Adr, 0x1, &ucInt0);
  // Clear interrupt sources
  return ADF_MMapWrite(MCR_interrupt_source_0_Adr, 0x1, &ucInt0);
}
/*---------------------------------------------------------------------------*/
unsigned char adi_ADF7023_SetPhyTx15D4(void)
{
  switch(mPHYState_15D4)
  {
  case FW_OFF:
  case FW_RX:
  case FW_TX:
    if( ADF_GoToOnState( ) != TRE_NO_ERR )
      break;

  case FW_ON:
    if( ADF_GoTo15D4OnMode( ) != TRE_NO_ERR )
      break;

  case FW_ON_15D4:
  case FW_RX_CCA_15D4:
  case FW_TX_15D4:
    if( ADF_GoTo15D4TxMode( ) != TRE_NO_ERR )
      break;

    if(pkt_Rx_start_time)
    {
      //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
      pkt_Rx_start_time = 0;
    }

    pkt_Tx_start_time = TickCounter;

    return TRE_NO_ERR;

  default:
    break;
  }
  return TRE_DEBUG;
}

/*---------------------------------------------------------------------------*/
unsigned char adi_ADF7023_SetPhyOn15D4(void)
{
  switch(mPHYState_15D4)
  {
  case FW_OFF:
  case FW_RX:
  case FW_TX:
    if( ADF_GoToOnState( ) != TRE_NO_ERR )
      break;

  case FW_ON:
  case FW_ON_15D4:
  case FW_RX_CCA_15D4:
  case FW_TX_15D4:
    if( ADF_GoTo15D4OnMode( ) != TRE_NO_ERR )
      break;

    if(pkt_Rx_start_time)
    {
      //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
      pkt_Rx_start_time = 0;
    }

    return TRE_NO_ERR;

  default:
    break;
  }
  return TRE_DEBUG;
}

unsigned char adi_ADF7023_SetBackToPhyOn15D4(void)
{
  switch(mPHYState_15D4)
  {
  case FW_OFF:
  case FW_RX:
  case FW_TX:
    if( ADF_GoToOnState( ) != TRE_NO_ERR )
      break;

  case FW_ON:
  case FW_ON_15D4:
  case FW_RX_CCA_15D4:
  case FW_TX_15D4:
    if( ADF_GoTo15D4PHYOnMode( ) != TRE_NO_ERR )
      break;

    if(pkt_Rx_start_time)
    {
      //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
      pkt_Rx_start_time = 0;
    }

    return TRE_NO_ERR;

  default:
    break;
  }
  return TRE_DEBUG;
}
/*---------------------------------------------------------------------------*/
unsigned char adi_ADF7023_SetPhyRxCCA15D4(void)
{

//  if(mPHYState_15D4 != FW_RX_CCA_15D4)
//    pkt_Rx_start_time = TickCounter;  //restart the timer only if the state was different

  switch(mPHYState_15D4)
  {
  case FW_OFF:
  case FW_RX:
  case FW_TX:
    if( ADF_GoToOnState( ) != TRE_NO_ERR )
      break;

  case FW_ON:
    if( ADF_GoTo15D4OnMode( ) != TRE_NO_ERR )
      break;

  case FW_ON_15D4:
  case FW_RX_CCA_15D4:
  case CMD_PHY_TX_15d4:
    if( ADF_GoTo15D4RxCCAMode( ) != TRE_NO_ERR )
      break;

    return TRE_NO_ERR;

  default:
    break;
  }
  return TRE_DEBUG;
}

/*---------------------------------------------------------------------------*/
/*************************************************************************/
/* void adi_ADF7023_154D_PatchRoutine(void)                                      */
/* Parameters:                                                           */
/*
the 15.4d routine to the ADF7023                             */
/*************************************************************************/
unsigned char reason_for_failure;

void adi_ADF7023_154D_PatchRoutine(void)
{
  ADF_GoToOffState( );

  ADF_IssueCommandNW(CMD_RAM_LOAD_INIT);

  mADF7023Status.Bits.cmd_loader_empty = 1;

  // Write 15d4 firmware data to PRAM
  ADF_MMapWrite(0x600, sizeof(patch_802_15_4_d), (unsigned char *)&patch_802_15_4_d[0]);

  // Issue command PRAM loading complete
  ADF_IssueCommandNW(CMD_RAM_LOAD_DONE);

  ADF_WaitFWState(FW_OFF);

  // Issue Synchronize command
  ADF_IssueCommandNW(CMD_SYNC);

  ADF_WaitFWState(FW_OFF);
}
/*---------------------------------------------------------------------------*/
/**
* @brief Configures BBRAM registers specific to 15d4g firmware
*
* @param[in]   None.
*
* @return  unsigned char.
*
* First change the radio state to PHY_15D4_ON. Then initialize the BBRAM
* registers specific to 15d4g firmware module.
*/

unsigned char adi_ADF7023_config_15d4_BBRAM_Regs(void)
{
  TyBBRAM15_4d BBRAM154d;

  //Ensure radio device is PHY_ON_15D4 state
  adi_ADF7023_SetPhyOn15D4( );

  // Set 15d4 specific BBRAM registers
  adi_ADF7023_154d_bbram_setting(&BBRAM154d);

  // Initilize 15d4 specific BBRAM register with the set configuration.
  return ADF_MMapWrite(BBRam154d_MMemmap_Start, sizeof(TyBBRAM15_4d), (unsigned char *)&BBRAM154d);
}
/*---------------------------------------------------------------------------*/
/**
* @brief Configures BBRAM registers specific to 15d4g firmware specific values
*
* @param[in]   None.
*
* @return  void.
*
* First Set the register values to zero and assigning the non-zero values to the necessary registers.
*/
/*---------------------------------------------------------------------------*/
static void adi_ADF7023_154d_bbram_setting(TyBBRAM15_4d *pBBRAM)
{
  // Clear the BBRAM register specific to 15d4 firmware
  memset(pBBRAM,0x0,sizeof(TyBBRAM15_4d));

  // Configure each BBRAM register as per need

  // Number of pre-amble bytes
  pBBRAM->nb_preamble_bytes_low_r  = 0x08;   // 0x12E

  // Sync
  pBBRAM->sfd_low_r                = 0x09;   // 0x130
  pBBRAM->sfd_high_r               = 0x72;   // 0x131

  // Rolling Receive buffer config
  pBBRAM->rx_buff_signal_r         = RX_SZ_15_4D/2; // 0x134  Almost rx full
  pBBRAM->rx_buff_size_r           = RX_SZ_15_4D; // 0x135  Rx full
  // Rolling transmit buffer config
  pBBRAM->tx_buff_signal_r         = TX_SZ_15_4D/2; // 0x136  Almost tx full
  pBBRAM->tx_buff_size_r           = TX_SZ_15_4D;   // 0x137  Tx full

  // Reserved value
  pBBRAM->reserved2_r              = 0x0;   // 0x13A
}

#ifdef ANTENNA_DIVERSITY
/*---------------------------------------------------------------------------*/
/**
* @brief Configures the antenna 0 & 1 path & select an antenna for transmission.
*
* @param[in] 	Antenna_Path_0 		ATB Control bit setting for path 0 (uint8_t)
* @param[in] 	Antenna_Path_1		ATB Control bit setting for path 1 (uint8_t)
* @param[in] 	ATB_Level           ADI_ADF7023J_15D4_TRX_ATB_LEVEL type
* @param[in] 	Antenna_Select      ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED type
*
* @return unsigned char.
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

unsigned char ADF7023_15d4g_TxAntennaConfig(uint8_t Antenna_Path_0, uint8_t Antenna_Path_1, ADI_ADF7023J_15D4_TRX_ATB_LEVEL ATB_Level,ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED Antenna_Select)
{
  uint8_t value = 0;

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
  return ADF_MMapWrite(BB_TX_antenna_cfg_r, 1, &value);
}
/*---------------------------------------------------------------------------*/
/**
* @brief Configures the receive antenna path & enable/disable Antenna diversity.
*
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

unsigned char ADF7023_15d4g_RxAntennaDivConfig(uint8_t Antenna_Path_0, uint8_t Antenna_Path_1, ADI_ADF7023J_15D4_TRX_ATB_LEVEL ATB_Level, ADI_ADF7023J_15D4_RX_ANTENNA_DIVERSITY ADA_Enable, ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED Antenna_Select)
{
  uint8_t value = 0;

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
  return ADF_MMapWrite(BB_antenna_RX_diversity_cfg, 1, &value);
}

#endif // ANTENNA_DIVERSITY
/*---------------------------------------------------------------------------*/
/**
* @brief Turns ADF7023 phy 15d4 RX mode on and waits for a packet
*
* @param[in]   None
* @param[out]  None
*
* @return  void
*
* Sets the
* @sa adi_ADF7023_Transmit_15d4_Frame()
*/

unsigned char adi_ADF7023_Receive_15d4_Frame(void)
{
  AdfClearInterrupts( );

  return adi_ADF7023_SetPhyRxCCA15D4( );
}

/*---------------------------------------------------------------------------*/
#if (I_SUPPORT_4BYTE_FCS == 1)
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

  adi_crc_SetCrcSeedVal(hCrc, 0xFFFFFFFF);
#ifdef GENERATE_WRONG_CRC
  crc = 0;
#else
  /* 4 byte CRC should be handled in the code */
  if(data_length < 4)
    adi_crc_Compute (hCrc, databuff, 4);
  else
    adi_crc_Compute (hCrc, databuff, data_length);
#endif /* GENERATE_WRONG_CRC */

  bool bCRCProgress=true;

  while(bCRCProgress == true)
  {
    adi_crc_IsCrcInProgress(hCrc, &bCRCProgress);
  }

  /* Get the final result */
  adi_crc_GetFinalCrcVal(hCrc, &rem);
  crc = rem ^ 0xFFFFFFFF;


  // Append CRC to payload data
  databuff[data_length]   = (uint8_t) ((crc >> 0) & 0xff);
  databuff[data_length+1] = (uint8_t) ((crc >> 8 ) & 0xff);
  databuff[data_length+2] = (uint8_t) ((crc >> 16) & 0xff);
  databuff[data_length+3] = (uint8_t) ((crc >> 24) & 0xff);
}
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
  *data = (*data & 0x0F) << 4 | (*data & 0xF0) >> 4;
  *data = (*data & 0x33) << 2 | (*data & 0xCC) >> 2;
  *data = (*data & 0x55) << 1 | (*data & 0xAA) >> 1;

}
/*---------------------------------------------------------------------------*/
uint8_t Verify_4byte_CRC(uint8_t *RxBuffer, uint16_t data_length)
{
  uint32_t crc_calculated = 0;
  uint32_t verify_rem = 0;
  uint8_t *dataPtr;
  int i;
  dataPtr = RxBuffer;

  //Get packet length from received frame.
  //data_length = ((dataPtr[0] & 0x7) << 8) | (dataPtr[1]);
  data_length-=6; // PHR and CRC(4-byte FCS) Size

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


  adi_crc_SetCrcSeedVal(hCrc, 0xFFFFFFFF);

  adi_crc_Compute (hCrc, dataPtr, data_length);

  bool bCRCProgress=true;

  while(bCRCProgress == true)
  {
    adi_crc_IsCrcInProgress(hCrc, &bCRCProgress);
  }

  /* Get the final result */
  adi_crc_GetFinalCrcVal(hCrc, &verify_rem);

  crc_calculated = verify_rem ^ 0xFFFFFFFF;
  /* The unique number specified in the specification (0xC704DD7B),
  can achieved by reversing the bits and complementing them from the crc calculated(0x2144DF1C) */
  if(crc_calculated != 0x2144DF1C)
    return false;
  else
    return true;
}
#endif // (I_SUPPORT_4BYTE_FCS == 1)
/*---------------------------------------------------------------------------*/
/**
* @brief Starts the CCA timer.
*
* @param[in]   None
*
* @return unsigned char
*
* Starts CCA timer by issuing command ADI_ADF7023_CMD_CCA_START.
*
* @sa ADF7023_15d4g_StartCCATimer()
*
* @note This function should be called when radio device state is
* PHY_RX_CCA_15D4
*/

unsigned char ADF7023_15d4g_StartCCATimer(void)
{
  unsigned char result = TRE_NO_ERR;

  result = ADF_IssueCommandNW(CMD_CCA_START);

  ADF_WaitCmdLdr();

  mADF7023Status.Bits.cmd_loader_empty = 1;

  return result;
}

unsigned char ADF7023_15d4g_ReInit(void)
{
  unsigned char uhf_result;
  uint32_t timeout = 80000;
  uint16_t n;
  unsigned char ucInt0 = 0xFF;

  NVIC_DisableIRQ(SYS_GPIO_INTA_IRQn);
  UHF_FirstConnect();

  for(n=0; n<DEF_INIT_HW_WAIT_1MS; n++);
  if(ADF_WaitCmdLdr())
    return TRE_WAIT_CMD;
  ADF_MMapRead(PKT_RAM_SIVER, 4, mSiVer);
  if(ADF_IssueCommand(CMD_PHY_OFF))
    return TRE_TO_OFF_STATE;
  for(n=0; n<DEF_INIT_HW_WAIT_1MS; n++);
  if(ADF_MMapWrite(BBRam_MMemmap_Start, sizeof(gBBRAM), (unsigned char *)&gBBRAM))
    return TRE_MAPWR_CMD;

#ifndef DISABLE_PATCH_802_15_4D
  adi_ADF7023_154D_PatchRoutine( );
  adi_ADF7023_config_15d4_BBRAM_Regs( );
  /* Wait till TRX state changes to ON_15D4 state */
  if(ADF_WaitFWState(FW_ON_15D4)) return TRE_TO_ON_STATE;

#ifdef ANTENNA_DIVERSITY
  ADF7023_15d4g_TxAntennaConfig(2,1,ADI_ADF7023J_15D4_VDD_DRV,ADI_ADF7023J_15D4_ANTENNA0);

  ADF7023_15d4g_RxAntennaDivConfig(2,1,ADI_ADF7023J_15D4_VDD_DRV,
                                   ADI_ADF7023J_15D4_AD_DISABLE,ADI_ADF7023J_15D4_ANTENNA0);
#endif //ANTENNA_DIVERSITY

#endif // DISABLE_PATCH_802_15_4D

  if(ADF_IssueCommand(CMD_CONFIG_DEV)) return TRE_CFG_CMD;

  NVIC_ClearPendingIRQ(SYS_GPIO_INTA_IRQn);

  ADF_MMapWrite(MCR_interrupt_source_0_Adr, 0x1, &ucInt0);

  NVIC_EnableIRQ(SYS_GPIO_INTA_IRQn);

#ifdef DISABLE_PATCH_802_15_4D
  if(InitParm & INIT_TRSC_RX) if(ADF_IssueCommandNW(CMD_PHY_RX)) return TRE_TO_RX_STATE;
#else
  update_phy_params();
  /* Wait untill TRX state changes to RX_CCA_15D4*/
  if(ADF_WaitFWState(FW_RX_CCA_15D4)) return TRE_TO_ON_STATE;
#endif // DISABLE_PATCH_802_15_4D

  return TRE_NO_ERR;
}

uint8_t TRX_re_init_status;
static void
ADF7023_ReInit(void *ptr)
{
  do {
    rf_stats.re_init_count++;
    TRX_re_init_status = ADF7023_15d4g_ReInit();
    memset(&TRxFrame,0,sizeof(TRxFrame));
  }while(TRX_re_init_status != TRE_NO_ERR);
}

void SysTick_Handler(void)
{
  TickCounter++;


  if((pkt_recv_err_timeout_set) && (pkt_recv_err_timeout == TickCounter))
  {
    rf_stats.Rx_timeout_re_init_count++;
    ADF7023_ReInit(NULL);
    pkt_recv_err_timeout_set = false;
  }

  if((pkt_transmit_err_timeout_set) && (pkt_transmit_err_timeout == TickCounter))
  {
    bTxDone = 1;
    rf_stats.Tx_timeout_re_init_count++;
    ADF7023_ReInit(NULL);
    pkt_transmit_err_timeout_set = false;
  }
}

//@debug
volatile uint32_t dbg_mADF7023Status_and_TRXStatus_diff = 0;
volatile uint32_t dbg_goto_rxcca_failed = 0;
unsigned char ADF7023_ReadAntennaRSSI(int8_t *RSSI_r)
{
  unsigned char uhf_result;
  uint8_t value;

  //@debug
  *RSSI_r = 0;
  /* Read current status of TRX, if it is not in FW_RX_CCA_15D4 state then
  change it to FW_RX_CCA_15D4 state, before reading RSSI.*/
  ADF_ReadStatus(&mADF7023Status);
  if(mADF7023Status.Bits.fw_state != FW_RX_CCA_15D4){
    dbg_mADF7023Status_and_TRXStatus_diff++;
    if(ADF_GoTo15D4RxCCAMode( )) {
      dbg_goto_rxcca_failed++;
    }
  }

  uhf_result = ADF_MMapRead(BB_antenna_RX_diversity_cfg, 0x1, &value);
  // Read the the RSSI value from antenna selected either manually or
  // by antenna diversity algorithm (if enabled)
  if(value & ANTENNA_1)
  {
    uhf_result = ADF_MMapRead(BB_antenna1_rssi, 0x1, (unsigned char *)RSSI_r);
  }
  else
  {
    uhf_result = ADF_MMapRead(BB_antenna0_rssi, 0x1, (unsigned char *)RSSI_r);
  }
  *RSSI_r -= 107;
  return uhf_result;
}

/*---------------------------------------------------------------------------*/
#endif // DISABLE_PATCH_802_15_4D


static int PhyWakeUp(void)
{

  uint32_t i = 0;
  uint16_t value = 0;

  disable_CS3();

  if(adi_gpio_SetLow (ADI_GPIO_PORT2, ADI_GPIO_PIN_7) != ADI_GPIO_SUCCESS)
  {
    adi_initpinmux();
    return -1;
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
    return -1;
  }

  /* if at first you don't succeed... */
  if (i > 4999)
  {
    i = 0;
    if(adi_gpio_SetLow (ADI_GPIO_PORT2, ADI_GPIO_PIN_7) != ADI_GPIO_SUCCESS)
    {
      adi_initpinmux();
      return -1;
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
      return -1;
    }
  }

  // if second try fails, return error
  if (i > 4999)
  {
    adi_initpinmux();
    return -1;
  }
  adi_initpinmux();
  return 0;
}

static int UHF_FirstConnect()
{
  volatile int uhf_result;
  volatile uint64_t timeout = 2000;

  //hopefully it's in PHY_SLEEP at this point
  uhf_result = PhyWakeUp();

  do
  {
    timeout--;
  }while(0 < timeout);

  //issue a hard reset
  if(ADF_IssueCommandNW(CMD_HW_RESET)) return TRE_RST_CMD;
  //hard reset -> PHY_SLEEP timing is still TBD
  //TODO replace in future with a more accurate value
  //No point as it is optimized.
  timeout = 20000;
  do
  {
    timeout--;
  }while(0 < timeout);

  if(ADF_WaitFWState(FW_OFF)) return TRE_WAIT_CMD;

  return 0;
}

static unsigned char SetupIrqFlag(IRQn_Type Flag)
{
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

    return 0;
}


ADI_SPI_HANDLE get_spi_handle()
{
  return hSpi;
}

void TRX_Set_Data_Rate(uint16_t phy_data_rate)
{
    uint8_t dataRate[2];
    uint8_t status = ADF_MMapWrite(RADIO_CFG_0, 0x02, (unsigned char*) &phy_data_rate);
    //  return (ADF_MMapWrite(RADIO_CFG_0, 0x02, (unsigned char*) &phy_data_rate));
    ADF_MMapRead(RADIO_CFG_0, 0x02, (unsigned char*) dataRate);
}

void TRX_Freq_Deviation(uint16_t freq_dev_in_units)
{
  uint8_t freq_dev[2];
  unsigned char radio_cfg_1;
  ADF_MMapRead(RADIO_CFG_1,0x01,&radio_cfg_1);

  radio_cfg_1 &= 0x0F;

  freq_dev[0] = (freq_dev_in_units & 0x0F00) >> 8;
  freq_dev[0] = (freq_dev[0] << 4) | radio_cfg_1;

  freq_dev[1] = (freq_dev_in_units & 0x00FF);

  ADF_MMapWrite(RADIO_CFG_1, 0x02, &freq_dev[0]);
}

void TRX_IF_Filter_Bandwidth(uint8_t ifbw)
{
  unsigned char radio_cfg_9_reg;

  ADF_MMapRead(0x115,0x01,&radio_cfg_9_reg);

  radio_cfg_9_reg &= ~(0x3 << 6);
  radio_cfg_9_reg |= (ifbw << 6);

  ADF_MMapWrite(0x115, 0x01, &radio_cfg_9_reg);
}
int Retireve_rf_statistics(unsigned char *buf, unsigned int Bufpos, int len)
{
  if(Is_StatisticsEnabled())
  {  
    buf[Bufpos++] = 'S'; // start of statistics

    Bufpos = AddLongToBuffer(rf_stats.Tx_EOF + rf_stats.Rx_EOF , buf, Bufpos, len);                             // Long 0 - 5
    Bufpos = AddLongToBuffer(rf_stats.CRC_failure + rf_stats.CCA_Failed, buf, Bufpos, len);                     // Long 1 - 5

    Bufpos = AddLongToBuffer(clock_seconds(), buf, Bufpos, len);                                                // Int 0 - 3
    Bufpos = AddLongToBuffer((rf_stats.pkt_Tx_time * 4) /1000 , buf, Bufpos, len);                              // Int 1 - 3
    Bufpos = AddLongToBuffer((rf_stats.pkt_Rx_time * 4) /1000 , buf, Bufpos, len);                              // Int 2 - 3
    Bufpos = AddLongToBuffer((rf_stats.cca_time  * 4)   /1000 , buf, Bufpos, len);                              // Int 2 - 3
    Bufpos = AddLongToBuffer(rf_stats.CCA_Failed, buf, Bufpos, len);                                            // Int 3 - 3
    //#ifdef ENABLE_LOW_POWER_MODE  
    Bufpos = AddLongToBuffer(system_hibernate_duration, buf, Bufpos, len);                                      // Int 3 - 3
    //#endif    
    Bufpos = AddLongToBuffer(system_flexi_mode_sleep_duration, buf, Bufpos, len);                               // Int 3 - 3

    //Active Up-Channel Index
    Bufpos = AddByteToBuffer(phy_current_channel,buf, Bufpos, len);                                             // Int 3 - 3

    //RSSI pack
    Bufpos = AddByteToBuffer(rf_stats.RSSI_val, buf, Bufpos, len);                                              // Byte 1 - 2  

    //Transmit Power
    Bufpos = AddByteToBuffer(transmit_pwr, buf, Bufpos, len);                                                   // Int 3 - 3

    //Rx Repeats Stat
    Bufpos = AddByteToBuffer(0, buf, Bufpos, len);                                                              // Int 3 - 3

    //CCA Treshold
    Bufpos = AddByteToBuffer(-70, buf, Bufpos, len);                                                            // Byte 1 - 2

    //BatVolt
    Bufpos = AddWordToBuffer(0, buf, Bufpos, len); 
  }
  
  return Bufpos;
}
