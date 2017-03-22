/*------------------------------------------------------------------------------
THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES INC. ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT, ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES INC. BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

YOU ASSUME ANY AND ALL RISK FROM THE USE OF THIS CODE OR SUPPORT FILE.

IT IS THE RESPONSIBILITY OF THE PERSON INTEGRATING THIS CODE INTO AN APPLICATION
TO ENSURE THAT THE RESULTING APPLICATION PERFORMS AS REQUIRED AND IS SAFE.
-------------------------------------------------------------------------------*/


/*------------------------------------------------------------------*/
/* 	    Stages of the radio link state machine	            */
/*------------------------------------------------------------------*/

#ifndef __ADF7023_H__
#define __ADF7023_H__

#include <stdbool.h>
#include "net/netstack.h"
#include "device.h"

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
typedef enum {FALSE = 0, TRUE = !FALSE} boolean;

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
  PHY_NOEVENT            = 0x00,
#ifdef DISABLE_PATCH_802_15_4D
  PHY_RX_CRCOK           = 0x01,
  PHY_SYNCHDET           = 0x02,
  PHY_TX_END             = 0x04,
  PHY_HDR_DETECT         = 0x08
#else
  PHY_PREAMBLE_DETECT    = 0x01,
  PHY_PHR_DETECT         = 0x02,
  PHY_CRC_CORRECT        = 0x04,
  PHY_TX_EOF             = 0x08,
  PHY_RX_EOF             = 0x10,
  PHY_BUFFER_ALMOST_FULL = 0x20,
  PHY_BUFFER_FULL        = 0x40,
  PHY_CCA                = 0x80
#endif // DISABLE_PATCH_802_15_4D      
}PHYEVENTS;

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

#ifndef DISABLE_PATCH_802_15_4D
typedef struct 
{
  union 
  {
    uint16_t Number_Of_Bytes_To_Send;
    uint16_t Number_Of_Bytes_To_Receive;
  } Remaining;
  
  union
  {  
    uint16_t Number_Of_Bytes_Sent;
    uint16_t Number_Of_Bytes_Received;
  } Accomplished;
  
  uint8_t *pPacket;
} ADI_ADF7023_BUFFER_TYPE;
#endif // DISABLE_PATCH_802_15_4D

typedef struct rf_statistics{
  uint32_t PHR_detect;
  uint32_t CRC_failure;
  uint32_t Rx_EOF;
  int8_t  RSSI_val;
  uint32_t re_init_count;
  uint32_t Tx_EOF;
  uint32_t CCA_Failed;
  uint32_t pkt_Tx_time;
  uint32_t pkt_Rx_time;
  uint32_t cca_time;
  uint32_t Rx_timeout_re_init_count;
  uint32_t Tx_timeout_re_init_count;
  uint32_t TxStateFail_re_init_count;
  //@debug
  uint32_t tx_drop_count_for_full_buff;
  uint32_t tx_drop_count_for_full_nbr;
}rf_statistics_t;

signed char     GetRSSIReadBackReg(void);
unsigned char   ADF_IssueCommand(unsigned char Cmd);
unsigned char   ADF_ChangeChans(unsigned long ChFreq);
unsigned char*  ADF_GetSiVer(void);
unsigned char   ADF_GetMSBSiVer(void);
unsigned char   ADF_MMapRead(unsigned long ulAdr,unsigned long ulLen,unsigned char *pData);
unsigned char   ADF_TransmitPacket(unsigned char* pucTXData, unsigned char TxPwr);
unsigned char   GetPHYON_RSSI(signed char* RSSIVal);
unsigned char   ADF_RadioInit(unsigned char ProfileNo, ISMBAND ISMBand, U32 Freq, U8 InitParm);
unsigned char   ADF_GetNoPHYProfiles(void);

unsigned char   ADF_PwrMgt(TRPSSTATE PDState);
unsigned char   ADF_PrepBSMode(void);

unsigned char ADF_GoToOnState(void);
unsigned char ADF_GoToOffState(void);

U16             ADF_GetPATTO(void);
U16             ADF_GetPackTO(void);

#ifdef DISABLE_PATCH_802_15_4D
PHYEVENTS GetPHYEventStatus(unsigned char *PackBuffer);
#else 
PHYEVENTS GetPHYEventStatus_15D4(unsigned char *PackBuffer);
#endif // DISABLE_PATCH_802_15_4D

#define SUCCESS                             0
#define FAILURE                             1

#define SI_MSBVER_02                        0x02
#define BBRam_MMemmap_Start                 0x100
#define ANODE_DEFAULT_PROFILE               4 // 100kbps
#define INIT_TRSC_PORTS                     0x01
#define INIT_TRSC_CNGPHY                    0x02
#define INIT_TRSC_RX                        0x04

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

#ifndef DISABLE_PATCH_802_15_4D
#define TRE_INVALID_CCA_RSSI_THERSHOLD        0xB5
#define TRE_INVALID_TIMER_DURATION            0xB6
#define TRE_PACKET_SIZE_OVERFLOW              0xB7
#define TRE_TO_15D4_ON_STATE                  0xB8
#define TRE_TO_15D4_TX_STATE                  0xB9
#define TRE_TO_15D4_RX_CCA_STATE              0xBA
#define TRE_TO_ON_FROM_15D4_ON_STATE          0xBB
#define TRE_EXIT_15D4                         0xBC

#define MAKE_UINT16(lsb,msb)                  (uint16_t)(lsb) | (((uint16_t)(msb)) << 8)

#define ADI_SPI_MASTERCON_INITIALIZER      BITM_SPI_CTL_MASEN

#define ADI_SPI_DMA_INITIALIZER     0
        
#endif // DISABLE_PATCH_802_15_4D

#define adi_int_EnterCriticalRegion(arg)    __disable_irq()

#define adi_int_ExitCriticalRegion(arg)    __enable_irq()

#define TRANSMIT_POWER                        0x06

#if (I_SUPPORT_4BYTE_FCS == 0x01)
/* Polynomial to calculate 32 bit CRC */
#define CRCPOLY2 0xEDB88320UL
#endif // I_SUPPORT_4BYTE_FCS

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
  unsigned char antenna_RX_diversity_cfg_r;          // 0x129
  unsigned char TX_anetnna_cfg_r;                    // 0x12A   
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
  unsigned char vco_band_readback_r;                 // 0x13B
  unsigned char vco_ampl_readback_r;                 // 0x13C
} TyBBRAM15_4d;

typedef struct
{
  unsigned char bb_cca_cfg_0;                        // 0x103   
  unsigned char bb_cca_cfg_1;                        // 0x104   
  unsigned char bb_ca_threshold;                     // 0x105
} TyBBRAM15_4d_CCA;

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

#endif // DISABLE_PATCH_802_15_4D


typedef struct
{
  unsigned char ProfileNo;
  ISMBAND ISMBand;
  U32 Freq;
  U8 InitParm;
} ADI_ADF7023_GENERIC_SETTINGS_TYPE; 

typedef struct
{                       
  VU32 fw_state          : 5;   // Indicate the current state of the MAC Processor
  VU32 cmd_loader_empty  : 1;   // 0:RC not ready for MAC, 1 RC is ready for MAC cmd
  VU32 irq_status        : 1;   // 1: Pending interrupt condition
  VU32 spi_ready         : 1;   // 0:SPI is not ready for access, 1 SPI is ready
}ADF_STA;


typedef union
{
  VU32          Reg;
  ADF_STA       Bits;
}ADFSTA_Reg;

__packed
typedef struct
{
  unsigned char gain_status;
  unsigned char gain_correction;
} agc_gain_map;

extern TyBBRAM         gBBRAM;
extern ADF_CmdCodes    mPHYState;
extern ADFSTA_Reg mADF7023Status;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
// BBRAM Constants
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef DISABLE_PATCH_802_15_4D

#define interrupt_mask_0_interrupt_num_wakeups           (0x1 << 7) 
#define interrupt_mask_0_interrupt_swm_rssi_det          (0x1 << 6) 
#define interrupt_mask_0_interrupt_aes_done              (0x1 << 5) 
#define interrupt_mask_0_interrupt_tx_eof                (0x1 << 4) 
#define interrupt_mask_0_interrupt_address_match         (0x1 << 3) 
#define interrupt_mask_0_interrupt_crc_correct           (0x1 << 2) 
#define interrupt_mask_0_interrupt_sync_detect           (0x1 << 1) 
#define interrupt_mask_0_interrupt_premable_detect       (0x1 << 0)

#else

#define interrupt_mask_0_interrupt_cca                   (0x1 << 7)
#define interrupt_mask_0_interrupt_buffer_full           (0x1 << 6)
#define interrupt_mask_0_interrupt_buffer_almost_full    (0x1 << 5)
#define interrupt_mask_0_interrupt_rx_eof                (0x1 << 4)
#define interrupt_mask_0_interrupt_tx_eof                (0x1 << 3)
#define interrupt_mask_0_interrupt_crc_correct           (0x1 << 2) 
#define interrupt_mask_0_phr_detect                      (0x1 << 1) 
#define interrupt_mask_0_interrupt_premable_detect       (0x1 << 0)

#define BB_CCA_THRESHOLD                                 0x105
#define BB_CCA_CFG_0                                     0x103
#define BB_antenna0_rssi 		                 0x12B
#define BB_antenna1_rssi 		                 0x12C

#define ANTENNA_0					0 
#define ANTENNA_1 					2

#define MAX_PDSU_15_4D  157 //0x800 // v4.01 firmware supports this for old v4.00 it should be max 0x400
#define USE_PDSU_15_4D  (MAX_PDSU_15_4D-1)

#define BB_antenna_RX_diversity_cfg		         0x129
#define BB_TX_antenna_cfg_r 		                 0x12A
#define PHR_FCSMASK					  0x10
#define PHR_LENMSBMASK 					     7

#define RADIO_CFG_0                                      0x10C
#define RADIO_CFG_1                                      0x10D
#define RADIO_CFG_2                                      0x10E
#define PREAMBLE_LENGTH_REG                              0x11D
#define PREAMBLE_LENGTH_LOW_REG                          0x12E


#define PR_phy_rx_status                                 0x01F  // for 4.01 version
#define PREAMBLE_SFD_DETECT				     3

//#define TX_BASE_154D  PKT_RAM_TXBASE
#define TX_SZ_15_4D   20 /*0x80*/
//#define RX_BASE_154D  PKT_RAM_RXBASE
#define RX_SZ_15_4D   20 /*0x80*/

#endif // DISABLE_PATCH_802_15_4D

#define MCR_interrupt_source_0_Adr                       0x336  
#define MCR_interrupt_source_1_Adr                       0x337
#define MCR_calibration_control_Adr                      0x338
#define MCR_calibration_status_Adr                       0x339
#define MCR_PA_Level_Adr              			 0x307

#define MCR_PA_Level_Adr              			 0x307
#define MCR_CONFIG_HIGH              			 0x30C
#define MCR_CONFIG_LOW              			 0x30D

#define BB_CHANNEL_FREQ_0                                0x109

#define AGC_MODE_REG                                     0x35D
#define AGC_GAIN_STATUS_REG                              0x360
#define ADC_READBACK_HIGH_REG                            0x327
#define ADC_READBACK_LOW_REG                             0x328

#define FREE_RUNNING_AGC                                     0x00
#define HOLD_AGC                                             0x40

#define PKT_RAM_SIVER                       0x01
#define ANODE_NO_OF_BANDS                   3
#define ANODE_NO_OF_PROFILES                6

#define MAX_TX_PWR                  63



/*************************************************************************
        Function Prototypes
*************************************************************************/
#ifndef DISABLE_PATCH_802_15_4D
extern unsigned char ADF7023_15d4g_SetCCARSSIThreshold( uint8_t cca_threshold );
extern unsigned char ADF7023_15d4g_SetCCATimerDuration( uint8_t TimeValue );
extern unsigned char ADF7023_15d4g_EnableAutoTx( bool AutoTxflag );
extern unsigned char ADF7023_15d4g_StartCCATimer(void);
extern void cbTxBufferAlmostFull (void);
extern void cbTxBufferFull (void);
extern void cbRxBufferAlmostFull(unsigned char *pBuffer);
extern void cbRxBufferFull(unsigned char *pBuffer);
extern unsigned char ADF_GoTo15D4OnMode(void);
extern unsigned char ADF_GoTo15D4TxMode(void);
extern unsigned char adi_ADF7023_SetPhyTx15D4(void);
extern unsigned char adi_ADF7023_SetPhyOn15D4(void);
extern unsigned char adi_ADF7023_SetPhyRxCCA15D4(void);
extern unsigned char ADF_GoTo15D4RxCCAMode(void);
extern unsigned char adi_ADF7023_Transmit_15d4_Frame(void);
extern unsigned char cb154dPacketReceived (unsigned char *pBuffer);
static unsigned char AdfClearInterrupts(void);
extern void adi_ADF7023_154D_PatchRoutine(void);
extern unsigned char adi_ADF7023_config_15d4_BBRAM_Regs(void);
static void adi_ADF7023_154d_bbram_setting(TyBBRAM15_4d *pBBRAM);
extern unsigned char ADF7023_15d4g_TxAntennaConfig(uint8_t Antenna_Path_0, uint8_t Antenna_Path_1, 
                                                   ADI_ADF7023J_15D4_TRX_ATB_LEVEL ATB_Level,ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED Antenna_Select);
extern unsigned char ADF7023_15d4g_RxAntennaDivConfig(uint8_t Antenna_Path_0, uint8_t Antenna_Path_1, ADI_ADF7023J_15D4_TRX_ATB_LEVEL ATB_Level, 
                                                      ADI_ADF7023J_15D4_RX_ANTENNA_DIVERSITY ADA_Enable, ADI_ADF7023J_15D4_TRX_ANTENNA_SELECTED Antenna_Select);
extern unsigned char adi_ADF7023_Receive_15d4_Frame(void);
extern unsigned char ADF7023_15d4g_Send_Pkt(uint16_t Phr_Header, uint8_t *PSDUdata, uint8_t CCA_Therhold, uint8_t TimerValue, bool CCA_EnableFlag, unsigned char TxPwr);
extern unsigned char ADF7023_ReadAntennaRSSI(int8_t *RSSI_r);
extern unsigned char ADF_GoTo15D4PHYOnMode(void);
extern unsigned char ADF_Exit15D4(void);

#if (I_SUPPORT_4BYTE_FCS == 1)
extern void Compute_4byte_CRC(uint8_t *databuff, uint16_t packet_length);
extern void swapbits(uint8_t *data);
extern unsigned int CRC32(unsigned char *c, int n);
#endif // (I_SUPPORT_4BYTE_FCS == 1)

#endif // DISABLE_PATCH_802_15_4D

uint16_t get_pkt_tx_time(void);

#endif // __ADF7023_H__