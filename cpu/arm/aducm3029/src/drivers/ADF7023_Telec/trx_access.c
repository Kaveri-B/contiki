
/** \file trx_access.c
 **
 ** \brief Functions for accessing the transeiver functionality
 **
 ** \cond STD_FILE_HEADER
 **
 ** COPYRIGHT(c) 2010-11 Procubed Technology Solutions Pvt Ltd.
 ** All rights reserved.
 **
 ** THIS SOFTWARE IS PROVIDED BY "AS IS" AND ALL WARRANTIES OF ANY KIND,
 ** INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR USE,
 ** ARE EXPRESSLY DISCLAIMED.  THE DEVELOPER SHALL NOT BE LIABLE FOR ANY
 ** DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE. THIS SOFTWARE
 ** MAY NOT BE USED IN PRODUCTS INTENDED FOR USE IN IMPLANTATION OR OTHER
 ** DIRECT LIFE SUPPORT APPLICATIONS WHERE MALFUNCTION MAY RESULT IN THE DIRECT
 ** PHYSICAL HARM OR INJURY TO PERSONS. ALL SUCH IS USE IS EXPRESSLY PROHIBITED.
 **
 *******************************************************************************
 **  \endcond
 */

/*! \addtogroup TRX_Access TRX Access
 *  @{
 */
/*******************************************************************************
* File inclusion
*******************************************************************************/
#ifndef ADI_IAR
/* system service includes  */
#include <services/services.h>
/* device manager includes  */
#include <drivers/adi_dev.h>
/* System services init includes */
#include "adi_ssl_init.h"
/* UHF includes */
#endif
#include "system.h"
#include "telec_common.h"
#include <adf7023.h>
#include "phy/phy.h"
#ifndef ADI_IAR
#include "board.h"
#endif
#include "queue_latest.h"
#include "event_manager.h"
#include "list_latest.h"
#include "trx_access.h"
#ifndef ADI_IAR
#include "bf_timer.h"
#endif
#include "hw_tmr.h"
#include "sw_timer.h"
#ifdef FEC_LIB_INCLUDED
#include "lib802_15_4_NEC.h"
#endif

//#include "fec.h"
//#include "statistics.h"
#include "timer_service.h"
#include "utest_support.h"

extern void raise_int( ushort int_num );

/*
** ============================================================================
** Private Macro definitions
** ============================================================================
*/

#define MAX_TX_INTERVAL      ( (2047 * 20 * 8 ) + (10000) + 1000 )
#define MAX_RX_INTERVAL      ( (2047 * 20 * 8 ) + (10000) + 1000 )

/*Enable the following macro if it is required to timestamp the ack timings*/

#define MRFSK_SFD_LEN						2 //in bytes

#define MRFSK_POSTAMBLE_LEN 				2 //in bytes

#define MRFSK_PHR_LEN						2 //in bytes

#define DATA_WHITENING_ENABLED				0x0010

#define TX_2BUF

#define INDEX_TO_PHR_LSB					0x01

#define INDEX_TO_PD_PLD						0x03

#define PHR_DW_BIT_MASK						0x10

#define DW_INDICATOR						0x8000

#define MAX_ACK_FRAME_SIZE_WITHOUT_FEC		7

#define ACK_REQUIRED						(1 << 5)

#define SYNTH_CAL_TIME_OUT_VAL				(60)  //in seconds


#define Is_PSDU_PN9_Encoded()					\
	( rx_frame_info.FCSLength & DW_INDICATOR )

#define Get_PA_Level( pwr_dbm ) \
	pa_level_mcr[ ((pwr_dbm) - MIN_OUTPUT_POWER )/2 ]


#define FREQ_BAND_ID_MASK		0x07c00000
/*
** ============================================================================
** Private Structures, Unions & enums Type Definitions
** ============================================================================
*/

typedef struct rx_frame_info_tag
{
	uint8_t* link;
	uint32_t sfd_rx_time;
	uint16_t psduLength;
	uint16_t FCSLength;
	int psduLinkQuality;
	uint32_t channel;
	uint8_t* psdu;
}rx_frame_info_t;

#ifdef DEBUG_ACK_TIMINGS
p3time_t rx_complete_time;
p3time_t rx_start_time;
#endif


enum
{
    TRX_EVENT_CMD_READY 		= ADI_ADF7023_EVENT_CMD_READY,
    TRX_EVENT_CMD_FINISHED 		= ADI_ADF7023_EVENT_CMD_FINISHED,
    TRX_EVENT_PKT_TX_COMPLETE 	= ADI_ADF7023_EVENT_PACKET_TRANSMITTED,
    TRX_EVENT_PKT_RECIEVED 		= ADI_ADF7023_EVENT_PACKET_RECEIVED,
    TRX_EVENT_SYNC_WORD_DETECT 	= ADI_ADF7023_EVENT_SYNC_WORD_DETECT,
	TRX_EVENT_SPORT_RX_DONE		= ADI_ADF7023_EVENT_SPORT_RX_DONE,
    TRX_EVENT_UNSUPPORTED 		= ADI_ADF7023_UNSUPPORTED_EVENT,
#ifdef PATCH_802_15_4D
    TRX_EVENT_CCA_DONE			= ADI_ADF7023_15D4_EVENT_CCATIMER_EXPIRED,
    TRX_EVENT_PHR_DETECT		= ADI_ADF7023_15D4_EVENT_PHR_DETECT,
#endif
	TRX_EVENT_ERR_PKT_RECIEVED 	= ADI_ADF7023_EVENT_ERR_PACKET_RECEIVED,
};

volatile uint8_t transceiver_event;

#ifdef SPORT_SUPPORT
typedef enum {
	IDLE = 0,
	RECEIVING_PHR, 		//here we have started the first RX dma to receive data up to including PHR
	RECEIVING_PKT_REMAINDER,		//here we are dma'ing the rest of the packet.
	DETECT_SYNC,
	RECEIVE_CONTINUOUS
} SPORT_PACKET_RX_STATE;

typedef struct {
	SPORT_PACKET_RX_STATE state;
	uint8_t *pkt_data;		//pointer to buffer containing the packet data
	uint8_t *pkt_wptr;		//write pointer where we are filling in the packet data
	ADI_ADF7023_BUFFER_TYPE *rxbuf;
	ADI_ADF7023_BUFFER_TYPE *buf1; //PPV
	ADI_ADF7023_BUFFER_TYPE *buf2; //PPV new
	int rx_buf_len;		//total length of buffer used to rx the packet
	int first_dma_len;	//number of bytes received in the first dma to get the PHR
    int rx_pkt_len; 	//number of bytes of packet data received including sync header
    int remaining_pkt_dmabytes;
    int next_dma_len;
    int prev_dma_len;
    int rx_dma_irq_cnt;
    int ping_pong_flag;	 //PPV

} SPORT_PACKET_RX;
#endif //#ifdef SPORT_SUPPORT

/*
** ============================================================================
** Private Variable Definitions
** ============================================================================
*/

sw_tmr_t sw_timer1, sw_timer2;

uint8_t rx_buffer[ RX_SZ_15_4D ];

// Handle to the ADF7023 device driver
#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif
static ADI_ADF7023_DEV_HANDLE   hDevice;

#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif

static ADI_ADF7023_BUFFER_TYPE RxFrame;

static ADI_ADF7023_BUFFER_TYPE *pRxFrame;

#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif

static rx_frame_info_t rx_frame_info;

/*frame to be transmitted and the frame received will
be in this location*/

#ifndef PATCH_802_15_4D

#pragma section ("L1_data_a")

static uint8_t tx_buffer[ TX_BUFFER_LEN ];
#ifdef TX_2BUF
#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif
static uint8_t tx_buffer2[ TX_BUFFER_LEN * 2 ];
#endif

#endif //#ifndef PATCH_802_15_4D

#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif

u8 Continuous_Rx_Mode = 0;

#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif
u8 IR_Calibration_Active = 0;

#ifdef TRX_ACCESS_DEBUG
#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif
static int pkt_tx_irq = 0;
#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif
static int pkt_rx_irq = 0;

static int sync_detect_irq = 0;
#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif
static int soft_irq = 0;

static int trx_error_counter = 0;

#endif //#ifdef TRX_ACCESS_DEBUG

static uint32_t total_symbols_transmitted, total_symbols_tx = 0;

static uint8_t const pa_level_mcr[] =
{
	2,
	3,
	5,
	6,
	8,
	11,
	16,
	20,
	25,
	29,
	33,
	37,
	42,
	47,
	51,
	55,
	58,
	63
};

static trx_data_event_cb_t trx_cb;

static volatile uint32_t trx_event;

#ifdef SPORT_SUPPORT
#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif
static SPORT_PACKET_RX packet_receiver = {IDLE, NULL, NULL, NULL,NULL,NULL,0,0,0,0,0,0,0,0};

#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif
static int sport_rx_pkt_count=0;
#ifndef ADI_IAR
#pragma section ("L1_data_a")
#endif
static int sport_err=0;

#endif //#ifdef SPORT_SUPPORT

uint8_t currentTxMode = 0;

#ifdef BENCH_MARKING

extern uint64_t ccaTimeStamp1,ccaTimeStamp2,ackTimeStamp1,ackTimeStamp2, TxToRxTimeStamp1, TxToRxTimeStamp2;
extern uint32_t cca_time_diff[256],ack_time_diff[256],time_diff_TX_to_RX[256];
extern uint8_t cca_ind, ack_ind, TxtoRxInd;

#endif

extern uint8_t transmit_pwr;
extern uint16_t phy_current_channel;
extern uint32_t system_hibernate_duration;
extern uint32_t system_flexi_mode_sleep_duration;

extern phy_pib_t phy_pib;

extern sw_tmr_t sw_timer1, sw_timer2;

static void start_synth_cal_timer( void );

void set_synth_cal_pending(void *ptr);

static sw_tmr_t synth_cal_timer;

void wakeup_GPIO_configure(bool_t input);

/*
** ============================================================================
** Public Variable Definitions
** ============================================================================
*/

rf_statistics_t rf_statistics;

#define ADUCM3025_MAX_PACKET_LEN      0x96

/* No of bytes occupied by various fields in the Tx'd/Rx'd Radio Frames */
#define SIZE_LEN_FIELD  (1)

extern uint8_t rx_buf[ ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD ];

extern u32 ApplicationError(u32 nError);

void trx_error_trap(void);
ADI_ADF7023_RESULT_TYPE TRX_ContReceiveEnable(bool bState);

#if 1
#define trap()    trx_error_trap()
#else
#define trap()    trx_error_counter++
#endif

extern uint8_t rx_buffer[];



/*
** ============================================================================
** External Variable Declarations
** ============================================================================
*/

extern ADI_ADF7023_DEV_HANDLE sport_uhf_handle;

extern void WakeupCallback1(void *pCBParam, uint32_t Event, void *pArg);

/*
** ============================================================================
** Private Function Prototypes
** ============================================================================
*/

static uint32_t Build_SHR
					(
						SHR_t* p_shr_info,
						uint8_t* p_frame
					);

static uint32_t Get_SHRLen( SHR_t* p_shr_info);

static void RadioCallback
		(
			void* pCBParam,
			uint32_t nEvent,
			void* pArg
		);

#ifndef ADI_IAR
static ADI_INT_HANDLER(TRX_SoftInterruptHandler);
#else
void TRX_SoftInterruptHandler(void *pCBParam, uint32_t Event, void *pArg);
#endif

ADI_ADF7023_RESULT_TYPE ADF7023_15d4g_SetBitContinousRX(ADI_ADF7023_DEV_HANDLE const hDevice, bool state);

void adi_ADF7023_SPORT_RX_Continuous_Data_App_cb(ADI_ADF7023_BUFFER_TYPE *current_buf);

void Reset_sport_generic_RX( void );

void TRX_Reload_15d4g_firmware(void);
/*
** ============================================================================
** Public Function Definitions
** ============================================================================
*/

#define Check_IRcal_Active()		\
	if(IR_Calibration_Active)	\
	{	\
		return ADI_ADF7023_ERR_RADIO_IR_CAL_BUSY;	\
	}

int global_trap_cnt=0;
void global_trap(void)
{
	global_trap_cnt++;
}
void trx_error_trap(void)
{
#ifdef STATISTICS_ENABLED
	rf_statistics.radio_trx_err++;
#endif//#ifdef STATISTICS_ENABLED
	//while(1);
	global_trap();
}
#ifdef SPORT_SUPPORT
void sport_err_trap(void)
{
	sport_err++;
#ifdef STATISTICS_ENABLED
	rf_statistics.radio_sport_err++;
#endif//#ifdef STATISTICS_ENABLED
	global_trap();
}
#endif
#ifdef ADI_IAR
uint8_t TRX_Init(ADI_SPI_HANDLE hDevice_SPI, ADI_SPI_DEV_ID_TYPE spidev, IRQn_Type const irq_flag)
#else
uint8_t TRX_Init( void )
#endif
{
    ADI_ADF7023_RESULT_TYPE	result;
    volatile uint32_t i,j;

    //init ATB1 flag pin and reset to 0
    //InitTestFlags();

    sw_tmr_create
    (
      gptmr_mod_ins,
      &(sw_timer1),
      SW_TMR_ONESHOT,
      MAX_TX_INTERVAL,
      (sw_tmr_cb_t)&phy_reset_802_15_4D_driver,
      NULL
    );

    sw_tmr_create
    (
      gptmr_mod_ins,
      &(sw_timer2),
      SW_TMR_ONESHOT,
      MAX_RX_INTERVAL,
      (sw_tmr_cb_t)&phy_reset_802_15_4D_rx_driver,
      NULL
    );

	ADI_ADF7023_GENERIC_SETTINGS_TYPE gen_settings;
    gen_settings.Frequency               = 901000000;
#ifdef USE_50K_DATARATE
    gen_settings.DataRate                = 500;
#else
    gen_settings.DataRate                = 1000;
#endif
    gen_settings.ShortAddr.ShortAddr     = 0x01FF;
    gen_settings.ShortAddr.ShortAddrMask = 0xFFFF;
    gen_settings.ModScheme               = MOD_2GFSK;
    gen_settings.DemodScheme             = DEMOD_2FSK_GFSK_MSK_GMSK;

	transceiver_event = 0;
#ifdef STATISTICS_ENABLED
	rf_statistics.radio_trx_init++;
#endif//#ifdef STATISTICS_ENABLED
#ifdef ADI_IAR
	result = adi_ADF7023_Init(&hDevice, hDevice_SPI, spidev, irq_flag, (ADI_ADF7023_CALLBACK)RadioCallback, NULL);
#else
	result = adi_ADF7023_Init(&hDevice, (u32)SPI_DEV_NUMBER, ADI_FLAG_PH0, ADI_FLAG_PH6, (ADI_ADF7023_CALLBACK)RadioCallback, NULL);
#endif
	// Set Discriminator bandwidth & post demodulater bandwidth  based on data rate
    if(!result)
    {
    	if(gen_settings.DataRate == 1000)
    	{
       	 // Setting for 100kbps rate
    	 result = adi_ADF7023_SetFrequencyDeviation(hDevice,(gen_settings.DataRate)/2);
    	 result = adi_ADF7023_SetDiscrimBW(hDevice,0x20);
    	 result = adi_ADF7023_SetPostDemodBW(hDevice, 0x26);
	    }
    	else
    	{
    	 // Setting for 50kbps rate will be used
    	 result = adi_ADF7023_SetFrequencyDeviation(hDevice,(gen_settings.DataRate)/2);
       	 result = adi_ADF7023_SetDiscrimBW(hDevice, 0x82);
    	 result = adi_ADF7023_SetPostDemodBW(hDevice, 0x13);
	    }
    }

	if(!result)
		result = adi_ADF7023_SetGenericSettings(hDevice, &gen_settings);

#ifdef ADI_DEBUG
	    ADI_ADF7023_GENERIC_SETTINGS_TYPE settings_test;
	    if(!result)
	        result = adi_ADF7023_GetGenericSettings(hDevice, &settings_test);
#endif

#ifndef  PATCH_802_15_4D

	if(!result)
        result = adi_ADF7023_SetPreambleLen(hDevice, (u8)0x4);
    if(!result)
        result = adi_ADF7023_SetAddressMatchOffset(hDevice, (u8)1);
    if(!result)
        result = adi_ADF7023_SetSyncWord(hDevice, (u8)16, 0x904e); //802.15.4 sync word
    if(!result)
        result = adi_ADF7023_SetPacketEndian(hDevice, (bool)1); //set LSB first (802.15.4)
    if(!result)
        result = adi_ADF7023_SetClearAutoCRC(hDevice, (bool)0);
    if(!result)
        result = adi_ADF7023_SetClearFixedPktLen(hDevice, (bool)1); //fixed packet length
    if(!result)
        result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);
    volatile int timeout = 200000;

    while(!result && timeout--);    /* the command config dev requires some time to complete. either
                                       use the callback to figure out exactly when the function completes
                                       or sit here for a little while. either is ok */
#endif

	if(!result)
        result = adi_ADF7023_SetBlockingMode(hDevice, true);


    if(!result)
        result = adi_ADF7023_SetPhyOn(hDevice);

#ifdef  DO_IR_CALIBRATION

	TRX_do_IR_Calibration();

#endif

#ifdef PATCH_802_15_4D

    if(!result)
    	result = adi_ADF7023_154D_PatchRoutine(hDevice);

    if(!result)
    	result = adi_ADF7023_config_15d4_BBRAM_Regs(hDevice); //config BBRAM registers for 15d4


	adi_ADF7023_EnableSynthCalPending(hDevice, 0x01);

	//start_synth_cal_timer();
    SetSynthCalPending(TRUE);

	if(!result)
		result = adi_ADF7023_15d4g_Enable_Disable_Auto_SynthCalibration(hDevice, 0);

    hDevice->FastTransitionsEnabled = 1;

#else

	if(!result)
		result = adi_ADF7023_SportInit(hDevice, adi_dev_ManagerHandle, adi_dma_ManagerHandle, SPORT_DEVICE_NUMBER);

    if(!result)
        result = adi_ADF7023_EnableSportMode(hDevice, false);
#endif

#ifdef ANTENNA_DIVERSITY
	if(!result)
		result = ADF7023_15d4g_RxAntennaDivConfig(hDevice, 2, 1, ADI_ADF7023J_15D4_VDD_DRV, ADI_ADF7023J_15D4_AD_ENABLE, ADI_ADF7023J_15D4_ANTENNA0);//Both Antenna
	if(!result)
		result = ADF7023_15d4g_TxAntennaConfig(hDevice,2,1,ADI_ADF7023J_15D4_VDD_DRV,ADI_ADF7023J_15D4_ANTENNA1);
#else
	if(!result)
		result = ADF7023_15d4g_RxAntennaDivConfig(hDevice, 2, 1, ADI_ADF7023J_15D4_VDD_DRV, ADI_ADF7023J_15D4_AD_DISABLE, ADI_ADF7023J_15D4_ANTENNA0);//Both Antenna
	if(!result)
		result = ADF7023_15d4g_TxAntennaConfig(hDevice,2,1,ADI_ADF7023J_15D4_VDD_DRV,ADI_ADF7023J_15D4_ANTENNA1);
#endif

    if ( result )
    {
    	trap();
    	return FAILURE;
    }


	return SUCCESS;
}
/******************************************************************************/

uint8_t TRX_Register_Data_Event_cb(trx_data_event_cb_t cb)
{
	if ( NULL != cb )
	{
#ifdef ADI_IAR
		if(adi_gpio_RegisterCallback(RADIO_INT_NUM, TRX_SoftInterruptHandler, NULL) != ADI_GPIO_SUCCESS)
#else
		if(adi_int_CECHook(RADIO_INT_NUM, TRX_SoftInterruptHandler, NULL, TRUE) != ADI_INT_RESULT_SUCCESS)
#endif
	    {
	        trap();
	    	return FAILURE;
	    }

	    trx_cb = cb;
	    return SUCCESS;
	}

	return FAILURE;
}

/******************************************************************************/

uint8_t TRX_Reset(void)
{
	return SUCCESS;
}


/******************************************************************************/

uint8_t TRX_On( void )
{
#ifdef UTEST_TRX
	utu_timestamp(UTUL_TRX_OFF,0);
#endif
	Check_IRcal_Active();

	ADI_ADF7023_RESULT_TYPE uhf_result = adi_ADF7023_SetPhyOn(hDevice);


	if ( uhf_result )
    {
    	trap();
    	return FAILURE;
    }
	return SUCCESS;
}

/******************************************************************************/
#ifndef ADI_IAR
#pragma default_section (CODE,"L1_code")
#endif
uint8_t TRX_TX_On( void )
{
#ifdef BENCH_MARKING
	benchMark_set_time(22);
#endif
#ifdef UTEST_TRX
	utu_timestamp(UTUL_TRX_TX,0);
#endif
	Check_IRcal_Active();

	ADI_ADF7023_RESULT_TYPE uhf_result = adi_ADF7023_SetPhyTx(hDevice);
	if ( uhf_result )
    {
    	trap();
    	return FAILURE;
    }

#ifdef BENCH_MARKING

    benchMark_set_time(30);

#endif

	return SUCCESS;
}

/******************************************************************************/
uint8_t TRX_Rx_On( void )
{
#ifdef UTEST_TRX
	utu_timestamp(UTUL_TRX_RX,0);
#endif
	Check_IRcal_Active();

	ADI_ADF7023_RESULT_TYPE uhf_result = adi_ADF7023_SetPhyRx(hDevice);

	if ( uhf_result )
    {
    	trap();
    	return FAILURE;
    }
	return SUCCESS;
}

/******************************************************************************/
#ifndef ADI_IAR
#pragma default_section (CODE,"sdram0_bank1")
#endif
uint8_t TRX_Off( void )
{
	Check_IRcal_Active();

	ADI_ADF7023_RESULT_TYPE uhf_result = adi_ADF7023_SetPhyOff(hDevice);
	if ( uhf_result )
    {
    	trap();
    	return FAILURE;
    }
	return SUCCESS;
}

/******************************************************************************/

uint8_t TRX_Wakeup(void)
{
	ADI_ADF7023_RESULT_TYPE uhf_result = (ADI_ADF7023_RESULT_TYPE)SUCCESS;
	if ( uhf_result )
    {
    	trap();
    	return FAILURE;
    }
	return SUCCESS;
}

/******************************************************************************/

#ifdef PATCH_802_15_4D

uint8_t TRX_Get_RSSI_Method_1( int *pRSSIval )
{
    ADI_ADF7023_PHY_STATE_TYPE prev_state = hDevice->PhyState;
	ADI_ADF7023_RESULT_TYPE uhf_result = SUCCESS;
    TRX_ContReceiveEnable(true);
	if(!uhf_result)
	{

		uhf_result = ADF7023_ReadAntennaRSSI
					 (
						 	hDevice,
						 	(int *)pRSSIval
					 );

		if ( uhf_result )
	    {
#ifdef TRX_ACCESS_DEBUG
	    	trx_error_counter++;
#endif //#ifdef TRX_ACCESS_DEBUG
	    	uhf_result = (ADI_ADF7023_RESULT_TYPE)FAILURE;
	    }
	    else
	    {
			uhf_result = (ADI_ADF7023_RESULT_TYPE)SUCCESS;
	    }
	}

    if ( !uhf_result )
    {
        uhf_result = TRX_ContReceiveEnable(false);
        if((*pRSSIval > -90) && (prev_state != PHY_ON_15D4))
        {
            if(!uhf_result)
                uhf_result = TRX_Rx_15d4On();
        }
    }

	return (uint8_t)uhf_result;
}

uint8_t TRX_Get_RSSI_Method_4( int *pRSSIval )
{

	ADI_ADF7023_RESULT_TYPE uhf_result = SUCCESS;
    uhf_result = ADF7023_15d4g_SetBitContinousRX(hDevice, true);
	if(!uhf_result)
	{

		uhf_result = ADF7023_ReadAntennaRSSI
					 (
						 	hDevice,
						 	(int *)pRSSIval
					 );

		if ( uhf_result )
	    {
#ifdef TRX_ACCESS_DEBUG
	    	trx_error_counter++;
#endif //#ifdef TRX_ACCESS_DEBUG
	    	uhf_result = (ADI_ADF7023_RESULT_TYPE)FAILURE;
	    }
	    else
	    {
			uhf_result = (ADI_ADF7023_RESULT_TYPE)SUCCESS;
	    }
	}

        if(!uhf_result){
          uhf_result =  ADF7023_15d4g_SetBitContinousRX(hDevice, false);
        }

	return (uint8_t)uhf_result;
}

#endif

#ifndef PATCH_802_15_4D
uint8_t TRX_Get_RSSI_Method_2( char* pRSSIval )
{
	ADI_ADF7023_RESULT_TYPE uhf_result = adi_ADF7023_GetRSSI
										 (
										 	hDevice,
										 	(u8 *) pRSSIval
										 );

	if ( uhf_result )
    {
    	//trap();
#ifdef STATISTICS_ENABLED
		rf_statistics.radio_rssi_fail++;
#endif//#ifdef STATISTICS_ENABLED
    	return FAILURE;
    }

	return SUCCESS;
}

/******************************************************************************/

#pragma section ("L1_code")
uint8_t TRX_Get_RSSI_Method_3( int* pRSSIval )
{
	ADI_ADF7023_RESULT_TYPE uhf_result = adi_ADF7023_SPORTmode_GetRSSI
										 (
										 	hDevice,
										 	pRSSIval
										 );


	if ( uhf_result )
    {
    	//trap();
#ifdef STATISTICS_ENABLED
		rf_statistics.radio_rssi_fail++;
#endif//#ifdef STATISTICS_ENABLED
    	return FAILURE;
    }

    //SetTestFlag(0,0);
	return SUCCESS;
}

//#pragma section ("L1_code")
uint8_t TRX_Get_RSSI_Method_CCA( int* pRSSIval )
{
	ADI_ADF7023_RESULT_TYPE uhf_result = adi_ADF7023_GetRSSI_CCA_mode
										 (
										 	hDevice,
										 	pRSSIval
										 );


	if ( uhf_result )
    {
    	//trap();
#ifdef STATISTICS_ENABLED
		rf_statistics.radio_rssi_fail++;
#endif//#ifdef STATISTICS_ENABLED
    	return FAILURE;
    }

//    SetTestFlag(0,0);
	return SUCCESS;
}

//#pragma section ("L1_code")
uint8_t TRX_CCA_RSSI_START()
{
	ADI_ADF7023_RESULT_TYPE uhf_result = adi_ADF7023_RSSI_CCA_mode_start
										 (
										 	hDevice
										 );


	if ( uhf_result )
    {
    	//trap();
#ifdef STATISTICS_ENABLED
		rf_statistics.radio_rssi_fail++;
#endif//#ifdef STATISTICS_ENABLED
    	return FAILURE;
    }

//    SetTestFlag(0,0);
	return SUCCESS;
}

#endif //#ifndef PATCH_802_15_4D

//#pragma section ("L1_code")
uint8_t TRX_CCA_RSSI_STOP()
{
	ADI_ADF7023_RESULT_TYPE uhf_result = adi_ADF7023_RSSI_CCA_mode_stop
										 (
										 	hDevice
										 );


	if ( uhf_result )
    {
    	//trap();
#ifdef STATISTICS_ENABLED
		rf_statistics.radio_rssi_fail++;
#endif//#ifdef STATISTICS_ENABLED
    	return FAILURE;
    }

//    SetTestFlag(0,0);
	return SUCCESS;
}


uint32_t TRX_Total_Symbols_Txed(void)
{
    return total_symbols_transmitted;

}

uint32_t TRX_Total_TX_Duration()
{
    return total_symbols_tx;
}

void TRX_Reset_Total_Symbols_Txed(void)
{
    total_symbols_transmitted = 0;
}



/******************************************************************************/
#ifndef ADI_IAR
#pragma default_section (CODE,"L1_code")
#endif

uint8_t TRX_Write
		(
			SHR_t* p_shr_info,
			uint16_t phr,
			uint8_t* pd_data,  //includes even the FCS field
			uint32_t pd_length // includes even the FCS field len
#ifdef PATCH_802_15_4D
			, bool CCA_Enable,
			uint8_t CCAThreshHold15D4,
			uint8_t CCATimerDuration15D4
#endif
		)
{

    static int sixlowpan_tx_time = 0;

#ifdef UTEST_TRX
	utu_timestamp(UTUL_TRX_WRITE,0);
#endif


	ADI_ADF7023_RESULT_TYPE	uhf_result;

#ifndef PATCH_802_15_4D
	uint16_t i;

	uchar extra_byte_cnt = 0;

	uint32_t shr_len;

	uint32_t loop_var = 0;

	uint8_t fec_coding;

	uint8_t* p_frame = tx_buffer;

	uint16_t fec_frame_len_in_bits = (pd_length + 2)*8;

	uint8_t* p_phr;

#endif //PATCH_802_15_4D

	uint8_t ack = 0x00;

	if(( pd_data[0] & 0x07 ) == 0x02 )
	{
		ack = 0x01;
	}

#ifndef PATCH_802_15_4D
	if( !( shr_len = Build_SHR(p_shr_info, p_frame ) ) )
	 	return FAILURE;

	if( (shr_len + pd_length) > TX_BUFFER_LEN)
		return FAILURE;

#ifdef BENCH_MARKING
	benchMark_set_time(57);
#endif

	p_frame += shr_len;

	p_phr = p_frame;

	*p_frame++ = phr & 0xff;
	*p_frame++ = phr >> 8;

	/* copy the PHY payload.  */
	memcpy(p_frame,pd_data,pd_length);

#ifdef BENCH_MARKING
	benchMark_set_time(58);
#endif

	fec_coding = p_shr_info->shr_info.mr_fsk_shr.fec_coding;

	/*check if FEC Encoding is needed*/
	if( fec_coding & MASK_FEC_BIT )
	{
		/*make sure that the buffer holding the frame is big enough to accomodate the
		FEC coding which doubles the number of bytes*/
		if( fec_coding & MASK_FEC_SCHEME_BIT )
		{


		}
		else
		{
			/*making sure that there are 3 tail bits at the end*/
			//p_frame[pd_length] = 0x00;
			//fec_frame_len_in_bits += 3;

			/*RSC scheme should be used*/
			/*NRNSC scheme should be used*/

#ifdef TX_2BUF

#ifdef BENCH_MARKING
					benchMark_set_time(80);
#endif

			 	memcpy(tx_buffer2, tx_buffer, shr_len);
#ifdef BENCH_MARKING
					benchMark_set_time(81);
#endif

				extra_byte_cnt = FEC_Encode(p_phr, (pd_length + 2), tx_buffer2 + shr_len);
#ifdef BENCH_MARKING
					benchMark_set_time(91);
#endif

#else
			//if fec coding happens, the new packet data will be copied to the
			//start of tx_buffer so we dont have to use a temp buffer.
			memcpy(tx_buffer, tx_buffer + TX_BUFFER_LEN/2, shr_len);
			extra_byte_cnt = FEC_Encode(p_phr, (pd_length + 2), tx_buffer + shr_len );
#endif

			pd_length = ( pd_length * 0x02 ) + extra_byte_cnt/*one extra tail byte need to be sent for the decoder on the dest node*/;


		}
	}

#ifdef BENCH_MARKING
	benchMark_set_time(59);
#endif

	/*check if data whitening (PN9 encoding) is needed*/
	if( phr & DATA_WHITENING_ENABLED )
	{
		uint8_t* p_pld;

#ifdef TX_2BUF
		if( fec_coding & MASK_FEC_BIT )
			p_pld = &tx_buffer2[shr_len + MRFSK_PHR_LEN];
		else
			p_pld = &tx_buffer[shr_len + MRFSK_PHR_LEN];
#else
		if( fec_coding & MASK_FEC_BIT )
			p_pld = &tx_buffer[shr_len + MRFSK_PHR_LEN];
		else
			p_pld = &tx_buffer[TX_BUFFER_LEN/2 + shr_len + MRFSK_PHR_LEN];
#endif

		/*perform data whitening on the psdu. The scrambled data displaces the
		input data in the memory*/
		if( fec_coding & MASK_FEC_BIT  )
		{

			p_pld += sizeof(phr);
		}
		for(i = 0; i < pd_length; i++)
		{
			swapbits(&p_pld[i]);
		}
		ScramblePSDU
		(
			p_pld,
			p_pld,
			pd_length
		);
		for(i = 0; i < pd_length; i++)
		{
			swapbits(&p_pld[i]);
		}
	}

#ifdef BENCH_MARKING
	benchMark_set_time(60);
#endif


	/*at this stage we have a packet which would have gone through
	1) FEC a
	2) Data whitening
	at the lcaotion specified by p_raw_tx_frame parameter which can be simply
	taken and sent over SPORT interface*/

   TxFrame.ElementCount = shr_len + sizeof(phr) + pd_length;


   if( fec_coding & MASK_FEC_BIT )
   {
   		TxFrame.ElementCount += sizeof(phr);
   }
#endif /*PATCH_802_15_4D*/

   /*add up the tx symbols to the total only if the frame under transmission
   is not an MAC ACK frame*/
   if(!ack)
   {
#ifndef PATCH_802_15_4D
	   total_symbols_transmitted += (uint32_t)(TxFrame.ElementCount << 3);
#else
	   total_symbols_transmitted += ((Get_SHRLen(p_shr_info) + sizeof(phr) + pd_length) << 3);
#endif
   }

   total_symbols_tx += ((Get_SHRLen(p_shr_info) + sizeof(phr) + pd_length) << 3) * HWTIMER_SYMBOL_LENGTH;

   sixlowpan_tx_time += ((10 + sizeof(phr) + pd_length) << 3) * HWTIMER_SYMBOL_LENGTH;

   rf_statistics.pkt_Tx_time = (sixlowpan_tx_time/4000);

#ifndef PATCH_802_15_4D

#ifdef TX_2BUF
	if( fec_coding & MASK_FEC_BIT )
	   TxFrame.pData = tx_buffer2;
    else
	   TxFrame.pData = tx_buffer;

#else
    if( fec_coding & MASK_FEC_BIT )
	    TxFrame.pData = tx_buffer;
    else
	   TxFrame.pData = tx_buffer  + TX_BUFFER_LEN/2;
#endif

   TxFrame.bBufferProcessed = false;

#endif /*PATCH_802_15_4D*/

#ifdef FAST_TRANSITION_ENABLED

	hDevice->auto_tx_to_rx_enabled = pd_data[0] & ACK_REQUIRED;
	adi_ADF7023_SetTxToRxAutoTurnAround(hDevice, hDevice->auto_tx_to_rx_enabled);

#endif

#ifdef  PATCH_802_15_4D

	/* 1) Transmits the packet in 15D4 mode.
	   2) If the CCA_Enable set to true driver does the CCA and
	      transmits the packet if channel is free.
	   3) CCA status(Channel Busy or Free) is notified by interrupt callback.
	 */
	uhf_result = ADF7023_15d4g_Send_Pkt(hDevice, phr, pd_data, CCAThreshHold15D4, CCATimerDuration15D4, CCA_Enable);

#else
	//we do not want to bring the PHY to ON state after completion of the
   	//transmission. This function should just initiate transmission and come out
   	uhf_result = adi_ADF7023_SPORT_TX_packet(hDevice, &TxFrame);

#endif


	if ( uhf_result )
	{
		trap();
		return FAILURE;
	}

#ifdef BENCH_MARKING

#ifndef  PATCH_802_15_4D

	timeStamp2 = gpTmr_mod_ins->hw_timer_if.hw_tmr_get_time(
					gpTmr_mod_ins->hw_timer_if.pHw_tmr_ins );

	time_diff[ind++] = timeStamp2 - timeStamp1;


	benchMark_set_time(61);

#endif //PATCH_802_15_4D

	ackTimeStamp2 = gpTmr_mod_ins->hw_timer_if.hw_tmr_get_time(
						gpTmr_mod_ins->hw_timer_if.pHw_tmr_ins );

	ack_time_diff[ack_ind++] = ackTimeStamp2 - ackTimeStamp1;

#endif

    if (CCA_Enable)
        start_802_15_4D_reset_timer(0);
    else
        start_802_15_4D_reset_timer(2);

	return SUCCESS;
}

/******************************************************************************/

uint8_t TRX_Set_RX_Buffer( void )
{
    ADI_ADF7023_RESULT_TYPE	uhf_result = (ADI_ADF7023_RESULT_TYPE)SUCCESS;
#ifdef UTEST_TRX
	utu_timestamp(UTUL_SET_RX_BUF,0);
#endif

    //RxFrame.ElementCount = RX_BUFFER_LEN;
    RxFrame.ElementCount = MAX_PDSU_15_4D;
    RxFrame.pData = rx_buffer;

    RxFrame.bBufferProcessed = false;

#ifndef PATCH_802_15_4D

    uhf_result = adi_ADF7023_SPORT_RX_802_15_4g_packet(hDevice, &RxFrame);

#endif
    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}

/******************************************************************************/

uint32_t TRX_get_RX_packet_len( void )
{
	return rx_frame_info.psduLength;
}

/******************************************************************************/

uint32_t TRX_get_RX_packet_timestamp( void )
{
	return hw_tmr_get_symbols(rx_frame_info.sfd_rx_time);
}

/******************************************************************************/

uint8_t TRX_Read_RX_Buffer(uint8_t* pd_data, uint32_t pd_length)
{
    uint32_t length = 0;
    bool is_data_whitened = false;
    uint16_t i;



    if( pd_data != NULL )
    {
#ifdef PATCH_802_15_4D

    	memcpy( pd_data, (uint8_t*)&rx_frame_info, sizeof(rx_frame_info_t) );

#else
    	is_data_whitened = (rx_frame_info.FCSLength & DW_INDICATOR)?true:false;

    	rx_frame_info.FCSLength  &=  ~DW_INDICATOR;

    	memcpy( pd_data, (uint8_t*)&rx_frame_info, sizeof(rx_frame_info_t) );



    	pd_data += sizeof( rx_frame_info_t );

    	if( is_data_whitened )
    	{
    		for(i = 0; i < pd_length; i++)
			{
				swapbits(&(pRxFrame->pData[INDEX_TO_PD_PLD + i]));
			}

    		DescramblePSDU
			(
				&(pRxFrame->pData[INDEX_TO_PD_PLD]),
				pd_data,
				pd_length
			);
			for(i = 0; i < pd_length; i++)
			{
				swapbits(&pd_data[i]);
			}

    	}
    	else
    	{
    		memcpy( pd_data , &(pRxFrame->pData[ INDEX_TO_PD_PLD ]),pd_length );
    	}
#endif

    	memset((uint8_t*)&rx_frame_info, 0, sizeof(rx_frame_info_t));


    	return SUCCESS;

    }

    trap();
    return FAILURE;

}
/******************************************************************************/
#ifndef ADI_IAR
#pragma default_section (CODE,"sdram0_bank1")
#endif
uint8_t TRX_Set_Output_Power( int8_t pwr_dbm, uint8_t* p_pa_mcr_level )
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	uint8_t status, pa_level;

	*p_pa_mcr_level = Get_PA_Level( pwr_dbm );
	pa_level = ( *p_pa_mcr_level - 3)/4;

	uhf_result = adi_ADF7023_SetTransmitterPowerLevel( hDevice,pa_level );

	if(!uhf_result)
    	uhf_result = adi_ADF7023_SetTransmitterPowerLevelMCR(hDevice, *p_pa_mcr_level);

	if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}

/******************************************************************************/

uint8_t TRX_Set_PA_Level_MCR( uint8_t level )
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;
	uint8_t status;

	uint8_t pa_level = ( level - 3)/4;

	uhf_result = adi_ADF7023_SetTransmitterPowerLevel( hDevice, pa_level );

	if(!uhf_result)
		uhf_result = adi_ADF7023_SetTransmitterPowerLevelMCR(hDevice, level);

	if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}
int TRX_Get_PA_Level_MCR(void)
{
    return hDevice->mcrPALevel;
}
uint32_t TRX_Get_Channel_Frequency()
{
    return adi_ADF7023_GetChannelFrequency(hDevice);
}

/******************************************************************************/

uint8_t TRX_Set_Channel_Frequency( uint32_t center_freq )
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

	uhf_result = adi_ADF7023_SetChannelFrequency( hDevice,center_freq );
//	if(!uhf_result)
//       uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    volatile int timeout = 1000;
    do {
        timeout--;
    } while(timeout > 0);

    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}

/******************************************************************************/

uint8_t TRX_Freq_Deviation( uint16_t freq_dev )
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

	uhf_result = adi_ADF7023_SetFrequencyDeviation( hDevice, freq_dev );
	if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    volatile int timeout = 1000;
    do {
        timeout--;
    } while(timeout > 0);

    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}

/******************************************************************************/

uint8_t TRX_IF_Filter_Bandwidth( uint8_t ifbw )
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

	uhf_result = adi_ADF7023_SetIFFilterBandwidth( hDevice, ifbw );

	if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    volatile int timeout = 1000;
    do {
        timeout--;
    } while(timeout > 0);

    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}


/******************************************************************************/
#ifdef PATCH_802_15_4D
uint8_t TRX_CCA_Filter_Bandwidth( uint8_t cca_filter_bw )
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

	uhf_result = DynamicIFFilterSet_CCATimerMode( hDevice,
												  ENABLE_IFBW_AUTO_SWITCH,
												  ENABLE_IFBW_SWITCH_ON_PREAMBLE,
												  cca_filter_bw);

    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}

#endif

/******************************************************************************/

uint8_t TRX_Discrim_PostDemod_Bandwidth( uint8_t discrim_bw, uint8_t post_demod_bw )
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

	uhf_result = adi_ADF7023_SetDiscrimBW( hDevice, discrim_bw );
	if(!uhf_result)
		uhf_result = adi_ADF7023_SetPostDemodBW( hDevice, post_demod_bw );
	if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    volatile int timeout = 1000;
    do {
        timeout--;
    } while(timeout > 0);

    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}

/******************************************************************************/

uint8_t TRX_Set_Data_Rate( uint16_t data_rate )
{
    Check_IRcal_Active();
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uhf_result = adi_ADF7023_SetDataRate( hDevice, data_rate );

    if(!uhf_result)
    uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    volatile int timeout = 1000;
    do {
        timeout--;
    } while(timeout > 0);

    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}

/******************************************************************************/

uint16_t TRX_Get_Data_Rate( void )
{
  return( adi_ADF7023_GetDataRate( hDevice ));
}

/******************************************************************************/

uint8_t TRX_Mod_Scheme( uint8_t mod_scheme )
{
    Check_IRcal_Active();
    ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

    uhf_result = adi_ADF7023_SetModulationScheme(hDevice,(ADI_ADF7023_MOD_TYPE)mod_scheme );
    if(!uhf_result)
    uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    volatile int timeout = 1000;
    do {
        timeout--;
    } while(timeout > 0);

    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}

/******************************************************************************/

uint8_t TRX_DeMod_Scheme( uint8_t demod_scheme )
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

	uhf_result = adi_ADF7023_SetDemodulationScheme(hDevice,(ADI_ADF7023_DEMOD_TYPE)demod_scheme );
	if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    volatile int timeout = 1000;
    do {
        timeout--;
    } while(timeout > 0);

    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}


/******************************************************************************/

uint8_t TRX_SetSyncWord( uint8_t bitlen, uint32_t word )
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

	uhf_result = adi_ADF7023_SetSyncWord(hDevice, bitlen, word );
	if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);

    volatile int timeout = 1000;
    do {
        timeout--;
    } while(timeout > 0);

    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}


/******************************************************************************/
uint8_t TRX_Read_pa_mcr_level( void )
{
	uint8_t pa_level;
	adi_ADF7023_BlockMemRead(hDevice, &pa_level, 1, POWER_AMPLIFIER);
	return pa_level;
}

/******************************************************************************/

int8_t TRX_Convert_pa_mcr_level_to_dBm( uint8_t pa_mcr_level )
{
	int i = 0;
	if ( ( pa_mcr_level>0 ) && ( pa_mcr_level<=63 ) )
	{
		for (i=0; i<sizeof(pa_level_mcr); i++ )
		{
			if ( pa_level_mcr[ i ] >= pa_mcr_level )
			{
				return ( pa_level_mcr[ i ] == pa_mcr_level )?
				(2*i + MIN_OUTPUT_POWER):(2*(i-1) + MIN_OUTPUT_POWER);
			}
		}

		return i;
	}
	else
	{
		/*should not happen*/
		return i;
	}
}

/******************************************************************************/

uint8_t TRX_Get_State(uint8_t* p_phy_state)
{
	ADI_ADF7023_PHY_STATE_TYPE phy_state = PHY_BUSY_TRX;
	ADI_ADF7023_RESULT_TYPE uhf_result =
	adi_ADF7023_GetPhyState(hDevice, &phy_state);

	if(!uhf_result)
	{
		*p_phy_state = (uint8_t)phy_state;
		return SUCCESS;
	}

    trap();
	return FAILURE;
}

/******************************************************************************/


/*
** ============================================================================
** Private Function Definitions
** ============================================================================
*/

/**
 * @brief   Example ADF7023 callback
 * @param[in]   *pCBParam       Callback parameter provided initially to driver by application
 * @param[in]   nEvent          The event that generated this callback
 * @param[in]   *pArg           pArg varies depending upon the event. It could either be a
 *                              command that was completed, or the address of a buffer either
 *                              transmitted or received.
 * @return  None
 */

#ifndef ADI_IAR
#pragma default_section (CODE,"L1_code")
#endif
static void RadioCallback
			(
				void* pCBParam,
                uint32_t  nEvent,
                void* pArg
            )
{
	uint64_t cycles_now;

	if( TRX_EVENT_CMD_FINISHED == nEvent )
	{
        ADI_ADF7023_CMD_TYPE cmd = (ADI_ADF7023_CMD_TYPE)pArg;
    }

    if(TRX_EVENT_PKT_TX_COMPLETE == nEvent )
    {
        rf_statistics.Tx_EOF++;

    /*	testVal1++;
    	if((testVal1 % 50) == 0)
    	{
    		UpdateMask(hDevice, 1);
    	}*/
		if(Continuous_Rx_Mode)
			return;		//disable calls to MAC/PHY code.

#ifdef BENCH_MARKING

#ifdef  PATCH_802_15_4D


		cca_time_diff[cca_ind++] = ccaTimeStamp2 - ccaTimeStamp1;


#endif //PATCH_802_15_4D

#endif
		ADI_ADF7023_BUFFER_TYPE *TxFrame = (ADI_ADF7023_BUFFER_TYPE * ) pArg;
#ifdef TRX_ACCESS_DEBUG
    	pkt_tx_irq++;
#endif //#ifdef TRX_ACCESS_DEBUG

    	if(trx_cb == NULL)
        {
        	trx_packet_sent_cb();
        }
        else
        {

#ifdef UTEST_TRX
	utu_timestamp(UTUL_TRX_RADIOCB_TX_PKT_DONE,0);
#endif
        	transceiver_event |= TX_COMPLETE;
        	//asm("raise 12;");
        	raise_int(RADIO_INT_NUM);
        }


    }
#ifdef PATCH_802_15_4D

    if( TRX_EVENT_CCA_DONE == nEvent )
    {
#ifdef BENCH_MARKING

		ccaTimeStamp1 = gpTmr_mod_ins->hw_timer_if.hw_tmr_get_time(
							gpTmr_mod_ins->hw_timer_if.pHw_tmr_ins );

#endif
    	if (((ADI_ADF7023_DEV_HANDLE)pArg)->bCCA_Status)
    	{

    		/* Notify the higher layer, as channel is busy */
    		transceiver_event |= CCA_DONE_BUSY;
    	}

    	else
    	{

    		/* Notify the higher layer, as channel free and
    		   data transmission follows. */
    		transceiver_event |= CCA_DONE_SUCCESS;
    	}

    	//asm("raise 12;");
		raise_int(RADIO_INT_NUM);

    }
#endif

    if( TRX_EVENT_PKT_RECIEVED == nEvent )
    {
        rf_statistics.Rx_EOF++;

		if(Continuous_Rx_Mode)
			return;		//disable calls to MAC/PHY code.


#ifdef BENCH_MARKING

		benchMark_set_time(0);

#endif

#ifdef PATCH_802_15_4D
    	pRxFrame = (((ADI_ADF7023_DEV_HANDLE)pArg)->pPacket);
#else
		pRxFrame = (ADI_ADF7023_BUFFER_TYPE *) pArg;
#endif
#ifdef TRX_ACCESS_DEBUG
        pkt_rx_irq++;
#endif //#ifdef TRX_ACCESS_DEBUG

#ifdef PATCH_802_15_4D
		/* get the FCS type of data received. */
        rx_frame_info.FCSLength = (((ADI_ADF7023_DEV_HANDLE)pArg)->PHR_FCS_Type)?2:4;

		rx_frame_info.psdu = pRxFrame->pData;

		if ((((ADI_ADF7023_DEV_HANDLE)pArg)->bCRC_Correct_Status == false) )
		{
            rf_statistics.CRC_failure++;
//            if(phy_trx_enable_status())
//            {
                transceiver_event |= RX_COMPLETE;
                //asm("raise 12;");
                raise_int(RADIO_INT_NUM);
//            }
//            else
//            {
//                rf_statistics.CRC_failure++;
//                TRX_Rx_15d4On();
//                phy_pib.TRXState = PHY_RX_ON;
//            }

     		return;
		}

        pRxFrame->ElementCount = (((pRxFrame->pData[0] & 0x07) << 8) | pRxFrame->pData[1]);
#endif

#ifdef DEBUG_ACK_TIMINGS
		rx_complete_time = sw_current_time_get(gptmr_mod_ins);
#endif


#ifndef PATCH_802_15_4D
		rx_frame_info.psduLength = pRxFrame->ElementCount - 3;
        /*b3 in the first byte of the PHR contains the info of FCS length.
        0 = 32-bit, 1 = 16-bit */
        rx_frame_info.FCSLength = ( pRxFrame->pData[ INDEX_TO_PHR_LSB ] & 0x08 )?2:4;

        /*set the last bit to  indicate presence of PN9 encoding
        in the recived packet*/

        if( pRxFrame->pData[ INDEX_TO_PHR_LSB ] & PHR_DW_BIT_MASK )
        {
        	rx_frame_info.FCSLength |= DW_INDICATOR;
        }
#else
		rx_frame_info.psduLength = pRxFrame->ElementCount;

#endif
        stop_802_15_4D_rx_reset_timer();

        if(trx_cb == NULL)
        {
        	trx_packet_rx_cb();
        }
        else
        {

#ifdef UTEST_TRX
	utu_timestamp(UTUL_TRX_RADIOCB_RX_PKT_DONE,0);
#endif
            if(phy_trx_enable_status())
            {
                //trx_event |= TRX_EVENT_PKT_RECIEVED;
                //trx_event |= RX_COMPLETE;
                transceiver_event |= RX_COMPLETE;
                //asm("raise 12;");
                raise_int(RADIO_INT_NUM);
            }
            else
            {
                adi_ADF7023_ReceiveIsrCallback ( rx_buf );
                return;
            }
        }

    }

    if( TRX_EVENT_ERR_PKT_RECIEVED == nEvent )
    {
    	((ADI_ADF7023_DEV_HANDLE)pArg)->bCRC_Correct_Status = false;

    	rx_frame_info.psduLength = 0;

    	transceiver_event |= RX_COMPLETE;
        //asm("raise 12;");
        raise_int(RADIO_INT_NUM);
    }


#ifdef PATCH_802_15_4D
    if( TRX_EVENT_PHR_DETECT == nEvent)
    {
        rf_statistics.PHR_detect++;
#ifdef TRX_ACCESS_DEBUG
    	sync_detect_irq++;
#endif //#ifdef TRX_ACCESS_DEBUG

    	if(hDevice->pPacket != &RxFrame)
    	{
    		hDevice->pPacket = &RxFrame;
            RxFrame.pData = rx_buffer;
    	}

		if(Continuous_Rx_Mode)
			return;		//disable calls to MAC/PHY code.

        rx_frame_info.psduLinkQuality = 0;
        /* Read the antenna RSSI to calculate LQI */
    	if(ADF7023_ReadAntennaRSSI(pArg, (int *)(&(rx_frame_info.psduLinkQuality)))
        	== ADI_ADF7023_SUCCESS )
        {
            rf_statistics.RSSI_val = rx_frame_info.psduLinkQuality;

			/* time stamp the received packet*/
	    	rx_frame_info.sfd_rx_time = gpTmr_mod_ins->hw_timer_if.hw_tmr_get_time(
						gpTmr_mod_ins->hw_timer_if.pHw_tmr_ins );

#ifdef DEBUG_ACK_TIMINGS
			rx_start_time = rx_frame_info.sfd_rx_time;
#endif
			if(trx_cb != NULL)
			{
#ifdef UTEST_TRX
	utu_timestamp(UTUL_TRX_RADIOCB_SYNC,0);
#endif
				//trx_event |= TRX_EVENT_SYNC_WORD_DETECT;
				//trx_event |= SFD_DETECTED;
				transceiver_event |= SFD_DETECTED;
				//asm("raise 12;");
				raise_int(RADIO_INT_NUM);

			}
        }
    }
#else

    if( TRX_EVENT_SYNC_WORD_DETECT == nEvent )
    {

        sync_detect_irq++;

		if(Continuous_Rx_Mode)
			return;		//disable calls to MAC/PHY code.

    	/*TBD: RSSI need to be converted to LQI*/
     	if(adi_ADF7023_SPORTmode_GetRSSI(pArg, &(rx_frame_info.psduLinkQuality))
        	== ADI_ADF7023_SUCCESS )
        {

			/* time stamp the received packet*/

			rx_frame_info.sfd_rx_time = gpTmr_mod_ins->hw_timer_if.hw_tmr_get_time(
						gpTmr_mod_ins->hw_timer_if.pHw_tmr_ins );
#ifdef DEBUG_ACK_TIMINGS
			rx_start_time = rx_frame_info.sfd_rx_time;
#endif
			if(trx_cb != NULL)
			{
#ifdef UTEST_TRX
	utu_timestamp(UTUL_TRX_RADIOCB_SYNC,0);
#endif
				//trx_event |= TRX_EVENT_SYNC_WORD_DETECT;
				//trx_event |= SFD_DETECTED;
				transceiver_event |= SFD_DETECTED;
				//asm("raise 12;");
				raise_int(RADIO_INT_NUM);

			}
        }
    }

	if(TRX_EVENT_SPORT_RX_DONE == nEvent) {
		if(Continuous_Rx_Mode == 0)
    		adi_ADF7023_SPORT_RX_802_15_4g_packet_cb(hDevice);
		else
			adi_ADF7023_SPORT_RX_Continuous_Data_cb(hDevice);
    }
#endif //PATCH_802_15_4D
}


/******************************************************************************/
#ifndef ADI_IAR
static ADI_INT_HANDLER(TRX_SoftInterruptHandler)
{

#ifdef BENCH_MARKING

    if(transceiver_event == RX_COMPLETE)
    {
		//intHandler++;

		benchMark_set_time(1);
    }

#endif
    ADI_INT_HANDLER_RESULT  Result = ADI_INT_RESULT_NOT_PROCESSED;

#ifdef TRX_ACCESS_DEBUG
    soft_irq++;
#endif //#ifdef TRX_ACCESS_DEBUG
    trx_cb( &transceiver_event );

    Result =  ADI_INT_RESULT_PROCESSED;
    return Result;
}
#else
void TRX_SoftInterruptHandler(void *pCBParam, uint32_t Event, void *pArg)
{
    trx_cb(&transceiver_event);
}
#endif
#ifndef PATCH_802_15_4D
/******************************************************************************/

/*!
 * @brief builds and copies the SHR into the place specified
 * @param[in] p_shr_info - contains all the info for building the SHR
 * @param[in] p_frame - place here the SHR needs to be built in the memory
 * @return SUCCESS or FAILURE.
 */

#pragma default_section (CODE,"sdram0_bank1")

static uint32_t Build_SHR( SHR_t* p_shr_info, uint8_t* p_frame  )
{
	uint32_t shr_len = 0 ;
	uint32_t loop_var;
	switch( p_shr_info->shr_type )
	{
		case SHR_TYPE_MR_FSK_SHR:
		{

			mr_fsk_shr_t* p_mr_fsk = &( p_shr_info->shr_info.mr_fsk_shr );

			for(loop_var = 0; loop_var<p_mr_fsk->preamble_rep; loop_var++)	//set preamble
				*p_frame++ = p_mr_fsk->preamble_byte;

			*p_frame++ = p_mr_fsk->sfd & 0xff;
			*p_frame++ = p_mr_fsk->sfd >> 8;

			shr_len = p_mr_fsk->preamble_rep + MRFSK_SFD_LEN;
		}
		break;
		default:
		break;
	}

	return shr_len;
}

#endif //#ifndef PATCH_802_15_4D

/******************************************************************************/
static uint32_t Get_SHRLen( SHR_t* p_shr_info)
{
	uint32_t shr_len = 0 ;
	switch( p_shr_info->shr_type )
	{
		case SHR_TYPE_MR_FSK_SHR:
			shr_len = (&( p_shr_info->shr_info.mr_fsk_shr ))->preamble_rep + MRFSK_SFD_LEN + MRFSK_POSTAMBLE_LEN;
		break;

		default:
		break;
	}

	return shr_len;
}

#ifdef SPORT_SUPPORT

/*********************************************************************

    Function:       get_PHR_pkt_len


    Description:   returns packet lenght field of PHR from
    				802.15.4g packet.

    				data passed is start of packet sync word

*********************************************************************/
int get_PHR_pkt_len(u8 *data)
{
	u8 *ptr = data;
	u16 psdu_len;
	int i;

	for(i=0; i<SYNC_WORD_LEN; i++)	//PHR starts right sync word. skip over.
		ptr++;

#if defined(MODE_802_15_4_B)
	return (int) *ptr;		//lenght is a simple byte
#endif

#if defined(MODE_802_15_4_G)
	{
		u16 phr = *ptr++;

		phr |= *ptr << 8;
		psdu_len = phr & 0xFFE0;

		psdu_len = (psdu_len & 0x00FF) << 8 | (psdu_len & 0xFF00) >> 8;
		psdu_len = (psdu_len & 0x0F0F) << 4 | (psdu_len & 0xF0F0) >> 4;
		psdu_len = (psdu_len & 0x3333) << 2 | (psdu_len & 0xCCCC) >> 2;
		psdu_len = (psdu_len & 0x5555) << 1 | (psdu_len & 0xAAAA) >> 1;


		return psdu_len;	//get PHR bits 5-15
	}
#endif
}
#endif //#ifdef SPORT_SUPPORT

/******************************************************************************/
#ifndef PATCH_802_15_4D

/*!
 * @brief receives a 802.15.4 PHY PSDU packet from the radio over the SPORT
 *
 * @param[in]   buf			type ADI_ADF7023_BUFFER_TYPE *.
							Pointer to buffer which will receive packet data.
 *							This will contain a pointer to the packet data buffer (pData)
 *							and the lenght of the buffer in bytes (ElementCount). bBufferProcessed
 *							should be initialied to false. When bBufferProcessed becomes true
 *							then the packet will have been received.
 *
 *
 * @return      ADI_ADF7023_RESULT_TYPE
 *
 * receives a 802.15.4 PHY PSDU packet from the radio over the SPORT.
 * A buffer should be passed to the function which can hold the complete received packet.
 * The packet will contain the last byte of the sync word followed by the PHR
 * and PSDU payload.  The function submits the data buffer for DMA on
 * the SPORT RX end and enables the SPORT data transfer. The application should
 * send a separate command to the ADF7023 to put the radio in RX mode. A callback
 * to the radio driver callback function passed to function adi_ADF7023_Init()
 * will be called with the event ADI_ADF7023_EVENT_PACKET_RECEIVED when the packet
 * has been received on the SPORT. The 3rd parameter, pArg, to the callback function
 * will contain the pointer to the received buffer of type (ADI_ADF7023_BUFFER_TYPE *).
 *
 * @note	The lenght of the buffer should be longer than the lenght of the
 *			expected packets to be received. The data buffer should remain
 *			allocated until after the SPORT transfer has completed. This function
 *			should not be called again until the previous packet has been
 *			received.
 */
//#pragma default_section (CODE,"L1_code")

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SPORT_RX_802_15_4g_packet(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const buf)
{
	int i;
	ADI_ADF7023_RESULT_TYPE	nResult;
	SPORT_PACKET_RX *ppacket_receiver = &packet_receiver;
	u8 *data = buf->pData;
	int len = buf->ElementCount;

	ppacket_receiver->rxbuf = buf;

	if(ppacket_receiver->state != IDLE)
	{
        //ApplicationError(nResult);
        sport_err_trap();
#if defined(ADI_DEBUG)
        printf("SPORT_RX_802_15_4_packet failed state is not IDLE\n");
#endif

		return ADI_ADF7023_ERR_UNKNOWN;
    }

    /* when starting DMA for RX of packet, we will start with 1 DMA which
    will include the packet header containing the lenght of the packet.
    We will get an callback on the first DMA so we can issue a DMA for the length
    of the rest of the packet. */


    ppacket_receiver->next_dma_len = raw_frame_handler_cb( ppacket_receiver->state );


    if((nResult = adi_ADF7023_Start_SPORT_RX(hDevice,data, ppacket_receiver->next_dma_len))
		!= ADI_DEV_RESULT_SUCCESS)
		return nResult;

    ppacket_receiver->state = RECEIVING_PHR;
    ppacket_receiver->pkt_data = data;
    ppacket_receiver->rx_buf_len = len;

    ppacket_receiver->rx_pkt_len = ppacket_receiver->next_dma_len;

    ppacket_receiver->prev_dma_len = ppacket_receiver->next_dma_len;

    return ADI_ADF7023_SUCCESS;
}

/******************************************************************************/
/* called back when the SPORT has completed an RX transfer. Gets PHR packet lenght for the
first RX transfer and then starts a new RX transfer for the rest of the packet. After the
second RX transfer, it passes the received packet to the upper layer. */
void adi_ADF7023_SPORT_RX_802_15_4g_packet_cb(ADI_ADF7023_DEV_HANDLE uhf_handle)
{
	SPORT_PACKET_RX *ppacket_receiver = &packet_receiver;
	u32 nResult;
	int packet_len = 0;
	int chunk_len = 0;
	int remaining_len;

	uint16_t prev_dma_len = ppacket_receiver->next_dma_len;

    ppacket_receiver->rx_dma_irq_cnt++;

    switch(ppacket_receiver->state)
    {
        case RECEIVING_PHR:		//got the Phy header. Read pkt len and get the rest of the pkt
        {
			u8 *data = ppacket_receiver->pkt_data;


			ppacket_receiver->next_dma_len = raw_frame_handler_cb( ppacket_receiver->state );

			packet_len = get_pld_len_cb();

			//packet_len will be lenght of data starting after the PHR.
			//so this will be the data coming after the 1st DMA
			if((packet_len < 0) ||
				(packet_len > (ppacket_receiver->rx_buf_len - ppacket_receiver->prev_dma_len)))
			{
				//error -- we do not have enough room in buffer!
				//abort operation.
				/* disable dataflow */
			    if((nResult = adi_ADF7023_SPORT_RX_stop_RX(uhf_handle))!= ADI_DEV_RESULT_SUCCESS)
			    {
			    	//dev_control_err
			    	//ApplicationError(nResult);
			    }
			    //ApplicationError(-1);
			    sport_err_trap();
			}

			if(packet_len == 0)
			{
				ppacket_receiver->rx_pkt_len = ppacket_receiver->prev_dma_len;
				ppacket_receiver->next_dma_len = 0;
				goto pkt_rx_done;
			}

						//submit DMA for rest of packet.
			if((nResult = adi_ADF7023_SPORT_RX_set_next_databuf(uhf_handle,
				(u8*) ppacket_receiver->pkt_data + ppacket_receiver->rx_pkt_len, ppacket_receiver->next_dma_len )
				)!= ADI_DEV_RESULT_SUCCESS)
			{
			    //ApplicationError(nResult);
			    sport_err_trap();
			}

			ppacket_receiver->state = RECEIVING_PKT_REMAINDER;
		    ppacket_receiver->rx_pkt_len += ppacket_receiver->next_dma_len;

		    ppacket_receiver->prev_dma_len = ppacket_receiver->next_dma_len;

		    break;
        }
        case RECEIVING_PKT_REMAINDER:	//got the rest of the packet

#ifdef BENCH_MARKING
        benchMark_set_time(70);
#endif
        //SetTestFlag(0,1);	//set ATB1 test flag
        ppacket_receiver->next_dma_len = raw_frame_handler_cb( ppacket_receiver->state );
pkt_rx_done:
        {


        	if( !( ppacket_receiver->next_dma_len ) )
			{

				/*all bytes have been read and this would be the last stage of decoding*/
					/* disable dataflow */
				if((nResult = adi_ADF7023_SPORT_RX_stop_RX(uhf_handle))!= ADI_DEV_RESULT_SUCCESS)
				{
				    //dev_control_err
				    //ApplicationError(nResult);
				    sport_err_trap();
				}

			    ppacket_receiver->state = IDLE;

				//if(packet_receiver->cb)
				//	packet_receiver->cb(packet_receiver->pkt_data, packet_receiver->rx_pkt_len, 0 /*no erro*/);
			    //ReceivingPkt = FALSE;
				//ppacket_receiver->rxbuf->ElementCount = ppacket_receiver->rx_pkt_len;
				ppacket_receiver->rxbuf->ElementCount = get_element_count_cb( ppacket_receiver->rx_pkt_len );
				ppacket_receiver->rxbuf->bBufferProcessed = true;
				sport_rx_pkt_count++;

#ifdef BENCH_MARKING
				benchMark_set_time(71);
#endif

				RadioCallback(NULL, TRX_EVENT_PKT_RECIEVED, ppacket_receiver->rxbuf);
				//SetTestFlag(0,0);
			}
			else
			{
				if((nResult = adi_ADF7023_SPORT_RX_set_next_databuf(uhf_handle,
				(u8*) ppacket_receiver->pkt_data + ppacket_receiver->rx_pkt_len, ppacket_receiver->next_dma_len )
				)!= ADI_DEV_RESULT_SUCCESS)
				{
				    //ApplicationError(nResult);
				    sport_err_trap();
				}

				ppacket_receiver->state = RECEIVING_PKT_REMAINDER;
			    ppacket_receiver->rx_pkt_len += ppacket_receiver->next_dma_len;

			    ppacket_receiver->prev_dma_len = ppacket_receiver->next_dma_len;

			}

			break;
		}
    }
}

/******************************************************************************/
#pragma default_section (CODE,"sdram0_bank1")

int Disable_RX_SPORT(void)
{

     u32 nResult;
	 ADI_DEV_DEVICE_HANDLE hSportRXHandle = sport_uhf_handle->hSportRX;

/* disable dataflow */
	if((nResult = adi_dev_Control(
			                    hSportRXHandle,
			                    ADI_DEV_CMD_SET_DATAFLOW,
			                    (void *)FALSE
			                    ))!= ADI_DEV_RESULT_SUCCESS)
	{
		//dev_control_err
		//ApplicationError(nResult);
		sport_err_trap();
	}

	packet_receiver.state = IDLE;

	Reset_sport_generic_RX();

    return ADI_ADF7023_SUCCESS;
}
#endif //PATCH_802_15_4D

/******************************************************************************/
#ifdef  DO_IR_CALIBRATION

/*!
 * @brief calls the ADF7023 driver IR calibration routine.
 *
 * @return ADI_ADF7023_RESULT_TYPE
 *
 * Calls the ADF7023 driver IR calibration routine. This function sets the flag
 * IR_Calibration_Active while the IR calibration routine is executing. This
 * flag is used by the TRX_access layer to detect if the IR calibration routine
 * is executing when another radio API is called. If that is true, then the
 * error code ADI_ADF7023_ERR_RADIO_IR_CAL_BUSY is returned by those API's.
 *
 */

ADI_ADF7023_RESULT_TYPE TRX_do_IR_Calibration(void)
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhfResult;

#ifdef UTEST_TRX
	utu_timestamp(UTUL_TRX_IRCAL,0);		//start
#endif
	IR_Calibration_Active = 1;
#ifdef STATISTICS_ENABLED
	rf_statistics.radio_ircal_cnt++;
#endif//#ifdef STATISTICS_ENABLED
	uhfResult = adi_ADF7023_do_IR_Calibration(hDevice);
	if(!uhfResult)
		TRX_Reload_15d4g_firmware();
	IR_Calibration_Active = 0;

#ifdef UTEST_TRX
	utu_timestamp(UTUL_TRX_IRCAL,0xffff);	//done
#endif

#ifdef STATISTICS_ENABLED
	if(uhfResult != ADI_ADF7023_SUCCESS)
		rf_statistics.radio_ircal_fail_cnt++;
#endif//#ifdef STATISTICS_ENABLED

	return(uhfResult);
}

#endif

#ifdef BENCH_MARKING

//#pragma section ("L1_code")

void benchMark_set_time(int i)
{
	ack_time_diff[i] = gpTmr_mod_ins->hw_timer_if.hw_tmr_get_time(
						gpTmr_mod_ins->hw_timer_if.pHw_tmr_ins );
}

#endif


#ifdef SPORT_SUPPORT
/*****************************************************************************************
For Continuous Receive functionality

Use double buffer scheme application pass both buffers to this function call.
Application should allocate these two buffers structures,
allocate the data buffers for them and initialize the buffer structure pointer pData
and elementcount before passing to this function.
*****************************************************************************************/

ADI_ADF7023_RESULT_TYPE adi_ADF7023_SPORT_RX_Continuous_Data(ADI_ADF7023_DEV_HANDLE const hDevice, ADI_ADF7023_BUFFER_TYPE* const buf1, ADI_ADF7023_BUFFER_TYPE* const buf2)
{
	int i;
	u32 nResult;
	SPORT_PACKET_RX *ppacket_receiver = &packet_receiver;
	u8 *data = buf1->pData;
	int len = buf1->ElementCount;

    //Two different buffer variable buf1 & buf2 added to packet_receiver structure and flag to keep track the 2 buffers.

	ppacket_receiver->rxbuf = buf1;  // Kept this to use earlier version code.
	ppacket_receiver->buf1 = buf1;
	ppacket_receiver->buf2 = buf2;

	if(ppacket_receiver->state != IDLE)
	{
        ApplicationError((u32) -1);
#if defined(ADI_DEBUG)
        printf("SPORT_RX_802_15_4_packet failed state is not IDLE\n");
#endif

		return ADI_ADF7023_ERR_UNKNOWN;
    }

#ifndef SPORT_RX_DETECT_SYNC
    /* when starting DMA for RX of packet, we will start with 1 DMA which
    will include the packet header containing the lenght of the packet.
    We will get an callback on the first DMA so we can issue a DMA for the length
    of the rest of the packet. */


    #ifdef NO_PREAMBLE_SYNCWORD
	// For NEC deliverables
	 if((nResult = adi_ADF7023_Start_SPORT_RX(hDevice,data, len))
		!= ADI_DEV_RESULT_SUCCESS)
		return (ADI_ADF7023_RESULT_TYPE) nResult;
	#else
	// Only for internal testing with first packet has header & PHR
	 if((nResult = adi_ADF7023_Start_SPORT_RX(hDevice,data, HEADER_LEN_INCL_PHR))
		!= ADI_DEV_RESULT_SUCCESS)
	    return (ADI_ADF7023_RESULT_TYPE) nResult;
	#endif

    #ifdef NO_PREAMBLE_SYNCWORD
    ppacket_receiver->state = RECEIVE_CONTINUOUS;
    #else
    ppacket_receiver->state = RECEIVING_PHR;
    #endif

    ppacket_receiver->pkt_data = data;
    ppacket_receiver->rx_buf_len = len;
    ppacket_receiver->first_dma_len = HEADER_LEN_INCL_PHR;
#else	/*SPORT_RX_DETECT_SYNC*/
	pSync = &Sync;
	pSync->State =  0;
	pSync->sync_offset = pSync->SFD_found = 0;

	#ifndef SYNC_DEBUG_SIMULATE
		if((nResult = adi_ADF7023_Start_SPORT_RX(hDevice, pkt_dma_buf, SYNC_DMA_LEN))
			!= ADI_DEV_RESULT_SUCCESS)
			return nResult;
	#endif

	ppacket_receiver->state = DETECT_SYNC;
    ppacket_receiver->pkt_data = data;
    ppacket_receiver->pkt_wptr = data;
    ppacket_receiver->rx_buf_len = len;
    ppacket_receiver->rx_pkt_len = 0;
    ppacket_receiver->next_dma_len = SYNC_DMA_LEN;

	#ifdef SYNC_DEBUG_BUFF
		init_sync_debug();
	#endif

	#ifdef SYNC_DEBUG_SIMULATE
	//simulate getting RX data from the sport
	init_sync_simu_debug();

	while(!buf->bBufferProcessed)
	{
		adi_ADF7023_SPORT_RX_802_15_4g_packet_cb(hDevice);
	}

	#endif
#endif

    return ADI_ADF7023_SUCCESS;
}

/****************************************************************************************************************/

void adi_ADF7023_SPORT_RX_Continuous_Data_cb(ADI_ADF7023_DEV_HANDLE uhf_handle)

{
	SPORT_PACKET_RX *ppacket_receiver = &packet_receiver;
	u32 nResult;
	u16 *p_fifo_short;
	u32 fifo_tmp;
	u8 data_cnt;

	// New ping pong buffer to submit for read continously


    ppacket_receiver->rx_dma_irq_cnt++;

#ifdef SYNC_DEBUG_BUFF
	if(ppacket_receiver->rx_dma_irq_cnt == 6)
		;//debug_halt();
#endif

    switch(ppacket_receiver->state)
    {
#ifdef SPORT_RX_DETECT_SYNC
        case DETECT_SYNC:
        	//save data from dma buf
        	if(LoadRX_fifo(&rx_fifo, pkt_dma_buf, SYNC_DMA_LEN) == -1)
        	{
        		ApplicationError((u32)-1);
        	}

#ifndef SYNC_DEBUG_SIMULATE
        	//start next dma
        	if((nResult = adi_ADF7023_SPORT_RX_set_next_databuf(uhf_handle, pkt_dma_buf, SYNC_DMA_LEN))
				!= ADI_DEV_RESULT_SUCCESS)
			{
        		ApplicationError(nResult);
        	}
#endif

        	//check for sync
        	data_cnt = SYNC_DMA_LEN;
        	while((p_fifo_short = GetRx_fifo_short(&rx_fifo)) != NULL)
        	{
				data_cnt -= 2;
        		pSync->inp = p_fifo_short;
        		FSK4g_Sync_Detection(pSync);

				if(pSync->SFD_found)
				{
					//sync found. Take the data which was not
					//part of the sync word and shift it all the way to the
					//lsb bits of an unsigned 32 bit word, int_bit_fifo
					//which will be used to assemble incoming data.
					int_bit_fifo = *p_fifo_short;
					int_bit_fifo >>= pSync->sync_offset;

					if(data_cnt > 0)
						get_pktdata_from_fifo(&rx_fifo, data_cnt);

        			ppacket_receiver->state = RECEIVING_PHR;
				}
        	}

        break;

        case RECEIVING_PHR:		//get and check the Phy header.
        {
			u8 *phr = ppacket_receiver->pkt_data;	//PHR will be first 2 bytes in pkt
			int psdulen;
			int remaining_pkt_dmabits;


			//save data from dma buf
        	if(LoadRX_fifo(&rx_fifo, pkt_dma_buf, SYNC_DMA_LEN) == -1)
        	{
        		ApplicationError(-1);
        	}

			get_pktdata_from_fifo(&rx_fifo, SYNC_DMA_LEN);

			//swap byte order of PHR -- not needed.
			//*(u16*) ppacket_receiver->pkt_data =
			//	ppacket_receiver->pkt_data[1] | (ppacket_receiver->pkt_data[0] << 8);

			psdulen = get_PHR_pkt_len(phr);

			/*
			Calculate how much remaining data we need to dma for the
			current packet:
			Remaining data to dma is:
			total data needed for dma -
			data that has been dma'd so far.
			Total data we need to dma in for packet after SFD is:
			(psdulen*8) + PHR_BITLEN
			Data we have dma'd so far after SFD is:
			data bits in int_bit_fifo +
			data already moved into packet buffer.
			*/

			remaining_pkt_dmabits = ((psdulen*8) + PHR_BITLEN) -
				((16-pSync->sync_offset) + /*SYNC_DMA_LEN*8 +*/
				ppacket_receiver->rx_pkt_len*8);

			if(remaining_pkt_dmabits <= 0)
			{
				//ppacket_receiver->rx_pkt_len = 2 /*PHR len*/ + psdulen;
				//for very short psdu (few bytes), there can be extra data in the packet buffer
				//beyond psdu data. This is a special case that maybe can be ignored.
				goto pkt_rx_done;
			}
			ppacket_receiver->remaining_pkt_dmabytes = remaining_pkt_dmabits >> 3;  //divide by 8

			//check if extra byte is needed in case data is not byte
			//aligned.
			if(remaining_pkt_dmabits & 0x7)
				ppacket_receiver->remaining_pkt_dmabytes += 1;

			//set size of next dma
			if(ppacket_receiver->remaining_pkt_dmabytes >= PKT_DMA_WINDOW_SIZE)
				ppacket_receiver->next_dma_len = PKT_DMA_WINDOW_SIZE;
			else
				ppacket_receiver->next_dma_len = ppacket_receiver->remaining_pkt_dmabytes;

#ifndef SYNC_DEBUG_SIMULATE

			//start next dma
        	if((nResult = adi_ADF7023_SPORT_RX_set_next_databuf(uhf_handle,
        						pkt_dma_buf, ppacket_receiver->next_dma_len))
				!= ADI_DEV_RESULT_SUCCESS)
			{
        		ApplicationError(nResult);
        	}
#endif

        	ppacket_receiver->remaining_pkt_dmabytes -= ppacket_receiver->next_dma_len;
        	ppacket_receiver->state = RECEIVING_PKT_REMAINDER;

			break;
        }

        case RECEIVING_PKT_REMAINDER:		//got the Phy header. Read pkt len and get the rest of the pkt
        {
        	//save data from dma buf
        	if(LoadRX_fifo(&rx_fifo, pkt_dma_buf, ppacket_receiver->next_dma_len) == -1)
        	{
        		ApplicationError(-1);
        	}

        	get_pktdata_from_fifo(&rx_fifo, ppacket_receiver->next_dma_len);

        	//more data? keep reading it.
        	if(ppacket_receiver->remaining_pkt_dmabytes > 0)
        	{
        		//set size of next dma
				if(ppacket_receiver->remaining_pkt_dmabytes >= PKT_DMA_WINDOW_SIZE)
					ppacket_receiver->next_dma_len = PKT_DMA_WINDOW_SIZE;
				else
					ppacket_receiver->next_dma_len = ppacket_receiver->remaining_pkt_dmabytes;

#ifndef SYNC_DEBUG_SIMULATE
				//start next dma
	        	if((nResult = adi_ADF7023_SPORT_RX_set_next_databuf(uhf_handle,
	        						pkt_dma_buf, ppacket_receiver->next_dma_len))
					!= ADI_DEV_RESULT_SUCCESS)
				{
	        		ApplicationError(nResult);
	        	}
#endif
	        	ppacket_receiver->remaining_pkt_dmabytes -= ppacket_receiver->next_dma_len;
        	}
        	else
        	{
pkt_rx_done:
        		//end of data, provide packet to upper layer.
#ifndef SYNC_DEBUG_SIMULATE
        		/* disable dataflow */
				if((nResult = adi_ADF7023_SPORT_RX_stop_RX(uhf_handle))!= ADI_DEV_RESULT_SUCCESS)
				{
				    //dev_control_err
				    ApplicationError(nResult);
				}
#endif

			    ppacket_receiver->state = IDLE;
                            flush_bit_fifo();
                            ppacket_receiver->rxbuf->ElementCount = ppacket_receiver->rx_pkt_len;
                            ppacket_receiver->rxbuf->bBufferProcessed = true;
                            sport_rx_pkt_count++;
        	}
        	break;
        }

#else  /* SPORT_RX_DETECT_SYNC */

    	case RECEIVING_PHR:		//got the Phy header. Read pkt len and get the rest of the pkt
        {
			u8 *data = ppacket_receiver->pkt_data;
			int packet_len = get_PHR_pkt_len(data);

			//packet_len will be lenght of data starting after the PHR.
			//so this will be the data coming after the 1st DMA
			if((packet_len < 0) || (packet_len > (ppacket_receiver->rx_buf_len)))
			{
				//error -- we do not have enough room in buffer!
				//abort operation.
				/* disable dataflow */
			    if((nResult = adi_ADF7023_SPORT_RX_stop_RX(uhf_handle))!= ADI_DEV_RESULT_SUCCESS)
			    {
			    	//dev_control_err
			    	ApplicationError(nResult);
			    }
			    ApplicationError((u32)-1);
			}

			if(packet_len == 0)
			{
				ppacket_receiver->rx_pkt_len = ppacket_receiver->first_dma_len;
				goto pkt_rx_done;
			}

			/* Here we can start by submitting the first buffer from the beginning. We
			dont need to keep the PHR data, that can be overwritten



			//submit DMA for rest of packet.
			if((nResult = adi_ADF7023_SPORT_RX_set_next_databuf(uhf_handle,
				(u8*) ppacket_receiver->pkt_data + ppacket_receiver->first_dma_len, packet_len)
				)!= ADI_DEV_RESULT_SUCCESS)
			{
			    ApplicationError(nResult);
			}

  			*/
			ppacket_receiver->buf1->bBufferProcessed = false;
			ppacket_receiver->ping_pong_flag = 0;

			if((nResult = adi_ADF7023_SPORT_RX_set_next_databuf(uhf_handle,
				(u8*) ppacket_receiver->buf1->pData, (ppacket_receiver->buf1->ElementCount))
				)!= ADI_DEV_RESULT_SUCCESS)
			{
			    ApplicationError(nResult);
			}

			ppacket_receiver->state = RECEIVE_CONTINUOUS;
		   	sport_rx_pkt_count++;
		    break;
        }
        case RECEIVING_PKT_REMAINDER:	//got the rest of the packet
pkt_rx_done:
        {
			/* disable dataflow */
			if((nResult = adi_ADF7023_SPORT_RX_stop_RX(uhf_handle))!= ADI_DEV_RESULT_SUCCESS)
			{
			    //dev_control_err
			    ApplicationError(nResult);
			}

		    ppacket_receiver->state = IDLE;
                    ppacket_receiver->rxbuf->ElementCount = ppacket_receiver->rx_pkt_len;
                    ppacket_receiver->rxbuf->bBufferProcessed = true;
                    sport_rx_pkt_count++;

                    //!!! Put application callback function to process packet here...

                    break;
		}

        // PPV
		case RECEIVE_CONTINUOUS:
		{

		/* Here keep track of pingpong and submit the next ping pong buffer	*/

					ADI_ADF7023_BUFFER_TYPE *current_buf, *next_buf;

					if(ppacket_receiver->ping_pong_flag == 0)
					{
						current_buf = ppacket_receiver->buf1;
						next_buf = ppacket_receiver->buf2;
					}
					else
					{
						current_buf = ppacket_receiver->buf2;
						next_buf = ppacket_receiver->buf1;
					}
					current_buf->bBufferProcessed = true;
					next_buf->bBufferProcessed = false;
					ppacket_receiver->ping_pong_flag ^=1;

					if((nResult = adi_ADF7023_SPORT_RX_set_next_databuf(uhf_handle,
						(u8*) next_buf->pData, (next_buf->ElementCount))
						)!= ADI_DEV_RESULT_SUCCESS)
					{
						ApplicationError(nResult);
						}


		/****************************************************************************
			Callback the application with the current buffer
			Do the data verification in the application.

		*****************************************************************************/
		adi_ADF7023_SPORT_RX_Continuous_Data_App_cb(current_buf);
		sport_rx_pkt_count++;

		break;
		}
#endif
	}
}
#endif //#ifdef SPORT_SUPPORT

#ifdef SPORT_SUPPORT
/******************************************************************************/
/*!
 * @brief enables or disables the radio continuous RX mode
 *
 * @param[in]   mode         0 for disable, 1 for enable
 *
 * @return none
 *
 * Enables or disables the radio continuous RX mode. When continuous RX mode
 * is enabled, the MAC/PHY software functions envoked by the callback function
 * RadioCallback will be disabled. And the function adi_ADF7023_SPORT_RX_Continuous_Data_cb
 * will be called when the TRX_EVENT_SPORT_RX_DONE event occurs. When continuous RX mode
 * is disabled, the MAC/PHY software will function normally.
 *
 */
void TRX_Set_Continuous_Rx_Mode(int mode)
{
	Continuous_Rx_Mode = mode;
}

/******************************************************************************/
/*!
 * @brief Resets the packet receiver module to the IDLE state
 *
 * @return none
 *
 * Resets the packet receiver module to the IDLE state. This should be done when
 * the application is finished with using RX continuous mode.
 *
 */
void SetReceiverToIdleState(void)
{
	SPORT_PACKET_RX *ppacket_receiver = &packet_receiver;
	ppacket_receiver->state = IDLE;
}


/******************************************************************************/
/*!
 * @brief Returns the ADF7023 radio driver handle
 *
 * @return ADI_ADF7023_DEV_HANDLE ADF7023 radio driver handle
 *
 * Returns the ADF7023 radio driver handle
 *
 */
ADI_ADF7023_DEV_HANDLE TRX_GetRadioHandle()
{
	return hDevice;
}

#endif //#ifdef SPORT_SUPPORT

#ifdef TEMPERATURE_TEST

/******************************************************************************/

/*!
 * @brief Calls the ADF7023 driver function to get the current temprature.
 *
 * @param[in]   None
 *
 * @param[out]  temp Temperature Value.
 *
 * @return      Status returned when ADF7023 Temperature reading
 * driver function is invoked
 *
 * This function returns the current temperature using driver
 * function provided.
 */
uint8_t TRX_Read_Temperature( double *temp )
{
	Check_IRcal_Active();
	ADI_ADF7023_RESULT_TYPE uhf_result = ADI_ADF7023_SUCCESS;

	uhf_result = adi_ADF7023_GetTemperatureReading(hDevice, temp);

    if(!uhf_result)
        return SUCCESS;

    trap();
    return FAILURE;
}

#endif //#ifdef TEMPERATURE_TEST

#ifdef PATCH_802_15_4D
/******************************************************************************/
/*!
 * @brief Calls the ADF7023 function to set the tranceiver state to
 * PHY_ON in 15D4 mode.
 *
 * @param[in]   None
 *
 * @param[out]  None
 *
 * @return      None
 *
 * To set the tranceiver state to PHY_ON(DEV_ON mode) in 15D4, this function
 * should be called.
 *
 * @note      Before invoking this function the tranceiver state
 * should be in PHY_RX_CCA_15D4. TRX_Rx_15d4On() function can be used to set the
 * tranceiver state to PHY_RX_CCA_15D4 state.
 */
void TRX_15d4On(void)
{
	ADI_ADF7023_RESULT_TYPE result = ADI_ADF7023_SUCCESS;

	if ((hDevice->PhyState == PHY_RX_CCA_15D4) ||
	    (hDevice->PhyState == PHY_TX_15D4))
	{
		result = adi_ADF7023_SetPhyOn15d4(hDevice);
	}
	if(result)
	{
		trap();
	}

}

/******************************************************************************/
/*!
 * @brief Calls the ADF7023 function to set the tranceiver state to
 * RX_ON in 15D4 mode.
 *
 * @param[in]   None
 *
 * @param[out]  None
 *
 * @return      None
 *
 * To set the tranceiver state to receiving mode(RX mode) in 15D4, this function
 * should be called. This function call adi_ADF7023_Receive_15d4_Frame which needs
 * reception buffer which will store the data received.
 *
 * @note      Before invoking this function the tranceiver state
 * should be in PHY_ON_15D4. TRX_15d4On() function can be used to set the
 * tranceiver state to PHY_ON_15D4 state.
 */
uint8_t TRX_Rx_15d4On(void)
{
	ADI_ADF7023_RESULT_TYPE	uhf_result = ADI_ADF7023_SUCCESS;

	if (hDevice->PhyState == PHY_ON_15D4)
	{
		TRX_Set_RX_Buffer();
		uhf_result = adi_ADF7023_Receive_15d4_Frame(hDevice, &RxFrame);

	}
	if(uhf_result)
	{
		trap();
	}
        return uhf_result;
}

void TRX_cont_Rx_15d4On(void)
{
	ADI_ADF7023_RESULT_TYPE	uhf_result = ADI_ADF7023_SUCCESS;

    TRX_Set_RX_Buffer();
    uhf_result = adi_ADF7023_Receive_15d4_Frame(hDevice, &RxFrame);

	if(uhf_result)
	{
		trap();
	}
}

/*!
 * @brief Calls the ADF7023 function to submit a buffer for reception without
 * changing the PHY state.
 *
 * @param[in]   None
 *
 * @param[out]  None
 *
 * @return      None
 *
 * To set the receive buffer without changing the PHY state, this function
 * should be called. This function call adi_ADF7023_Set_15d4_Receive_Buffer
 * which sets the reception buffer which will store the data received. This
 * function is used when auto tx to rx turnaround is enabled.
 *
 */
void TRX_set_15D4_rx_buffer(void)
{
	ADI_ADF7023_RESULT_TYPE	uhf_result = ADI_ADF7023_SUCCESS;

	TRX_Set_RX_Buffer();

	uhf_result = adi_ADF7023_Set_15d4_Receive_Buffer(hDevice, &RxFrame);

	if(uhf_result)
	{
		trap();
	}
}

/******************************************************************************/
/*!
 * @brief This function is called by higher layer to get the
 * CRC status of the recently received packet.
 *
 * @param[in]   None
 *
 * @param[out]  None
 *
 * @return      CRC status(SUCCESS/FAILURE)
 *
 * Returns the CRC status of the recently received packet. On reception
 * of packet, as 2 byte CRC verification is done by tranceiver to get its
 * status this function should be used. Driver will update the status in
 * device handler(hDevice) depending on CRC interrupt.
 */
bool TRX_crc_status(void)
{
	return hDevice->bCRC_Correct_Status;
}

/******************************************************************************/
/*!
 * @brief calls the ADF7023 driver function to set the Preamble length.
 *
 * @param[in]   Preamble length to be set in the tranceiver.
 *
 * @param[out]  None
 *
 * @return None
 *
 * Calls the ADF7023 driver function to set the preamble length in the
 * tranceiver. Once the preamble length is set in the tranceiver,
 * it will use this number to send the preamble bytes
 * before any packet transmission.
 *
 */
void TRX_154d_setPreambleLength(uint16_t PreambleLength)
{
	ADI_ADF7023_RESULT_TYPE uhf_result = adi_adf7023_154d_Set_Preamble_Length(hDevice, PreambleLength);
	if(uhf_result)
	{
		trap();
	}
}

/******************************************************************************/
/*!
 * @brief calls the ADF7023 driver function to reset the radio and reload
 * the 15_4D firmware.
 *
 * @param[in]   None.
 *
 * @param[out]  None.
 *
 * @return None.
 *
 * Calls the ADF7023 driver function to reload the 15_4D firmware. Currently
 * this is used to solve the issue of interrupt not triggering after
 * data transmit initiation.
 */
void TRX_Reload_15d4g_firmware(void)
{
	ADI_ADF7023_RESULT_TYPE	result = ADI_ADF7023_SUCCESS;

    /* Reset the radio and reload the 15_4D firmware */
	if(!result)
    	result = adi_ADF7023_154D_PatchRoutine(hDevice);

    //if(!result)
    //	result = adi_ADF7023_config_15d4_BBRAM_Regs(hDevice); //config BBRAM registers for 15d4

	/* set the radio to PHY_ON_15_4D state. */
	if(!result)
		result = adi_ADF7023_SetPhyOn15d4(hDevice);

	if(hDevice->FastTransitionsEnabled)
		adi_ADF7023_15d4g_Enable_Disable_Auto_SynthCalibration(hDevice, 0);

	adi_ADF7023_EnableSynthCalPending(hDevice, 0x01);

	if(result)
	{
		trap();
	}
}

#endif
/******************************************************************************/

#ifdef BENCH_MARKING
/*!
 * @brief Get current hardware time, which is used for benchMarking
 *
 * @param[in]   None
 *
 * @param[out]  None
 *
 * @return      Current Hardware time
 *
 * Returns the current harware time in micro seconds, which can be used
 * for benchmarking. To calculate the time taken for a particular operation,
 * call this function before and after that operation and get the difference
 * in returned value to get the time taken for that operation in micro seconds.
 */
uint64_t getBenchMarkTime(void)
{
	return gpTmr_mod_ins->hw_timer_if.hw_tmr_get_time(
						gpTmr_mod_ins->hw_timer_if.pHw_tmr_ins );
}
#endif	//BENCH_MARKING

#ifdef PATCH_802_15_4D
/*!
 * @brief Reset the driver device handle values to issue radio reset
 * command and reload 15_4D firmware.
 *
 * @param[in]   None
 *
 * @param[out]  None
 *
 * @return      None
 *
 */
void resetDeviceHandle(void)
{
	/* As interrupt are not triggering to continue with polling
	   operation change the init state value to uninitialized */
	hDevice->InitState = ADI_ADF7023_STATE_UNKNOWN;
	/* To come out of the while loop of issue cmd driver function
	 for resetting the radio change this value */
	hDevice->bIsCmdRdy = true;
	/*  */
	hDevice->pPacket = NULL;

	/* To create the interrupt not triggering for test purpose by
	   disabling the interrupt mask */
	//UpdateMask(hDevice, 0);
}
void resetDeviceHandleRx(void)
{
	/* As interrupt are not triggering to continue with polling
	   operation change the init state value to uninitialized */
	hDevice->InitState = ADI_ADF7023_STATE_UNKNOWN;
	/* To come out of the while loop of issue cmd driver function
	 for resetting the radio change this value */
	hDevice->bIsCmdRdy = true;
	/*  */
	hDevice->pPacket = NULL;

    adi_ADF7023_15d4g_Reset_vars();

	/* To create the interrupt not triggering for test purpose by
	   disabling the interrupt mask */
	//UpdateMask(hDevice, 0);
}
#if 0
/*!
 * @brief Read the BBRAM registers for verification.
 *
 * @param[in]   None
 *
 * @param[out]  BBRAM_ReadBack - BBRAM registers value read from the tranceiver.
 *
 * @return      None
 */
void readBBRAMRegs(u8 *BBRAM_ReadBack)
{
	ADI_ADF7023_RESULT_TYPE uhf_result;

	/* Read the BBRAM register values from the radio. */
	uhf_result = adi_ADF7023_BlockMemRead(hDevice, BBRAM_ReadBack, 0x40 /* Size */,
	0x100 /* Start Address */ );
}
#endif
#endif //PATCH_802_15_4D

/******************************************************************************/
/*!
 * @brief Calls the ADF7023 driver function to set the flag to indicate that
 * a synthesizer calibration is pending
 *
 * @param[in]   value	Zero 				- Disable synth_cal_pending flag.
 *						NonZero (+ve values)- Enable synth_cal_pending flag.
 * @param[out]  None.
 *
 * @return None.
 *
 * Calls the ADF7023 driver function to set the flag to indicate that a synthesizer
 * calibration is pending. Based on this flag the driver will perform synthesizer
 * calibration.
 */

void SetSynthCalPending(uint8_t value)
{
	adi_ADF7023_EnableSynthCalPending(hDevice, value);
}

/******************************************************************************/
/*!
 * @brief Calls the ADF7023 driver function to perform synthesizer calibration
 * by issuing command 0xEE.
 *
 * @param[in]   None.
 *
 * @param[out]  None.
 *
 * @return None.
 *
 * Calls the ADF7023 driver function to perform synthesizer calibration by
 * issuing command 0xEE. After the synthesizer calibration is completed, the
 * driver will also disable auto synthesizer calibration.
 */


void TRX_Perform_Manual_Synth_Cal()
{
	if(hDevice->PhyState == PHY_ON_15D4)
	{
		adi_ADF7023_Synthesizer_Calibration(hDevice);
	}
}

#ifdef UTEST_TRX
/******************************************************************************/
/*!
 * @brief Sets UTU timestamp
 *
 * @param[in]   id			UTU timestamp ID
 *
 * @param[in]   value		value to be stored
 *
 * @param[out]  None.
 *
 * @return None.
 *
 * Used to log the vco_band_read_back and vco_ampl_read_back values from the
 * transceiver.
 */

void Set_UTU_TimeStamp(uint8_t id, long value)
{
	utu_timestamp(id, value);
}
#endif //#ifdef UTEST_TRX

/******************************************************************************/
/*!
 * @brief Starts the periodic synthesizer calibration timer
 *
 * @param[in]   None
 *
 * @param[out]  None.
 *
 * @return None.
 *
 * When Auto synthesizer calibration is disabled it is a good practice to
 * periodically perform a full synthesizer calibration. A software timer has been
 * provided to indicate the driver that a full synthesizer calibration is
 * pending and needs to be performed. This function starts a software timer for
 * SYNTH_CAL_TIME_OUT_VAL (60 seconds by default). On the expiry of this timer,
 * the synth_cal_pending flag (in hDevice) is enabled to indicate synthesizer
 * calibration is pending.
 */

static void start_synth_cal_timer( void )
{
	uint8_t synth_cal_timeout = SYNTH_CAL_TIME_OUT_VAL;

	tmr_create_one_shot_timer
	(
		&synth_cal_timer,
		synth_cal_timeout * 1000000, // usecs
		(sw_tmr_cb_t)set_synth_cal_pending,
		NULL
	);

	tmr_start_relative( &synth_cal_timer );
}

/******************************************************************************/
/*!
 * @brief Callback for synth_cal_timer.
 *
 * @param[in]   NULL
 *
 * @param[out]  None.
 *
 * @return None.
 *
 * This is the callback provided by synth_cal_timer (s/w timer to enable synth_cal_pending flag).
 * When this callback is received the synth_cal_pending flag is enabled and the s/w timer is
 * restared.
 */

void set_synth_cal_pending(void *ptr)
{
	tmr_stop( &synth_cal_timer );

	SetSynthCalPending(TRUE);

	tmr_start_relative( &synth_cal_timer );
}

#ifdef UNIT_TEST_PRINT
uint32_t get_time(void)
{
	return (gpTmr_mod_ins->hw_timer_if.hw_tmr_get_time(
						gpTmr_mod_ins->hw_timer_if.pHw_tmr_ins ));
}
#endif /* UNIT_TEST_PRINT */

uint8_t *TRX_Get_RX_Buffer(void)
{
	return hDevice->pBuf;
}

void TRX_clear_RX_Buffer()
{
    hDevice->pBuf = NULL;
}

uint8_t TRX_radioWakeup(void)
{
    return adi_ADF7023_Wakeup(hDevice);

}
uint8_t TRX_radioWakeup_seq(uint8_t sequence)
{
    if (sequence == 4)
    {
        return TRX_ContReceiveEnable(true);
    }
    else if (sequence == 5)
    {
        return TRX_ContReceiveEnable(false);
    }
    else
    {
        return adi_ADF7023_sequencial_Wakeup(hDevice, sequence);
    }

}

void TRX_radioSleep(void)
{
    adi_ADF7023_Sleep(hDevice);
}

ADI_ADF7023_RESULT_TYPE TRX_ContTransmitSettings(uint8_t mode)
{
  ADI_ADF7023_RESULT_TYPE result;

  result = adi_ADF7023_SetVarTxTestMode(hDevice,mode);

  if( mode != 0)
  {
          if(!result)
          result = adi_ADF7023_SetPhyTx(hDevice);
  }
  else
          result= adi_ADF7023_SetPhyOn(hDevice);

  return result;
}

ADI_ADF7023_RESULT_TYPE TRX_ContReceiveEnable(bool bState)
{
    ADI_ADF7023_RESULT_TYPE result;

    result = ADF7023_15d4g_SetBitContinousRX(hDevice, bState);

    if( bState != 0)
    {
        if(!result)
            TRX_Rx_15d4On();
    }
    else
    {
      if(!result)
        TRX_15d4On();
      hDevice->PhyState = PHY_ON_15D4;
    }

    return result;
}

void TRX_radioRead(uint8_t *bbram_array, uint8_t bbram_len, uint16_t bbram_addr)
{
    ADI_ADF7023_RESULT_TYPE uhf_result;
    uhf_result = adi_ADF7023_BlockMemRead(hDevice, bbram_array, bbram_len, bbram_addr);
}

void TRX_radioWrite(uint8_t *bbram_array, uint8_t bbram_len, uint16_t bbram_addr)
{
    ADI_ADF7023_RESULT_TYPE uhf_result;
    uint8_t status;
    uhf_result = adi_ADF7023_BlockMemWrite(hDevice, bbram_array, &status, bbram_len, bbram_addr);

    if(!uhf_result)
        uhf_result = adi_ADF7023_IssueCmd(hDevice, ADI_ADF7023_CMD_CONFIG_DEV);
}

unsigned char   ADF_MMapRead(unsigned long ulAdr,unsigned long ulLen,unsigned char *pData)
{
    TRX_radioRead(pData, ulLen, ulAdr);
    return 0;
}


/*----------------------------------------------*/
/* Power down the transceiver to various states */
/*----------------------------------------------*/
#ifndef ADF7242
unsigned char ADF_PwrMgt(TRPSSTATE PDState)
{
    if(PDState == TRPS_DOWN)
    {
        TRX_radioSleep();
    }
    if(PDState == TRPS_UP)
    {
        TRX_radioWakeup();
    }
    return TRE_NO_ERR;
}
#endif
phy_band_config_t phy_band_config[MAX_PHY_BANDS_SUPPORTED]=
{
#if defined(CFG_PHY_BAND_920_928)
	PHY_BAND_CONFIG_920_MHz,
#endif
};

phy_mode_config_t* gp_phy_mode_config;

/*
** ============================================================================
** External Variable Declarations
** ============================================================================
*/

extern phy_pib_t phy_pib;

extern void phy_process_data_event( volatile uint8_t* trx_event );

extern void restore_trx_state( phy_status_t  state );

extern phy_status_t backup_trx_state( void );

/*
** ============================================================================
** Private Function Prototypes
** ============================================================================
*/

static phy_status_t phy_set_defaults( void );

/*
** ============================================================================
** Public Function Definitions
** ============================================================================
*/
#ifndef ADI_IAR
void PHY_Init( uint8_t cold_start )
#else
void PHY_Init( uint8_t cold_start, ADI_SPI_HANDLE hDevice_SPI, ADI_SPI_DEV_ID_TYPE spidev, IRQn_Type const irq_flag )
#endif
{
	//uint32_t i = 100000L;
#ifndef ADI_IAR
	TRX_Init();
#else
	TRX_Init(hDevice_SPI, spidev, irq_flag);
#endif

	//while(i--);


	TRX_Register_Data_Event_cb(phy_process_data_event);

	if ( cold_start )
	{
		phy_pib.TRXState = PHY_DEV_ON;

		phy_pib.pendingStateChange = PHY_INVALID_PARAMETER;

		gp_phy_mode_config = &(phy_band_config[0].phy_mode_config[0]);

		PHY_Configure_Radio(gp_phy_mode_config);
#ifndef PATCH_802_15_4D
		TRX_Set_RX_Buffer();
#endif

		phy_set_defaults();
	}
}

/*****************************************************************************/

void PHY_Reset ( void )
{
	TRX_Reset();

	/*go to default PHY mode*/
	gp_phy_mode_config = &(phy_band_config[0].phy_mode_config[0]);

	PHY_Configure_Radio( gp_phy_mode_config );

	phy_pib.TRXState = PHY_DEV_ON;

	phy_pib.pendingStateChange = PHY_INVALID_PARAMETER;

	phy_set_defaults();

#ifndef PATCH_802_15_4D
	TRX_On();
#else
	TRX_15d4On();
#endif

	//TRX_Set_RX_Buffer();
}

/*****************************************************************************/

void PHY_Configure_Radio( phy_mode_config_t* p_phy_config )
{
	phy_status_t original_state;

    //volatile uint32_t i=0xFFFFF;
	//TRX_On();

    gp_phy_mode_config = p_phy_config;

    original_state = backup_trx_state();

#ifndef PATCH_802_15_4D
	/*bring the radio to phy off state as the following IF bandwidth*/
	TRX_Off();

#endif

	TRX_IF_Filter_Bandwidth( p_phy_config->if_bandwidth );

#ifdef PATCH_802_15_4D

	TRX_CCA_Filter_Bandwidth(p_phy_config->cca_filter_bandwidth);

#endif

	//while(--i);

#ifndef PATCH_802_15_4D

	TRX_On();//TRX_On();

#endif

	TRX_Set_Data_Rate( (p_phy_config->data_rate_kbps) * 10 );

	TRX_Discrim_PostDemod_Bandwidth(p_phy_config->radio_discrim_bw, p_phy_config->radio_post_demod_bw);

	TRX_Freq_Deviation( (p_phy_config->freq_deviation_khz) * 10 );
	//i=0xFFFFF;
	//while(--i);

    restore_trx_state( original_state );

}

/*****************************************************************************/

uint8_t PHY_rcv_in_progress( void )
{
    return ( plme_get_trx_state_request() == PHY_BUSY_RX )?1:0;
	//return 0;
}

/*****************************************************************************/
#ifndef ADI_IAR
#pragma section ("L1_code")
#endif
uint8_t PHY_Get_Symbol_Rate( void )
{
	return gp_phy_mode_config->symbol_duration;
}

/*****************************************************************************/

void plme_set_phy_mode_request( void )
{
	/*set the PHY in the mode requested only if the PHY
	indicates that it has the capability of that */
	uint32_t val;
	uint32_t i,j,k;
	phy_mode_config_t* p_phy_config;
	phy_mode_t mode;
	uint32_t delay = 0xFFFF;

	uint32_t freq_band_id = (phy_pib.CurrentSUNPageEntry & FREQ_BAND_ID_MASK)>>22;

	for(i=0;i<(MAX_PHY_BANDS_SUPPORTED);i++)
	{
		if(phy_band_config[i].freq_band_id == freq_band_id )
		{
			for ( j = 0; j < 4; j++ )
			{
				mode = (phy_mode_t)(1<<(j));
				if ( phy_pib.CurrentSUNPageEntry & mode )
				{
					//if ( g_phy_mode != mode )
					{
						//g_phy_mode = mode;

						p_phy_config = &phy_band_config[i].phy_mode_config[mode/2];

						PHY_Configure_Radio(p_phy_config);


						while(--delay);
#ifndef PATCH_802_15_4D
						TRX_On();
#else
						TRX_15d4On();
#endif

						//TRX_Set_RX_Buffer();


						phy_pib.TRXState = PHY_DEV_ON;

						phy_pib.pendingStateChange = PHY_INVALID_PARAMETER;

						phy_pib.MaxSUNChannelSupported = p_phy_config->num_of_channels;

						val = 80;

						PLME_set_request(phyRSSIThreshold,1,&val);

						memset(phy_pib.SUNChannelsSupported,0x0,MAX_CHANNELS_CAPABILITY);

						for (k = 0 ;k< phy_pib.MaxSUNChannelSupported; k++ )
					    {
					    	phy_pib.SUNChannelsSupported[k/8] |= (1<<(k%8));
					    }
					    /*Set the center frequency corresponding to the phy current channel.This
					     will make sure that the center freq is changed whenever CurrentSUNPageEntry
					     is changed.Channel1 of PHY Mode1:951.2MHz,Channel1 of PHY Mode2:951.5MHz.
					     This step of setting the exact center freq should be taken care by application.
					     This can be removed if application can take care of that*/
					    phy_set_current_channel(phy_pib.CurrentChannel);

					}
				}
			}
		}
	}


}

/*
** ============================================================================
** Private Function Definitions
** ============================================================================
*/

static phy_status_t phy_set_defaults( void )
{
	uint32_t i,j, val = DEF_TX_POWER;
	phy_mode_config_t* p_phy_config = gp_phy_mode_config;


	phy_pib.MaxSUNChannelSupported = p_phy_config->num_of_channels;


	/*set the read only paramaters */
	phy_pib.SymbolsPerOctet = p_phy_config->symbols_per_octet;

    phy_pib.SHRDuration =
    	phy_pib.SymbolsPerOctet * ( DEF_PREAMBLE_REPITITIONS + aMRFSKSFDLength );

    phy_pib.MaxFrameDuration = phy_pib.SHRDuration +
    ( (aMRFSKPHRLength + aMaxPHYPacketSize) * phy_pib.SymbolsPerOctet );

    for (i = 0 ;i< phy_pib.MaxSUNChannelSupported; i++ )
    {
    	phy_pib.SUNChannelsSupported[i/8] |= (1<<(i%8));
    }

    val = DEF_CHANNEL;
	PLME_set_request(phyCurrentChannel,2, &val);

	val = DEF_TX_POWER;
	PLME_set_request(phyTransmitPower,1,&val);


	val = DEF_CCA_MODE;
	PLME_set_request(phyCCAMode,1,&val);

	phy_pib.CCADuration = aCCATime;

	val = 9;
	PLME_set_request(phyCurrentPage,1,&val);

	/*look through the supported phy bands configuration table if this freq band is supported*/
	for(i=0;i<MAX_PHY_BANDS_SUPPORTED;i++)
	{
		val = PAGE_NUMBER_9|MOD_FSK;

		if( phy_band_config[i].freq_band_id ) // need to change this. even 0 is an ID for a freq band
		{
			val |= (phy_band_config[i].freq_band_id << 22);
			for(j=0;j<MAX_PHY_MODES_SUPPORTED;j++)
			{
				val |= (phy_band_config[i].phy_mode_config[j].phy_mode);

			}

			PLME_set_request(phySUNPageEntriesSupported,4,&val);

			if(!i)
			{
				val &= (~FSK_PHY_MODE_MASK);
				val |= p_phy_config->phy_mode;
				PLME_set_request(phyCurrentSUNPageEntry,4,&val);
			}
		}
	}

	val = DEF_FSKFEC_SWITCH;
	PLME_set_request(phyFECEnabled,1,&val);

	val = DEF_FSKFEC_SCHEME;
	PLME_set_request(phyFSKFECScheme,1,&val);

	val = DEF_FSKFEC_INTERLEAVING;
	PLME_set_request(phyFSKFECInterleaving,1,&val);

	val = DEF_MRFSKSFD;
	PLME_set_request(phyMRFSKSFD,1,&val);

	val = DEF_PREAMBLE_REPITITIONS;
	PLME_set_request(phyFSKPreambleRepetitions,2,&val);

	val = DEF_FSKSCRAMBLE;
	PLME_set_request(phyFSKScramblePSDU,1,&val);

	val = DEF_RSSI_THRESHOLD;
	PLME_set_request(phyRSSIThreshold,1,&val);

#ifdef TEMPERATURE_TEST
	phy_pib.temperature_read = 0.0;
#endif //#ifdef TEMPERATURE_TEST

	phy_pib.SyncWord = DEF_SYNC_WORD;

	return PHY_SUCCESS;
}

/*****************************************************************************/

uint32_t phy_get_total_symbols_txed(void)
{
    return TRX_Total_Symbols_Txed();

}

/*****************************************************************************/
void phy_reset_total_symbols_txed(void)
{
    TRX_Reset_Total_Symbols_Txed();
	return;
}

/*******************************************************************************/

void start_802_15_4D_reset_timer(uint8_t startMode)
{
    currentTxMode = startMode;

    tmr_stop(&sw_timer1);

    /* restart the data interval timer */
    tmr_start_relative( &sw_timer1 );
}

void stop_802_15_4D_reset_timer(uint8_t stopMode)
{
    currentTxMode = stopMode;

    if (stopMode == 2)
            return;

    tmr_stop(&sw_timer1);
}

void stop_802_15_4D_rx_reset_timer()
{
    tmr_stop(&sw_timer2);
}

void start_802_15_4D_rx_reset_timer()
{
    tmr_stop(&sw_timer2);
    /* restart the data interval timer */
    tmr_start_relative( &sw_timer2 );
}

void cca_done_cb(void* s, uint8_t status)
{

}

/******************************************************************************/

#if TELEC_TEST_HARNESS
#ifdef ADCM3029_EZKIT_USED

void wakeup_GPIO_configure(bool_t input)
{
    pADI_GPIO0->CFG &= ~(3<<30);

    if (input)
    {

        if (adi_gpio_RegisterCallback(XINT_EVT0_IRQn, WakeupCallback1, NULL))
        {
        }

        if (adi_gpio_InputEnable(ADI_GPIO_PORT0, ADI_GPIO_PIN_15, true)) {
        }

        if (adi_gpio_EnableExIRQ(XINT_EVT0_IRQn, ADI_GPIO_IRQ_FALLING_EDGE)) {
        }
    }
    else
    {
        if (adi_gpio_OutputEnable(ADI_GPIO_PORT0, ADI_GPIO_PIN_15, true)) {
        }
    }
}

/******************************************************************************/

uint16_t sleep_couner;

volatile bool_t bHibernateExitFlag = true;

uint8_t process_sleep()
{
    uint8_t status;

    while(check_uart_tx_status());
    bHibernateExitFlag = false;
    TRX_radioSleep();
    adi_pwr_EnterLowPowerMode(ADI_PWR_MODE_HIBERNATE, &bHibernateExitFlag, 0);

    status = TRX_radioWakeup();

    sleep_couner++;

    return status;
}

/******************************************************************************/
#else
uint8_t process_sleep()
{
	uint8_t status;

	while(check_uart_tx_status());
	bHibernateExitFlag = false;
	TRX_radioSleep();
	adi_pwr_EnterLowPowerMode(ADI_PWR_MODE_HIBERNATE, &bHibernateExitFlag, 0);

	status = TRX_radioWakeup();
    return status;
}

/******************************************************************************/

void wakeup_GPIO_configure(bool_t input)
{
	pADI_GPIO2->CFG &= ~(3<<2);

	if (input)
	{
		adi_gpio_RegisterCallback(XINT_EVT3_IRQn, WakeupCallback1, NULL);
		adi_gpio_InputEnable(ADI_GPIO_PORT2, ADI_GPIO_PIN_1, true);
		adi_gpio_EnableExIRQ(XINT_EVT3_IRQn, ADI_GPIO_IRQ_FALLING_EDGE);
	}
	else
	{
		adi_gpio_OutputEnable(ADI_GPIO_PORT2, ADI_GPIO_PIN_1, true);
	}
}
#endif
#endif

/******************************************************************************/

void signal_event_to_uart_task( void )
{
    raise_int(UART_INT_NUM);
}
void UART_Task( void )
{
    uart_hal_rx_low_priority_cb();
}

/******************************************************************************/

uint8_t hif_Telec_send_msg(char* p_Msg, base_t msg_len)
{
    return (send_uart_frame(p_Msg, msg_len));
}

void raiseUartEvent()
{
    signal_event_to_uart_task();
}

void UART_SoftInterruptHandler(void *pCBParam, uint32_t Event, void *pArg)
{
    //UART_Task();
}

unsigned char adi_ADF7023_SetPhyRxCCA15D4(void)
{
  uint8_t result;
  result = TRX_Rx_15d4On();
  phy_pib.TRXState = PHY_RX_ON;
  return result;
}
#ifndef ADF7242
int Retireve_rf_statistics(unsigned char *buf, unsigned int Bufpos, int len)
{
  rf_statistics.pkt_Rx_time = (get_rx_duration()/4000);
  if(Is_StatisticsEnabled())
  {
      buf[Bufpos++] = 'S'; // start of statistics

      Bufpos = AddLongToBuffer(rf_statistics.Tx_EOF + rf_statistics.Rx_EOF , buf, Bufpos, len);                             // Long 0 - 5
      Bufpos = AddLongToBuffer(rf_statistics.CRC_failure + rf_statistics.CCA_Failed, buf, Bufpos, len);                     // Long 1 - 5

      Bufpos = AddLongToBuffer(clock_seconds(), buf, Bufpos, len);                                                // Int 0 - 3
      Bufpos = AddLongToBuffer((rf_statistics.pkt_Tx_time * 4) /1000 , buf, Bufpos, len);                              // Int 1 - 3
      Bufpos = AddLongToBuffer((rf_statistics.pkt_Rx_time * 4) /1000 , buf, Bufpos, len);                              // Int 2 - 3
      Bufpos = AddLongToBuffer((rf_statistics.cca_time  * 4)   /1000 , buf, Bufpos, len);                              // Int 2 - 3
      Bufpos = AddLongToBuffer(rf_statistics.CCA_Failed, buf, Bufpos, len);                                            // Int 3 - 3
      //#ifdef ENABLE_LOW_POWER_MODE
      Bufpos = AddLongToBuffer(system_hibernate_duration, buf, Bufpos, len);                                      // Int 3 - 3
      //#endif
      Bufpos = AddLongToBuffer(system_flexi_mode_sleep_duration, buf, Bufpos, len);                               // Int 3 - 3

      //Active Up-Channel Index
      Bufpos = AddByteToBuffer(phy_current_channel,buf, Bufpos, len);                                             // Int 3 - 3

      //RSSI pack
      Bufpos = AddByteToBuffer(rf_statistics.RSSI_val, buf, Bufpos, len);                                              // Byte 1 - 2

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
#endif
/*****************************************************************************/


/*
 ** EOF
 */

/*@}*/
