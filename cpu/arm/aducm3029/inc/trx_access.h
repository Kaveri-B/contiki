/** \file trx_access.h
 *******************************************************************************
 ** \brief provides APIs to access the RF driver functionality
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

#ifndef _TRX_ACCESS_H_
#define _TRX_ACCESS_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
** =============================================================================
** Public Macro definitions
** =============================================================================
*/

/**
 ** \defgroup trx TRX Access Layer Interface
 */

/**
 ** \defgroup trx_access_defs  TRX Layer Definitions
 ** \ingroup trx
 */

/*@{*/

/*! Minimum out power supported (in dBm) */
#define MIN_OUTPUT_POWER				-22

/*! Maximum out power supported (in dBm) */
#define MAX_OUTPUT_POWER				12

/*! API for enabling continuous transmission*/
#define Enable_Continuous_Tx()		\
	Set_TX_SPORT_Continuous_TX(1)

/*! API for disabling continuous transmission*/
#define Disable_Continuous_Tx()		\
	Disable_TX_SPORT();				\
	Set_TX_SPORT_Continuous_TX(0);

/*! API reading the RSSI value */
#ifdef PATCH_802_15_4D

	#define TRX_Get_RSSI(x) TRX_Get_RSSI_Method_4(x)

#else

#ifdef USE_RSSI_METHOD_2
	#define TRX_Get_RSSI(x) TRX_Get_RSSI_Method_2(x)
#else
	#define TRX_Get_RSSI(x) TRX_Get_RSSI_Method_3(x)
#endif

#endif //PATCH_802_15_4D

/*
** =============================================================================
** Public Structures, Unions & enums Type Definitions
** =============================================================================
*/

typedef void (*trx_data_event_cb_t)( volatile uint8_t*  nEvent );

/*!
 *******************************************************************************
 ** \struct mr_fsk_shr_t
 ** structure to hold all the information needed by TRX access module for
 ** building the MR-FSK SHR.
 ******************************************************************************/
typedef struct mr_fsk_shr_tag
{
	unsigned char preamble_byte; /**< holds MR_FSK Preamble byte*/
	unsigned short preamble_rep; /**< holds MR_FSK Preamble length in bytes*/
	unsigned short sfd;			/**< holds sfd*/
	unsigned char fec_coding;	/**< b0: for fec enable/disble,
								b1: for NRNSC or RSC, b2: Interleave enabled/disabled */
}mr_fsk_shr_t;

/*!
 *******************************************************************************
 ** \union shr_info_t
 **     shr union to hold all the inforamtion needed by TRX access module to
 **     build the SHR.
 ******************************************************************************/
typedef union shr_info
{
	mr_fsk_shr_t	mr_fsk_shr;	/**< Holds information about MR_FSK_SHR*/
}shr_info_t;

/*!
 *******************************************************************************
 ** \struct SHR_t
 **		SHR structure to hold all the inforamtion needed by TRX access module to
 **	    build the SHR.
 ******************************************************************************/
typedef struct shr_tag
{
	unsigned char shr_type; /**< holds the type of the SHR to be built*/
	shr_info_t shr_info;	/**< holds the information for building the indicated SHR type*/
}SHR_t;

/*!
 *******************************************************************************
 ** \enum event
 **      Enumeration for different TRX event types
 ******************************************************************************/
enum
{
	NO_EVENT 	= 0x00,   /**< Indicates that there is no event from the TRX */
	TX_COMPLETE = 0x01,   /**< Indicates that TRX successfully transmitted a frame */
	RX_COMPLETE = 0x02,    /**< Indicates that TRX successfully received a frame*/
	SFD_DETECTED = 0x04,	  /**< Indicates that TRX has detected a SFD and is busy
							in getting the whole packet*/
#ifdef PATCH_802_15_4D
	CCA_DONE_SUCCESS = 0x08,
	CCA_DONE_BUSY = 0x10,
#endif
};

/*!
 *******************************************************************************
 ** \enum SHR_TYPES
 **		Enumeration for differnt types of SHR that can be built by the TRX Access
 **		module
 ******************************************************************************/
enum
{
	SHR_TYPE_MR_FSK_SHR,	/**< SHR type as MR_FSK_SHR */
	SHR_TYPE_MR_O_QPSK,		/**< SHR type as MR_O_QPSK */
	SHR_TYPE_MR_OFDM		/**< SHR type as MR_ODMK */
};

/*!
 *******************************************************************************
 ** \enum FEC_coding
 ** Enumeration for values that can be specified for creating the MR-FSK SHR.
 ******************************************************************************/
enum
{
	MASK_FEC_BIT	 			= 0x01,
	NRNSC_FEC_BIT				= 0x02,
	RSC_FEC_BIT					= 0x00,
	MASK_FEC_SCHEME_BIT			= 0x02,
	MASK_INTERLEAVING_BIT		= 0x04
};

/*!
 *******************************************************************************
 ** \enum IFBW AUTO SWITCH
 **		Enumeration for enabling and disabling of IFBW AUTO SWITCH that
 **		can be built by the TRX Access module .
 ******************************************************************************/
enum
{
	DISABLE_IFBW_AUTO_SWITCH = 0x00,	/**< DISABLE  IFBW AUTO SWITCH */
	ENABLE_IFBW_AUTO_SWITCH				/**< ENABLE  IFBW AUTO SWITCH */
};

/*!
 *******************************************************************************
 ** \enum IFBW SWITCH ON PREAMBLE
 **		Enumeration for enabling and disabling of IFBW SWITCH ON PREAMBLE that
 **		can be built by the TRX Access module .
 ******************************************************************************/
enum
{
	DISABLE_IFBW_SWITCH_ON_PREAMBLE = 0x00,	/**< DISABLE  IFBW SWITCH ON PREAMBLE */
	ENABLE_IFBW_SWITCH_ON_PREAMBLE			/**< ENABLE  IFBW SWITCH ON PREAMBLE */
};


/*@}*/

/*
** =============================================================================
** Public Variable Declarations
** =============================================================================
*/

/* None */
/*
** ============================================================================
** Private Variable Definitions
** ============================================================================
*/

/* None */

/*
** =============================================================================
** Public Variables Definitions
** =============================================================================
**/

/* None */

/*
** ============================================================================
** External Variable or function Declarations
** ============================================================================
*/

extern void trx_packet_rx_cb( void );
extern void trx_packet_sent_cb( void );

/*
** =============================================================================
** Public Function Prototypes
** =============================================================================
*/

/** \defgroup trx_access_req_func TRX Required Functions
 ** \ingroup trx
 */

/*@{*/

/**
 *******************************************************************************
 ** \brief Initialises the radio with the configured settings and brings it into
 **        PHY on state and also initializes adf7023 SPORTs interface
 ** \param - None
 ** \return SUCCESS or FAILURE
 ** \note This function should be called once during the system initialization
 ******************************************************************************/
uint8_t TRX_Init(ADI_SPI_HANDLE hDevice_SPI, ADI_SPI_DEV_ID_TYPE spidev, IRQn_Type const irq_flag);

/**
 *******************************************************************************
 ** \brief Resets the TRX module and brings the RF to PHY ON state and
 ** 	uninitialises all device resources
 ** \param - None
 ** \return SUCCESS or FAILURE
 ** \note Not being used.
 ******************************************************************************/
uint8_t TRX_Reset(void);

/**
 *******************************************************************************
 ** \brief Provides the state of the TRX
 ** \param[out] *p_phy_state - pointer where the frame needs to be copied
 ** \return SUCCESS or FAILURE.
 ** \note None
 ******************************************************************************/
uint8_t TRX_Get_State(uint8_t* p_phy_state);

/**
 *******************************************************************************
 ** \brief registers call back for processing the data events
 ** \param - cb - callback function to be called whenever there are events
 *  from the TRX.
 ** \return SUCCESS or FAILURE
 ******************************************************************************/
uint8_t TRX_Register_Data_Event_cb(trx_data_event_cb_t cb);

/*@}*/

/** \defgroup trx_access_control_functions TRX Control Functions
 ** \ingroup trx
 */

/*@{*/

/**
 *******************************************************************************
 ** \brief Queries and sets the radio to phy ON state
 ** \param - None
 ** \return SUCCESS or FAILURE
 *******************************************************************************/
uint8_t TRX_On( void );

/**
 *******************************************************************************
 ** \brief Queries and sets the radio to phy TX_ON state
 ** \param - None
 ** \return SUCCESS or FAILURE
 ******************************************************************************/
uint8_t TRX_TX_On( void );

/**
 *******************************************************************************
 ** \brief Queries and sets the radio to phy RX_ON state
 ** \param - None
 ** \return SUCCESS or FAILURE
 ******************************************************************************/
uint8_t TRX_Rx_On( void );

/**
 *******************************************************************************
 ** \brief Queries and sets the radio to phy OFF state
 ** \param - None
 ** \return SUCCESS or FAILURE
 ******************************************************************************/
uint8_t TRX_Off( void );

/**
 *******************************************************************************
 ** \brief Wakes up the radio
 ** \param - None
 ** \return SUCCESS or FAILURE
 ******************************************************************************/
uint8_t TRX_Wakeup(void);

#ifdef PATCH_802_15_4D
/**
 *******************************************************************************
 ** \brief Provides RSSI reading
 ** \param[out] *pRSSIval - pointer to RSSI value
 ** \return SUCCESS or FAILURE
 ** \note 1)This function should be called in PHY_ON state\n
 **		  2)Reads the received input power in 2's complement dBm
 ******************************************************************************/
uint8_t TRX_Get_RSSI_Method_1( int * pRSSIval );

#endif //PATCH_802_15_4D

/**
 *******************************************************************************
 ** \brief Provides RSSI reading
 ** \param[out] *pRSSIval - pointer to RSSI value
 ** \return SUCCESS or FAILURE
 ** \note 1)This function should be called in PHY_ON state\n
 **		  2)Reads the received input power in 2's complement dBm
 ******************************************************************************/
uint8_t TRX_Get_RSSI_Method_2( char* pRSSIval );


/**
 *******************************************************************************
 ** \brief Provides RSSI reading using mode 3
 ** \param[out] *pRSSIval - pointer to RSSI value.
 ** \return SUCCESS or FAILURE
 ** \note 1)This function should be called in PHY_RX state\n
 **		  2)Reads the received input power in 2's complement dBm
 ******************************************************************************/
uint8_t TRX_Get_RSSI_Method_3( int* pRSSIval );

/**
 *******************************************************************************
 ** \brief Sets the transmitter PA power level
 ** \param[in] pwr_dbm - transmit power value
 ** \param[out] *p_pa_mcr_level - pointer to pa_mcr_level  value.
 ** \return SUCCESS or FAILURE
 ** \note 1)This function calculates the PA level by using pa_level = ( *p_pa_mcr_level - 3)/4;
 ******************************************************************************/
uint8_t TRX_Set_Output_Power( int8_t pwr_dbm, uint8_t* p_pa_mcr_level );

/**
 *******************************************************************************
 ** \brief Sets the transmitter PA power level
 ** \param[in] level - transmit power value(0 to 63)
 ** \return SUCCESS or FAILURE
 ** \note 1)This function calculates the PA level by using pa_level = ( level - 3)/4; \n
 **       2) Valid range for level is 0 to 63
 ******************************************************************************/
uint8_t TRX_Set_PA_Level_MCR( uint8_t level );

/**
 *******************************************************************************
 ** \brief Sets the Channel Frequency
 ** \param[in] center_freq - frequency value in Hz
 ** \return SUCCESS or FAILURE
 ** \note Sets the frequency to the closest possible value.
 ******************************************************************************/
uint8_t TRX_Set_Channel_Frequency( uint32_t center_freq );

/**
 *******************************************************************************
 ** \brief Sets the radio Discriminator BW and Post demod BW
 ** \param[in]discrim_bw - discriminator BW value
 ** \param[in] post_demod_bw - Post Demod BW value
 ** \return SUCCESS or FAILURE
 ** \note The value passed in should be the desired frequency deviation in kHz*10
 *******************************************************************************/
uint8_t TRX_Discrim_PostDemod_Bandwidth( uint8_t discrim_bw, uint8_t post_demod_bw );
/**
 *******************************************************************************
 ** \brief Sets the Frequency Deviation
 ** \param[in] freq_dev - frequency deviation value
 ** \return SUCCESS or FAILURE
 ** \note The value passed in should be the desired frequency deviation in kHz*10
 *******************************************************************************/
uint8_t TRX_Freq_Deviation( uint16_t freq_dev );

/**
 *******************************************************************************
 ** \brief Sets the IF filter bandwidth
 ** \param[in] ifbw - filter bandwidth value
 ** \return SUCCESS or FAILURE.
 ** \note The value passed in should be 0x0(100 kHz),0x1(150 kHz),0x2(200 kHz),0x3(300 kHz)
 ******************************************************************************/
uint8_t TRX_IF_Filter_Bandwidth( uint8_t ifbw );

/**
 *******************************************************************************
 ** \brief Sets the CCA filter bandwidth
 ** \param[in] ifbw - CCA filter bandwidth value
 ** \return SUCCESS or FAILURE.
 ** \note The value passed in should be 0x0(100 kHz),0x1(150 kHz),0x2(200 kHz),0x3(300 kHz)
 ******************************************************************************/
uint8_t TRX_CCA_Filter_Bandwidth( uint8_t cca_filter_bw );

/**
 *******************************************************************************
 ** \brief Sets the data rate
 ** \param[in] data_rate - data_rate value
 ** \return SUCCESS or FAILURE
 ** \note The value passed in must be 10 times of the desired data rate in kbps
 ******************************************************************************/
uint8_t TRX_Set_Data_Rate( uint16_t data_rate );

/**
 *******************************************************************************
 ** \brief Sets the modulation scheme
 ** \param[in] mod_scheme - mod_scheme value
 ** \return SUCCESS or FAILURE.
 ** \note MOD_2FSK = 0x0\n MOD_2GFSK = 0x1\n MOD_OOK = 0x2\n MOD_CARRIER = 0x3\n
 ******************************************************************************/
uint8_t TRX_Mod_Scheme( uint8_t mod_scheme );

/**
 *******************************************************************************
 ** \brief Sets the demodulation scheme
 ** \param[in] demod_scheme - demod_scheme value
 ** \return SUCCESS or FAILURE.
 ** \note MOD_2FSK = 0x0\n MOD_2GFSK = 0x1\n MOD_OOK = 0x2\n MOD_CARRIER = 0x3\n
 **/
uint8_t TRX_DeMod_Scheme( uint8_t demod_scheme );

/**
 *******************************************************************************
 ** \brief Reads the PA level
 ** \param - None
 ** \return PA MCR value
 ******************************************************************************/
uint8_t TRX_Read_pa_mcr_level( void );

/**
 *******************************************************************************
 ** \brief Converts pa_mcr_level value to corresponding dBm value
 ** \param[in] pa_mcr_level - value to be converted
 ** \return index value
 ** \note Valid range for pa_mcr_level is 0 to 63
 ******************************************************************************/
int8_t TRX_Convert_pa_mcr_level_to_dBm(uint8_t pa_mcr_level );

/*@}*/

/** \defgroup trx_access_data_functions TRX Data Functions
 ** \ingroup trx
 */

/*@{*/

/**
 *******************************************************************************
 ** \brief Builds and writes the PHY packet into the TX buffer
 ** and enables the SPOTRTS interface for transmission
 ** \param[in] p_shr_info - contains all the info for building the SHR,
 ** \param[in] phr - holds the 2 byte PHR to be put in the frame,
 ** \param[in] *pd_data - pointer where the payload including the FCS is present
 ** \param[in] pd_length - payload length including the FCS field length
 ** \param[in] CCA_Enable - indicates whether CCA needs to be performed before data tx
 ** \param[in] CCAThreshHold15D4 - ThreshHold value of CCA, below which data can be transmitted.
 ** \param[in] CCATimerDuration15D4 - Timer duration for which CCA has to be performed
 ** \return SUCCESS or FAILURE.
 ** \note Creates the frame in the rx buffer of the SPORTS interface and
 **			enables the SPORTS data flow.
 ******************************************************************************/
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
		);

/**
 *******************************************************************************
 ** \brief Sets the rx buffer for reception and also enables the SPORTS interface
 **  for data reception
 ** \param - None
 ** \return SUCCESS or FAILURE.
 ** \note None.
 ******************************************************************************/
uint8_t TRX_Set_RX_Buffer( void );

/**
 *******************************************************************************
 ** \brief provides the the length of the frame, pending to be read from the rx buffer
 ** \param - None
 ** \return frame length in bytes.
 ******************************************************************************/
uint32_t TRX_get_RX_packet_len( void );

/**
 *******************************************************************************
 ** \brief provides the the timestamp of the frame, taken when the SFD interrupt arrives
 ** \param - None
 ** \return timestamp in symbols.
 ******************************************************************************/
uint32_t TRX_get_RX_packet_timestamp( void );

/**
 *******************************************************************************
 ** \brief Reads the received frame from the rx buffer and copies to
 **			the application buffer
 ** \param[out] *pd_data - pointer where the frame needs to be copied
 ** \param[in]  pd_length - length of the frame to be copied
 ** \return SUCCESS or FAILURE.
 ** \note Applciation has to allocate a buffer and call this function to
 **			get the frame recieved. Applciation needs to use
 **			TRX_get_RX_packet_len() to know the length of the frame being read
 **			before allocating a buffer
 ******************************************************************************/
uint8_t TRX_Read_RX_Buffer( uint8_t* pd_data, uint32_t pd_length );

/**
 *******************************************************************************
 ** \brief Function to get the payload length
 ** \param - None
 ** \return paload length in bytes.
 ******************************************************************************/
extern int get_pld_len( void );

/**
 *******************************************************************************
 ** \brief Function to reset the received packet
 ** \param - None
 ** \return - None
 ******************************************************************************/
void Reset_packet_Rx( void );

#ifdef BENCH_MARKING

void benchMark_set_time(int i);

#endif

/**
 *******************************************************************************
 ** \brief Function get the total number of symbols transmitted count
 ** \param - None
 ** \return - Total number of symbols transmitted
 ******************************************************************************/
uint32_t TRX_Total_Symbols_Txed(void);

/**
 *******************************************************************************
 ** \brief Function to reset the total number of symbols transmitted count
 ** \param - None
 ** \return - None
 ******************************************************************************/
void TRX_Reset_Total_Symbols_Txed(void);

/**
 *******************************************************************************
 ** \brief Function to do IR Calibration
 ** \param - None
 ** \return - Result of IR Calibration
 ******************************************************************************/
ADI_ADF7023_RESULT_TYPE TRX_do_IR_Calibration(void);

/**
 *******************************************************************************
 ** \brief Function to put the radio to ContinuousRxMode and viceversa.
 ** \param - mode -	"true":put the radio to ContinuousRxMode.
 **					"false":put the radio to normal mode.
 ** \return - None
 ******************************************************************************/
void TRX_Set_Continuous_Rx_Mode(int mode);

uint8_t TRX_Rx_15d4On(void);

void TRX_15d4On(void);

bool TRX_crc_status(void);

void TRX_154d_setPreambleLength(uint16_t PreambleLength);

ADI_ADF7023_DEV_HANDLE TRX_GetRadioHandle();

uint8_t TRX_Read_Temperature( double *temp );

uint8_t TRX_SetSyncWord( uint8_t bitlen, uint32_t word );

void TRX_set_15D4_rx_buffer(void);

void SetSynthCalPending(uint8_t value);

void TRX_Reload_15d4g_firmware(void);

ADI_ADF7023_RESULT_TYPE TRX_ContTransmitSettings(uint8_t mode);

#ifdef BENCH_MARKING
uint64_t getBenchMarkTime(void);
#endif

#define MAKE_UINT16(lsb,msb)                            (u16)(lsb) | (((u16)(msb)) << 8)

/*! Total bands supported */
#define MAX_PHY_BANDS_SUPPORTED				1

/*! Total phy modes supported within each bands */
#define MAX_PHY_MODES_SUPPORTED				2

/******************************************************************************/

/**
 *******************************************************************************
 ** \enum phy_band_t
 **  Phy frequency Bands
 *******************************************************************************
 **/
typedef enum freq_band_id_tag
{
	FREQ_BAND_ID_920_928_JAPAN  = 9
}freq_band_id_t;

#if !defined(CFG_PHY_BAND_920_928)
#define CFG_PHY_BAND_920_928
#endif



#if defined(CFG_PHY_BAND_920_928)
#if !defined(CFG_PHY_BAND_920_928_MODE_1)
#if !defined(CFG_PHY_BAND_920_928_MODE_2)
#define CFG_PHY_BAND_920_928_MODE_ALL
#endif
#endif
#endif

/**
 *******************************************************************************
 ** \enum FSK_Phy_Mode
 **  IDs for different modes in the SUN Page entry
 *******************************************************************************
 **/
typedef enum phy_mode_tag
{
	FSK_PHY_MODE_NONE	= 0x00000000,
	FSK_PHY_MODE_1 		= 0x00000001,
	FSK_PHY_MODE_2 		= 0x00000002,
	FSK_PHY_MODE_3 		= 0x00000004,
	FSK_PHY_MODE_4 		= 0x00000008,
	FSK_PHY_MODE_MASK 	= 0x0000000F
}phy_mode_t;

/**
 *******************************************************************************
 ** \enum phy_if_bw_t
 **  Bandwidth rate
 *******************************************************************************
 **/
typedef enum phy_if_bw_tag
{
	IF_BW_100_KHz  = 0,
	IF_BW_150_KHz  = 1,
	IF_BW_200_KHz  = 2,
	IF_BW_300_KHz  = 3
}phy_if_bw_t;

/**
 *******************************************************************************
 ** \enum Data_Rates
 **  Enumerations for different Data Rates
 *******************************************************************************
 **/
typedef enum phy_data_rate_tag
{
	DR_50_KBPS		=  50,
	DR_100_KBPS		= 100,
	DR_200_KBPS		= 200,
	DR_400_KBPS		= 400
}phy_data_rate_t;

/**
 *******************************************************************************
 ** \enum SYMBOL RATES
 **  Enumerations for different SYMBOL rates
 *******************************************************************************
 **/
typedef enum symbol_rate_tag
{
	SYM_RATE_50_KSPS = 50000,
	SYM_RATE_100_KSPS = 100000,
	SYM_RATE_200_KSPS = 200000,
}symbol_rate_t;

/*! Default 0 channel number is used*/
#define DEF_CHANNEL					0x1

/*! Default 10dBm TX Power is used*/
#define DEF_TX_POWER				TX_POWER_10 // 10 dBm

/*! Default CCA Mode1 is used to access the Channel is busy or free*/
#define DEF_CCA_MODE				0x01

/*! Default FEC Enable value*/
#define DEF_FSKFEC_SWITCH			0x00 // Disabled

/*! Default FEC used for FSK*/
#define DEF_FSKFEC_SCHEME			0x00 // NRNSC

/*! Default FEC interleaving used for FSK*/
#define DEF_FSKFEC_INTERLEAVING		        0x01

/*! Default Multi-Rate and Multi-Regional used for FSK*/
#define DEF_MRFSKSFD				0x00


/*! Default Preamble Repitions used for FSK*/
#define	DEF_PREAMBLE_REPITITIONS	0x000A	// bytes

/*! Default scramble used for FSK*/
#define DEF_FSKSCRAMBLE				0x1

/*! Default -75dBm of RSSI Threshold is used*/
#define DEF_RSSI_THRESHOLD			80 //dBm

/*! Default sync word set to 15.4g uncoded */
#define DEF_SYNC_WORD				0x904e

/*! Defines receiver sensitivity value*/
#define RX_SENSITIVITY			(-106)  // TBD

/*! Defines RSSI upper limit value*/
#define RSSI_UL					(-30)  // -26

/*! Defines RSSI lower limit value*/
#define RSSI_LL					(-110) // -97

/*! Defines default common signalling mode*/
#define COMMON_SIGNALLING_MODE		FSK_PHY_MODE_1

/*! Default maximum channel capabilty */
#define MAX_CHANNELS_CAPABILITY					0x08

/*! Default maximum SUN Page entries capabilty */
#define MAX_SUN_PAGE_ENTRIES_CAPABILITY			0x03


#if defined(CFG_PHY_BAND_920_928)

#if defined(CFG_PHY_BAND_920_928_MODE_1) || defined(CFG_PHY_BAND_920_928_MODE_ALL)
/*! Defines 920 MHz PHY Band's phy mode 1  configurations*/
#define PHY_BAND_920_MODE_1		{FSK_PHY_MODE_1,\
								SYM_RATE_50_KSPS,\
								DR_50_KBPS,\
								IF_BW_100_KHz,\
								FILTERED_2FSK_MODN,\
								200,\
								25,\
								920600,\
								38,\
								20,\
								8,\
								0x82,\
								0x13,\
								IF_BW_200_KHz }
#else
/*! Defines 920 MHz PHY Band's phy mode 1  configurations*/
#define PHY_BAND_920_MODE_1	{FSK_PHY_MODE_NONE}
#endif		/* #if defined(CFG_PHY_BAND_920_928_MODE_1) || defined(CFG_PHY_BAND_920_928_MODE_ALL) */

#if defined( CFG_PHY_BAND_920_928_MODE_2 )	|| defined(CFG_PHY_BAND_920_928_MODE_ALL)
/*! Defines 920 MHz PHY Band's phy mode 2 configurations*/
#define PHY_BAND_920_MODE_2		{FSK_PHY_MODE_2,\
								SYM_RATE_100_KSPS,\
								DR_100_KBPS,\
								IF_BW_150_KHz,\
								FILTERED_2FSK_MODN,\
								400,\
								50,\
								920900,\
								18,\
								10,\
								8,\
								0x20,\
								0x26,\
								IF_BW_300_KHz}
#else
/*! Defines 920 MHz PHY Band's phy mode 2 configurations*/
#define PHY_BAND_920_MODE_2	{FSK_PHY_MODE_NONE}
#endif		/* #if defined( CFG_PHY_BAND_920_928_MODE_2 )	|| defined(CFG_PHY_BAND_920_928_MODE_ALL) */
#endif		/* #if defined(CFG_PHY_BAND_920_928) */

#if defined(CFG_PHY_BAND_920_928)
/*! Defines 920 MHz PHY Band configurations */
#define PHY_BAND_CONFIG_920_MHz		{FREQ_BAND_ID_920_928_JAPAN,\
									{\
									PHY_BAND_920_MODE_1,\
									PHY_BAND_920_MODE_2\
									}\
									}
#else
/*! Defines 920 MHz PHY Band configurations */
#define PHY_BAND_CONFIG_920_MHz {0x0}
#endif /*defined(CFG_PHY_BAND_920_928)*/


/******************************************************************************/

/* PHY Constants */

/*!< Max SUN phy packet size */
#define aMaxPHYPacketSize			MAX_PDSU_15_4D

/*!< CCA time in symbols for MR-FSK SUN phy */
#define aCCATime					8

/*!< Size of MR-FSK SUN phy SFD*/
#define aMRFSKSFDLength				2

/*!< Size of MR-FSK SUN phy PHR*/
#define aMRFSKPHRLength				2

/*! Defines duration of transmission in symbols*/
#define phy_tx_duration( length ) phy_tx_duration_calculate(length)

/******************************************************************************/

/* Macros for PHY PIB Attribute ids */

/*!< Attribute ID for phyCurrentChannel*/
#define phyCurrentChannel       		0x00	/* 2 byte  */

/*!< Attribute ID for phyTransmitPower*/
#define phyTransmitPower        		0x02	/* 1 byte  */

/*!< Attribute ID for phyCCAMode*/
#define phyCCAMode              		0x03	/* 1 byte  */

/*!< Attribute ID for phyCurrentPage*/
#define phyCurrentPage          		0x04    /* 1 byte  */

/*!< Attribute ID for phyMaxFrameDuration*/
#define phyMaxFrameDuration			0x05	/*RO*/

/*!< Attribute ID for phySHRDuration*/
#define phySHRDuration				0x06	/*RO*/

/*!< Attribute ID for phySymbolsPerOctet*/
#define phySymbolsPerOctet			0x07	/*RO*/

/*!< Attribute ID for phyCCADuration*/
#define phyCCADuration				0x21

/*!< Attribute ID for phyCurrentSUNPageEntry*/
#define phyCurrentSUNPageEntry			0x22

/*!< Attribute ID for phyFSKFECScheme*/
#define phyFSKFECScheme				0x23

/*!< Attribute ID for phyFSKFECInterleaving*/
#define phyFSKFECInterleaving			0x24

/*!< Attribute ID for phyMaxSUNChannelSupported*/
#define phyMaxSUNChannelSupported		0x26	/*RO*/

/*!< Attribute ID for phyMRFSKSFD*/
#define phyMRFSKSFD				0x28

/*!< Attribute ID for phyNumSUNPageEntriesSupported*/
#define phyNumSUNPageEntriesSupported	        0x2a

/*!< Attribute ID for phySUNChannelsSupported*/
#define phySUNChannelsSupported			0x2b	/*RO*/

/*!< Attribute ID for phySUNPageEntriesSupported*/
#define phySUNPageEntriesSupported		0x2c

/*!< Attribute ID for phyFSKPreambleRepetitions*/
#define phyFSKPreambleRepetitions		0x2d

/*!< Attribute ID for phyFSKScramblePSDU*/
#define phyFSKScramblePSDU			0x2e

/*!< Attribute ID for phyRSSIThreshold*/
#define phyFECEnabled   			0x2f	/*propritery phy pib*/

/*!< Attribute ID for PhyTRXState*/
#define PhyTRXState				0x30	/*propritery phy pib*/

/*!< Attribute ID for phySUNPageEntriesIndex*/
#define phySUNPageEntriesIndex			0x31	/*propritery phy pib*/

/*!< Attribute ID for phyRSSIThreshold*/
#define phyRSSIThreshold 			0x32	/*propritery phy pib*/

/*!< Attribute ID for ADF7023 temperature reading*/
#define phyTemperatureRead 			0x33	/*propritery phy pib*/

/*!< Default total PHY_PIB*/
#define TOTAL_PHY_PIBS				22

/**
 *******************************************************************************
 ** \enum Modulation
 **  Enumerations for different Modulations.
 *******************************************************************************
 **/
typedef enum modulation_type_tag
{
	FILTERED_2FSK_MODN,
	FILTERED_4FSK_MODN,
	OFDM_MODN,
	OQPSK_MODN
}modulation_type_t;

/**
 *******************************************************************************
 ** \enum Freq_Ban_IDs
 ** Frequency band ids used for setting the freq band field in the SUN Page entry
 *******************************************************************************
 **/
enum
{
	FREQ_BAND_169 		= 0x00000000,
	FREQ_BAND_450_470 	= 0x00400000,
	FREQ_BAND_470_510 	= 0x00800000,
	FREQ_BAND_779_787 	= 0x00c00000,
	FREQ_BAND_863_870 	= 0x01000000,
	FREQ_BAND_896_901 	= 0x01400000,
	FREQ_BAND_901_902 	= 0x01800000,
	FREQ_BAND_902_928 	= 0x01c00000,
	FREQ_BAND_917_923 	= 0x02000000,
	FREQ_BAND_920_928 	= 0x02400000,
	FREQ_BAND_928_960 	= 0x02800000,
	FREQ_BAND_950_958 	= 0x02c00000,
	FREQ_BAND_1427_1518     = 0x03000000,
	FREQ_BAND_2400_2483     = 0x03400000,
	FREQ_BAND_MASK		= 0x07C00000
};

/**
 *******************************************************************************
 ** \enum Mod_Type_IDs
 **	 Modulation type ids used for setting the Modulation type field in the SUN
 **  Page entry
 *******************************************************************************
 **/
enum
{
	MOD_FSK			= 0x00000000,
	MOD_OFDM		= 0x00100000,
	MOD_O_QPSK		= 0x00200000,
	MOD_BIT_MASK	= 0x00300000
};

/**
 *******************************************************************************
 ** \enum Page_Number
 **  Page Number ids used for setting the Page Number field in the SUN
 **  Page entry
 *******************************************************************************
 **/
enum
{
	PAGE_NUMBER_7		= (uint32_t)0x38000000,
	PAGE_NUMBER_8		= (uint32_t)0x40000000,
	PAGE_NUMBER_9		= (uint32_t)0x48000000,
	PAGE_NUMBER_10		= (uint32_t)0x50000000,
	PAGE_NUMBER_MASK	= (uint32_t)0x78000000
};

/**
 *******************************************************************************
 ** \enum Tx_Power
 **  Enumerations for different Tx power(in dBm)
 *******************************************************************************
 **/
enum
{
	TX_POWER_MINUS_20	 	= -20,
	TX_POWER_MINUS_18	 	= -18,
	TX_POWER_MINUS_16	 	= -16,
	TX_POWER_MINUS_14	 	= -14,
	TX_POWER_MINUS_12	 	= -12,
	TX_POWER_MINUS_10	 	= -10,
	TX_POWER_MINUS_8	 	= -8,
	TX_POWER_MINUS_6	 	= -6,
	TX_POWER_MINUS_4	 	= -4,
	TX_POWER_MINUS_2	 	= -2,
	TX_POWER_0	 		=  0,
	TX_POWER_2			=  2,
	TX_POWER_4	 		=  4,
	TX_POWER_6	 		=  6,
	TX_POWER_8	 		=  8,
	TX_POWER_10	 		=  10,
	TX_POWER_12	 		=  12
};

/**
 *******************************************************************************
 ** \enum PHY_Status
 **  Enumerations for different Status of phy
 *******************************************************************************
 **/
typedef enum phy_status_tag
{
	PHY_BUSY					= 0,	/**< Phy layer is in busy state*/
	PHY_BUSY_RX					= 1,	/**< Transceiver is busy receiving data*/
	PHY_BUSY_TX					= 2,	/**< Transceiver is busy transmitting data*/
	PHY_FORCE_TRX_OFF			= 3,	/**< Transceiver gets OFF irrespective of the state it is in*/
	PHY_IDLE					= 4,	/**< Phy layer is in idle state*/
	PHY_INVALID_PARAMETER		= 5,	/**< Error while an invalid range or an supported parameter is entered */
	PHY_RX_ON					= 6,	/**< Transceiver is in receiving mode*/
	PHY_SUCCESS					= 7,	/**< Successful completion of call*/
	PHY_TRX_OFF					= 8,	/**< Transceiver is in OFF state*/
	PHY_TX_ON					= 9,	/**< Transceiver is in transmitting mode*/
	PHY_UNSUPPORTED_ATTRIBUTE	= 0xa,  /**< Error while an supported attribute is entered */
	PHY_READ_ONLY				= 0xb,  /**< Attempt to set value for a read only pib*/
	PHY_DEV_ON					= 0xc,  /**< Default state of Phy*/
	UNSUPPORTED_MODE_SWITCH		= 0x22, /**< Indicates that mode switching is not supported*/
	UNSUPPORTED_PPDU_FEC		= 0x23, /**< Indicates that FEC is not supported*/
	UNSUPPORTED_TX_CHANNEL		= 0x24, /**< Indicates that the specified channel is not supported*/
	UNSUPPORTED_MR_OQPSK_SPREADING_MODE = 0x25, /**< Indicates that the O-QPSK spreading is not supported*/
	PHY_CRC_ERROR				= 0x26,
	PHY_CCA_FAIL				= 0x27,
	PHY_HW_ERROR				= 0xFF
}phy_status_t;

/**
 *******************************************************************************
 ** \struct phy_mode_config_t
 ** Storage structure for PHY mode configuartion parameters
 *******************************************************************************
 **/
typedef struct  phy_mode_config_tag
{
	phy_mode_t phy_mode;				/**< stores phy mode id*/
	symbol_rate_t symbol_rate; 			/**< stores symbol rate for the corresponding phy mode*/
	phy_data_rate_t data_rate_kbps;		/**< stores data rate for the corresponding phy mode*/
	phy_if_bw_t if_bandwidth; 			/**< stores IF bandwidth for the corresponding phy mode*/
	modulation_type_t modulation_type;	/**< stores modulation type for the corresponding phy mode*/
	uint16_t channel_spacing_kHz;		/**< stores channel spacing in the corresponding phy mode*/
	uint16_t freq_deviation_khz;		/**< stores frequency deviation used in the corresponding phy mode*/
	uint32_t chan_center_freq0;			/**< stores centre frequency of channel 0 in the corresponding phy mode*/
	uint16_t num_of_channels;			/**< stores total number of channels present in the corresponding phy mode*/
	uint8_t symbol_duration; 			/**< stores the symbols duration corresponding to the phy mode*/
    uint8_t	symbols_per_octet; 			/**< stores the per octet symbol count info w.r.t the corresponding phy mode*/
	uchar radio_discrim_bw;				/**< stores radio discriminator bandwidth setting w.r.t the corresponding phy mode*/
	uchar radio_post_demod_bw;			/**< stores radio post demod bandwidth setting w.r.t the corresponding phy mode*/
	phy_if_bw_t cca_filter_bandwidth; 	/**< stores CCA filter bandwidth for the corresponding phy mode*/
}phy_mode_config_t;

/**
 *******************************************************************************
 ** \struct phy_band_config_t
 ** Storage structure for PHY Band configuartion parameters
 *******************************************************************************
 **/
typedef struct phy_band_config_tag
{
	freq_band_id_t freq_band_id;									/**< stores frequency band ID */
	phy_mode_config_t phy_mode_config[ MAX_PHY_MODES_SUPPORTED ];	/**< stores PHY mode configuartion parameters for each mode supported */
}phy_band_config_t;

/**
 *******************************************************************************
 ** \struct PHY_PIB
 ** Storage structure for PHY PIBs
 *******************************************************************************
 **/
typedef struct phy_pib_tag
{
	uint16_t 	CurrentChannel;  /* Holds the current channel*/
	uint8_t 	TransmitPower;   /* Holds the Tx power*/
	uint8_t 	CCAMode;         /* Holds the type of CCA used*/
	uint8_t 	CurrentPage;     /* Holds the current page*/
	uint16_t 	MaxFrameDuration;/* Holds the Maximum frame duration value */
	uint16_t 	SHRDuration;	 /* Holds the duration value of the SHR in symbols*/
	uint8_t 	SymbolsPerOctet; /* indicates the number of symbols needed for transmitting 1 byte*/
	uint16_t 	CCADuration;	 /* Duration used for performing CCA*/
	uint32_t 	CurrentSUNPageEntry; /* Holds the currently active sun page*/
	uint8_t 	FSKFECScheme;      /* Holds the FEC scheme*/
	uint8_t 	FSKFECInterleaving;	/* Indicates if FEC inteleaving is supported or not*/
	uint16_t 	MaxSUNChannelSupported;	/**< Stores the total number of channels supported in the currntly active phy mode*/
	uint8_t 	MRFSKSFD;
	uint8_t 	NumSUNPageEntriesSupported;	/**< Number of stored sun page entries in the SUNPageEntriesSupported table*/
	uint8_t 	SUNChannelsSupported[ MAX_CHANNELS_CAPABILITY ];			/**< Stores the bit map indicating which all channels are supported for the active phy mode*/
	uint32_t 	SUNPageEntriesSupported[ MAX_SUN_PAGE_ENTRIES_CAPABILITY ];	/**< Stores all the supported sun page entries*/
	uint16_t 	FSKPreambleRepetitions;	/**< Stores the number of preamble bytes that should be sent as part of SHR*/
	uint8_t 	FSKScramblePSDU;
	uint8_t         FECEnabled;			/**< Indicates if FEC is enabled or disabled*/
	phy_status_t    TRXState;			/**< Stores the TRX state*/
	uint8_t         SUNPageEntryIndex;	/**< Stores the current sun page entry which can be used for storing a new entry*/
	int8_t          RSSIThreshold;		/**< Stores the RSSI threshold to be used for CCA*/
	phy_status_t    pendingStateChange;/**< Stores the pending state to be changed to if TRX state change is not allowed at the time of request*/
#ifdef TEMPERATURE_TEST
	double temperature_read;				/**< Stores the most recent temperature reading from the ADF7023*/
#endif //#ifdef TEMPERATURE_TEST
	uint32_t SyncWord;					/**< Stores the current sync word set to the ADF7023*/

}phy_pib_t;

/**
 *******************************************************************************
 ** \struct phy_transmission(phy_tx_t)
 **			Used by MAC Layer(NHLE to PHY) for issuing PD-Data request
 *******************************************************************************
 **/
typedef struct phy_tx_struct
{
	struct phy_tx_struct *link;		/**< Ptr to next msg in list */
	uint16_t TxChannel;				/**< Holds the channel on which tx should be done */
	uint8_t PPDUCoding;				/**< Indicates if the PPDU should be FEC encoded or not*/
	uint8_t FCSLength;				/**< The size of the FCS in the passed PPDU. True for 32-bit CRC, false for 16-bit CRC */
	uint8_t ModeSwitch;				/**< Indicates if PPDU should be transmitted in a different mode */
	uint8_t NewModeSUNPage;			/**< The new SUN page if mode switching is required */
	uint8_t ModeSwitchParameterEntry;/**< The index to the table holding the mode switch info */
	uint16_t psduLength;			/**< Length of the psdu to be transmitted excluding CRC */
	uint8_t *psdu;
	uint16_t reservedPHR;			/**< 2 bytes reserved for PHR for updating at driver*/
}phy_tx_t;

/**
 *******************************************************************************
 ** \struct phy_reception(phy_rx_t)
 **		Used by the PHY Layer to receive a packet,which is then passed to the
 **		MAC Layer for further processing
 *******************************************************************************
 **/
typedef struct phy_rx_struct
{
    struct phy_rx_struct *link;			/**< Ptr to next msg */
    uint32_t sfd_rx_time;				/**< Packet reception timestamp */
    uint16_t psduLength;				/**< Length of the psdu in the received packet*/
    uint16_t FCSLength;					/**< Length of FCS recieved along with this packet */
    int psduLinkQuality;				/**< LQI of the received packet */
	uint32_t channel;					/**< The channel on which packet is received*/
    uint8_t *psdu;					/**< Place holder from where the psdu bytes are placed*/
}phy_rx_t;

/*
** ============================================================================
** Public Function Prototypes
** ============================================================================
*/

/**
 *******************************************************************************
 ** \brief Function to configure the PHY Radio
 ** \param phy_mode - phy mode to be set
 ** \retval - None
 ******************************************************************************/
void PHY_Configure_Radio( phy_mode_config_t* p_phy_config );

/**
 *******************************************************************************
 ** \brief Implements the RSSI to ED conversion
 **        This function performs the conversion of RSSI to ED value
 ** \param  rssi - holds the RSSI value which is to be converted
 ** \retval - The calculated ED value
 *******************************************************************************/
uint8_t convert_RSSI_To_ED( int8_t rssi );

/**
 *******************************************************************************
 ** \brief Implements to reset the PHY SPI driver.
 **        This function is used to forcefully reset the PHY SPI driver,
 **        when the radio stop triggering interrupts. This issue is mainly
 **        observed during transmission of data and TxEOF interrupt is not
 **	       triggered.
 **
 ** \param id - Not Used (Requied for timer callback)
 ** \param *data - Not Used (Requied for timer callback)
 ** \retval - None
 *******************************************************************************/
void phy_reset_802_15_4D_driver(void *data);

/**
 *******************************************************************************
 ** \brief Implements to reset the PHY SPI driver.
 **        This function is used to forcefully reset the PHY SPI driver,
 **        when the radio stop triggering interrupts. This issue is mainly
 **        observed during transmission of data and TxEOF interrupt is not
 **	       triggered.
 **
 ** \param id - Not Used (Requied for timer callback)
 ** \param *data - Not Used (Requied for timer callback)
 ** \retval - None
 *******************************************************************************/
void phy_reset_802_15_4D_rx_driver(void *data);

/**
 *******************************************************************************
 ** \brief Implements to reset sports driver
 **        This function is used to reset sports driver
 ** \param  - None
 ** \retval - None
 *******************************************************************************/
void reset_sports_driver( void );

void PD_Data_Indication_cb( uint8_t *pBuf, bool state_change_req
#ifdef PATCH_802_15_4D
, u8 PHY_Status
#endif //PATCH_802_15_4D
);

void PD_Data_Confirmation_cb( void* pHandle,uchar status );

void start_802_15_4D_reset_timer(uint8_t startMode);
void stop_802_15_4D_reset_timer(uint8_t stopMode);
void stop_802_15_4D_rx_reset_timer();
uint8_t PHY_Get_Symbol_Rate( void );

/*@}*/

#ifdef __cplusplus
}
#endif
#endif /*_TRX_ACCESS_H_*/

