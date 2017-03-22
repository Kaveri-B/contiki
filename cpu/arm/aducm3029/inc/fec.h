/** \file fec.h
 *******************************************************************************
 ** \brief Provides interface for accessing FEC functionality.
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

#ifndef FEC_H
#define FEC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
** =============================================================================
** Public Macro definitions
** =============================================================================
*/

/*! Defines Receving Buffer Length */
//#define RX_BUFFER_LEN						2100
/*! Defines that FEC Capable is disabled */
#define NO_FEC_CAPABLE						0x0000
/*! Macro defining value for FEC NRNSC Capable */
#define FEC_NRNSC_CAPABLE					0x0001
/*! Macro defining value for FEC RSC Capable */
#define FEC_RSC_CAPABLE						0x0002
/*! Defines FEC Interleaving */
#define FEC_INTERLEAVING					0x0004
/*! Defines Configuration of FEC Capability */
#define CONFIG_FEC_CAPABILITY				(FEC_NRNSC_CAPABLE|FEC_INTERLEAVING)			
/*! Defines MAX Packet Length  in Bits */
#define MAX_PACKET_LENGTH_IN_BITS 			512//0x3FF8
/*! Defines value for VD N bits */
#define VD_N_bits    MAX_PACKET_LENGTH_IN_BITS
/*! Defines value for VD N bytes */
#define VD_N_bytes   (VD_N_bits / bpB) // 512/8  = 64
/*! Defines value for VD N short */
#define VD_N_Shorts  (VD_N_bits / bpS) // 512/16 = 32
/*! Defines maximum VD N bits */	
#define MAX_VD_N_bits	0x4000 // 2048 bytes * 8
/*! Defines maximum VD N bytes */
#define MAX_VD_N_bytes   (MAX_VD_N_bits / bpB) 
/*! Defines maximum VD N shorts */
#define MAX_VD_N_Shorts  (MAX_VD_N_bits / bpS)
/*! Macro defining value for chunk length */
#define CHUNK_LENGTH			128

/*
** ============================================================================
** Public Structures, Unions & enums Type Definitions
** ============================================================================
*/
/**
 *******************************************************************************
 ** \enum fec_sm_trigger_t
 **  Specific events for general FEC decoder state machine
 *******************************************************************************
 **/
typedef enum
{
    //TRIGGER_ENTRY = SM_TRIGGER_ENTRY, /**< Event to enter into a new state*/
    //TRIGGER_EXIT  = SM_TRIGGER_EXIT,  /**< Event to exit from the current state*/
    TRIGGER_GET_1ST_DMA_LEN,	/**< Event to trigger to get the first DMA length*/
    EXTRACT_PHR,  	   		  /**< Event for triggering NRNSC decoding for the received PHR */
    EXTRACT_REMAINDER,    	  /**< Event for triggering RSC decoding for the received PHY Payload part */
}fec_sm_trigger_t;


/**
 *******************************************************************************
 ** \enum fec_sm_state_t
 ** Different states the FEC State Machine. The state indicator is used to refer 
 ** to a state outside of the state machine.   
 *******************************************************************************
 **/
typedef enum {
	NO_FEC_IDLE_STATE,			/**< Indicates IDLE state*/
	FEC_IDLE_STATE,				/**< Indicates FEC IDLE state*/
	NRNSC_DECODING_PHR_STATE,   /**< Indicates that the machine is decoding the PHR in NRNSC*/
	RSC_DECODING_PHR_STATE,		/**< Indicates that the machine is decoding the PHR in RSC*/
	NRNSC_DECODING_PLD_STATE,	/**< Indicates that the machine is decoding the PLD in NRNSC*/
	RSC_DECODING_PLD_STATE		/**< Indicates that the machine is decoding the PLD in RSC*/
} fec_sm_state_t;

/**
 *******************************************************************************
 ** \struct fec_dec_sm_struct
 ** Structure holding information about FEC decoder State Machine  
 *******************************************************************************
 **/
typedef struct fec_sm_struct
{
    //sm_t super;					/* current state, represented as a general state machine */
    fec_sm_state_t state_ind;	/* current state indicator */
    uint8_t result;				/* result of a previous operation */
    uint8_t enable;
    uint8_t interleave;
    uint8_t scheme;
    uint16_t next_read_len;
	uint16_t rem_read_enc_bytes;
    uint16_t iInBuff;
    uint16_t iOutBuff;
    uint16_t Ninp;
	uint16_t Nskip;
	uint16_t Ntail;
	uint16_t Nout;
}fec_sm_t;

/*
** ============================================================================
** Public Variable Declarations
** ============================================================================
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
** External Variable Declarations
** ============================================================================
*/

extern fec_sm_t fec_sm;

/*
** ============================================================================
** Public Function Prototypes
** ============================================================================
*/
/**
 *****************************************************************************
 * @brief Function to enable the FEC
 * @param *p_block - pointer to the packet for FEC is to be applied
 * @param block_len_in_bytes - packet length in bytes
 * @retval - returns the extra bits to be added after FEC
 *****************************************************************************/
uchar FEC_Encode(uint8_t* p_block, uint16_t block_len_in_bytes, uint8_t* dest_buf );

/**
 *****************************************************************************
 * @brief Function used to handle the raw packet based on the event
 * @param event - type of event
 * @retval - returns length of the raw packet 
 *****************************************************************************/
uint32_t raw_frame_handler_cb( uint32_t event ); 

/**
 *****************************************************************************
 * @brief Function to get the payload length
 * @param - None
 * @retval - returns the psdu length
 *****************************************************************************/
int get_pld_len_cb(void);

/**
 *****************************************************************************
 * @brief Function to get the total length of the received packet based on FEC 
 *        and Interleaving.
 * @param total_rx_count - received packet count
 * @retval - returns the actual length of the received packet 
 *****************************************************************************/
uint16_t get_element_count_cb( uint16_t total_rx_count );

#ifdef __cplusplus
}
#endif
#endif /* FEC_H */
