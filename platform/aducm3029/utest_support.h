/***********************************************************************************************************************

Copyright (c) 2013 - 2015, Analog Devices, Inc.  All rights reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.

***********************************************************************************************************************/

/** \file utest_support.h
 *******************************************************************************
 ** \brief Provides APIs for Utest Support
 **
 ** \cond STD_FILE_HEADER
 *******************************************************************************
 **  \endcond
 */

#ifndef UTEST_SUPPORT_H
#define UTEST_SUPPORT_H
#ifdef UTEST_SUPPORT
#include "sys/clock.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
** =============================================================================
** Public Macro definitions
** =============================================================================
*/

/*None*/

/*
** ============================================================================
** Public Structures, Unions & enums Type Definitions
** ============================================================================
*/

#if 0
	#define UTEST_TRXSM
	#define UTEST_TRX
	#define UTEST_PD
	#define UTEST_TRX
	#define UTEST_SW_TIMER
	#define UTEST_MAC_TEST
#endif

/*******************************************************************************
 ** \enum ts_lable_t
 **  Specific labels for making timestamps
 *******************************************************************************
 **/
/* labels for marking timestamps */

typedef enum
{
UTUL_NONE,


ADF7023_CCA_BUSY	= 1001,
ADF7023_TX_EOF	        = 1002,
ADF7023_RX_EOF	        = 1003,
ADF7023_CRC_FAILURE	= 1004,
ADF7023_PHY_OFF	        = 1005,
ADF7023_STATE_PHY_ON_15D4 = 1006,
ADF7023_STATE_PHY_TX_15D4 = 1008,
ADF7023_STATE_PHY_RX_15D4 = 1009,
ADF7023_CCA_TX	        = 1010,
ADF7023_NON_CCA_TX	= 1011,


ADUCM3025_PKT_DROP_LEN_ISSUE =	1101,
ADUCM3025_PKT_TRANSMIT	= 1102,

_6MAC_MAC_OUTPUT	= 2001,
_6MAC_MAC_INPUT 	= 2002,
_6MAC_MAC_ACK_SEND	= 2003,


_6LP_PROCESS_PAD1        = 3001,
_6LP_PROCESS_PADn	 = 3002,
_6LP_PKT_DISCARD_TYPE_01 = 3003,
_6LP_PKT_DISCARD_TYPE_11 = 3004,
_6LP_PKT_DISCARD_TYPE_10 = 3005,
_6LP_UDP_PORT_UNCOMPRESSED = 3006,
_6LP_NEXT_HEADER_INLINE	 = 3007,
_6LP_PREFIX_CONTEXT_FOUND = 3008,
_6LP_PREFIX_CONTEXT_NOT_FOUND =	3009,
_6LP_CONTEXT_BY_NUMB_FOUND = 3010,
_6LP_CONTEXT_BY_NUMB_NOT_FOUND	= 3011,
_6LP_IN_HC06_COMPRESSION   = 3012,
_6LP_IN_HC06_UNCOMPRESSION = 3013,
_6LP_IN_HC01_COMPRESSION   = 3014,
_6LP_IN_HC01_UNCOMPRESSION = 3015,
_6LP_IN_IPV6_DISPATCH_COMPRESSION = 3016,
_6LP_PKT_SENT_TO_MAC	 = 3017,
_6LP_OUTPUT_1ST_FRAG	 = 3018,
_6LP_OUTPUT_DROP_1ST_FRAG_NO_BUF = 3019,
_6LP_OUTPUT_NTH_FRAG	 = 3020	,
_6LP_OUTPUT_DROP_NTH_FRAG_NO_BUF = 3021,
_6LP_DROP_BIG_PKT_NO_FRAG = 3022,
_6LP_NO_FRAG_REQUIRED	 = 3023,
_6LP_PKT_RECEIVE_FROM_MAC = 3024,
_6LP_INPUT_DROP_REASSEMBLY_TIMEOUT = 3025,
_6LP_INPUT_1ST_FRAG	= 3026,
_6LP_INPUT_NTH_FRAG	= 3027,
_6LP_INPUT_DROP_FRAG_NOT_BELONG	= 3028,
_6LP_REASSEMBLY_INPROGRESS = 3029,
_6LP_REASSEMBLY_OFF	= 3030,
_6LP_UNKNOWN_HDR        = 3031,

DS6_TIME_EXPIRE_NEIGH_RM = 4001,
DS6_ADD_NEIGH	        = 4002,
DS6_NO_SPACE_IN_NEIGH_TBL= 4003,
DS6_REMOVE_NEIGH	= 4004,
DS6_NEIGH_FOUND	        = 4005,
DS6_NEIGH_NOT_FOUND	= 4006,
DS6_ADD_DEFRT   	= 4007,
DS6_NO_SPACE_IN_DEFRT	= 4008,
DS6_REMOVE_DEFRT	= 4009,
DS6_DEFRT_FOUND 	= 4010,
DS6_DEFRT_NOT_FOUND	= 4011,
DS6_DEFRT_INCOMPLETE	= 4013,
DS6_PREFIX_ADD  	= 4014,
DS6_NO_SPACE_PREFIX_LIST= 4015,
DS6_PREFIX_FOUND	= 4016,
DS6_PREFIX_NOT_FOUND	= 4017,
DS6_ADDR_ONLINK	        = 4018,
DS6_ADDR_NOT_ONLINK	= 4019,
DS6_IF_ADDR_ADDED	= 4020,
DS6_IF_ADDR_NOT_ADDED	= 4021,
DS6_REMOVE_IF_ADDR	= 4022,
DS6_ADDR_FOUND	        = 4023,
DS6_ADDR_NOT_FOUND	= 4024,
DS6_LL_ADDR_FOUND	= 4025,
DS6_LL_ADDR_NOT_FOUND	= 4026,
DS6_GLOBAL_ADDR_FOUND	= 4027,
DS6_GLOBAL_ADDR_NOT_FOUND= 4028,
DSC6_GLOBAL_SHRT_ADDR_FOUND = 4029,
DSC6_GLOBAL_SHRT_ADDR_NOT_FOUND	= 4030,
DS6_LL_SHRT_ADDR_FOUND	= 4031,
DS6_LL_SHRT_ADDR_NOT_FOUND = 4032,
DS6_MADDR_FOUND	        = 4033,
DS6_MADDR_NOT_FOUND	= 4034,
DS6_MADDR_REMOVE	= 4035,
DS6_AADDR_ADD   	= 4038,
DS6_NO_SPACE_AADDR	= 4039,
DS6_AADDR_REMOVED	= 4040,
DS6_AADDR_FOUND 	= 4041,
DS6_AADDR_NOT_FOUND	= 4042,
DS6_ROUTE_FOUND 	= 4043,
DS6_ROUTE_NOT_FOUND	= 4044,
DS6_ROUTE_ADD	        = 4045,
DS6_ROUTE_REMOVED	= 4046,
DS6_ROUTE_REMOVED_NXT_HOP= 4047,
DS6_ADD_SRC_ROUTE	= 4048,
DS6_SRC_ROUTE_FOUND	= 4049,
DS6_NO_SRC_ROUTE_FOUND	= 4050,
DS6_SRC_ROUTE_FOUND_TRANS_INDEX	= 4051,
DS6_NO_SRC_ROUTE_FOUND_TRANS_INDEX = 4052,
DS6_NO_DEFAULT_ROUTER	= 4053,
DS6_DAD_SUCCEDED	= 4054,
DS6_DAD_FAILED_LOCAL_ADDR = 4055,
DS6_SEND_RA	        = 4056,
DS6_SEND_PERIODIC_RA	= 4057,
DS6_SEND_RS	        = 4058,
DS6_STOP_RS     	= 4059,
DS6_6LBR_ADD	        = 4060,
DS6_NO_SPACE_6LBR	= 4061,
ND6_NS_RECEIVED 	= 4101,
ND6_NS_DROP_HOP_LIMIT_ICODE = 4102,
ND6_NS_DROP_LEN	        = 4103,
ND6_NS_DROP_SRC_ADDR_UNSPECIFIED = 4104,
ND6_ADD_NEIGH	        = 4105,
ND6_NEIGH_FOUND	        = 4106,
ND6_ND_OPTION_NOT_SUPPORTED = 4107,
ND6_SEND_NS	        = 4108,
ND6_ND_DISCARD_DEST_NULL = 4109,
ND6_NA_INPUT	        = 4110,
ND6_NA_DROP_HOP_LIMIT_ISSUE = 4111,
ND6_RS_INPUT	        = 4112,
ND6_RS_DROP_HOP_LIMIT_ISSUE = 4113,
ND6_RS_DROP_LEN	        = 4114,
ND6_RS_DROP_SRC_ADDR_UNSPECIFIED = 4115,
ND6_RA_SEND	        = 4116,
ND6_RS_SEND	        = 4117,
ND6_RA_INPUT	        = 4118,
ND6_RA_DROP_HOP_LIMIT_ISSUE	= 4119,
ND6_RA_DROP_LEN	        = 4120,
ND6_RA_DROP_6LBR_NULL	= 4121,
ND6_ND_NOT_SUPPORTED_IN_RA = 4122,
TCPIP_IP_PKT_RECEIVED	=	5001,
TCPIP_TCP_OUTPUT	=	5002,
TCPIP_DROP_BIG_PKT	=	5003,
TCPIP_DST_ADD_UNSPECIFIED =	5004,
TCPIP_DST_ADD_ONLINK	=	5005,
TCPIP_USE_SRC_RT_FOR_NEXT_HOP = 5006,
TCPIP_CHOOSE_DEF_ROUTE	=	5007,
TCPIP_DROP_DST_OFFLINK	=	5008,
TCPIP_NBR_ENTRY_INCOMPLETE =    5009,
TCPIP_REASSEMBLY_STARTED =	5101,
TCPIP_REASSMBLY_BUF_OVERFLOW =	5102,
TCPIP_PKT_SHORTER	=	5103,
TCPIP_DROP_SRC_MULICAST	=	5104,
TCPIP_DROP_MTU_EXCEED	=	5105,
TCPIP_DROP_TTL_FAIL	=	5106,
TCPIP_FORWARD_PCK	=	5107,
TCPIP_DROP_LL_SRC_ADDR_ISSUE =	5108,
TCPIP_DROP_NOT_FOR_ME	=	5109,
TCPIP_HOP_BY_HOP_HDR_PROCESS =	5111,
TCPIP_ROUTING_HDR_PROCESS    =	5112,
TCPIP_SRC_ADDR_IS_MULTICAST  =  5114,
TCPIP_LOOP_DETECT_IN_SRT_HDR =	5115,
TCPIP_NXT_ADR_IN_RH_NOT_ONLINK =5116,
TCPIP_HOP_LIMIT_EXCEDDED_ON_RH	=5117,
TCPIP_UNRECOGNISED_ROUTING_TYPE	=5118,
TCPIP_FRAG_HDR_PROCESS	=	 5119,
TCPIP_REASSEMBLY_HDR_PROCESS	=5120,
TCPIP_UNRECOGNISED_HDR	=	5121,
TCPIP_UNKNOWN_ICMP_TYPE	=	5122,
TCPIP_UDP_PKT_RECEIVED	=	5123,
TCPIP_UDP_BAD_CHECKSUM	=	5124,
TCPIP_UDP_ZERO_DST_PORT	=	5125,
TCPIP_UDP_CONNECTION_FOUND =   5126,
TCPIP_UDP_NO_CONNECTION_FOUND =	5127,
TCPIP_UDP_SEND	        =       5128,
TCPIP_UDP_DROP_LEN_ZERO	=	5129,
UIP_ICMP_ECHO_REQ_RECEIVED =	5201,
UIP_ICMP_ECHO_REPLY_SEND   =	5202,
UIP_ICMP_ICMP_ERR_MSG_SEND =	5203,
UIP_ICMP_ICMP_SEND	   =	5204,
RPL_MAX_DAO_RETRIES_REACHED	= 6001,
RPL_GOT_GLOBAL_ADDR		= 6002,
RPL_NO_GLOBBAL_ADDR		= 6003,
RPL_RECEIVED_DIS		= 6004,
RPL_MULTICAST_DIS_RECEIVED	= 6005,
RPL_UNICAST_DIS_RECEIVED	= 6006,
RPL_SENDING_MULICAST_DIS	= 6007,
RPL_RECEIVED_DIO		= 6008,
RPL_NEIGHBOR_ADDED		= 6009,
RPL_NOT_ADDED_NEIGHBOR 		= 6010,
RPL_NEIGHBOR_FOUND 		= 6011,
RPL_INVALID_DIO_LEN_ISSUE 	= 6012,
RPL_INVALID_DIO_MC_ISSUE 	= 6013,
RPL_DAG_MC_NOT_HANDLED		= 6014,
RPL_INVALID_DEST_PREFIX_OPT	= 6015,
RPL_INVALID_ROUTE_INFO_PREFIX_OPT = 6016,
RPL_INVALID_DAG_OPT		= 6017,
RPL_DAG_PREFIX_INFO_NOT_OK 	= 6018,
RPL_UNSUPPORTED_SUB_OPTION_DIO  = 6019,
RPL_DIO_NOT_SENT_UNHANDLED_DAG_MC = 6020,
RPL_MULTICAST_DIO_SENT 		= 6021,
RPL_UNICAST_DIO_SENT 		= 6022,
RPL_DAO_RECEIVED 		= 6023,
RPL_DIFFERENT_RPL_INSTANCE 	= 6024,
RPL_DAO_FORWARD			= 6025,
RPL_REMOVE_DAO_SOURCE 		= 6026,
RPL_SRC_ENTRY_FULL 		= 6027,
RPL_ADD_TO_SRC_ROUTE_TABLE	= 6028,
RPL_NO_GLOBAL_ADDRESS_SET 	= 6029,
RPL_NOT_JOINED_DAG 		= 6030,
RPL_DAO_SENDING_RETRY 		= 6031,
RPL_DAO_ACK_RECEIVED 		= 6032,
RPL_DAO_ACK_SENT 		= 6033,
RPL_MSG_RECEIVED 		= 6034,
RPL_SENDING_UNICAST_DIS         = 6035,
RPL_PURGE_ROUTE			= 6101,
RPL_REMOVE_ROUTE 		= 6102,
RPL_ROUTE_TABLE_FULL 		= 6103,
RPL_NEXT_HOP_UPDATED 		= 6104,
RPL_ROUTE_ADDED			= 6105,
RPL_REMOVE_PARENT 		= 6106,
RPL_REMOVE_NEIGHBOR		= 6107,
RPL_STARTED			= 6108,
RPL_DIO_TIMER_RESET		= 6201,
RPL_HANDLE_DAO_TIMER 		= 6202,
RPL_DAO_SCHEDULE 		= 6203,
RPL_ETX_PATH_MATRIC_CALC 	= 6301,
RPL_ETX_RANK_CALC 		= 6302,
RPL_ETX_BEST_PARENT_CALC 	= 6303,
RPL_ETX_UPDATE_MATRIC_CONTAINER	= 6304,
RPL_REMOVE_PARENTS              = 6401,
RPL_REMOVE_WORST_PARENTS	= 6402,
RPL_DAG_ALLOCATION_FAILED	= 6403,
RPL_DAG_ROOT_SET	        = 6404,
RPL_PREFIX_SET	                = 6405,
RPL_PREFIX_SET_FAILED	        = 6406,
RPL_DEFT_ROUTE_REMOVED	        = 6407,
RPL_DEFT_ROUTE_ADD           	= 6408,
RPL_DEFT_ROUTE_NULL	        = 6409,
RPL_DEFT_ROUTE_SET	        = 6410,
RPL_DAG_ALLOCATED       	= 6411,
RPL_LEAVE_DAG           	= 6413,
RPL_NO_SPACE_RPL_PARENT	        = 6414,
RPL_PARENT_ADD          	= 6415,
RPL_PARENT_FOUND	        = 6416,
RPL_NO_PARENT_FOUND	        = 6417,
RPL_BEST_PARENT_SELECTION	= 6418,
RPL_NO_BEST_PARENT	        = 6419,
RPL_CUR_PARENT_NOT_BEST	        = 6420,
RPL_NEW_PARENT_SELECTED	        = 6421,
RPL_REMOVE_DFFT_ROUTE	        = 6422,
RPL_REMOVE_NEIGH_ENTRY	        = 6423,
RPL_INSTANCE_ID_FOUND	        = 6425,
RPL_INSTANCE_ID_NOT_FOUND	= 6426,
RPL_SUPPORT_OF	                = 6427,
RPL_NOT_SUPPORT_OF      	= 6428,
RPL_PARENT_ADD_FAILED	        = 6430,
RPL_PARENT_ADDED        	= 6431,
RPL_DIO_DROP_OF_NOT_SUPPORTED	= 6432,
RPL_GLOBAL_IP_ADDED	        = 6433,
RPL_JOINED_DAG	                = 6434,
RPL_GBL_REPAIR_ADD_PARENT_FAILED= 6435,
RPL_DAG_REPAIR          	= 6436,
RPL_LOCAL_DAG_REPAIR	        = 6437,
RPL_RANK_RECALCULATE	        = 6438,
RPL_LOCAL_REPAIR_TRIGER	        = 6439,
RPL_RANK_CHANGING       	= 6440,
RPL_CANDIDATE_PARENT_NOT_VALID	= 6441,
RPL_DIO_DROP_MOP_NO_SUPPORT	= 6442,
RPL_DIO_DROP_LESS_RANK	        = 6443,
RPL_DIO_DROP_INFINITE_RANK	= 6445,
RPL_DADAG_ID_CHANGE_RESTART_NODE= 6446,
RPL_HIGHER_DTSN_RECEIVED	= 6447,
RPL_HIGHER_VERSION_RECEIVED	= 6448,
RPL_INCONSISTANCY_VER_NUMB	= 6449,
RPL_OLD_VER_NUMB_INCONSISTANCY	= 6450,
RPL_NEW_CANDIDATE_PARENT_ADDED	= 6453,
RPL_CONSISTANT_DIO	        = 6454,
RPL_PREF_PARENT_FOUND	        = 6455,
RPL_PREF_PARENT_NOT_FOUND	= 6456,
}ts_lable_t;
    
  
  
/*******************************************************************************
 ** \enum utu_status_t
 **  Specific status
 *******************************************************************************
 **/
typedef int utu_status_t;

/*******************************************************************************
 ** \enum utu_stamp_t
 **  Structure to store utu timestamps
 *******************************************************************************
 **/
typedef struct utu_stamp_struct utu_stamp_t;

/*******************************************************************************
 ** \enum utu_sfs_t
 **  sturcture to store superframe details
 *******************************************************************************
 **/
/* superframe structure */
typedef struct
{
    uint8_t bo;
    uint8_t so;
    uint8_t lcap;
    uint8_t ble;
} utu_sfs_t;

/*******************************************************************************
 ** \enum utu_stamp_struct
 **  structure to store utu timestamp
 *******************************************************************************
 **/
/* holder of stamp info */
struct utu_stamp_struct
{
    ts_lable_t label;
    long data;
    clock_time_t time;
    clock_time_t diff;
} ;

/* max number of stamps */
/*! Macro defining maximum number of stamps order*/
#define MAX_STAMPS_ORDER (4)  

/*! Macro defining maximum number of stamps*/
#define MAX_STAMPS ( 1 << MAX_STAMPS_ORDER )

/* storage for stamps */
extern utu_stamp_t utu_stamp[MAX_STAMPS];
extern volatile int utu_index;
  


/* take a timestamp */

// utu_stamp[utu_index].time = (p3time_t) sw_current_time_get(gpTmr_mod_ins); \

void utu_timestamp( ts_lable_t l, unsigned long d );

/* check if storage is full */
#define utu_is_full() ( utu_index >= MAX_STAMPS )

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

/* None */

/*
**=========================================================================
**  Public Function Prototypes
**=========================================================================
*/
      void utu_initialise( void);
   uint8_t Write_log_To_Flash(void);
     
/* None */

#ifdef __cplusplus
}
#endif
#else
#define Write_log_To_Flash()
#define utu_initialise()
#define utu_timestamp(l,d)
#endif 
#endif /* UTEST_SUPPORT_H */
