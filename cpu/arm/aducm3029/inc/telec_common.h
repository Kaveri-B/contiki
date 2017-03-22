/*!
 *****************************************************************************
 * @file:     common.h
 * @brief:    common include file for all example
 * @version: $Revision: 32612 $
 * @date:    $Date: 2015-11-08 13:00:01 -0500 (Sun, 08 Nov 2015) $
 *-----------------------------------------------------------------------------
 *
 * Copyright (C) 2009-2013 ARM Limited. All rights reserved.
 *
 * ARM Limited (ARM) is supplying this software for use with Cortex-M3
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************/


#ifndef __TELEC_COMMON_H__
#define __TELEC_COMMON_H__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include <services/wdt/adi_wdt.h>
#include <drivers/uart/adi_uart.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int 	base_t;
typedef base_t 			stime_t;
typedef base_t 			p3time_t;

typedef  uint8_t  			ubit;
typedef  ubit				uchar;
typedef  uint16_t			ushort;
typedef  uint32_t			ulong;

typedef uint8_t 			uint8;
typedef ushort 				uint16;

typedef uint8_t 			u8;
typedef uint8_t 			bitfield;

typedef uint32_t u32;
typedef uint16_t u16;

#define UART_INT_NUM        XINT_EVT1_IRQn

/*! Macro for defining Radio number*/
#define RADIO_INT_NUM		XINT_EVT2_IRQn

#define NULL_POINTER                (void*)0x0

#define SUCCESS 0
#define FAILURE 1

#define MAX_BASE_VALUE (~((base_t)0x0))

typedef __istate_t irq_state_t;

//#define ADI_ENTER_CRITICAL_REGION    adi_int_EnterCriticalRegion
//#define ADI_EXIT_CRITICAL_REGION     adi_int_ExitCriticalRegion

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define PATCH_802_15_4D

#ifdef ADCM3029_EZKIT_USED
/*! Macro for defining soft int number*/
#define MAC_SOFT_INT_NUM	XINT_EVT3_IRQn

#else
/*! Macro for defining soft int number*/
#define MAC_SOFT_INT_NUM    XINT_EVT0_IRQn
#endif

#define CCA_THRESHOLD -83
//#define TELEC_TEST_HARNESS
//#define STATISTICS_ENABLED

#define RADIO_STATS
#define PHY_STATS
#define MAC_STATS
#define UTIL_STATS


typedef struct {
#ifdef RADIO_STATS
	int radio_trx_init;			//number of radio initializations
	int radio_trx_err;			//any error in trx_access.c
	int radio_sport_err;		//sport related error
	int radio_rssi_fail;		//RSSI related radio failure
	int radio_ircal_cnt;		//number of IR calibrations done
	int radio_ircal_fail_cnt;	//IR calibration failures
	int radio_synth_cal_performed;
    int RSSI_val;
    int Tx_timeout_re_init_count;
    int Rx_timeout_re_init_count;
#endif

#ifdef PHY_STATS
	int phy_sfd_detect_event;	//sync detect event recv by PHY
	union   {
                int phy_rx_complete_event;	//rx complete event recv by PHY
                int Rx_EOF;
            };
	union   {
                int phy_tx_complete_event;	//tx complete event recv by PHY
                int Tx_EOF;
            };
    int PHR_detect;
    int CRC_failure;
    int CCA_Failed;
    int pkt_Tx_time;
    int pkt_Rx_time;
    int cca_time;

	int phy_reset_sport;		//sport reset forced
	int phy_reset_sport_plme_set;	//sport reset due to PLME.SET with FEC.
	int phy_sfd_detect_event_cancel;	//SDF event was cancelled due to transmission started
	int	phy_sfd_detect_event_during_cca;	//CCA fail due to SDF event happen during backoff period
	int phy_cca_interrupt_err;
	int phy_tx_eof_interrupt_err;
	int phy_rx_eof_interrupt_err;
#endif

#ifdef MAC_STATS
	int mac_cca_fail_busy;		//A single CCA attempt failed
	int mac_csma_fail_busy;		//A full CSMA-CA attempt failed
	int mac_cca_abort;			//A single CCA aborted due to pkt received.
	int mac_no_ack_fail;		//A single TX fail due to no ACK received
	int mac_tx_max_retry_fail;	//pkt TX fail due to max retries from ACK failures
	int mac_rcv_pkt;			//packet recv in PD_Data_Indication_cb
	int mac_rcv_crc_fail;		//CRC fail on recv packet
	int mac_rcv_parse_err;		//recv pkt parse error
	int mac_rcv_ack_pkt;		//ACK pkt recv in mac_frame_parse
	int mac_rcv_bcn_pkt;		//beacon pkt recv in mac_frame_parse
	int mac_rcv_data_pkt;		//data pkt recv in mac_frame_parse
	int mac_rcv_cmd_pkt;		//command pkt recv in mac_frame_parse
	int mac_trxsm_requeue_err;	//error related to requeue mechanism of TRX state machine
	int mac_backoff_suspend;	//No of times backoff suspended, due to reception of packet
	int mac_backoff_resume;		//No of times backoff resumed, after backoff suspended.
#endif

#ifdef UTIL_STATS
	int util_bm_alloc_err;		//happens when memory allocation fails
	int util_bm_alloc_err2;		//misch mememory manager error
	int util_queue_err;				//misch queue error
#endif

} rf_statistics_t;

#ifdef __cplusplus
}
#endif

#endif /* __TELEC_COMMON_H__ */
