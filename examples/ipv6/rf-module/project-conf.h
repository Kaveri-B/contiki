/*
 * Copyright (c) 2015, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#include "net/ipv6/multicast/uip-mcast6-engines.h"

/* Change this to switch engines. Engine codes in uip-mcast6-engines.h */
#define UIP_MCAST6_CONF_ENGINE UIP_MCAST6_ENGINE_ROLL_TM

#ifdef RPL_CONF_WITH_NON_STORING
#undef RPL_CONF_WITH_NON_STORING
#endif

#define RPL_CONF_WITH_NON_STORING	1

#define RPL_CONF_WITH_DAO_ACK		0

//@ESL
#define RPL_CONF_DIS_INTERVAL			30
/* Notification message interval in seconds */
#define RFM_NOTIFY_MSG_INTERVAL_SEC		12
#define RFM_NOTIFY_MSG_INTERVAL			(RFM_NOTIFY_MSG_INTERVAL_SEC * CLOCK_SECOND)
/* Time interval required by each nodes in milliseconds */
#define RFM_ND6_SLOT_INTERVAL_MILISEC		100
#define RFM_NODES_ID_START		2
#define RFM_NODES_ID_END		22
/* Multihop Notification message interval in seconds */
#define RFM_NOTIFY_MULTIHOP_MSG_DELAY_SEC	((RFM_ND6_SLOT_INTERVAL_MILISEC) * (RFM_NODES_ID_END - RFM_NODES_ID_START + 1))
#define RFM_NOTIFY_MULTIHOP_MSG_DELAY		(RFM_NOTIFY_MULTIHOP_MSG_DELAY_SEC * CLOCK_SECOND /1000)
/* Time interval required by each nodes in seconds */
#define RFM_MULTIHOP_ND6_SLOT_INTERVAL_MILISEC		(RFM_ND6_SLOT_INTERVAL_MILISEC * 2)
#define RFM_MULTIHOP_NODES_ID_START		24
#define RFM_MULTIHOP_NODES_ID_END		24
/* Time interval for which the BR stores the nbr entry in the nbr table for a node */
#define RFM_UIP_ND6_REACHABLE_TIME_MILISEC	(3 * RFM_ND6_SLOT_INTERVAL_MILISEC * CLOCK_SECOND)
#define RFM_UDP_NOTIFY_SERVER_PORT	8764	
#define RFM_UDP_NOTIFY_CLIENT_PORT	8764
#define RFM_NOTIFY_MSG_SIZE		20
#define RFM_NOTIFY_DEST_IPADDR		{0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}

#define RFM_UDP_NOTIFY_MULTIHOP_CLIENT_PORT  8763
#define RFM_UDP_NOTIFY_MULTIHOP_SERVER_PORT  8763

/* TFTP related macros */
#define RFM_TFTP_SERVER_PORT		     69
#define TFTP_CLIENT_TX_MAX_SIZE		     5
#define TFTP_CLIENT_RX_MAX_SIZE		     50
#define TFTP_BLOCK_SIZE		     	     512
#define TFTP_RETXMT_INTERVAL_MILLISEC	     50
#define TFTP_MAX_NUM_OF_RETXMTS		     1

#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#undef UIP_CONF_MAX_ROUTES

#ifdef TEST_MORE_ROUTES
/* configure number of neighbors and routes */
#define NBR_TABLE_CONF_MAX_NEIGHBORS     5
#define UIP_CONF_MAX_ROUTES   30
#else
/* configure number of neighbors and routes */
#define NBR_TABLE_CONF_MAX_NEIGHBORS     5
#define UIP_CONF_MAX_ROUTES   10
#endif /* TEST_MORE_ROUTES */

#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     nullrdc_driver
#undef NULLRDC_CONF_802154_AUTOACK
#define NULLRDC_CONF_802154_AUTOACK       1

/* Define as minutes */
#define RPL_CONF_DEFAULT_LIFETIME_UNIT   60

/* 10 minutes lifetime of routes */
#define RPL_CONF_DEFAULT_LIFETIME        10

#define RPL_CONF_DEFAULT_ROUTE_INFINITE_LIFETIME 1

#ifndef RPL_CONF_WITH_NON_STORING
#define RPL_CONF_WITH_NON_STORING 1 /* Set this to run with non-storing mode */
#endif /* RPL_CONF_WITH_NON_STORING */

#if WITH_NON_STORING
#undef RPL_NS_CONF_LINK_NUM
#define RPL_NS_CONF_LINK_NUM 40 /* Number of links maintained at the root. Can be set to 0 at non-root nodes. */
//#undef UIP_CONF_MAX_ROUTES
//#define UIP_CONF_MAX_ROUTES 0 /* No need for routes */
#undef RPL_CONF_MOP
#define RPL_CONF_MOP RPL_MOP_NON_STORING /* Mode of operation*/
#endif /* WITH_NON_STORING */


/* Configurations related to HTTP socket*/
#define HTTP_CONF_MAX_HTTP_SOCKETS      0
#define HTTP_CONF_MAX_HTTPS_SOCKETS     1
#define HTTP_CONF_TOTAL_HTTP_SOCKETS    (HTTP_CONF_MAX_HTTP_SOCKETS + HTTP_CONF_MAX_HTTPS_SOCKETS)
#define HTTP_CONF_MAX_SSL_CONTEXT       HTTP_CONF_MAX_HTTPS_SOCKETS
#define HTTP_CONF_MAX_POST_DATA         (HTTP_CONF_MAX_HTTP_SOCKETS + HTTP_CONF_MAX_HTTPS_SOCKETS)

/* TCP socket command related configurations.*/
#define TCP_SOCKET_MAX_NUM_CONNECTIONS  1

#if (RFM_NOTIFY_MSG_INTERVAL_SEC < (((RFM_NODES_ID_END - RFM_NODES_ID_START + 1) *RFM_ND6_SLOT_INTERVAL_MILISEC)/1000) ) 
#error "Error: Increase RFM_NOTIFY_MSG_INTERVAL_SEC as per the device ID range"
#elif (RFM_NOTIFY_MSG_INTERVAL_SEC < (RFM_NOTIFY_MULTIHOP_MSG_DELAY_SEC/1000 +((RFM_MULTIHOP_NODES_ID_END - RFM_MULTIHOP_NODES_ID_START + 1) * RFM_MULTIHOP_ND6_SLOT_INTERVAL)/1000))
#error "Error: Increase RFM_NOTIFY_MSG_INTERVAL_SEC as per single and multihop device ID range"
#endif

#endif
