/*
Copyright (c) 2013 - 2015, Analog Devices, Inc.  All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted 
(subject to the limitations in the disclaimer below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation 
  and/or other materials provided with the distribution.
* Neither the name of Analog Devices, Inc. nor the names of its contributors 
  may be used to endorse or promote products derived from this software without 
  specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* This file is part of the Contiki operating system.
*
*/

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"
#include "net/ipv6/uip-nd6.h"
#include "net/ipv6/sicslowpan.h"

#include "net/netstack.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define DEBUG	 DEBUG_NONE

#include "net/ip/uip-debug.h"
#include <uart/adi_uart.h>

#include "uart_handler.h"

extern ADI_UART_HANDLE hUartDevice;


#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_SERVER_PORT	1035
#define MCAST_SINK_UDP_PORT 3001 /* Host byte order */

static struct uip_udp_conn *server_conn;
#define DEV_TYPE_HOST           1
#define DEV_TYPE_ROUTER         2
#define DEV_TYPE_EDGE_ROUTER    3
#define DEV_TYPE_NONE           4
#define DEV_TYPE                DEV_TYPE_NONE

/*****************************************************************************/

PROCESS(udp_server_process, "UDP server process");
#if ((DEV_TYPE == DEV_TYPE_EDGE_ROUTER))
AUTOSTART_PROCESSES(&udp_server_process,NULL);
#endif //
/*---------------------------------------------------------------------------*/
void
tcpip_handler(void)
{
  char *str;
  char len = 0;
  len = uip_datalen();
    
  if(uip_newdata()) {
    str = (char*)uip_appdata;
    str[len] = '\0';
    PRINTF("DATA recv '%s'\n", str);
 
   // if(ANode_GetAutoSend())
      Send_ReceivedData2Uart((unsigned char *)str,0,len);
  }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  server_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));
 
 
 while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}

void form_udp_packet(uip_ipaddr_t *to_address, char *buf, int len)
{
    uip_udp_packet_sendto(server_conn, buf, len, to_address, UIP_HTONS(UDP_SERVER_PORT));
}
