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
*/

// a simple IEEE.802.15.4 <-> SLIP adapter

#include "contiki.h"
#include <string.h>
#include <stdio.h>
#include "rime.h"
#include "radio.h"
#include "uip.h"
#include "netstack.h"
#include "slip.h"

#define DEBUG 1

/*---------------------------------------------------------------------------*/
PROCESS(adapter_init, "Adapter Init");
//AUTOSTART_PROCESSES(&adapter_init, &uart_handler_process);
/*---------------------------------------------------------------------------*/

static unsigned short

cca(void)
{
  return 0;
  
}


static void

input (void)

{
  int len;
  len = packetbuf_datalen();
  if(len>0) 
  {
    slip_write(packetbuf_dataptr(), len);
  }
  
}



static void

send_packet(mac_callback_t sent, void *ptr)
{
}
static void

send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
}


static int
on(void)
{
  return NETSTACK_RADIO.on();
}



static int
off(int keep_radio_on)
{
  if(keep_radio_on) {
    return NETSTACK_RADIO.on();
  } else {
    return NETSTACK_RADIO.off();
  }
}


static void

init(void)
{
  on();
}


const struct rdc_driver stubrdc_driver =

{
  "stub-rdc",
  init,
  send_packet,
  input,
  on,
  off,
  cca,
};

/*---------------------------------------------------------------------------*/

static void slip_callback (void)
{
  const char* frame = &uip_buf[UIP_LLH_LEN]; // from slip_process in slip.c 
  
  NETSTACK_RADIO.on();
  
  if (NETSTACK_RADIO.send (frame, uip_len) == RADIO_TX_ERR)
  {
  }
  memset(&uip_buf,0,uip_len);
}

PROCESS_THREAD(adapter_init, ev, data)
{
  PROCESS_BEGIN();
  
  slip_set_tcpip_input_callback(slip_callback);
  
  PROCESS_END();
}

