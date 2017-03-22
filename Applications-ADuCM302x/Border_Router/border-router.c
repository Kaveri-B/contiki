/*
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
#include "startup.h"             /* Obtain defintion of WEAK */

#include "uart_handler.h"
#include <gpio/adi_gpio.h>
#define DEBUG	 DEBUG_NONE

#include "net/ip/uip-debug.h"
#include "net/rpl/rpl-private.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define PREFIX_VALID_LT 		((unsigned long)-1)
#define PREFIX_PREFERRED_LT ((unsigned long)-1)

PROCESS(border_router_process, "border router process");

extern uip_ipaddr_t  prefix_configured;   
/*****************************************************************************/
void led_on(void)
{
  adi_gpio_SetLow( ADI_GPIO_PORT1,ADI_GPIO_PIN_12);	
}
/*****************************************************************************/
void led_off(void)
{
  adi_gpio_SetHigh( ADI_GPIO_PORT1,ADI_GPIO_PIN_12);	  
}
/*****************************************************************************/
void
set_prefix(uip_ipaddr_t *prefix_64)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;
  static uip_ipaddr_t prefix;
  
  memcpy(&prefix, prefix_64, 16);
  memcpy(&ipaddr, prefix_64, 16);
  
  memcpy(&prefix_configured, prefix_64, sizeof(prefix_configured));
  store_prefix();
  NVIC_SystemReset(); 
}

uint8_t rpl_instance = 0x01;

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(border_router_process, ev, data)
{
  
  uip_ipaddr_t ipaddr;
     
  PROCESS_BEGIN();

  /* Assign a unique local address (RFC4193,
     http://tools.ietf.org/html/rfc4193). */
  memcpy(&ipaddr, &prefix_configured, sizeof(prefix_configured));  
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

 
  rpl_dag_root_init_dag();

  while(1) {
    PROCESS_WAIT_EVENT();
  }
  
  PROCESS_END();
}
