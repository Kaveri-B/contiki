/*
* Interface file between mbedTLS and contiki-os
*
*/

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "sys/ctimer.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdbool.h>
#include "sys/node-id.h"
#include "net/rpl/rpl.h"
#include "RF_Module_API_Handler.h"
#include "contiki-net.h"
#include "ip64-addr.h"
#include "http-socket.h"
//#include "mbedtls/net.h"

#include "mbedtls/platform.h"
#include "mbedtls/config.h"
#include "mbedtls/net.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
//#include "mbedtls/debug.h"
#include "mbedtls_Interface.h"
#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"

/************** Global variables declaration **********************************/
bool SSL_Handshake_Completed = false;

/************** Static variables declaration **********************************/
static unsigned char *g_ssl_rcvd_data_ptr = NULL;
static size_t g_ssl_rcvd_data_len = 0;

/************** External variables declaration ********************************/
extern uip_ip6addr_t g_server_ip6addr;
extern struct http_socket g_https_socket;
extern unsigned int dbg_g_mbedtls_net_recv;

/************** External functions prototype **********************************/
extern int mbedtls_ssl_data_rcvd(uint8_t *rcvd_data_ptr, int rcvd_data_len);

/******************* Global function definations ******************************/
int mbedtls_net_recv( void *ctx, unsigned char *buf, size_t len )
{
  int ret = 0;
  
  if((g_ssl_rcvd_data_ptr == NULL) || (g_ssl_rcvd_data_len == 0)){
    return MBEDTLS_ERR_SSL_WANT_READ;
  }
  
  if(len < g_ssl_rcvd_data_len) {
    memcpy(buf, g_ssl_rcvd_data_ptr, len);
    g_ssl_rcvd_data_len -= len;
    g_ssl_rcvd_data_ptr += len;
    ret = len;
  }
  else {
    memcpy(buf, g_ssl_rcvd_data_ptr, g_ssl_rcvd_data_len);
    g_ssl_rcvd_data_ptr = NULL;
    ret = g_ssl_rcvd_data_len;
    g_ssl_rcvd_data_len = 0;
  }
  return (ret);
}
/*----------------------------------------------------------------------------*/

int mbedtls_net_send( void *ctx, const unsigned char *buf, size_t len )
{
  tcp_socket_send(&g_https_socket.s, buf, len);
  return len;
}

/*----------------------------------------------------------------------------*/
int mbedtls_handle_rcvd_data(uint8_t *rcvd_data_ptr, int rcvd_data_len)
{
  int ret = -1; 
  g_ssl_rcvd_data_len = rcvd_data_len;
  g_ssl_rcvd_data_ptr = rcvd_data_ptr;
  if(SSL_Handshake_Completed){
    ret = mbedtls_ssl_data_rcvd(rcvd_data_ptr, rcvd_data_len);
    return ret;
  }
  else {
    mbedtls_start_ssl_handshake();
    return 0;
  }
}
/*----------------------------------------------------------------------------*/


