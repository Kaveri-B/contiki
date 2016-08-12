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

/************** External variables declaration ********************************/

/******************* Global function definations ******************************/
int mbedtls_net_recv( void *ctx, unsigned char *buf, size_t len )
{
  int ret = 0;
  struct ssl_info *ssl_info = (struct ssl_info *)ctx;
  
  if((ssl_info->rcvd_data_ptr == NULL) || (ssl_info->rcvd_data_len == 0)){
    return MBEDTLS_ERR_SSL_WANT_READ;
  }
  
  if(len < ssl_info->rcvd_data_len) {
    memcpy(buf, ssl_info->rcvd_data_ptr, len);
    ssl_info->rcvd_data_len -= len;
    ssl_info->rcvd_data_ptr += len;
    ret = len;
  }
  else {
    memcpy(buf, ssl_info->rcvd_data_ptr, ssl_info->rcvd_data_len);
    ssl_info->rcvd_data_ptr = NULL;
    ret = ssl_info->rcvd_data_len;
    ssl_info->rcvd_data_len = 0;
  }
  return (ret);
}
/*----------------------------------------------------------------------------*/

int mbedtls_net_send( void *ctx, const unsigned char *buf, size_t len )
{
  struct ssl_info *ssl_info = (struct ssl_info *)ctx;
  struct http_socket *http_socket = (struct http_socket *)ssl_info->app_struct_ptr;

  tcp_socket_send(&http_socket->s, buf, len);
  return len;
}

/*----------------------------------------------------------------------------*/
int mbedtls_handle_rcvd_data(struct ssl_info *ssl_info, uint8_t *rcvd_data_ptr, int rcvd_data_len)
{
  int ret = -1; 
  ssl_info->rcvd_data_len = rcvd_data_len;
  ssl_info->rcvd_data_ptr = rcvd_data_ptr;
  if(ssl_info->ssl_handshake_done){
    ret = mbedtls_ssl_read( ssl_info->ssl, rcvd_data_ptr, rcvd_data_len );
    return ret;
  }
  else {
    mbedtls_start_ssl_handshake(ssl_info);
    return 0;
  }
}
/*----------------------------------------------------------------------------*/


