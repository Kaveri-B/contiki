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
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "sys/ctimer.h"
#ifdef WITH_COMPOWER
#include "powertrace.h"
#endif
#include <stdio.h>
#include <string.h>

/* Only for TMOTE Sky? */
#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "net/ipv6/uip-ds6-route.h"

#include "sys/node-id.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-dag-root.h"
#include "RF_Module_API_Handler.h"
#include <ctype.h>
#include <stdio.h>
#include <stdbool.h>

#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"
#include "mbedtls_Interface.h"
#include "http-socket.h"
#include "tftp.h"

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 8765
#define UDP_EXAMPLE_ID  190
#ifndef PERIOD
#define PERIOD 20
#endif

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
#define MAX_PAYLOAD_LEN		50

#define UIP_IP_BUF                          ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_ICMP_BUF                      ((struct uip_icmp_hdr *)&uip_buf[uip_l2_l3_hdr_len])
#define UIP_UDP_BUF                        ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])

/********************** Enum/Structure declarations ***************************/
typedef enum RFModuleHttpNotifications_tag {
  RFMODULE_HTTP_NOTIFY_HTTP_SOCKET_OPENED,
  RFMODULE_HTTP_NOTIFY_DEVICE_NOT_JOINED,
  RFMODULE_HTTP_NOTIFY_URL_LEN_EXCEEDS,
  RFMODULE_HTTP_NOTIFY_WRONG_SECURITY_INDEX,
  RFMODULE_HTTP_NOTIFY_SOCKET_MEM_ALLOC_FAILED,
  
}RFModuleHttpNotifications_t;

typedef enum RFModuleHttpMethods_tag {
  RFMODULE_HTTP_METHOD_POST,
  RFMODULE_HTTP_METHOD_GET
}RFModuleHttpMethods_t;


typedef enum RFModuleTCPNotifications_tag {
  RFMODULE_TCP_NOTIFY_SOCKET_OPENED,
  RFMODULE_TCP_NOTIFY_SOCKET_CLOSED,
  RFMODULE_TCP_NOTIFY_SOCKET_MEM_ALLOC_FAILED,
  RFMODULE_TCP_NOTIFY_DATA_SENT,
  RFMODULE_TCP_NOTIFY_DATA_RCVD,
  RFMODULE_TCP_NOTIFY_SOCKET_CONNECTED,
  RFMODULE_TCP_NOTIFY_SOCKET_TIMEDOUT,
  RFMODULE_TCP_NOTIFY_SOCKET_ABORTED,
  RFMODULE_TCP_NOTIFY_INVALID_SOCKET_ID,
  RFMODULE_TCP_NOTIFY_CONNECTION_CLOSED,
  RFMODULE_TCP_NOTIFY_LENGTH_EXCEEDED,
}RFModuleTCPNotifications_t;

/***************** External variables declaration *****************************/
extern NodeType_t g_node_type;
extern RPL_MOP_Type_t g_RPL_MOP_type;
extern unsigned char g_FAN_compliant;

/***************** Static variables declaration *******************************/
static uip_ipaddr_t server_ipaddr;
static uip_ipaddr_t br_prefix = {{0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
static uip_ipaddr_t prefix;
static uint8_t prefix_set;
static int seq_id;
static int reply;
static struct uip_ds6_notification n;
static unsigned int temp_router_callback = 0;

static struct http_socket http_socket_array[HTTP_CONF_TOTAL_HTTP_SOCKETS];
static struct ssl_info ssl_info[HTTP_CONF_MAX_SSL_CONTEXT];
static mbedtls_ssl_context ssl[HTTP_CONF_MAX_SSL_CONTEXT];
static mbedtls_ssl_config conf[HTTP_CONF_MAX_SSL_CONTEXT];
static uint8_t g_device_joined;

tcp_socket_info_t tcp_socket_array[TCP_SOCKET_MAX_NUM_CONNECTIONS];

static struct uip_udp_conn *notify_server_conn;
static struct uip_udp_conn *notify_multihop_server_conn;
static struct ctimer backoff_timer;

/***************** Global variables declaration *******************************/
uint8_t g_dbg = 0;

/***************** Static functions declaration *******************************/
static void  router_callback(int event, uip_ipaddr_t *route, uip_ipaddr_t *ipaddr,
                             int numroutes);
static struct http_socket * RFM_get_http_socket(uint8_t *http_socket_id);
static void RFM_handle_http_callback(struct http_socket *s,
                                     void *ptr,
                                     http_socket_event_t ev,
                                     const uint8_t *data,
                                     uint16_t datalen);
static void RFM_free_http_socket(struct http_socket *http_socket);
static void RFM_http_socket_close(struct http_socket *s);

static tcp_socket_info_t * RFM_get_tcp_socket(uint8_t *socket_id);
static void RFM_free_tcp_socket(tcp_socket_info_t *tcp_socket_info);
static int RFM_TCP_input_callback(struct tcp_socket *tcps, void *ptr,
      const uint8_t *inputptr, int inputdatalen);
static void RFM_TCP_event_callback(struct tcp_socket *tcps, void *ptr,
      tcp_socket_event_t e);
static tcp_socket_info_t * RFM_get_tcp_socket_info(uint8_t socket_id);

static void send_packet(void *ptr);
/***************** Process declarations ***************************************/
PROCESS(udp_client_process, "UDP client process");
PROCESS(udp_client_process_2, "UDP client process 2");
PROCESS(uart_handler_process, "uart_handler_process process");
PROCESS(border_router_process, "Border router process");
//AUTOSTART_PROCESSES(&uart_handler_process);
PROCESS(network_start_process, "network_start_process process");
AUTOSTART_PROCESSES(&network_start_process);

/***************** Static function definations ********************************/
static void
tcpip_handler(void)
{
  char *str;
  uint32_t slot_interval;
  uint16_t start_devid;
  uint16_t end_devid;
  uint16_t cur_devid;
  rpl_dag_t *current_dag;
  uip_ipaddr_t *dft_ip;
  
  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
    reply++;
    if((uip_datalen() >= 6) && ((UIP_UDP_BUF->destport == notify_server_conn->lport) || 
         (UIP_UDP_BUF->destport == notify_multihop_server_conn->lport))) {
       PRINTF(" Received notification \n");
       /* Set timer to send tftp client message, based on the interval and device id */
        slot_interval = (uint16_t)str[0] << 8;
        slot_interval |= str[1] & 0xFF;
        
        start_devid = (uint16_t)str[2] << 8;
        start_devid |= str[3] & 0xFF;
        end_devid = (uint16_t)str[4] << 8;
        end_devid |= str[5] & 0xFF;
        cur_devid = (uint16_t)uip_lladdr.addr[6] << 8;
        cur_devid |= uip_lladdr.addr[7] & 0xFF;

        if((cur_devid >= start_devid) && (cur_devid <= end_devid)) {
          /* Is node single hop away or multihop away */
          current_dag = rpl_get_any_dag();
          dft_ip = uip_ds6_defrt_choose();

          if(memcmp((uint8_t *)dft_ip + 8, (uint8_t *)&current_dag->dag_id + 8, 8) == 0) {
             PRINTF("Single hop away\n");
             cur_devid = cur_devid - RFM_NODES_ID_START;  //@TBD End node device ID starts from 2, hence subtracted 2
          }
          else {
             PRINTF("Multi hop away\n");
             cur_devid = cur_devid - RFM_MULTIHOP_NODES_ID_START;  //@TBD Multihop away
          }
          slot_interval = slot_interval * cur_devid;
	  ctimer_set(&backoff_timer, (slot_interval * CLOCK_SECOND)/1000, send_packet, NULL);
        }       
    }
    else {
      tftp_post_event(tftp_rcvd_udp);
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
  tftp_post_event(tftp_send_rrq);
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;
  
  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
         PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
         PRINTF("\n");
         /* hack to make address "final" */
         if (state == ADDR_TENTATIVE) {
           uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
         }
       }
  }
}
/*---------------------------------------------------------------------------*/
#if 0
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;
  
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
  
  /* The choice of server address determines its 6LoPAN header compression.
  * (Our address will be compressed Mode 3 since it is derived from our link-local address)
  * Obviously the choice made here must also be selected in udp-server.c.
  *
  * For correct Wireshark decoding using a sniffer, add the /64 prefix to the 6LowPAN protocol preferences,
  * e.g. set Context 0 to fd00::.  At present Wireshark copies Context/128 and then overwrites it.
  * (Setting Context 0 to fd00::1111:2222:3333:4444 will report a 16 bit compressed address of fd00::1111:22ff:fe33:xxxx)
  *
  * Note the IPCMV6 checksum verification depends on the correct uncompressed addresses.
  */
  
#if 0
  /* Mode 1 - 64 bits inline */
  uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 1);
#elif 1
  /* Mode 2 - 16 bits inline */
  uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
#else
  /* Mode 3 - derived from server link-local (MAC) address */
  uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
}
#endif
/*---------------------------------------------------------------------------*/

static struct http_socket * RFM_get_http_socket(uint8_t *http_socket_id)
{
  uint8_t i;
  struct http_socket *http_socket = NULL;
  *http_socket_id = 0xFF;
  
  for(i = 0; i < HTTP_CONF_TOTAL_HTTP_SOCKETS; i++) {
    if(http_socket_array[i].used == 0) {
      memset(&http_socket_array[i], 0, sizeof(http_socket_array[i]));
      http_socket_array[i].used = 1;
      http_socket = &http_socket_array[i];
      *http_socket_id = i;
      break;
    }
  }
  return http_socket;
}

static void RFM_free_http_socket(struct http_socket *http_socket)
{
  uint8_t i;
  
  for(i = 0; i < HTTP_CONF_TOTAL_HTTP_SOCKETS; i++) {
    if(http_socket == &http_socket_array[i]) {
      http_socket_array[i].used = 0;
      break;
    }
  }
}
/*----------------------------------------------------------------------------*/

static void RFM_http_socket_close(struct http_socket *s)
{
  /* Give indication to host as http socket closed.*/
  PRINTF("HTTP_NOTIFY: HTTP_SOCKET_CLOSED\n");
  RFM_free_http_socket(s);
}
/*----------------------------------------------------------------------------*/

static void set_prefix_64(uip_ipaddr_t *prefix_64)
{
  uip_ipaddr_t ipaddr;
  memcpy(&prefix, prefix_64, 16);
  memcpy(&ipaddr, prefix_64, 16);
  prefix_set = 1;
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
}
/*----------------------------------------------------------------------------*/

#include "mbedtls/platform.h"
#include "mbedtls/config.h"
#include "mbedtls/net.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/certs.h"
#include "mbedtls/x509.h"
#include "mbedtls/x509_crt.h"
#if defined(MBEDTLS_X509_CRT_PARSE_C)
extern mbedtls_x509_crt cacert;
extern mbedtls_x509_crt clicert;
extern mbedtls_pk_context pkey;
#endif
extern mbedtls_entropy_context entropy;
extern mbedtls_ctr_drbg_context ctr_drbg;

static void RFM_handle_http_callback(struct http_socket *s,
                              void *ptr,
                              http_socket_event_t ev,
                              const uint8_t *data,
                              uint16_t datalen)
{
  uint16_t i;
  
  if((ev == HTTP_SOCKET_HEADER) || (ev == HTTP_SOCKET_DATA)) {
    if(ev == HTTP_SOCKET_HEADER)
      PRINTF("Received HTTP header:\n");
    if(ev == HTTP_SOCKET_DATA)
      PRINTF("Received HTTP body of %d bytes:\n", datalen);
    for(i = 0; i<datalen; i++)
      PRINTF("%c",data[i]);
    PRINTF("\n");
  }
  else if(ev == HTTP_SOCKET_ERR) {
    PRINTF("HTTP socket error\n");
  }
  else if(ev == HTTP_SOCKET_TIMEDOUT) {
    PRINTF("HTTP socket error: timed out\n");
  }
  else if(ev == HTTP_SOCKET_ABORTED) {
    PRINTF("HTTP socket error: aborted\n");
  }
  else if(ev == HTTP_SOCKET_HOSTNAME_NOT_FOUND) {
    PRINTF("HTTP socket error: hostname not found\n");
  }
  else if(ev == HTTP_SOCKET_CLOSED) {
    PRINTF("HTTP socket closed\n");
#if defined(MBEDTLS_X509_CRT_PARSE_C)
    mbedtls_x509_crt_free( &clicert );
    mbedtls_x509_crt_free( &cacert );
    mbedtls_pk_free( &pkey );
#endif
    mbedtls_ssl_free( s->ssl_info->ssl );
    mbedtls_ssl_config_free( s->ssl_info->conf );
    mbedtls_ctr_drbg_free( &ctr_drbg );
    mbedtls_entropy_free( &entropy );
    RFM_http_socket_close(s);
  }
}
/*----------------------------------------------------------------------------*/

static void  router_callback(int event, uip_ipaddr_t *route, uip_ipaddr_t *ipaddr,
                             int numroutes)
{
  if( event == UIP_DS6_NOTIFICATION_DEFRT_ADD ){
    
    
  }
  if(event == UIP_DS6_NOTIFICATION_DEFRT_RM) {   
  }
}
/*----------------------------------------------------------------------------*/

static void start_network(char *str)
{
  /* IPv6 CONFIGURATION */
  int i;
  uint8_t addr[sizeof(uip_lladdr.addr)];
  char rpl_mop;
  char FAN_compliant;
  
  if(memcmp(str, "6LBR", 4) == 0){
    g_node_type = NODE_6LBR;
    rpl_mop = str[5];
    FAN_compliant = str[7];
    PRINTF("MOP: %c\n",rpl_mop);
  }
  else if(memcmp(str, "6LR", 3) == 0){
    g_node_type = NODE_6LR;
    rpl_mop = str[4];
    FAN_compliant = str[6];
  }
  else if(memcmp(str, "6LN", 3) == 0){
    g_node_type = NODE_6LN;
    rpl_mop = str[4];
    FAN_compliant = str[6];
  }
  else {
    PRINTF("ERROR: Invalid node type.\n");
    return;
  }
  if(rpl_mop == '1') {
    g_RPL_MOP_type = RPL_MOP_TYPE_NON_STORING;
  }
  else if(rpl_mop == '2') {
    g_RPL_MOP_type = RPL_MOP_TYPE_STORING_NO_MULTICAST;
  }
  else {
    PRINTF("ERROR: Invalid RPL MOP type.\n");
    return;      
  }
  
  if(FAN_compliant == '0'){
    g_FAN_compliant = 0;
  }
  else if(FAN_compliant == '1'){
    g_FAN_compliant = 1;
  }
  else {
    PRINTF("ERROR: Invalid entry for FAN_compliant.\n");
    return;      
  }
  
  for(i = 0; i < sizeof(uip_lladdr.addr); i += 2) {
    addr[i + 1] = node_id & 0xff;
    addr[i + 0] = node_id >> 8;
  }
  linkaddr_copy((linkaddr_t *)addr, &linkaddr_node_addr);
  memcpy(&uip_lladdr.addr, addr, sizeof(uip_lladdr.addr));
  
  process_start(&tcpip_process, NULL);
  
  printf("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n", lladdr->ipaddr.u8[14],
           lladdr->ipaddr.u8[15]);
  }
  
  if(1) {
    uip_ipaddr_t ipaddr;
    int i;
    uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    printf("Tentative global IPv6 address ");
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:",
             ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n",
           ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
  }
  
  uip_ds6_notification_add(&n, router_callback);
  if(memcmp(str, "6LBR", 4) == 0){
    process_start(&border_router_process, NULL);
  }
  
  mbedtls_client_init();
}
/*----------------------------------------------------------------------------*/

static void RFM_handle_http_req_cmd(char *str)
{
  uint8_t str_pos;
  char url[HTTP_SOCKET_URLLEN];// = "https://[aaaa::1]:443";
  struct http_socket *http_socket;
  uint8_t ssl_security_index = 0; //Default value
  uint8_t http_socket_id = 0xFF;

  /* Parse entered URL.*/
  str_pos = 0;
  while(str[str_pos] != ' ') {
    url[str_pos] = str[str_pos];
    str_pos++;
  }

  /* Allocate memory for new HTTP socket. */
  http_socket = RFM_get_http_socket(&http_socket_id);
  if( http_socket == NULL) {
    /* Failed to allocate memory for new HTTP socket.*/
    PRINTF("ERROR: RFMODULE_HTTP_NOTIFY_SOCKET_MEM_ALLOC_FAILED\n");
    return;
  }
  /* If https request then update SSL information.*/
  if(strncmp(url, "https:", strlen("https:")) == 0) {
    /*@TBD SSL security info sholud be set prior to http request. Currently 
    SSL configuration is done here. It should be removed after providing APIs to set SSL info. */
    ssl_info[ssl_security_index].used = 1;
    ssl_info[ssl_security_index].ssl = &ssl[ssl_security_index];
    ssl_info[ssl_security_index].conf = &conf[ssl_security_index];
    mbedtls_client_init();
    mbedtls_configure_ssl_info(&ssl_info[ssl_security_index]);

    /* Link security info with http socket structure. */
    http_socket->ssl_info = &ssl_info[ssl_security_index];
    http_socket->ssl_info->app_struct_ptr = http_socket;
  }
  else {
    http_socket->ssl_info = NULL;
  }

  PRINTF("Send HTTP request\n");
  http_socket_init(http_socket);
  http_socket_post(http_socket, url, NULL, 0, NULL, RFM_handle_http_callback, NULL); 
}
/*----------------------------------------------------------------------------*/

static uint16_t rport = 8169;
static uint16_t lport = 8169;
static void RFM_handle_tcp_socket_req(char *str)
{
  tcp_socket_info_t *tcp_socket_info_ptr;
  uint8_t tcp_socket_id;
  
  tcp_socket_info_ptr = RFM_get_tcp_socket(&tcp_socket_id);
  if(tcp_socket_info_ptr == NULL){
    /* Failed to allocate memory for new TCP socket.*/
    PRINTF("TCP_SOCKET_NOTIFY: SOCKET_MEM_ALLOC_FAILED \n");
    return;        
  }
  tcp_socket_info_ptr->socket_id = tcp_socket_id;
   /* TCP listen value */
  if (str[0] == '1'){
      tcp_socket_info_ptr->tcp_listen = 1;
  }
  else{
     tcp_socket_info_ptr->tcp_listen = 0;
  }
  /* Register TCP socket.*/
  tcp_socket_register(&tcp_socket_info_ptr->tcp_socket, tcp_socket_info_ptr,
    tcp_socket_info_ptr->inputbuf, sizeof(tcp_socket_info_ptr->inputbuf),
    tcp_socket_info_ptr->outputbuf, sizeof(tcp_socket_info_ptr->outputbuf),
    RFM_TCP_input_callback,
    RFM_TCP_event_callback);
  /* Based on listen value start connect or listen.*/
  if(tcp_socket_info_ptr->tcp_listen == 1){
     tcp_socket_listen(&tcp_socket_info_ptr->tcp_socket, lport);
  }
  else {
    tcp_socket_connect(&tcp_socket_info_ptr->tcp_socket,
          &server_ipaddr, rport);
  }
  PRINTF("TCP_SOCKET_NOTIFY: SOCKET_OPENED. ID: %d\n", tcp_socket_id);
}
/*----------------------------------------------------------------------------*/

static void RFM_handle_send_tcp_data(char *str)
{
  tcp_socket_info_t *tcp_socket_info_ptr;
  uint8_t tcp_socket_id;
  char tcp_socket_id_str[2];
  uint16_t str_count;
  
  tcp_socket_id_str[0] = str[0];
  tcp_socket_id_str[1] = '\0';
  tcp_socket_id = atoi(tcp_socket_id_str);
  tcp_socket_info_ptr = RFM_get_tcp_socket_info(tcp_socket_id);
  if(tcp_socket_info_ptr == NULL) {
    PRINTF("TCP_SOCKET_NOTIFY: INVALID_SOCKET_ID %d\n",tcp_socket_id);
    return ;
  }
  //@TBD
  str = str + 2;
  tcp_socket_info_ptr->outputdata_len = 0;
  for(str_count = 0; str[str_count] != ' '; str_count++, tcp_socket_info_ptr->outputdata_len++) {
    tcp_socket_info_ptr->outputbuf[str_count] = str[str_count];
  }
  tcp_socket_send(&tcp_socket_info_ptr->tcp_socket,
                tcp_socket_info_ptr->outputbuf, tcp_socket_info_ptr->outputdata_len);
  
}
/*----------------------------------------------------------------------------*/
static void RFM_handle_tcp_socket_close_req(char *str)
{
  tcp_socket_info_t *tcp_socket_info_ptr;
  uint8_t tcp_socket_id;
  char tcp_socket_id_str[2];

  tcp_socket_id_str[0] = str[0];
  tcp_socket_id_str[1] = '\0';
  tcp_socket_id = atoi(tcp_socket_id_str);
  tcp_socket_info_ptr = RFM_get_tcp_socket_info(tcp_socket_id);
  if(tcp_socket_info_ptr == NULL) {
    PRINTF("TCP_SOCKET_NOTIFY: INVALID_SOCKET_ID %d\n",tcp_socket_id);
    return ;
  }
  tcp_socket_info_ptr->tcp_listen = 0;
  tcp_socket_close(&tcp_socket_info_ptr->tcp_socket); 
}

/*----------------------------------------------------------------------------*/

/***************** Process definations ****************************************/
PROCESS_THREAD(udp_client_process, ev, data)
{
  //static struct etimer periodic;
  //static struct ctimer backoff_timer;
#if WITH_COMPOWER
  static int print = 0;
#endif
  
  PROCESS_BEGIN();
  
  PROCESS_PAUSE();
  
  PRINTF("udp_client_process started\n");
  
  while(1) {
    PROCESS_YIELD();
    //@kaveri
    
    if(ev == serial_line_event_message && data != NULL) {
      char *str;
      str = data;
      PRINTF("Serial event message received\n");
      if(memcmp(str, "udp_start", 9) == 0) {
        if(!process_is_running(&udp_client_process_2)){
          
          process_start(&udp_client_process_2, NULL);
        }
      }
      else if(memcmp(str, "udp_stop", 8) == 0) {
        if(process_is_running(&udp_client_process_2)){
          
          process_exit(&udp_client_process_2);
        }
      }
    }
    
  }  
  PROCESS_END();
}
/*----------------------------------------------------------------------------*/

PROCESS_THREAD(udp_client_process_2, ev, data)
{
  static struct etimer periodic;
#if WITH_COMPOWER
  static int print = 0;
#endif
  
  PROCESS_BEGIN();
  
  PROCESS_PAUSE();
  
  PRINTF("UDP client process_2 started nbr:%d routes:%d\n",
         NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);
  print_local_addresses();
  
  /* Start listening port for notification message */
  notify_server_conn = udp_new(NULL, UIP_HTONS(RFM_UDP_NOTIFY_CLIENT_PORT), NULL);
  if(notify_server_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(notify_server_conn, UIP_HTONS(RFM_UDP_NOTIFY_SERVER_PORT)); 

  /* Start listening port for multihop notification message */
  notify_multihop_server_conn = udp_new(NULL, UIP_HTONS(RFM_UDP_NOTIFY_MULTIHOP_CLIENT_PORT), NULL);
  if(notify_multihop_server_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(notify_multihop_server_conn, UIP_HTONS(RFM_UDP_NOTIFY_MULTIHOP_SERVER_PORT)); 

  /* initialize tftp client process */
  tftp_client_init(); 
  
#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif
  
  //etimer_set(&periodic, SEND_INTERVAL);
  //ctimer_set(&backoff_timer, SEND_INTERVAL, send_packet, NULL);
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
    
    if(etimer_expired(&periodic)) {
      etimer_reset(&periodic);
      //ctimer_set(&backoff_timer, SEND_TIME, send_packet, NULL);
      
#if WITH_COMPOWER
      if (print == 0) {
	powertrace_print("#P");
      }
      if (++print == 3) {
	print = 0;
      }
#endif
      
    }
  }
  
  PROCESS_END();
}
/*----------------------------------------------------------------------------*/

PROCESS_THREAD(border_router_process, ev, data)
{
  
  PROCESS_BEGIN();
  
  /* While waiting for the prefix to be sent through the SLIP connection, the future
  * border router can join an existing DAG as a parent or child, or acquire a default
  * router that will later take precedence over the SLIP fallback interface.
  * Prevent that by turning the radio off until we are initialized as a DAG root.
  */
  prefix_set = 0;
  NETSTACK_MAC.off(0);
  
  PROCESS_PAUSE();
  
  //SENSORS_ACTIVATE(button_sensor);
  
  PRINTF("RPL-Border router started\n");
#if 0
  /* The border router runs with a 100% duty cycle in order to ensure high
  packet reception rates.
  Note if the MAC RDC is not turned off now, aggressive power management of the
  cpu will interfere with establishing the SLIP connection */
  NETSTACK_MAC.off(1);
#endif
  
  /* Request prefix until it has been received */
  //while(!prefix_set) {
  // etimer_set(&et, CLOCK_SECOND);
  //  request_prefix();
  // PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  // }
  
  set_prefix_64(&br_prefix);
  
  /* Now turn the radio on, but disable radio duty cycling.
  * Since we are the DAG root, reception delays would constrain mesh throughbut.
  */
  NETSTACK_MAC.off(1);
  
#if DEBUG || 1
  print_local_addresses();
#endif
  
  /* Assign a unique local address (RFC4193,
     http://tools.ietf.org/html/rfc4193). */
  //memcpy(&ipaddr, &br_prefix, sizeof(br_prefix));  
  //uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  //uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  rpl_dag_root_init_dag();

  while(1) {
      PROCESS_WAIT_EVENT();
  }
  PROCESS_END();
}
/*----------------------------------------------------------------------------*/

PROCESS_THREAD(uart_handler_process, ev, data)
{
  
  PROCESS_BEGIN();
  
  while(1)
  {
    PROCESS_YIELD();
    if(ev == serial_line_event_message && data != NULL) {
      char *str;
      str = data;
      PRINTF("Serial event message received\n");
      if(memcmp(str, "JOIN_NETWORK", 12) == 0) {
        PRINTF("JOIN_NETWORK message received\n");
        if(!process_is_running(&tcpip_process)){
          
          start_network(&str[13]);
        }
      }
      if(memcmp(str, "SEND_TO_DEFAULT", 15) == 0) {
        PRINTF("SEND_TO_DEFAULT message received\n");
        if(!process_is_running(&udp_client_process_2)){
          
          process_start(&udp_client_process_2, NULL);
        }
      }
      else if(memcmp(str, "STOP_SEND_TO_DEFAULT", 20) == 0) {
        PRINTF("STOP_SEND_TO_DEFAULT message received\n");
        if(process_is_running(&udp_client_process_2)){
          
          process_exit(&udp_client_process_2);
        }
      }
      else if(memcmp(str, "HTTP_REQ", 8) == 0) {
        PRINTF("HTTP_REQ message received\n");
        if(g_device_joined == 0){
          PRINTF("ERROR: Device not yet joined to the network.");
          return 0;
        }
        RFM_handle_http_req_cmd(&str[9]);
      }
      else if(memcmp(str, "TCP_SOCKET_REQ", 14) == 0) {
	RFM_handle_tcp_socket_req(&str[15]);
      }
      else if(memcmp(str, "SEND_TCP_DATA", 13) == 0) {
        RFM_handle_send_tcp_data(&str[14]);
      }
      else if(memcmp(str, "TCP_SOCKET_CLOSE_REQ", 20) == 0) {
        RFM_handle_tcp_socket_close_req(&str[21]);
      }
      else if(memcmp(str, "DEBUG", 5) == 0){
	PRINTF("sizeof size_t: %d\n", sizeof(size_t));
      }
    }
    
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#define NODE_TYPE_CONF_6LN	1

PROCESS_THREAD(network_start_process, ev, data)
{
  uint8_t addr[sizeof(uip_lladdr.addr)];

  PROCESS_BEGIN();

  linkaddr_copy((linkaddr_t *)addr, &linkaddr_node_addr);
  memcpy(&uip_lladdr.addr, addr, sizeof(uip_lladdr.addr));

#ifdef NODE_TYPE_CONF_6LN
  g_node_type = NODE_6LN;
#endif

  process_start(&tcpip_process, NULL);

  printf("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n", lladdr->ipaddr.u8[14],
           lladdr->ipaddr.u8[15]);
  }

  if(1) {
    uip_ipaddr_t ipaddr;
    int i;
    uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    printf("Tentative global IPv6 address ");
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:",
             ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n",
           ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
  }
  
  if(g_node_type == NODE_6LBR)
      process_start(&border_router_process, NULL);

  PROCESS_END();
}  
/*---------------------------------------------------------------------------*/

void rpl_nwk_discovery_conf(uint8_t status)
{
   if(status == 1) {
     /* Successfully discovered the DODAG root, start sending UDP packet */
     if(!process_is_running(&udp_client_process_2)){
       process_start(&udp_client_process_2, NULL);
     }
   }
}
/*----------------------------------------------------------------------------*/

/***************** Public function definations ********************************/
void mbedtls_client_app_notification(uint8_t mbedtls_event)
{
#if 0
  int ret, i;
  if(mbedtls_event == MBEDTLS_EVENT_SSL_COMPLETED){
    PRINTF("SSL handshake done. Send http request\n");
    ret = mbedtls_ssl_write(&ssl, g_http_req, sizeof(g_http_req));
    if(ret <= 0){
      PRINTF("ssl_write failed.\n");
    }
    else {
      PRINTF("Sent http request of %d bytes: ", ret);
      for(i = 0; i < sizeof(g_http_req); i++){
        PRINTF("%c",g_http_req[i]);
      }
      PRINTF("\n");
    }
  }
#endif //0
}
/*----------------------------------------------------------------------------*/

void rpl_join_indication(unsigned int joined)
{
  if(joined){
    if(temp_router_callback == 0){
      temp_router_callback = 1;
      g_device_joined = 1;
      PRINTF("JOIN_NETWORK_CONF\n");
    }
  }
}
/*---------------------------------------------------------------------------*/

static tcp_socket_info_t * RFM_get_tcp_socket(uint8_t *socket_id)
{
  uint8_t array_count;
  tcp_socket_info_t *tcp_socket_info = NULL;
  
  for(array_count = 0; array_count < TCP_SOCKET_MAX_NUM_CONNECTIONS; array_count++) {
    if (tcp_socket_array[array_count].used == 0) {
      memset(&tcp_socket_array[array_count], 0, sizeof(tcp_socket_array[array_count]));
      tcp_socket_array[array_count].used = 1;
      tcp_socket_info = &tcp_socket_array[array_count];
      *socket_id = array_count;
      break;
    }
  }
  return tcp_socket_info;
}
/*----------------------------------------------------------------------------*/

static void RFM_free_tcp_socket(tcp_socket_info_t *tcp_socket_info)
{
  uint8_t array_count;
  
  for(array_count = 0; array_count < TCP_SOCKET_MAX_NUM_CONNECTIONS; array_count++) {
    if ((tcp_socket_array[array_count].used == 1) 
         && (&tcp_socket_array[array_count] == tcp_socket_info)) {
      tcp_socket_array[array_count].used = 0;
      break;
    }
  }
}
/*----------------------------------------------------------------------------*/

static int RFM_TCP_input_callback(struct tcp_socket *tcps, void *ptr,
      const uint8_t *inputptr, int inputdatalen)
{
  tcp_socket_info_t *tcp_socket_info = (tcp_socket_info_t *)ptr;
  uint16_t i;  

  /* Give indication as TCP data received.*/
  PRINTF("TCP_SOCKET_NOTIFY: DATA_RCVD. ID: %d\n",tcp_socket_info->socket_id);
  for(i = 0; i <inputdatalen; i++){
    PRINTF("%c",inputptr[i]);
  }
  return 0;
}
/*----------------------------------------------------------------------------*/

static void RFM_TCP_event_callback(struct tcp_socket *tcps, void *ptr,
      tcp_socket_event_t e)
{
  tcp_socket_info_t *tcp_socket_info = (tcp_socket_info_t *)ptr;
  
  /* Give generated TCP event to host.*/
  PRINTF("TCP_SOCKET_NOTIFY: ");
  switch(e) {
    case TCP_SOCKET_CONNECTED:
      PRINTF("SOCKET_CONNECTED. ID: %d\n",tcp_socket_info->socket_id);
      break;
    case TCP_SOCKET_CLOSED:
      if(tcp_socket_info->tcp_listen){
        /* If TCP socket is opened for listen then, start listening on next 
        connection. */
        tcp_socket_listen(&tcp_socket_info->tcp_socket, tcp_socket_info->tcp_socket.listen_port);
        PRINTF("CONNECTION_CLOSED. ID: %d\n",tcp_socket_info->socket_id);
      }
      else {
        tcp_socket_unregister(&tcp_socket_info->tcp_socket);
        RFM_free_tcp_socket(tcp_socket_info);
        PRINTF("SOCKET_CLOSED. ID: %d\n",tcp_socket_info->socket_id);
      }
      break;
    case TCP_SOCKET_TIMEDOUT:
      PRINTF("SOCKET_TIMEDOUT. ID: %d\n",tcp_socket_info->socket_id);
      break;
    case TCP_SOCKET_ABORTED:
      PRINTF("SOCKET_ABORTED. ID: %d\n",tcp_socket_info->socket_id);
      tcp_socket_unregister(&tcp_socket_info->tcp_socket);
      RFM_free_tcp_socket(tcp_socket_info);
      break;
    case TCP_SOCKET_DATA_SENT:
      PRINTF("DATA_SENT. ID: %d\n",tcp_socket_info->socket_id);
      break;
    default:
      break;
  }
}
/*----------------------------------------------------------------------------*/

static tcp_socket_info_t * RFM_get_tcp_socket_info(uint8_t socket_id)
{
  tcp_socket_info_t *tcp_socket_info_ptr = NULL;
  uint8_t array_count;
  
  for(array_count = 0; array_count < TCP_SOCKET_MAX_NUM_CONNECTIONS; array_count++) {
    if ((tcp_socket_array[array_count].used == 1) 
         && (tcp_socket_array[array_count].socket_id == socket_id)) {
      tcp_socket_info_ptr = &tcp_socket_array[array_count];
      break;
    }
  }
  return tcp_socket_info_ptr;
}

/* Stub to avoid linker error */
void removePortForwardEntry(uint8_t *ipaddr)
{
}

