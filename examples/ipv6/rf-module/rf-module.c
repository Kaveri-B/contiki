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
#include "RF_Module_API_Handler.h"
#include <ctype.h>
#include <stdio.h>
#include <stdbool.h>

#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"
#include "mbedtls_Interface.h"
#include "http-socket.h"

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678
#define UDP_EXAMPLE_ID  190
#ifndef PERIOD
#define PERIOD 60
#endif

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
#define MAX_PAYLOAD_LEN		30

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

/***************** External variables declaration *****************************/
extern NodeType_t g_node_type;
extern RPL_MOP_Type_t g_RPL_MOP_type;
extern unsigned char g_FAN_compliant;

/***************** Static variables declaration *******************************/
static struct uip_udp_conn *client_conn;
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

/***************** Process declarations ***************************************/
PROCESS(udp_client_process, "UDP client process");
PROCESS(udp_client_process_2, "UDP client process 2");
PROCESS(uart_handler_process, "uart_handler_process process");
PROCESS(border_router_process, "Border router process");
AUTOSTART_PROCESSES(&uart_handler_process);

/***************** Static function definations ********************************/
static void
tcpip_handler(void)
{
  char *str;
  
  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
    reply++;
    printf("DATA recv '%s' (s:%d, r:%d)\n", str, seq_id, reply);
  }
}
/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
  char buf[MAX_PAYLOAD_LEN];
  
#ifdef SERVER_REPLY
  uint8_t num_used = 0;
  uip_ds6_nbr_t *nbr;
  
  nbr = nbr_table_head(ds6_neighbors);
  while(nbr != NULL) {
    nbr = nbr_table_next(ds6_neighbors, nbr);
    num_used++;
  }
#endif /* SERVER_REPLY */
  
  seq_id++;
  PRINTF("DATA send to %d 'Hello %d'\n",
         server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);
  sprintf(buf, "Hello %d from the client", seq_id);
  uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
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
/*---------------------------------------------------------------------------*/

static struct http_socket * RFM_get_http_socket(uint8_t *http_socket_id)
{
  uint8_t i;
  struct http_socket *http_socket = NULL;
  *http_socket_id = 0xFF;
  
  for(i = 0; i < HTTP_CONF_TOTAL_HTTP_SOCKETS; i++) {
    if(http_socket_array[i].used == 0) {
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

static void set_prefix_64(uip_ipaddr_t *prefix_64)
{
  rpl_dag_t *dag;
  uip_ipaddr_t ipaddr;
  memcpy(&prefix, prefix_64, 16);
  memcpy(&ipaddr, prefix_64, 16);
  prefix_set = 1;
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
  
  dag = rpl_set_root(RPL_DEFAULT_INSTANCE, &ipaddr);
  if(dag != NULL) {
    rpl_set_prefix(dag, &prefix, 64);
    PRINTF("created a new RPL dag\n");
  }
}
/*----------------------------------------------------------------------------*/
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
  uint8_t http_post_data_len = 0;
  char url[HTTP_SOCKET_URLLEN];// = "https://[aaaa::1]:443";
  struct http_socket *http_socket;
  const char *post_data = NULL;
  const char *content_type = NULL;
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
  static struct ctimer backoff_timer;
#if WITH_COMPOWER
  static int print = 0;
#endif
  
  PROCESS_BEGIN();
  
  PROCESS_PAUSE();
  
  set_global_address();
  
  PRINTF("UDP client process_2 started nbr:%d routes:%d\n",
         NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);
  print_local_addresses();
  
  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 
  
  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
         UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
  
  /* initialize serial line */
  //uart1_set_input(serial_line_input_byte);
  //serial_line_init();
  
  
#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif
  
  etimer_set(&periodic, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
    
    if(etimer_expired(&periodic)) {
      etimer_reset(&periodic);
      ctimer_set(&backoff_timer, SEND_TIME, send_packet, NULL);
      
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
  //static struct etimer et;
  
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
  
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
    
    //@kaveri
    if (ev == serial_line_event_message ) {
      PRINTF("Received serial data\n");
      //rpl_repair_root(RPL_DEFAULT_INSTANCE);
    }
#if 0
    if (ev == sensors_event && data == &button_sensor) {
      PRINTF("Initiating global repair\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    }
#endif
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
          return;
        }
        RFM_handle_http_req_cmd(&str[9]);
      }
      else if(memcmp(str, "DEBUG", 5) == 0){
	PRINTF("sizeof size_t: %d\n", sizeof(size_t));
      }
    }
    
  }
  PROCESS_END();
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
