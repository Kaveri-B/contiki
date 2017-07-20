
#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "sys/ctimer.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include <stdio.h>
#include <string.h>

#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "net/ipv6/uip-ds6-route.h"

#include "sys/node-id.h"
#include "net/rpl/rpl.h"
#include "RF_Module_API_Handler.h"
#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>
#include "tftp.h"
#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF                          ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF                        ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])

/*---------------------------------------------------------------------------*/
extern NodeType_t g_node_type;
extern uint8_t g_mac_ack;
/*---------------------------------------------------------------------------*/
PROCESS(tftp_server_process, "TFTP server process");

/*---------------------------------------------------------------------------*/

static uint8_t tftp_server_inited = 0;
static struct uip_udp_conn *tftp_server_conn;
static uint8_t data_buf[TFTP_CLIENT_RX_MAX_SIZE] = "TFTP server reply for node: "; //String length 28
static uint8_t tx_buf[TFTP_BLOCK_SIZE + TFTP_DATA_HDR_LEN];
static uint16_t txd_bytes;
static uint16_t txd_blocks;

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(tftp_server_process, ev, data)
{
  uint16_t opcode;
  uint8_t lladdr[8];
  uint16_t len = 0;
  uint8_t *buf;
  uint16_t block_num;
  uip_ds6_nbr_t *nbr;
  uint16_t nodeId;
  char str[6];
  uint8_t index;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT();

    if((ev == tftp_rcvd_udp) &&
           (UIP_UDP_BUF->destport == tftp_server_conn->lport)) {
      buf = uip_appdata;
      opcode = (uint16_t)buf[0] << 8;
      opcode |= buf[1];
      if(opcode == OPCODE_RRQ) {
        PRINTF("\nRcvd RRQ from: ");
        PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
        PRINTF("\n");
        nodeId = (uint16_t)buf[2] << 8;
        nodeId |= buf[3];;
        /* Received Read request, send data */
        txd_bytes = txd_blocks = 0;
        /* Add this device to nbr table cache if already not there */
        if((nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr)) == NULL) {
          /* Derive MAC extended address from IID */
          memcpy(lladdr, &UIP_IP_BUF->srcipaddr.u8[8], 8);
          lladdr[0] ^= 0x02;
          if((nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, (uip_lladdr_t *)
                              lladdr, 0, NBR_REACHABLE, NBR_TABLE_REASON_UNDEFINED, NULL)) == NULL) {
                PRINTF("\n UNable to add nbr\n");
            return 0;
          }
        }
        else {
          /* Reset reachable time */
          stimer_reset(&nbr->reachable);
        }
        
        /* Create DATA packet */
        txd_blocks++;
        tx_buf[0] = ((uint16_t)OPCODE_DATA >> 8)  & 0xFF; 
        tx_buf[1] = OPCODE_DATA & 0xFF;
	tx_buf[2] = (txd_blocks >> 8) & 0xFF;
        tx_buf[3] = txd_blocks & 0xFF;

        len = TFTP_CLIENT_RX_MAX_SIZE;
        if(len > TFTP_BLOCK_SIZE) {
          len = TFTP_BLOCK_SIZE;
        }
        txd_bytes += len;
        PRINTF("Sending data for node: %d\n", nodeId);
        /* Added this only for ease of readability of messages*/
        sprintf(str,"%d",nodeId);
        for(index = 0; index <= strlen(str); index++){
          data_buf[28 + index] = str[index];
        }
        /* Copy data from data buffer to tx_buf */
        memcpy(tx_buf + TFTP_DATA_HDR_LEN, data_buf, len);
        g_mac_ack = 0;
        uip_udp_packet_sendto(tftp_server_conn, tx_buf, len + TFTP_DATA_HDR_LEN,
                       &UIP_IP_BUF->srcipaddr, UIP_UDP_BUF->srcport);
      }
      else if(opcode == OPCODE_ACK){
        PRINTF("Received ACK message \n");
        /* Rcvd Ack for previous block, send next block*/
        block_num = (uint16_t)buf[2] << 8; 
        block_num |= buf[3];
        if((block_num > txd_blocks) || ((TFTP_BLOCK_SIZE * block_num) > TFTP_CLIENT_RX_MAX_SIZE))
          return 0;
   
        /* Create DATA packet */
        len = TFTP_CLIENT_RX_MAX_SIZE - (block_num * TFTP_BLOCK_SIZE);
        if(len > TFTP_BLOCK_SIZE) {
          len = TFTP_BLOCK_SIZE;
        }
        txd_blocks = block_num + 1;
        tx_buf[0] = ((uint16_t)OPCODE_DATA >> 8)  & 0xFF; 
        tx_buf[1] = OPCODE_DATA & 0xFF;
	tx_buf[2] = (txd_blocks >> 8) & 0xFF;
        tx_buf[3] = txd_blocks & 0xFF;
        
        /* Need to copy data from data buffer to tx_buf */
        memcpy(tx_buf + TFTP_DATA_HDR_LEN, data_buf + (block_num * TFTP_BLOCK_SIZE), len);
        g_mac_ack = 0;
        uip_udp_packet_sendto(tftp_server_conn, tx_buf, len + TFTP_DATA_HDR_LEN,
                       &UIP_IP_BUF->srcipaddr, (UIP_UDP_BUF->srcport));
        txd_bytes += len;
      }
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

void tftp_server_init(void)
{
  if(tftp_server_inited == 0) {
    tftp_server_inited = 1;
    tftp_server_conn = udp_new(NULL, 0, NULL);
    if(tftp_server_conn == NULL) {
      PRINTF("No UDP connection available\n");
      return;
    }
    udp_bind(tftp_server_conn, UIP_HTONS(RFM_TFTP_SERVER_PORT));
    process_start(&tftp_server_process, NULL);
  }
}
/*---------------------------------------------------------------------------*/

void tftp_post_event(process_event_t event)
{
  if(g_node_type == NODE_6LN) {
    tftp_post_client_event(event);
  }
  else if(g_node_type == NODE_6LBR) {
    process_post(&tftp_server_process, event, NULL);
  }
}

