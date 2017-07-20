
#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "sys/ctimer.h"
#include <stdio.h>
#include <string.h>
#include "net/packetbuf.h"
#include "net/queuebuf.h"

#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "net/ipv6/uip-ds6-route.h"

#include "sys/node-id.h"
#include "net/rpl/rpl.h"
#include "RF_Module_API_Handler.h"
#include <ctype.h>
#include <stdbool.h>
#include "tftp.h"
#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF                          ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF                        ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])

/*---------------------------------------------------------------------------*/
extern NodeType_t g_node_type;

/*---------------------------------------------------------------------------*/

uint8_t g_mac_ack;
/*---------------------------------------------------------------------------*/

PROCESS(tftp_client_process, "TFTP client process");
/*---------------------------------------------------------------------------*/
static uint8_t tftp_client_inited = 0;
static struct uip_udp_conn *tftp_client_conn;
static char tx_buf[TFTP_CLIENT_TX_MAX_SIZE];
static uint16_t rcvd_bytes;
static uint8_t rx_buf[TFTP_CLIENT_RX_MAX_SIZE];
static struct ctimer retxmt_timer;
typedef struct {
  uint8_t tx_buf[TFTP_CLIENT_TX_MAX_SIZE];
  uint8_t tx_buf_len;
  uint8_t num_retxmt;
}last_pkt_t;
last_pkt_t last_pkt;
/*---------------------------------------------------------------------------*/

void handle_retxmt_timer(void *ptr)
{
  last_pkt.num_retxmt++;
  if(last_pkt.num_retxmt <= TFTP_MAX_NUM_OF_RETXMTS) {
   /* Set the timer again */
   ctimer_set(&retxmt_timer, (TFTP_RETXMT_INTERVAL_MILLISEC * CLOCK_SECOND)/1000, handle_retxmt_timer, NULL);
   /* Retransmit the last packet */
   uip_udp_packet_sendto(tftp_client_conn, last_pkt.tx_buf, last_pkt.tx_buf_len,
                        uip_ds6_defrt_choose(), UIP_HTONS(RFM_TFTP_SERVER_PORT));
  }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tftp_client_process, ev, data)
{
  uint16_t opcode;
  uint8_t *buf; 
  uint16_t len;
  uint16_t index;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT();
    
    if(ev == tftp_send_rrq) {
      /* Fill with opcode RRQ */
      tx_buf[0] = ((uint16_t)OPCODE_RRQ >> 8)  & 0xFF;
      tx_buf[1] = OPCODE_RRQ & 0xFF;
      /* Fill device id */
      tx_buf[2] = uip_lladdr.addr[6];
      tx_buf[3] = uip_lladdr.addr[7];
      tx_buf[4] = 0; /* End of string indication to avoid showing as Malformed packet in wireshark*/
      /* Reset length of rx_buf to zero to receive new data */
      rcvd_bytes = 0;
      /* Send UDP payload */
      PRINTF("Sending RRQ message\n");
      g_mac_ack = 0;
      /* Save and Set timer to retxmt if data not received */
      memcpy(last_pkt.tx_buf, tx_buf, 5);
      last_pkt.tx_buf_len = 5;
      last_pkt.num_retxmt = 0;
      ctimer_set(&retxmt_timer, (TFTP_RETXMT_INTERVAL_MILLISEC * CLOCK_SECOND)/1000, handle_retxmt_timer, NULL);
      uip_udp_packet_sendto(tftp_client_conn, tx_buf, 5,
                        uip_ds6_defrt_choose(), UIP_HTONS(RFM_TFTP_SERVER_PORT));
    }
    else if((ev == tftp_rcvd_udp) && 
           (UIP_UDP_BUF->destport == tftp_client_conn->lport)) {
      ctimer_stop(&retxmt_timer);
      buf = uip_appdata;
      opcode = (uint16_t)buf[0] << 8;
      opcode |= buf[1];
      if(opcode == OPCODE_DATA) {
        /* Save received data block */
        len = uip_datalen() - TFTP_DATA_HDR_LEN;
        if((len + rcvd_bytes) > TFTP_CLIENT_RX_MAX_SIZE) {
          len = TFTP_CLIENT_RX_MAX_SIZE - rcvd_bytes;
        }
        memcpy(rx_buf + rcvd_bytes, &buf[TFTP_DATA_HDR_LEN], len);
        rcvd_bytes += len;
        /* print whole data */
        if(len != TFTP_BLOCK_SIZE){
          PRINTF("The rcvd data:\n");
          for(index = 0; index < rcvd_bytes; index++){
            PRINTF("%c", rx_buf[index]);
          }
          PRINTF("\n");
        }
        /* Send Ack for the received block */
        tx_buf[0] = ((uint16_t)OPCODE_ACK >> 8)  & 0xFF;
        tx_buf[1] = OPCODE_ACK & 0xFF;
        tx_buf[2] = buf[2];
        tx_buf[3] = buf[3];
        /* Send UDP payload */
        PRINTF("Sending ACK message\n");
        /* Save and set retransmit timer to resend ACK, if rest of the data not received */
        if(len == TFTP_BLOCK_SIZE) {
          memcpy(last_pkt.tx_buf, tx_buf, 4);
          last_pkt.tx_buf_len = 4;
          last_pkt.num_retxmt = 0;
          ctimer_set(&retxmt_timer, (TFTP_RETXMT_INTERVAL_MILLISEC * CLOCK_SECOND)/1000, handle_retxmt_timer, NULL);
        }
        g_mac_ack = 0;
        uip_udp_packet_sendto(tftp_client_conn, tx_buf, 4,
                        uip_ds6_defrt_choose(), UIP_UDP_BUF->srcport);
      }
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

void tftp_client_init(void)
{
  if(tftp_client_inited == 0) {
    tftp_client_inited = 1;
    tftp_client_conn = udp_new(NULL, UIP_HTONS(RFM_TFTP_SERVER_PORT), NULL);
    if(tftp_client_conn == NULL) {
      PRINTF("No UDP connection available\n");
      return;
    }
    process_start(&tftp_client_process, NULL);
  }
}
/*---------------------------------------------------------------------------*/

void tftp_post_client_event(process_event_t event)
{
  process_post(&tftp_client_process, event, NULL);
}
/*---------------------------------------------------------------------------*/
