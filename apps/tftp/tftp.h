
#ifndef _TFTP_H_
#define _TFTP_H_


#define OPCODE_RRQ   1
#define OPCODE_WRQ   2
#define OPCODE_DATA  3
#define OPCODE_ACK   4
#define OPCODE_ERROR 5

#define TFTP_DATA_HDR_LEN	4
typedef enum {
  tftp_send_rrq,
  tftp_rcvd_udp,
}TFTP_EVENTS;

void tftp_client_init(void);
void tftp_server_init(void);
void tftp_post_event(process_event_t event);
void tftp_post_client_event(process_event_t event);


#endif /* _TFTP_H_ */
