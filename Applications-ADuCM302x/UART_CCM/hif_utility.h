
#ifndef __HIF_UTILITY_H__
#define __HIF_UTILITY_H__

/**< index to the SOF field in the received buffer*/
#define SOF		0xA5

/**< index to the SOF field in the received buffer*/
#define SOF_1		0x7E

/**< default maximum buffer size*/
#define ACK_PACKET                  0xFE
#define PRINT_PACKET                0xFB
#define PRINT_HEX_PACKET            0xFC
#define RF_MODULE_PACKET            0xFD

/* Maximum buffer size used for UART transmission. */
#define MAX_BUFF_SIZE           500

typedef enum uart_frame_type_tag
{
  FRAME_TYPE_UI,
  FRAME_TYPE_RF_MODULE,
}uart_frame_type_t;

typedef enum uart_frame_rx_states_tag
{
  RX_INIT,
  RX_SOF_1,
  READING_HDR,
  READING_PLD,
  READ_ERR_BYTES,
  TELEC_TEST
}uart_frame_rx_states_t;

typedef enum rcvd_frame_status_tag
{ 
    UART_DATA_SUCCESS,
    INCOMPLETE_DATA_RECEIVED,
    UART_DATA_CRC_FAILED
}rcvd_frame_status_t;


typedef struct hif_tag
{
  uart_frame_rx_states_t rx_state;
  uart_frame_type_t uart_frame_type;
}hif_t;

typedef struct uart_tx_data_tag
{
    uint8_t *data;
    uint16_t tx_rd_index;
    uint16_t tx_wr_index;
    bool_t tx_in_progress;
} uart_tx_data_t;

enum
{ 
    HIF_SUCCESS	= 0x00,
    HIF_SEND_CONF,
    HIF_NLLE_BUFF_FAILED,
    HIF_BUFF_FAILED,
};

extern uint8_t tx_data_pending;
extern hif_t hif;
extern uart_tx_data_t send_data;

uint8_t Generate_Checksum( uint8_t* pBuff,uint16_t len );
uint8_t send_uart_frame(uint8_t *TxBuffer, uint16_t tx_size);
void guard_timer_callback(void *s);
void send_uart_frame_ack(rcvd_frame_status_t status);
void guard_timer_disable(void);

#endif //__HIF_UTILITY_H__