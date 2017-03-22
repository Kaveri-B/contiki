/***********************************************************************************************************************

Copyright(c) 2016 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.

***********************************************************************************************************************/

/**
 * \file
 * 		Main implementation file for uart handler code
 * \author
 *    Analog Devices
 */

#include "common.h"
#include "hif_utility.h"
#include "sys/ctimer.h"
#include "uart_handler.h"

#define DEV_TYPE_HOST           1
#define DEV_TYPE_ROUTER         2
#define DEV_TYPE_EDGE_ROUTER    3
#define DEV_TYPE_NONE           4
#define DEV_TYPE                DEV_TYPE_NONE

/* Host application interface structure. */
hif_t hif;
uint8_t tx_data_pending = 0;
uart_tx_data_t send_data;
extern uint8_t uart_tx_buffer[MAX_BUFF_SIZE];
static struct ctimer guard_timer;
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function generates 1 byte checksum. 
 *
 * @param[in]   pBuff          Pointer from where the frame command starts.
 *
 * @param[in]   len           Length of the frame.
 *
 * @returns  Generated checksum
 */
uint8_t Generate_Checksum( uint8_t* pBuff,uint16_t len )
{
  uint8_t crc=0;
  uint8_t i;
  
  for ( i = 0; i < len; i++) {
    crc = crc + pBuff[i];
  }
  return ~crc + 1 ;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Transmits given bytes through UART. 
 *
 * @param[in]   TxBuffer          Pointer to tranmitting buffer..
 *
 * @param[in]   tx_size           Length of transmitting packet.
 *
 * @returns  Transmission status
 */
uint8_t send_uart_frame(uint8_t *TxBuffer, uint16_t tx_size)
{
  ADI_DISABLE_INT(UART_EVT_IRQn);
  
  if (send_data.tx_wr_index >= send_data.tx_rd_index) {
    if(((MAX_BUFF_SIZE - send_data.tx_wr_index) + send_data.tx_rd_index) < tx_size) {
      ADI_ENABLE_INT(UART_EVT_IRQn);
      return HIF_BUFF_FAILED;
    }
  }
  else if ((send_data.tx_rd_index - send_data.tx_wr_index) < tx_size ) {
    ADI_ENABLE_INT(UART_EVT_IRQn);
    return HIF_BUFF_FAILED;
  }
  
  if((send_data.tx_wr_index + tx_size - 1) > MAX_BUFF_SIZE) {
    memcpy(&send_data.data[send_data.tx_wr_index], TxBuffer,
           MAX_BUFF_SIZE - send_data.tx_wr_index);
    /* Copy the given bytes into UART Tx buffer */
    memcpy(send_data.data, &TxBuffer[MAX_BUFF_SIZE - send_data.tx_wr_index + 1],
           (tx_size) - (MAX_BUFF_SIZE - send_data.tx_wr_index));
    
    send_data.tx_wr_index = (tx_size) - (MAX_BUFF_SIZE - send_data.tx_wr_index);
  }
  else {
    /* Copy the given bytes into UART Tx buffer */
    memcpy(&send_data.data[send_data.tx_wr_index], TxBuffer, tx_size);
    send_data.tx_wr_index += tx_size;
  }
  if (hif.rx_state == RX_INIT) {
    /* If there is no reception under progress, then start transmission */
    uart_hal_write(&send_data);
  }
  else {
    /* If UART reception is under progress, then make Tx pending bit 1.*/
    tx_data_pending = 1;
  }
  ADI_ENABLE_INT(UART_EVT_IRQn);
  return HIF_SUCCESS;
}
/*----------------------------------------------------------------------------*/

#if  ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
uint8_t send_spi_frame(uint8_t *TxBuffer, uint16_t tx_size)
{
  if (send_data.tx_wr_index >= send_data.tx_rd_index) {
    if(((MAX_BUFF_SIZE - send_data.tx_wr_index) + send_data.tx_rd_index) < tx_size) {
      ADI_ENABLE_INT(UART_EVT_IRQn);
      return HIF_BUFF_FAILED;
    }
  }
  else if ((send_data.tx_rd_index - send_data.tx_wr_index) < tx_size ) {
    ADI_ENABLE_INT(UART_EVT_IRQn);
    return HIF_BUFF_FAILED;
  }
  
  if((send_data.tx_wr_index + tx_size - 1) > MAX_BUFF_SIZE) {
    memcpy(&send_data.data[send_data.tx_wr_index], TxBuffer,
           MAX_BUFF_SIZE - send_data.tx_wr_index);
    /* Copy the given bytes into UART Tx buffer */
    memcpy(send_data.data, &TxBuffer[MAX_BUFF_SIZE - send_data.tx_wr_index + 1],
           (tx_size) - (MAX_BUFF_SIZE - send_data.tx_wr_index));
    
    send_data.tx_wr_index = (tx_size) - (MAX_BUFF_SIZE - send_data.tx_wr_index);
  }
  else {
    /* Copy the given bytes into UART Tx buffer */
    memcpy(&send_data.data[send_data.tx_wr_index], TxBuffer, tx_size);
    send_data.tx_wr_index += tx_size;
  }
  if (hif.rx_state == RX_INIT) {
    /* If there is no reception under progress, then start transmission */
    spi_hal_write(&send_data);
  }
  else {
    /* If cmd reception is under progress, then make Tx pending bit 1.*/
    tx_data_pending = 1;
  }
}
#endif // ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function transmits UART ACK frame.
 *
 * @param[in]   rcvd_frame_status_t    Status of received UART frame.
 *
 * @returns  None
 */
void send_uart_frame_ack(rcvd_frame_status_t status)
{
  uint8_t ack_frame[20];
  uint8_t index = 0;
  uint16_t ack_frame_len = 2;
  
  ack_frame[0] = SOF;
  ack_frame[1] = SOF_1;
  ack_frame[2] = (ack_frame_len & 0xFF);
  ack_frame[3] = ((ack_frame_len >> 8) & 0xFF);
  ack_frame[4] = ACK_PACKET;
  ack_frame[5] = status;
  ack_frame[6] = Generate_Checksum(&ack_frame[4], ack_frame_len);
  send_uart_frame(ack_frame, ack_frame_len + 5);
}

#if  ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
void send_spi_frame_ack(rcvd_frame_status_t status)
{
  uint8_t ack_frame[20];
  uint8_t index = 0;
  uint16_t ack_frame_len = 2;
  
  ack_frame[0] = SOF;
  ack_frame[1] = SOF_1;
  ack_frame[2] = (ack_frame_len & 0xFF);
  ack_frame[3] = ((ack_frame_len >> 8) & 0xFF);
  ack_frame[4] = ACK_PACKET;
  ack_frame[5] = status;
  ack_frame[6] = Generate_Checksum(&ack_frame[4], ack_frame_len);
  send_spi_frame(ack_frame, ack_frame_len + 5);
}
#endif // ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Initializes UART Tx structure. 
 *
 * @param[in]   None
 *
 * @return  None
 */
void RFM_cmd_tx_struct_init(void){
  tx_data_pending = 0;
  send_data.data = uart_tx_buffer;
  send_data.tx_in_progress = false;
  send_data.tx_rd_index = 0;
  send_data.tx_wr_index = 0;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function handles guard timer expiry. 
 *
 * @param[in]  s Pointer to timer params.
 *
 * @return  None
 */
void guard_timer_callback(void *s)
{
    hif.rx_state = RX_INIT;
    //send_uart_frame_ack(INCOMPLETE_DATA_RECEIVED);
    /* Need to reset RxBuffer. Hence Reinitialize the UART.*/
    adi_uart_Close(hUartDevice);
    InitUART();
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function starts guard timer for given milliseconds. 
 *
 * @param[in]   time_in_milisec   Time in milliseconds to start the timer.
 *
 * @return  None
 */
void guard_timer_start(uint16_t time_in_milisec)
{
  ctimer_set(&guard_timer, time_in_milisec, guard_timer_callback, NULL);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function stops the running guard timer. 
 *
 * @param[in]   None
 *
 * @return  None
 */
void guard_timer_disable(void)
{
  ctimer_stop(&guard_timer);
}