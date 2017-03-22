#include "adi_uart.h"
#include "uart_handler.h" 

extern uip_prefix_t prefix_configured;
extern uip_802154_longaddr ieee_802154_extended_addr;
extern uint16_t accept_dio_rank = 0;
extern uint8_t current_dtsn = 0;
extern uint8_t sensor_end_node = 0;

UART_CONFIG_PARAMETERS uart_app_info;

uint8_t get_current_dtsn()
{
  return uart_app_info.dtsn;
}

void increament_and_store_dtsn()
{
  uart_app_info.dtsn++;
  App_UART_Write_To_Flash();
}

void store_prefix()
{
  memcpy(&uart_app_info.prefix, &prefix_configured, sizeof(prefix_configured));
  App_UART_Write_To_Flash();
}

/*  get the node id, it is used in mac filter functionality*/
uint8_t get_node_id (void)
{
  return ieee_802154_extended_addr.addr[0];
}

/*  get the host id, it is used in mac filter functionality,
though node_id and host_id are same , host id name for easy understanding if
more hosts are present */

uint8_t get_host_id (void)
{
  return ieee_802154_extended_addr.addr[0];
}

uint16_t get_src_pan_id(void)
{
  return uart_app_info.pan_id;
}

uint16_t get_dst_pan_id(void)
{
  return uart_app_info.pan_id;
}
