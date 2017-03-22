
#ifndef __WSN_UI_HANDLER_H__
#define __WSN_UI_HANDLER_H__

#define UART_HEADER_LEN		4
#define UART_START_OF_PAYLOAD	UART_HEADER_LEN

#define UARTCMDPOS    3

#define SINGLEOBJLEN  5
#define LONGOBJLEN    5
#define INTOBJLEN     3
#define BYTEOJLEN     2
#define STROBJLEN     2
#define UNITOBJLEN    3
#define BYTEARYOBJLEN  2
#define STR_MARK      's' // ASCII 115 or 0x73
#define LONG_MARK     'l' // ASCII 108 or 0x6C
#define SINGLE_MARK   'f' // ASCII 102 or 0x66
#define INT_MARK      'i' // ASCII 105 or 0x69
#define BYTE_MARK     'c' // ASCII 99 or 0x63
#define BYTEARY_MARK  'a' // ASCII 97 or 0x61
#define UNIT_MARK     'u' // ASCII 117 or 0x75


typedef enum 
{
  UART_SUBCMD_GETINITBLK = 'A',
  UART_SUBCMD_GETTRANSREG,
  UART_SUBCMD_MUTE,
  UART_SUBCMD_RST,
  UART_SUBCMD_ERASECONFIG,
  UART_SUBCMD_ADDRESMESG,
  UART_SUBCMD_SRD_RESPOND,
  UART_SUBCMD_AUTOSEND,
  UART_SUBCMD_FLUSHQ,
  UART_SUBCMD_SRDBUFFER,
  UART_SUBCMD_CHECKDEVICE,
  UART_SUBCMD_GETCONFIG,
  UART_SUBCMD_SETCONFIG,
  UART_SUBCMD_TXMSG,
  UART_SUBCMD_GETSYSSTR,
  UART_SUBCMD_SETSYSSTR,
  UART_WIREDNW_DATA,
  UART_IOT_GET_CONFIG = 0x56,
  UART_IOT_SET_CONFIG
}STANDARDCMDS;

void WSN_UI_Handler(uint8_t *rx_buff);
uint8_t UARTUIFrameTx(unsigned char Cmd, int DataLen);
uint8_t* get_server_addr(void);
uint16_t get_server_port(void);
uint8_t is_dtls_enabled(void);
uint8_t* get_psk_identity(void);
uint8_t* get_psk_key(void);
uint8_t* get_psk_key(void);
uint8_t is_dhcp(void);
uint8_t* get_ip_addr(void);
uint8_t* get_subnet_mask(void);
uint8_t* get_default_gateway(void);
uint8_t* get_dns_server_addr(void);
uint8_t* get_parent_address (void);
void set_iot_gateway_config(struct ip64_dhcpc_state *s);
#endif //__WSN_UI_HANDLER_H__