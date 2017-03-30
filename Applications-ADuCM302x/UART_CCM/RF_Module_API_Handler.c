/***********************************************************************************************************************

Copyright(c) 2016 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.

***********************************************************************************************************************/

/**
 * \file
 * 		Main implementation file for RF Module API handler
 * \author
 *    Analog Devices
 */
/******************************************************************************/

#include "common.h"
#include "hif_utility.h"
#include "uart_handler.h"
#include "contiki-net.h"
#include "net/rpl/rpl.h"
#include "RF_Module_API_Handler.h"
#include "phy.h"

#define UDP_SERVER_PORT	1035
#define MCAST_SINK_UDP_PORT 3001 /* Host byte order */
   
#define LED3  0   //used in main.c
#define LED4  1  //used in UDP Client.c and UDP Server.c  
#define LED5  2  //Not used
#define LED1  3

typedef enum 
{ 
  TURN_OFF = 0x00,
  TURN_ON	
} LED_OnOffState;

typedef enum RFModuleFrameCommands_tag {
  RFMODULE_SET_PARAMS_REQ,
  RFMODULE_SET_PARAMS_CONF,

  RFMODULE_SET_DEFAULT_CONFIG_REQ,
  RFMODULE_SET_DEFAULT_CONFIG_CONF,

  RFMODULE_GET_PARAMS_REQ,
  RFMODULE_GET_PARAMS_CONF,

  RFMODULE_SCAN_NETWORK_REQ,
  RFMODULE_SCAN_NETWORK_CONF,

  RFMODULE_JOIN_NETWORK_REQ,
  RFMODULE_JOIN_NETWORK_CONF,

  RFMODULE_DISCONNECT_FROM_NETWORK_REQ,
  RFMODULE_DISCONNECT_FROM_NETWORK_CONF,

  RFMODULE_GOTO_SLEEP_REQ,
  RFMODULE_GOTO_SLEEP_CONF,

  RFMODULE_DEVICE_RESET_REQ,
  RFMODULE_DEVICE_RESET_CONF,

  /* Open a socket and return socket id*/
  RFMODULE_OPEN_SOCKET_REQ,
  RFMODULE_OPEN_SOCKET_CONF,

  RFMODULE_CLOSE_SOCKET_REQ,
  RFMODULE_CLOSE_SOCKET_CONF,

  RFMODULE_BIND_SOCKET_REQ,
  RFMODULE_BIND_SOCKET_CONF,

  RFMODULE_SEND_TO_DEFAULT_REQ,
  RFMODULE_SEND_TO_DEFAULT_CONF,

  /*Req can contain: socket id, destination ipv6 addr, destination port,
  packet_id(unique identifier to get confirmation), payload based on packet id.
  If packet id is 0xffff, then no need to send packet_tx_done, socket id,
  source ip addr, source port, payload*/
  RFMODULE_SEND_TO_REQ,
  RFMODULE_PACKET_TX_DONE_CONF,

  RFMODULE_RCVD_PACKET_INDICATION,

  RFMODULE_NETWORK_EVENTS_INDICATION,

}RFModuleFrameCommands_t;

typedef enum RFModuleGetSetParamsTypes_tag {
  /* ---- SET or GET Parameters --- */
  RFMODULE_PARAM_MAC_ADDRESS,
  RFMODULE_PARAM_NWK_KEY,
  RFMODULE_PARAM_NETWORK_ID,
  RFMODULE_PARAM_TXPOWER,
  /*start network on power up //persistent*/
  RFMODULE_PARAM_AUTO_JOIN,
  RFMODULE_PARAM_PANID,
  RFMODULE_PARAM_ORBIT_NUMBER,
  RFMODULE_PARAM_PREFIX_INFO,
  RFMODULE_PARAM_ISM_BAND,
  RFMODULE_PARAM_PHY,
  RFMODULE_SLEEPY_OPTION,

  /* ---- GET Parameters --- */
  RFMODULE_IPV6_ADDRESS,
  /*If no short address then return 0*/
  RFMODULE_SHORT_ADDRESS,
  /*Hardware version, network stack version*/
  RFMODULE_STATIC_INFO,
  /* whether joined? number of parents */
  RFMODULE_NODE_CURRENT_STATUS,
  /* network ID */
  RFMODULE_NETWORK_INFO,

  /* TBD */
  /* Routing info */
  RFMODULE_ROUTING_INFO,
  /* time since start of network, UTC */
  RFMODULE_TIME,
}RFModuleGetSetParamsTypes_t;

typedef enum RFModuleConfirmStates_tag {
  RFMODULE_SUCCESS = 0X00,
  RFMODULE_DEFAULT_ROUTE_REMOVED,
  RFMODULE_PROCESS_ALREADY_STARTED,
  RFMODULE_INSUFICIENT_DATA = 0xF0,
  RFMODULE_INVALID_DATA,
  RFMODULE_INVALID_NODE_TYPE,
}RFModuleConfirmStates_t;

typedef enum RFModuleNetworkStates_tag {
  RFMODULE_DEFAULT_ROUTE_ADDED = 0X00,
}RFModuleNetworkStates_t;

extern ADI_UART_HANDLE hUartDevice;
extern uint8_t RxBuffer[];
extern unsigned char 	TxBuffer[];
extern NodeType_t g_node_type;
PROCESS_NAME(border_router_process);
PROCESS_NAME(udp_server_process);
extern ANODECONFIGSTATE *mConfigParams;
extern uint8_t mConfigFlashMemory[];
extern uint8_t g_rcvd_cmd_frame_interface_type;
extern uint16_t mac_pan_id;
extern uint16_t mac_dst_pan_id;
extern uint16_t mac_src_pan_id;
extern uip_802154_longaddr ieee_802154_extended_addr;
extern uint16_t pan_id_configured;
extern RPL_MOP_Type_t g_RPL_MOP_type;

/******************************************************************************/
static uint8_t send_confirmation = 0;
static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *sink_conn;
uint8_t udp_connected;
uint8_t udp_socket_opened = 0;

/********************* Static functions declarations **************************/
static uint8_t GetCmdId(uint8_t *rx_buff);
uint8_t RF_Module_FrameTx(uint16_t DataLen);
static void  router_callback(int event, uip_ipaddr_t *route, uip_ipaddr_t *ipaddr,
                                                                  int numroutes);
static uint16_t GetCmdLen(uint8_t *rx_buff);
static uint8_t IsNodeTypeValid(uint8_t node_type);
static uint8_t IsChannelValid(uint8_t channel_num);
static int8_t ValidateJoinNetworkReqCmd(uint8_t *rx_buff);
static void UpdateJoinParams(uint8_t node_type);
static void OpenDefaultUDPSocket(void);
static void send_udp_packet(uint8_t *buff, uint16_t buff_len);
static uint8_t send_udp_packet_to(uint8_t socket_id, uint8_t *dest_ip,
                                  uint16_t dest_port, uint8_t packet_id,
                                  uint8_t *payload, uint16_t payload_len);

/********************* Public function definitions ****************************/

/*!
 * @brief  This function handles RF Module commands.
 *
 * @param[in]   rx_buff    Pointer to the RF Module command.
 *
 * @returns  None
 */
void RF_Module_API_Handler(uint8_t *rx_buff)
{
  uint8_t command_id;
  static struct uip_ds6_notification n;
  uint16_t command_len = 0;

  command_id = GetCmdId(rx_buff);
  command_len = GetCmdLen(rx_buff);
  switch(command_id){
  case RFMODULE_JOIN_NETWORK_REQ:
    if( ValidateJoinNetworkReqCmd(rx_buff) != RFMODULE_SUCCESS ) {
      goto API_processed;
    }
    g_node_type = (NodeType_t)rx_buff[RFMODULE_FRAME_CMD_POS + 1];
    /* Start tcpip process. Which intern starts network joining */
    if(!process_is_running(&tcpip_process)){
      /* Update parameters into Flash*/
      if(g_node_type == NODE_6LN){
        mConfigParams->NodeFunct = NF_6LN;
      }
      else if(g_node_type == NODE_6LR){
        mConfigParams->NodeFunct = NF_6LR;
      }
      else if(g_node_type == NODE_6LBR){
        mConfigParams->NodeFunct = NF_6LBR;
      }
      mConfigParams->DWChanNo = rx_buff[RFMODULE_FRAME_CMD_POS + 2];
      mConfigParams->Orbit = rx_buff[RFMODULE_FRAME_CMD_POS + 3];
      UpdateJoinParams(g_node_type);

      send_confirmation = 1;
      process_start(&tcpip_process, NULL);
      uip_ds6_notification_add(&n, router_callback);
      if(g_node_type == NODE_6LN){
        rpl_set_mode(RPL_MODE_LEAF);
      }
#if !RF_MODULE_STAR_TOPOLOGY
      if((g_node_type == NODE_6LBR) && (!process_is_running(&border_router_process))){
        process_start (&border_router_process, NULL);
      }
#endif
      if ( (g_node_type == NODE_6LBR) && (!process_is_running(&udp_server_process))){
        process_start(&udp_server_process, NULL);
      }
    }
    else {
      /* Already started the process, hence give confirmation. */
      TxBuffer[RFMODULE_FRAME_CMD_POS] = RFMODULE_JOIN_NETWORK_CONF;
      TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = RFMODULE_PROCESS_ALREADY_STARTED;
      RF_Module_FrameTx(2);
    }
    break;

  case RFMODULE_DEVICE_RESET_REQ:
    /* Wait till the current transmission completes. */
    while(uart_tx_is_in_progress());
    ADI_DISABLE_INT(UART_EVT_IRQn);
    /* Create Reset confirmation frame.*/
    TxBuffer[RFMODULE_FRAME_CMD_POS] = RFMODULE_DEVICE_RESET_CONF;
    TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = RFMODULE_SUCCESS;
    TxBuffer[0] = SOF;
    TxBuffer[1] = SOF_1;
    TxBuffer[2] = 0x02;
    TxBuffer[3] = 0x00;
    TxBuffer[2 + RFMODULE_FRAME_HEADER_LENGTH + RFMODULE_FRAME_CRC_LENGTH - 1] = Generate_Checksum(&TxBuffer[4], 2 );
    /* Transmit Reset confirmation*/
    UARTTx(TxBuffer, 2 + RFMODULE_FRAME_HEADER_LENGTH + RFMODULE_FRAME_CRC_LENGTH );
    /* Reset the system */
    NVIC_SystemReset();
    break;

  case RFMODULE_SEND_TO_DEFAULT_REQ:
    send_udp_packet(&rx_buff[RFMODULE_FRAME_CMD_POS + 1], command_len - 1);
    break;

  case RFMODULE_SEND_TO_REQ:
    uint16_t dest_port = ((0x00FF & (uint16_t)rx_buff[RFMODULE_FRAME_CMD_POS + 19]) << 8) |
            (0x00FF & (uint16_t)rx_buff[RFMODULE_FRAME_CMD_POS + 18]);

    send_udp_packet_to(rx_buff[RFMODULE_FRAME_CMD_POS + 1],  //socket id
                       &rx_buff[RFMODULE_FRAME_CMD_POS + 2], //ip addr
                       dest_port, //dest_port
                       rx_buff[RFMODULE_FRAME_CMD_POS + 20], //packet_id
                       &rx_buff[RFMODULE_FRAME_CMD_POS + 21], //payload
                       command_len - 1 - 1 -16 - 2 - 1); //payload_len
    break;

  case RFMODULE_SET_PARAMS_REQ:
    {
      uint8_t rx_buff_pos = RFMODULE_FRAME_CMD_POS + 1;
      uint16_t payload_len = command_len - 1;

      TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = RFMODULE_SUCCESS;
      do{
      switch(rx_buff[rx_buff_pos]){

      case RFMODULE_PARAM_MAC_ADDRESS:
        memcpy(mConfigParams->AdrAry, &rx_buff[rx_buff_pos + 1], 8);
        rx_buff_pos += 9;
        payload_len = payload_len - 8 - 1;
        memcpy(&uip_lladdr.addr, mConfigParams->AdrAry, sizeof (uip_lladdr));
        linkaddr_set_node_addr((linkaddr_t *)mConfigParams->AdrAry);
        memcpy(&ieee_802154_extended_addr, &mConfigParams->AdrAry, sizeof(uip_802154_longaddr));
        phy_set_ExtendedAddress(mConfigParams->AdrAry);
        break;

      case RFMODULE_PARAM_PANID:
        mConfigParams->PANID[1] = rx_buff[rx_buff_pos + 1];
        mConfigParams->PANID[0] = rx_buff[rx_buff_pos + 2];
        rx_buff_pos += 3;
        payload_len = payload_len - 2 - 1;
        memcpy(&mac_pan_id, &mConfigParams->PANID , sizeof(mac_pan_id));
        memcpy(&mac_src_pan_id, &mConfigParams->PANID , sizeof(mac_src_pan_id));
        memcpy(&mac_dst_pan_id, &mConfigParams->PANID , sizeof(mac_dst_pan_id));
        memcpy(&pan_id_configured, &mConfigParams->PANID, sizeof(pan_id_configured));
        phy_set_panId(((mConfigParams->PANID[1] << 8) | mConfigParams->PANID[0]));
        break;

      case RFMODULE_PARAM_ISM_BAND:
        mConfigParams->ISMBand = (ISMBAND)rx_buff[rx_buff_pos + 1];
        rx_buff_pos += 2;
        payload_len = payload_len - 1 - 1;
        break;

      case RFMODULE_PARAM_PHY:
        if((mConfigParams->ISMBand == ISMBAND_2400MHZ) && (mConfigParams->DWChanPHY > 0)){
          TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = RFMODULE_INVALID_DATA;
          payload_len = 0;
        }
        else
        {
          mConfigParams->DWChanPHY = rx_buff[rx_buff_pos + 1];
          rx_buff_pos += 2;
          payload_len = payload_len - 1 - 1;
        }
        break;

      case RFMODULE_PARAM_TXPOWER:
        mConfigParams->MaxTxPwr = rx_buff[rx_buff_pos + 1];
        rx_buff_pos += 2;
        payload_len = payload_len - 1 - 1;
        break;

      case RFMODULE_PARAM_NWK_KEY:
        if(rx_buff[rx_buff_pos + 1]){
          mConfigParams->MainOpts |= 0x40;
        }
        else {
          mConfigParams->MainOpts &= ~0x40;
        }
        memcpy(mConfigParams->EncryptionWord, &rx_buff[rx_buff_pos + 2], 16);
        rx_buff_pos += 18;
        payload_len = payload_len - 17 - 1;

        break;

      default:
        payload_len = 0;
        TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = RFMODULE_INVALID_DATA;
        break;
      }

      }while(payload_len > 0);

      SaveConfigtoFlash();
      /* give confirmation. */
      TxBuffer[RFMODULE_FRAME_CMD_POS] = RFMODULE_SET_PARAMS_CONF;
      RF_Module_FrameTx(2);
    }
    break;

  case RFMODULE_SET_DEFAULT_CONFIG_REQ:
    ANode_SetConfig(CONFIG_NOT_OKAY); 	     // Configuration no longer valid
    FlashErase(MEM_LOC_AD6L_PARAMS, 1);      // Erase the configuration
    /* Wait till the current transmission completes. */
    while(uart_tx_is_in_progress());
    ADI_DISABLE_INT(UART_EVT_IRQn);
    /* Create default_config confirmation frame.*/
    TxBuffer[RFMODULE_FRAME_CMD_POS] = RFMODULE_SET_DEFAULT_CONFIG_CONF;
    TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = RFMODULE_SUCCESS;
    TxBuffer[0] = SOF;
    TxBuffer[1] = SOF_1;
    TxBuffer[2] = 0x02;
    TxBuffer[3] = 0x00;
    TxBuffer[2 + RFMODULE_FRAME_HEADER_LENGTH + RFMODULE_FRAME_CRC_LENGTH - 1] = Generate_Checksum(&TxBuffer[4], 2 );
    /* Transmit default_config confirmation*/
    UARTTx(TxBuffer, 2 + RFMODULE_FRAME_HEADER_LENGTH + RFMODULE_FRAME_CRC_LENGTH );
    /* Reset the system */
    NVIC_SystemReset();
    break;

  default:
    break;
  }
API_processed:
  /* After processing the received command, start reception of next command. */
  hif.rx_state = RX_INIT;

    /* Start trnamission if there is a frame to transmit*/
  if (tx_data_pending) {
      cmd_hal_write();
      tx_data_pending = 0;
  }

    adi_cmd_SubmitRxBuffer(g_rcvd_cmd_frame_interface_type, &RxBuffer[0], 1);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function starts client process and sends network joining status.
 *
 * @param[in]   status    Network joining status.
 *
 * @returns  None
 */
void RF_Module_Join_Callback(uint8_t status)
{
  if((send_confirmation == 1)){
    send_confirmation = 0;
    LEDControl(LED4, TURN_ON);
    TxBuffer[RFMODULE_FRAME_CMD_POS] = RFMODULE_JOIN_NETWORK_CONF;
    TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = status;
    RF_Module_FrameTx(2);
  }
//  else {
//    TxBuffer[RFMODULE_FRAME_CMD_POS] = RFMODULE_NETWORK_EVENTS_INDICATION;
//    TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = status;
//    RF_Module_FrameTx(2);
//  }
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function fills RF Module frame header and transmits the frame.
 *  For this function the payload should be available in an array TxBuffer.
 *
 * @param[in]   DataLen         Length of the the transmitting frame.
 *
 * @returns  0: If able to accept for trnamission.
 *           1: If buffer size is less than frame length.
 */
uint8_t RF_Module_FrameTx(uint16_t DataLen)
{
  static uint16_t tx_len;
  if(DataLen >
 (UART_BUFFER_SIZE - RFMODULE_FRAME_HEADER_LENGTH - RFMODULE_FRAME_CRC_LENGTH)){
    return 1;
  }
  tx_len = DataLen + RFMODULE_FRAME_HEADER_LENGTH + RFMODULE_FRAME_CRC_LENGTH;
  TxBuffer[0] = SOF;
  TxBuffer[1] = SOF_1;
  TxBuffer[2] = (DataLen & 0xFF);
  TxBuffer[3] = ((DataLen >> 8) & 0xFF);
  TxBuffer[tx_len - 1] = Generate_Checksum(&TxBuffer[4], DataLen );
  if(g_rcvd_cmd_frame_interface_type == UART_CB){
  send_uart_frame(TxBuffer, tx_len);
  }
  return 0;
}

void rpl_join_indication(uint8_t status)
{
}

/********************* Static functions definitions ***************************/

/*!
 * @brief  This function returns Command ID in the received command frame..
 *
 * @param[in]   rx_buff    Pointer to the received frame.
 *
 * @returns  command_ID
 */
static uint8_t GetCmdId(uint8_t *rx_buff)
{
  return rx_buff[RFMODULE_FRAME_CMD_POS];
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function returns frmae command length.
 *
 * @param[in]   rrx_buff    Pointer to the received frame.
 *
 * @returns  frmae command length.
 */
static uint16_t GetCmdLen(uint8_t *rx_buff)
{
  return (((0x00FF & (uint16_t)rx_buff[3]) << 8) |
            (0x00FF & (uint16_t)rx_buff[2]));

}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function validates received node_type.
 *
 * @param[in]   node_type       Value need to check for node type.
 *
 * @returns  1: If value is valid node type.
 *           0: If value is invalid node type.
 */
static uint8_t IsNodeTypeValid(uint8_t node_type)
{
  return (((node_type == NODE_6LN) || (node_type == NODE_6LR) ||
           (node_type == NODE_6LBR)) ? 1: 0);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function validates received channel number for corresponding
 *         ISMBand and PHY.
 *
 * @param[in]   node_type       Value need to check for channel number .
 *
 * @returns  1: If value is valid channel number.
 *           0: If value is invalid channel number.
 */
static uint8_t IsChannelValid(uint8_t channel_num)
{
  if((mConfigParams->ISMBand == ISMBAND_2400MHZ) &&
     ((channel_num >= 11) && (channel_num <= 26))){
    return 1;
  }
  if(mConfigParams->ISMBand == ISMBAND_863_870MHZ){
    if((mConfigParams->DWChanPHY == 0) &&
      (channel_num <= 33)) return 1;
    if((mConfigParams->DWChanPHY == 1) &&
      (channel_num <= 16)) return 1;
  }
  if(mConfigParams->ISMBand == ISMBAND_901_902MHZ){
    if((mConfigParams->DWChanPHY == 0) &&
       (channel_num <= 78)) return 1;
    if((mConfigParams->DWChanPHY == 1) &&
       (channel_num <= 76)) return 1;
    if((mConfigParams->DWChanPHY == 2) &&
       (channel_num <= 72)) return 1;
  }
  if((mConfigParams->ISMBand == ISMBAND_902_928MHZ) &&
     (channel_num <= 128)) return 1;
  if((mConfigParams->ISMBand == ISMBAND_917_923MHZ) &&
    (channel_num <= 31)) return 1;
  if(mConfigParams->ISMBand == ISMBAND_920_928MHZ){
    if((mConfigParams->DWChanPHY == 0) &&
       (channel_num <= 37)) return 1;
    if((mConfigParams->DWChanPHY == 1) &&
       (channel_num <= 17)) return 1;
  }
  if(mConfigParams->ISMBand == ISMBAND_950_958MHZ){
    if((mConfigParams->DWChanPHY == 0) &&
       (channel_num <= 32)) return 1;
    if((mConfigParams->DWChanPHY == 1) &&
       (channel_num <= 15)) return 1;
  }
  if((mConfigParams->ISMBand == ISMBAND_896_901MHZ) ||
     (mConfigParams->ISMBand == ISMBAND_928_960MHZ)) return 1;
  return 0;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function validates the received Join_Network command and sends
 * confirmation if the command parameters are invalid.
 *
 * @param[in]   rx_buff         pointer to the received frame command.
 *
 * @returns  0: If the command parameters are valid.
 *           -1:If the command parameters are invalid.
 */
static int8_t ValidateJoinNetworkReqCmd(uint8_t *rx_buff)
{
  uint16_t command_len = GetCmdLen(rx_buff);
  if( command_len < 4){
    TxBuffer[RFMODULE_FRAME_CMD_POS] = RFMODULE_JOIN_NETWORK_CONF;
    TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = RFMODULE_INSUFICIENT_DATA;
    RF_Module_FrameTx(2);
    return -1;
  }

  if(!IsNodeTypeValid(rx_buff[RFMODULE_FRAME_CMD_POS + 1])) {
    TxBuffer[RFMODULE_FRAME_CMD_POS] = RFMODULE_JOIN_NETWORK_CONF;
    TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = RFMODULE_INVALID_NODE_TYPE;
    RF_Module_FrameTx(2);
    return -1;
  }

  if(!IsChannelValid(rx_buff[RFMODULE_FRAME_CMD_POS + 2])){
    TxBuffer[RFMODULE_FRAME_CMD_POS] = RFMODULE_JOIN_NETWORK_CONF;
    TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = RFMODULE_INVALID_DATA;
    RF_Module_FrameTx(2);
    return -1;
  }

    /* Validate RPL mode */
  if(command_len >= 5) { 
    if(rx_buff[RFMODULE_FRAME_CMD_POS + 4] > RPL_MOP_TYPE_STORING_MULTICAST) {
      TxBuffer[RFMODULE_FRAME_CMD_POS] = RFMODULE_JOIN_NETWORK_CONF;
      TxBuffer[RFMODULE_FRAME_CMD_POS + 1] = RFMODULE_INVALID_DATA;
      RF_Module_FrameTx(2);
      return -1;
    }
    else {
      g_RPL_MOP_type = (RPL_MOP_Type_t)rx_buff[RFMODULE_FRAME_CMD_POS + 4];
    }
  }
  
  return RFMODULE_SUCCESS;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Callback function for network states.
 *
 * @param[in]   rcvd_frame_status_t    Status of received UART frame.
 *
 * @returns  None
 */
static void  router_callback(int event, uip_ipaddr_t *route, uip_ipaddr_t *ipaddr,
                                                                  int numroutes)
{
  if( event == UIP_DS6_NOTIFICATION_DEFRT_ADD ){
    udp_connected = 1;
    RF_Module_Join_Callback(RFMODULE_SUCCESS);
  }
  if(event == UIP_DS6_NOTIFICATION_DEFRT_RM) {
    udp_connected = 0;
    RF_Module_Join_Callback(RFMODULE_DEFAULT_ROUTE_REMOVED);
  }
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function updates node type, channel and orbit into flash.
 *
 * @param[in]   node_type    Node type needed to update in flash.
 *
 * @returns  None
 */
static void UpdateJoinParams(uint8_t node_type) {
  unsigned char* pDest = (unsigned char*)mConfigParams;
  int n;

  mConfigParams->CheckSum = 0;
  for(n=0; n<(sizeof(ANODECONFIGSTATE) - 4); n++){
    mConfigParams->CheckSum += *(pDest + n);
  }

  FlashErase(MEM_LOC_CONFIG_PARAMS_IN_FLASH, SIZE_OF_CONFIG_PARAMES);
  FlashWrite(MEM_LOC_CONFIG_PARAMS_IN_FLASH, SIZE_OF_CONFIG_PARAMES,
               (long*)mConfigFlashMemory);

  config_init(false);
  update_phy_params();
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function creates UDP socket for default port
 *
 * @params   None
 *
 * @returns  None
 */
static void OpenDefaultUDPSocket(void)
{
  uip_ipaddr_t addr;

  udp_socket_opened = 1;

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(UDP_SERVER_PORT));

  /*
  * IPHC will use stateless multicast compression for this destination
  * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
  */
  uip_ip6addr(&addr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  uip_ds6_maddr_add(&addr);

  sink_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(sink_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function sends given UDP payload through default UDP connection.
 *
 * @param[in]   *buff    Pointer to UDP payload.
 *
 * @param[in]   buff_len        Length of UDP payload.
 *
 * @returns  None
 */
static void send_udp_packet(uint8_t *buff, uint16_t buff_len)
{
  static int LocFailCount = 0;
  rpl_dag_t* currentDag;

  if(!udp_socket_opened){
    OpenDefaultUDPSocket();
  }


  if(!udp_connected)
    return;

//  if(Node_IsSleepy())
//  {
//    if(!Get_Ack_Status())
//    {
//      LocFailCount++;
//      if(LocFailCount >= 4)
//      {
//        if(NULL != rpl_get_default_instance())
//          rpl_free_instance(rpl_get_default_instance());
//        LocFailCount = 0;
//        udp_connected = 0;
//        return;
//      }
//    }
//    else
//    {
//      LocFailCount = 0;
//    }
//  }
  //LEDControl(LED4, TURN_ON);

  currentDag  = rpl_get_any_dag();
  if(currentDag != NULL){
    uip_udp_packet_sendto(client_conn, buff, buff_len,
                          &currentDag->dag_id, UIP_HTONS(UDP_SERVER_PORT));
  }
  //LEDControl(LED4, TURN_OFF);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function sends UDP packet through specified UDP connection.
 *
 * @param[in]   socket_id    Specifies the UDP socket ID.
 *
 * @param[in]   *dest_ip     Pointer to destination IPv6 address.
 *
 * @param[in]   dest_port    UDP packet destination port.
 *
 * @param[in]   packet_id    Sequence number.
 *
 * @param[in]   *payload     UDP payload.
 *
 * @param[in]   payload_len    UDP payload length.
 *
 * @returns  0  if it is able to transmit UDP packet.
 *           1  if it is not able to transmit.
 */
static uint8_t send_udp_packet_to(uint8_t socket_id, uint8_t *dest_ip,
                                  uint16_t dest_port, uint8_t packet_id,
                                  uint8_t *payload, uint16_t payload_len)
{
#if !RF_MODULE_STAR_TOPOLOGY
  if(!udp_connected)
    return 1;
#endif //RF_MODULE_STAR_TOPOLOGY

  if((socket_id == 0xFF) && (!udp_socket_opened)){
    OpenDefaultUDPSocket();
  }
  // else If socket not opened then create new socket or send failure confirmation

  /* Currently used the existed udp connection */
  uip_udp_packet_sendto(client_conn, payload, payload_len, (const uip_ipaddr_t *)dest_ip, dest_port);
  return 0;
}