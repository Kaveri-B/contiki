/***********************************************************************************************************************

Copyright (c) 2013 - 2015, Analog Devices, Inc.  All rights reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.

***********************************************************************************************************************/
/*---------------------------------------------------------------------------*/
/**
* \file
*					Machine dependent ADucM3025 radio driver code.
* \author
*					Analog Devices
*/
/*---------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <gpio\adi_gpio.h>
#include <uart\adi_uart.h>
//#include "uhf.h"
#include "utest_support.h"

#include "contiki.h"
#include "dev/radio.h"

#include "dev/aducm3029-radio.h"
#include "net/netstack.h"

#include "net/packetbuf.h"

#include "contiki-conf.h"

#include "ADF7023.h"

#include "RF_Module_API_Handler.h"

#ifdef INTERACTIVE_SETTINGS
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

//#define DEBUG

#ifdef DEBUG
#define PRINTF_DEBUG(...)	printf(__VA_ARGS__)
#else
#define PRINTF_DEBUG(...)
#endif

#define CLEAN_RXBUF() (aducm3025_rxbuf[0] = 0)
#define RXBUF_EMPTY() (aducm3025_rxbuf[0] == 0)

#define CLEAN_TXBUF() (aducm3025_txbuf[0] = 0)
#define TXBUF_EMPTY() (aducm3025_txbuf[0] == 0)

/*----------------------------- Local Defines ---------------------------------*/

/* The transceiver state */
#define ON     0
#define OFF    1
#define MAC_MAX_ACK_RETRY       1

/* Toggle the macro below to turn various UHF interrupts on and off */
#define ADI_ADF7023_IMASK_CON \
INT_TX_EOF     |\
  INT_CRC_CORRECT
    
#ifndef DISABLE_PATCH_802_15_4D
    
#define PHR_FCS_LEN_BIT_RESET				0x00
#define PHR_FCS_LEN_BIT_SET				0x10
#define PHR_DATA_WHITENING_ENABLED			0x08
#define PHR_DATA_WHITENING_DISABLED			0x00
    
#else
    
#define PHR_FCS_LEN_BIT_RESET				0x0000
#define PHR_FCS_LEN_BIT_SET				0x0008
#define PHR_DATA_WHITENING_ENABLED			0x0010
#define PHR_DATA_WHITENING_DISABLED			0x0000
    
#endif // DISABLE_PATCH_802_15_4D
    
#ifdef FOREN6_FEEDER
    static const char Foren6Header[5] = {0xC1, 0x1F, 0xFE, 0x72, 0x01};
#endif    
/*----------------------------- Local Function Protos ---------------------------------*/

static void GetRadioSettings(ADI_ADF7023_GENERIC_SETTINGS_TYPE* settings);

/* Radio driver functions exported through Radio Driver Struct */
static int aducm3025_radio_init(void);
static int aducm3025_radio_prepare(const void *payload, unsigned short payload_len);
static int aducm3025_radio_transmit(unsigned short payload_len);
static int aducm3025_radio_send(const void *data, unsigned short len);
static int aducm3025_radio_read(void *buf, unsigned short bufsize);
static int aducm3025_radio_channel_clear(void);
static int aducm3025_radio_receiving_packet(void);
static int aducm3025_radio_pending_packet(void);
static int aducm3025_radio_on(void);
static int aducm3025_radio_off(void);

void Set_Ack_Done(uint8_t ack_done);
void Set_Rx_Seq_No(uint8_t seq_no);
void Set_Tx_Seq_No(uint8_t seq_no);
void Set_Ack_Required(uint8_t ack_req);

#ifdef DEBUG
static void PrintRadioSettings(ADI_ADF7023_DEV_HANDLE hUhf);
#endif // DEBUG

unsigned char adi_ADF7023_AsyncRx( void );

static uint16_t create_Normal_MR_FSK_PHR
(
bool scramble_psdu,
bool crc_16,
uint16_t* p_psdu_len
);

uint8_t ACK[] = {
  0x00,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x00
};

typedef struct TxStateMachine {
  uint8_t ACK_RX_Done;
  uint8_t Tx_Seq_No;
  uint8_t Rx_Seq_No;
  uint8_t Ack_Req;
}TxStateMachine_t;

volatile TxStateMachine_t TxState;

uint8_t bAckTxProgress = 0;

static int16_t rcvd_pkt_len;
unsigned char adi_ADF7023_SetPhyRx( void );
uint8_t transmit_pwr = TRANSMIT_POWER;
uint16_t Ack_Missed = 0;

extern rf_statistics_t rf_stats;
/*---------------------------------------------------------------------------*/
PROCESS(aducm3025_radio_process, "ADucM3025 radio driver");
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

static volatile uint8_t onoroff = OFF;
uint8_t DBG_Phy_State = 0x00;

static uint8_t aducm3025_txbuf[ ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD + \
  SIZE_ADDR_FIELD + SIZE_CRC_FIELD ];

static uint8_t aducm3025_rxbuf[ ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD + \
  SIZE_ADDR_FIELD + SIZE_CRC_FIELD ];

const struct radio_driver aducm3025_radio_driver =
{
  aducm3025_radio_init,
  aducm3025_radio_prepare,
  aducm3025_radio_transmit,
  aducm3025_radio_send,
  aducm3025_radio_read,
  aducm3025_radio_channel_clear,
  aducm3025_radio_receiving_packet,
  aducm3025_radio_pending_packet,
  aducm3025_radio_on,
  aducm3025_radio_off,
};

extern ADI_UART_HANDLE hUartDevice;

extern NodeType_t g_node_type;

/*---------------------------------------------------------------------------*/
static int
aducm3025_radio_init(void)
{
  
  static const ADI_ADF7023_GENERIC_SETTINGS_TYPE ADF7023Configs = { 
    /* Profile Number */
    PROFILE_NUM,
    /* ISM Band */
    ISMBAND_915MHZ,
    /* Frequency */
    901000000,
    /* Initialization Params*/
    INIT_TRSC_PORTS
  };
  
  /* Initialize ADF7023 radio. Pass the parameters which are obtained by GetRadioSettings.*/
  if ( ADF_RadioInit( ADF7023Configs.ProfileNo, ADF7023Configs.ISMBand,
                     ADF7023Configs.Freq, ADF7023Configs.InitParm) != TRE_NO_ERR )
    return TRE_DEBUG;
  
  /* Reduce the UHF interrupt priority */
  //  NVIC_SetPriority (SYS_GPIO_INTA_IRQn, 2);
  
  CLEAN_RXBUF();
  CLEAN_TXBUF();
  
  process_start(&aducm3025_radio_process, NULL);
  
  return 0;
}

/*---------------------------------------------------------------------------*/
int
radio_set_channel(unsigned long channelFrequency)
{
  return (ADF_ChangeChans( channelFrequency )); 
}
/*---------------------------------------------------------------------------*/

static int
aducm3025_radio_prepare(const void *payload, unsigned short payload_len)
{
  if (payload_len > (ADUCM3025_MAX_PACKET_LEN -
                     ( SIZE_LEN_FIELD + SIZE_CRC_FIELD))){
                       utu_timestamp(ADUCM3025_PKT_DROP_LEN_ISSUE,0);
                       return RADIO_TX_ERR;
                     }
  
#ifdef DISABLE_PATCH_802_15_4D  
  aducm3025_txbuf[0] = payload_len + SIZE_LEN_FIELD + SIZE_CRC_FIELD ;
  
  /* Copy to the txbuf */
  memcpy(&aducm3025_txbuf[SIZE_LEN_FIELD], payload, payload_len);
#else
  /* Copy to the txbuf. First two bytes we are reserving to copy the PHR,so that we can eliminate the 
  use of Radio_TRX_Buffer for transmission */
  memcpy(&aducm3025_txbuf[2], payload, payload_len);
  
#endif /* DISABLE_PATCH_802_15_4D */
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
aducm3025_radio_transmit(unsigned short payload_len)
{
  uint8_t retry_counter = 0;
  
  utu_timestamp(ADUCM3025_PKT_TRANSMIT,0);
  while(MAC_MAX_ACK_RETRY > retry_counter)
  {
#ifndef DISABLE_PATCH_802_15_4D  
    uint16_t PHR;
    uint8_t  Thrvalue = 37,TimerValue =4;
    
    PHR = create_Normal_MR_FSK_PHR
      (
       true, // bool scramble_psdu,
       !(I_SUPPORT_4BYTE_FCS), // bool crc_16,
       &payload_len // uint16_t* p_psdu_len
         );
    
#ifdef ENABLE_HARDWARE_CCA
    if(ADF7023_15d4g_Send_Pkt(PHR,aducm3025_txbuf,Thrvalue,TimerValue,true,transmit_pwr) == TRE_NO_ERR)
#else
      if(aducm3025_radio_channel_clear() != TRE_NO_ERR )
      {
        return RADIO_TX_COLLISION;
      }
    
    if(ADF7023_15d4g_Send_Pkt(PHR,aducm3025_txbuf,Thrvalue,TimerValue,false,transmit_pwr) == TRE_NO_ERR)
#endif /* ENABLE_HARDWARE_CCA */
#else
      if( ADF_TransmitPacket( aducm3025_txbuf, transmit_pwr) == TRE_NO_ERR )
#endif // DISABLE_PATCH_802_15_4D
      {
        if(TxState.Ack_Req)
        {
          int timeout = 100000;
          
          while(timeout)
          {
            if((TxState.ACK_RX_Done) && (TxState.Tx_Seq_No == TxState.Rx_Seq_No))
              return RADIO_TX_OK;
            
            timeout--;
          }
        }
        else
        {
          return RADIO_TX_OK;
        }
      }
    retry_counter++;
  }
  
  Ack_Missed++;
  
  return RADIO_TX_NOACK;
}
/*---------------------------------------------------------------------------*/
static int
aducm3025_radio_send(const void *payload, unsigned short payload_len)
{
  bAckTxProgress = 0;
  if(aducm3025_radio_prepare(payload, payload_len) == RADIO_TX_ERR){
    return RADIO_TX_ERR;
  }
  
#ifndef DISABLE_PATCH_802_15_4D  
  /* Increment payload_len to accomodate crc */
  payload_len += SIZE_CRC_FIELD;
  return aducm3025_radio_transmit(payload_len);
#else
  /* For ADF7023: No need to pass Payload Length for aducm3025_radio_transmit */
  if ( aducm3025_radio_transmit(payload_len + SIZE_LEN_FIELD) == RADIO_TX_OK )
  {  
    return RADIO_TX_OK;
  } else {
    return RADIO_TX_ERR;
  }
#endif /* DISABLE_PATCH_802_15_4D */
}

/*---------------------------------------------------------------------------*/
static int
aducm3025_radio_channel_clear(void)
{
  //  uint8_t Rssi;
  
  //  if( GetPHYON_RSSI( &Rssi ) == TRE_NO_ERR ) {   
  //    return (Rssi < 0x31); /* 0x31 is RSSI threshold set in uhf.c */
  if(ADF7023_ReadAntennaRSSI (&rf_stats.RSSI_val) == TRE_NO_ERR ) {   
    return ((rf_stats.RSSI_val < -90)?TRE_NO_ERR:TRE_DEBUG);
  }
  
  return TRE_DEBUG;
}
/*---------------------------------------------------------------------------*/
static int
aducm3025_radio_receiving_packet(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
aducm3025_radio_pending_packet(void)
{
  /* ADucM3025 Transmit API is synchronous */
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
aducm3025_radio_off(void)
{
  return ADF_GoToOffState( );
}
/*---------------------------------------------------------------------------*/
static int
aducm3025_radio_on(void)
{
#ifdef DISABLE_PATCH_802_15_4D
  return adi_ADF7023_SetPhyRx( );
#else
  DBG_Phy_State = 0x03;
  return adi_ADF7023_Receive_15d4_Frame( );
#endif // DISABLE_PATCH_802_15_4D
}

/*---------------------------------------------------------------------------*/

static int
aducm3025_radio_read(void *buf, unsigned short bufsize)
{
  if( RXBUF_EMPTY() || (aducm3025_rxbuf[0] > bufsize)){
    /*Nothing to read */ /* buffer too small */
    return 0;
  }
  
#ifdef DISABLE_PATCH_802_15_4D
  uint8_t len = aducm3025_rxbuf[0] - SIZE_LEN_FIELD - SIZE_CRC_FIELD;
#else
  uint8_t len = aducm3025_rxbuf[0] - SIZE_CRC_FIELD;
#endif // DISABLE_PATCH_802_15_4D  
  
  memcpy(buf, aducm3025_rxbuf+1, len);
  //  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rf_stats.RSSI_val);
  
  return len;
}

/*---------------------------------------------------------------------------*/
#ifdef INTERACTIVE_SETTINGS

#define ValidFreq(x)        (((x >= 431.0) && (x <= 464.0)) || ((x >= 862.0) && (x <= 928.0)))
#define ValidDataRate(x)    ((x >= 1.0) && (x <= 300.0))
#define ValidModScheme(x)   ((x >= MOD_2FSK) && (x <= MOD_2GFSK))

#define ASCII_0             '0'
#define ASCII_ESC           27

static char
ECHO(void)
{
  int16_t size_l = 1;
  char c;
  if( adi_UART_BufRx(hUartDevice, &c, &size_l) != ADI_UART_SUCCESS){
    return c;
  }
  if(adi_uart_SubmitTxBuffer(hUartDevice,&c,size_l) != ADI_UART_SUCCESS)
  {  
    return c;
  }
  adi_uart_EnableTx(hUartDevice,true);
  
  return c;
}

static float
GetRadioOperatingFrequency(void)
{
  char rx_l[5] = {0};
  float ftemp = 0.0;
  char rx_char;
  int i = 0;
  
  do {
    i = 0;
    ftemp = 0;
    PRINTF("\nEnter basic settings for the radio. The 7023 will be configured to operate at the closest possible operating point.\n");
    PRINTF("\nEnter frequency in Mhz (Eg - For 905.52 Mhz enter 905.52): ");
    do {
      rx_char = ECHO();
      rx_l[i++] = rx_char;
    } while((uint8_t)rx_char != 0xD); /* check for enter key */
    
    ftemp = atof(rx_l);
    
    if( !ValidFreq(ftemp) ){
      PRINTF("\nPlease enter a valid frequency\nValid ranges are 431 - 464 Mhz or 862 - 928Mhz");
    }
    else {
      return ftemp;
    }
  } while(1);
}

static float
GetRadioOperatingDataRate(void)
{
  char rx_l[5] = {0};
  float ftemp = 0.0;
  char rx_char;
  int i = 0;
  
  do {
    i = 0;
    ftemp = 0;
    PRINTF("\n\nEnter data rate in kbps (Eg - For 38.5 kbps enter 38.5): ");
    do {
      rx_char = ECHO();
      rx_l[i++] = rx_char;
    } while((uint8_t)rx_char != 0xD); /* check for enter key */
    
    ftemp = atof(rx_l);
    
    if( !ValidDataRate(ftemp) ){
      PRINTF("\nInvalid data rate\nSupported rates are between 1 - 300 kbps");
    }
    else {
      return ftemp;
    }
  } while(1);
}

static char
GetRadioOperatingModulationScheme()
{
  char rx_char;
  
  do {
    PRINTF("\n\nEnter modulation scheme (Note - this sets both mod and demod schemes)  - \n 0 - FSK \n 1 - GFSK \nEnter choice: ");
    
    rx_char = ECHO() - ASCII_0;
    if( !ValidModScheme((uint8_t)rx_char) ){
      PRINTF("\n Invalid choice. Try again \n");
    }
    else {
      PRINTF("\n");
      return rx_char;
    }
  } while(1);
}

static void
GetRadioSettings(ADI_ADF7023_GENERIC_SETTINGS_TYPE* settings)
{
  float ftemp = 0.0;
  char rx_char;
  
  ftemp = GetRadioOperatingFrequency();
  settings->Frequency = (uint32_t)(ftemp * 1000000);
  
  ftemp = GetRadioOperatingDataRate();
  settings->DataRate = (uint16_t)(ftemp * 10);
  
  rx_char = GetRadioOperatingModulationScheme();
  settings->ModScheme = (ADI_ADF7023_MOD_TYPE)rx_char;
  settings->DemodScheme = (ADI_ADF7023_DEMOD_TYPE)rx_char;
  
  /* currently the radio address defaults to 0x1FF */
  settings->ShortAddr.ShortAddr = 0x1FF;
  settings->ShortAddr.ShortAddrMask = SHORT_ADDR_MASK;
}

#else

inline static void
GetRadioSettings(ADI_ADF7023_GENERIC_SETTINGS_TYPE* settings)
{
  // memcpy(settings, &ADF7023Configs, sizeof (ADI_ADF7023_GENERIC_SETTINGS_TYPE));
}

#endif // INTERACTIVE_SETTINGS

#ifdef DEBUG

static void
PrintRadioSettings(ADI_ADF7023_DEV_HANDLE hUhf)
{
  ADI_ADF7023_GENERIC_SETTINGS_TYPE settings;
  ADI_ADF7023_RESULT_TYPE result;
  ADI_ADF7023_PHY_STATE_TYPE phy_state;
  
  result = adi_ADF7023_GetGenericSettings(hUhf, &settings);
  
  if(!result){
    result = adi_ADF7023_GetPhyState(hUhf, &phy_state);
  }
  
  if(result != ADI_ADF7023_SUCCESS)	{
    while(1){
      ;   /* hang on failure here, will do for now */
    }
  }
  
  PRINTF("\nCurrent radio settings - \n");
  PRINTF("Frequency - %d Mhz\n", settings.Frequency);
  PRINTF("Data Rate - %d kbps\n", (settings.DataRate)/10);
  
  switch(settings.ModScheme) {
  case MOD_2FSK:
    PRINTF("Modulation and demodulation scheme - 2FSK\n");
    break;
    
  case MOD_2GFSK:
    PRINTF("Modulation and demodulation scheme - GFSK\n");
    break;
    
  case MOD_OOK:
    PRINTF("Modulation and demodulation scheme - OOK\n");
    break;
    
  default:
    PRINTF("Unknown modulation type - %d", settings.ModScheme);
    break;
  }
  
  switch(phy_state) {
  case PHY_OFF:
    PRINTF("Current phy state - Phy OFF\n");
    break;
  case PHY_ON:
    PRINTF("Current phy state - Phy ON\n");
    break;
  case PHY_TX:
    PRINTF("Current phy state - Phy TX\n");
    break;
  case PHY_RX:
    PRINTF("Current phy state - Phy RX\n");
    break;
  default:
    PRINTF("Unknown phy state - %d\n", phy_state);
  }
}

#endif // DEBUG

static uint16_t create_Normal_MR_FSK_PHR
(
bool scramble_psdu,
bool crc_16,
uint16_t* p_psdu_len
)
{
  
  uint16_t phr;
  
#ifndef DISABLE_PATCH_802_15_4D  
  
  /* Set the length in the PHR */
  phr = (((*p_psdu_len & 0x00FF) << 8) | ((*p_psdu_len & 0xFF00) >> 8));
  
#else
  
  phr = *p_psdu_len;
  
  //phr(b15 to b5) = *p_psdu_len(b0 to b10)
  
  phr = (phr & 0x00FF) << 8 | (phr & 0xFF00) >> 8;
  phr = (phr & 0x0F0F) << 4 | (phr & 0xF0F0) >> 4;
  phr = (phr & 0x3333) << 2 | (phr & 0xCCCC) >> 2;
  phr = (phr & 0x5555) << 1 | (phr & 0xAAAA) >> 1;
#endif // DISABLE_PATCH_802_15_4D
  
  /*set the FCS len bit in the PHR*/
  phr |= ( crc_16 )?PHR_FCS_LEN_BIT_SET:PHR_FCS_LEN_BIT_RESET; //0 :32-bit, 1 :16 -bit
  
  /*set the Data whitening bit accordingly */
  phr |= ( scramble_psdu)?PHR_DATA_WHITENING_ENABLED:PHR_DATA_WHITENING_DISABLED;
  
  return phr;
}

unsigned char adi_ADF7023_AsyncRx( void )
{
  switch(mPHYState)
  {
  case CMD_PHY_SLEEP:
    if( ADF_GoToOffState( ) != TRE_NO_ERR )
      break;
    
    /* To go to PHY_RX from OFF state.we have to switch on the radio first */
  case CMD_PHY_OFF:
    if( ADF_GoToOnState( ) != TRE_NO_ERR )
      break;
    
  case CMD_PHY_TX:  
  case CMD_PHY_ON:       
    if( ADF_GoToRXState( ) != TRE_NO_ERR )
      break;
    
    return TRE_NO_ERR;
    
  default:
    break;
  } 
  return TRE_DEBUG;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(aducm3025_radio_process, ev, data)
{
  int16_t len;
  
  PROCESS_BEGIN();
  
  PRINTF_DEBUG("aducm3025_radio_process: started\r\n");
  
  while(1) {
#ifdef DISABLE_PATCH_802_15_4D
    /* Enable Async reception of RF frames */
    adi_ADF7023_AsyncRx( );
#endif /* DISABLE_PATCH_802_15_4D */
    
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    
    PRINTF_DEBUG("aducm3025_radio_process: calling receiver callback\r\n");
    
#ifdef DISABLE_PATCH_802_15_4D    
    packetbuf_clear();
    len = aducm3025_radio_read(packetbuf_dataptr(), PACKETBUF_SIZE);
#else
    len = rcvd_pkt_len;
#endif // DISABLE_PATCH_802_15_4D
    
    if(len > 0) {
      
#ifdef WIRESHARK_FEEDER
      {
        extern ADI_UART_HANDLE hUartDevice;
        char i;
        int16_t j = 1;  
        //len += 0;
        i = len;
#if(SLIPRADIO)		
        slip_write(packetbuf_dataptr(), len);
#else	
        if(ANode_GetAutoSend())
        {
          if(adi_uart_SubmitTxBuffer(hUartDevice,(char *)&i,j) == ADI_UART_SUCCESS)
            adi_uart_EnableTx(hUartDevice,true);
          
          if(adi_uart_SubmitTxBuffer(hUartDevice,(char *)packetbuf_dataptr(),len) == ADI_UART_SUCCESS)
            adi_uart_EnableTx(hUartDevice,true);
          
        }
#endif		
      }
#elif FOREN6_FEEDER
      {
        extern ADI_UART_HANDLE hUartDevice;
        char i;
        int16_t j;  
        //len += 0;
        i = len;
        //add it under a macro for foren6
        j =5;
        
        if(adi_uart_SubmitTxBuffer(hUartDevice,(char *)Foren6Header, j) == ADI_UART_SUCCESS)
          adi_uart_EnableTx(hUartDevice,true);
        //end of macro for foren6
        j = 1;
        if(adi_uart_SubmitTxBuffer(hUartDevice,(char *)&i, j) == ADI_UART_SUCCESS)
          adi_uart_EnableTx(hUartDevice,true);
        //len = len + 2;
        if(adi_uart_SubmitTxBuffer(hUartDevice,(char *)packetbuf_dataptr(),len) == ADI_UART_SUCCESS)
          adi_uart_EnableTx(hUartDevice,true);
      }
#else     
      
#ifdef DEBUG
      {
        printf("\n********************************************************************************\n");
        char *ptr = packetbuf_dataptr();
        PRINTF_DEBUG("\nADURF101-radio:\n--------------\nReceived data len = %d bytes\n",len);
        printf("Received data:\n");
        
        for(u8_t c=0; c < len; c++){
          if(c%30 == 0 )
            printf("\n");
          PRINTF("%x:",ptr[c]);
        }
        
        PRINTF_DEBUG("\n");
      }
#endif	 //DEBUG
      
      packetbuf_set_datalen(len);
      NETSTACK_RDC.input();
      
#endif //WIRESHARK_FEEDER
    }
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/  

void adi_ADF7023_ReceiveIsrCallback ( uint8_t *prxbuf )
{
  if(g_node_type != NODE_6LN){
    adi_gpio_Toggle(ADI_GPIO_PORT0,ADI_GPIO_PIN_13);
  }

#ifdef DISABLE_PATCH_802_15_4D
  /* Copy the received data into the receive buffer */
  memcpy( aducm3025_rxbuf, prxbuf, prxbuf[0] ); 
#else
  DBG_Phy_State = 0x02;
  uint16_t Rcv_Packet_Length = MAKE_UINT16(prxbuf[1],(prxbuf[0] & PHR_LENMSBMASK));
  
  /* Copy the received data into the receive buffer */
  memcpy( (aducm3025_rxbuf+1), (prxbuf+2), Rcv_Packet_Length);
  
  /* truncate 2 byte variable Rec_Packet_Length to 1 byte variable as we are sure that the upper sicslowpan layer can handle
  packets with length < 256 for the current implementation */
  aducm3025_rxbuf[0] = Rcv_Packet_Length;
  
  //  packetbuf_clear();
  
  if(aducm3025_rxbuf[1] & 0x02)
  {
    Set_Ack_Done(0x01);
    Set_Rx_Seq_No(aducm3025_rxbuf[3]);
  }
  
  rcvd_pkt_len = aducm3025_radio_read(packetbuf_dataptr(), PACKETBUF_SIZE);
  
#if defined(WIRESHARK_FEEDER) || defined(FOREN6_FEEDER)
  adi_ADF7023_Receive_15d4_Frame( );
#endif // WIRESHARK_FEEDER  
  
#endif // DISABLE_PATCH_802_15_4D
  
  process_poll(&aducm3025_radio_process);
}

/*---------------------------------------------------------------------------*/  

//void Poll_Radio_CB( void )
//{
//  /* Poll the aducm3025_radio process to process the received packet */
//  process_poll(&aducm3025_radio_process);
//}

/*---------------------------------------------------------------------------*/  
unsigned char adi_ADF7023_SetPhyRx( void )
{
  switch(mPHYState)
  {
  case CMD_PHY_SLEEP:
    if( ADF_GoToOffState( ) != TRE_NO_ERR )
      break;
    
    /* To go to PHY_RX from OFF state.we have to switch on the radio first */
  case CMD_PHY_OFF:
    if( ADF_GoToOnState( ) != TRE_NO_ERR )
      break;
    
  case CMD_PHY_TX:  
  case CMD_PHY_ON:       
    if( ADF_GoToRXState( ) != TRE_NO_ERR )
      break;
    
    return TRE_NO_ERR;
    
  default:
    break;
  } 
  return TRE_DEBUG;
}

extern void pull_down_gpio();
int send_ack(uint8_t seq_no, bool flag)
{ 
  uint16_t payload_len = 3;
  if(flag){
    ACK[2] |= (flag << 4);
  }
  else {
    ACK[2] = 0x02;
  }

  ACK[4] = seq_no;
  bAckTxProgress = 1;
  
  payload_len += SIZE_CRC_FIELD;
  
  pull_down_gpio();
  
#ifndef DISABLE_PATCH_802_15_4D  
  uint16_t PHR;
  uint8_t  Thrvalue = 47,TimerValue =6;
  
  PHR = create_Normal_MR_FSK_PHR
    (
     true, // bool scramble_psdu,
     !(I_SUPPORT_4BYTE_FCS), // bool crc_16,
     &payload_len // uint16_t* p_psdu_len
       );  
  
  if(ADF7023_15d4g_Send_Pkt(PHR,ACK,Thrvalue,TimerValue,false,transmit_pwr) == TRE_NO_ERR)
#else
    if( ADF_TransmitPacket( ACK, transmit_pwr) == TRE_NO_ERR )
#endif // DISABLE_PATCH_802_15_4D
    {
      return RADIO_TX_OK;
    }
  
  return RADIO_TX_ERR;
}

void Set_Ack_Done(uint8_t ack_done)
{
  TxState.ACK_RX_Done = ack_done;
}

void Set_Rx_Seq_No(uint8_t seq_no)
{
  TxState.Rx_Seq_No = seq_no;
}

void Set_Tx_Seq_No(uint8_t seq_no)
{
  TxState.Tx_Seq_No = seq_no;
}

void Set_Ack_Required(uint8_t ack_req)
{
  TxState.Ack_Req = ack_req;
}

bool Get_Ack_Status()
{
  return((TxState.ACK_RX_Done) && (TxState.Tx_Seq_No == TxState.Rx_Seq_No));
}

void pull_up_gpio( )
{
#if 0
  adi_GPIO_ConfigurePinMuxing(ADI_GPIO_PORT_3, ADI_GPIO_P34);
  adi_GPIO_SetOutputEnable (ADI_GPIO_PORT_3, ADI_GPIO_PIN_4, true);
  adi_GPIO_SetOpenCircuitEnable (ADI_GPIO_PORT_3, ADI_GPIO_PIN_4, true);
  adi_GPIO_SetHigh (ADI_GPIO_PORT_3, ADI_GPIO_PIN_4);
#endif
}


void pull_down_gpio( )
{
#if 0
  adi_GPIO_ConfigurePinMuxing(ADI_GPIO_PORT_3, ADI_GPIO_P34);
  adi_GPIO_SetLow (ADI_GPIO_PORT_3, ADI_GPIO_PIN_4);
  adi_GPIO_SetOpenCircuitEnable (ADI_GPIO_PORT_3, ADI_GPIO_PIN_4, false);
#endif   
}

/*---------------------------------------------------------------------------*/ 
