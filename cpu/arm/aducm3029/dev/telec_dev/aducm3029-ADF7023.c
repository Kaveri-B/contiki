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

#include "system.h"
#include "telec_common.h"
#include "stack_support.h"
#include "ADF7023.h"
#include "trx_access.h"
#include "queue_latest.h"
#ifdef TELEC_TEST_HARNESS
#include "buff_mgmt.h"
#endif

#include "RF_Module_API_Handler.h"
#include "phy/phy.h"

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

#ifdef TELEC_TEST_HARNESS
#define BUFFER_POOL_SIZE	13000
#endif

#ifdef FOREN6_FEEDER
    static const char Foren6Header[5] = {0xC1, 0x1F, 0xFE, 0x72, 0x01};
#endif

ADI_SPI_HANDLE  hDevice_SPI;

volatile bool sixlowpan_tx_in_progress = false;

#ifdef TELEC_TEST_HARNESS
#pragma location="never_retained_ram"
uint8_t heap[ BUFFER_POOL_SIZE ];

buf_mem_pool_t buf_cfg[2] = {
		{ 512, 5 },
                { 2500, 4 }
                            };
#endif

uint8_t ack_required;
uint8_t *direct_data_ptr;

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
  0x00,
  0X00,
  0X00
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
ADI_ADF7023_RESULT_TYPE adi_ADF7023_SetPhyRx( ADI_ADF7023_DEV_HANDLE const );
uint8_t transmit_pwr = TRANSMIT_POWER;
uint16_t Ack_Missed = 0;

extern uint8_t rx_buf[ ADUCM3025_MAX_PACKET_LEN + SIZE_LEN_FIELD ];

extern rf_statistics_t rf_statistics;

extern phy_pib_t phy_pib;

extern int last_pkt_rcvd_rssi_val;

extern phy_mode_config_t* gp_phy_mode_config;
extern phy_band_config_t phy_band_config[];

uint32_t tx_packet_count, total_fail_count, cca_fail_count;
int32_t last_pkt_rcvd_lqi;

uint8_t *TRX_Get_RX_Buffer(void);

void phy_process_data_event( volatile uint8_t* p_trx_event );

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

  enable_phy_trx(false);

#ifdef TELEC_TEST_HARNESS
  bm_init(  BUFFER_POOL_SIZE, heap, 2, buf_cfg );
#endif

#if TELEC_TEST_HARNESS
  rtc_config_init();
#endif

  ADI_ENABLE_INT(RADIO_INT_NUM);

  gp_phy_mode_config = &(phy_band_config[0].phy_mode_config[0]);

    /************TELEC INIT ***********************/
#if TELEC_TEST_HARNESS
  wakeup_GPIO_configure(true);
#endif

  /************TELEC INIT ENDS ***********************/

  /* Initialize ADF7023 radio. Pass the parameters which are obtained by GetRadioSettings.*/
  if(TRX_Init(hDevice_SPI, ADI_SPI_DEVID_2, SYS_GPIO_INTA_IRQn) != SUCCESS)
    return TRE_DEBUG;

  TRX_Register_Data_Event_cb(phy_process_data_event);

//  if ( ADF_RadioInit( ADF7023Configs.ProfileNo, ADF7023Configs.ISMBand,
//                     ADF7023Configs.Freq, ADF7023Configs.InitParm) != TRE_NO_ERR )
//    return TRE_DEBUG;

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
  return (TRX_Set_Channel_Frequency( channelFrequency ));
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

    sixlowpan_tx_in_progress = true;

    if(TRX_Write(NULL, PHR,aducm3025_txbuf + 2, payload_len, false, Thrvalue,TimerValue) == SUCCESS)
#endif /* ENABLE_HARDWARE_CCA */
#else
      if( ADF_TransmitPacket( aducm3025_txbuf, transmit_pwr) == TRE_NO_ERR )
#endif // DISABLE_PATCH_802_15_4D
      {

        while(sixlowpan_tx_in_progress);

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
    rf_statistics.RSSI_val = 0;
  if(TRX_Get_RSSI_Method_1 (&rf_statistics.RSSI_val) == SUCCESS ) {
    return ((rf_statistics.RSSI_val < -90)?TRE_NO_ERR:TRE_DEBUG);
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
  return TRX_Off( );
}
/*---------------------------------------------------------------------------*/
static int
aducm3025_radio_on(void)
{
#ifdef DISABLE_PATCH_802_15_4D
  return adi_ADF7023_SetPhyRx( );
#else
  DBG_Phy_State = 0x03;
  return TRX_Rx_15d4On( );
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
    if( TRX_Off( ) != SUCCESS )
      break;

    /* To go to PHY_RX from OFF state.we have to switch on the radio first */
  case CMD_PHY_OFF:
    if( TRX_On( ) != SUCCESS )
      break;

  case CMD_PHY_TX:
  case CMD_PHY_ON:
    if( TRX_Rx_On( ) != SUCCESS )
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
volatile uint32_t dbg_rx_buff_overflow = 0;
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

  sixlowpan_tx_in_progress = true;

  if(TRX_Write(NULL, PHR,ACK + 2, payload_len /* OR 4?? */, false, Thrvalue,TimerValue) == SUCCESS)
#else
    if( ADF_TransmitPacket( ACK, transmit_pwr) == TRE_NO_ERR )
#endif // DISABLE_PATCH_802_15_4D
    {
      while(sixlowpan_tx_in_progress);

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

void phy_process_data_event( volatile uint8_t* p_trx_event )
{
    uint16_t psdulen;
    phy_rx_t* p_rx_frame;
    uint8_t *pBuf;
    bool state_change_req;

    adi_int_EnterCriticalRegion();

    if( *p_trx_event & SFD_DETECTED )
    {

      *p_trx_event &= ~( SFD_DETECTED );

      /* Due to interrupt disabling while changing the TRX state,
      state might have been changed. Make sure TRX state in RX_ON
      state before changing the state BUSY_RX*/
      if (phy_pib.TRXState  == PHY_RX_ON)
      {

#ifdef STATISTICS_ENABLED
        rf_statistics.phy_sfd_detect_event++;
#endif//#ifdef STATISTICS_ENABLED

#ifdef UTEST_PD
        utu_timestamp(UTUL_PHY_PROCESS_DATA_SYNC, 0);
#endif
        phy_pib.TRXState  = PHY_BUSY_RX;

        adi_int_ExitCriticalRegion();

        //sfd_detected_cb();

        adi_int_EnterCriticalRegion();
#ifndef PATCH_802_15_4D
        start_sports_reset_timer();
#else
        start_802_15_4D_rx_reset_timer();
      }
#endif
    }

    if( *p_trx_event & TX_COMPLETE )
    {
        *p_trx_event &= ~( TX_COMPLETE );
#ifdef STATISTICS_ENABLED
        rf_statistics.phy_tx_complete_event++;
#endif//#ifdef STATISTICS_ENABLED
#ifdef UTEST_PD
        utu_timestamp(UTUL_PHY_PROCESS_DATA_TX_PKT_DONE,0);
#endif

#ifdef	PATCH_802_15_4D
        phy_pib.TRXState  = PHY_DEV_ON;
#else
        phy_pib.TRXState  = PHY_TX_ON;
#endif

        adi_int_ExitCriticalRegion();

        stop_802_15_4D_reset_timer(3);

//        if( phy_pib.pendingStateChange != PHY_INVALID_PARAMETER )
//        {
//                /*check if there was any state transition which was waiting for transmission
//                to be completed. This takes care of setting TRX to PHY_RX_ON or PHY_TRX_OFF
//                state after making sure that the current transmission is completed */
//                PLME_Set_TRX_State( phy_pib.pendingStateChange );
//
//                phy_pib.pendingStateChange = PHY_INVALID_PARAMETER;
//        }

        PD_Data_Confirmation_cb( NULL, PHY_SUCCESS );

        adi_int_EnterCriticalRegion();
    }
    if ( *p_trx_event & RX_COMPLETE )
    {


#ifdef BENCH_MARKING

          benchMark_set_time(2);
#endif
          *p_trx_event &= ~( RX_COMPLETE );
#ifdef STATISTICS_ENABLED
          rf_statistics.phy_rx_complete_event++;
#endif//#ifdef STATISTICS_ENABLED

#ifdef UTEST_PD
	utu_timestamp(UTUL_PHY_PROCESS_DATA_RX_PKT_DONE, 0);
#endif

#ifdef PD_DEBUG
          phyProcDataEvent++;
#endif //#ifdef PD_DEBUG

          adi_int_ExitCriticalRegion();

#ifdef BENCH_MARKING

          benchMark_set_time(3);
#endif

#ifndef PATCH_802_15_4D
          stop_sports_reset_timer();
#else
          //stop_802_15_4D_rx_reset_timer();
#endif

#ifdef BENCH_MARKING

          benchMark_set_time(4);
#endif

          //TRX_On();		//change made

#ifdef BENCH_MARKING

          benchMark_set_time(5);
#endif

          phy_pib.TRXState = PHY_DEV_ON; //change made

          psdulen = TRX_get_RX_packet_len();

          /* allocate a buffer with only required size */
          pBuf = TRX_Get_RX_Buffer();

          TRX_clear_RX_Buffer();

          if (pBuf == NULL)
          {
                  PD_Data_Indication_cb( pBuf, false, PHY_CRC_ERROR );
                  return;
          }

          p_rx_frame = (phy_rx_t *)Get_phy_rx_struct_location(pBuf);

#ifdef BENCH_MARKING

          benchMark_set_time(6);
#endif
        if(phy_trx_enable_status())
        {
            TRX_Read_RX_Buffer( (uint8_t*)p_rx_frame, psdulen );
            p_rx_frame->psdu += 2;
            p_rx_frame->sfd_rx_time = hw_tmr_get_symbols( p_rx_frame->sfd_rx_time );
        }

#ifdef BENCH_MARKING

          benchMark_set_time(7);
#endif

          //TRX_Set_RX_Buffer();

#ifdef BENCH_MARKING

          benchMark_set_time(8);
#endif
          if(phy_trx_enable_status())
          {
              p_rx_frame->channel = phy_pib.CurrentChannel;

              last_pkt_rcvd_rssi_val = p_rx_frame->psduLinkQuality;

              p_rx_frame->psduLinkQuality  =
              convert_RSSI_To_LQI(p_rx_frame->psduLinkQuality);
          }

          if ( phy_pib.pendingStateChange != PHY_INVALID_PARAMETER )
          {
                  /*state transition to PHY_TRX_OFF is pending since the reception was under
                  progress. The required change is already done before reading the frame*/
                  phy_pib.pendingStateChange = PHY_INVALID_PARAMETER;
                  state_change_req = true;
          }
          else
          {
                  state_change_req = true;
          }

#ifdef TEMPERATURE_TEST

          uhf_result = (ADI_ADF7023_RESULT_TYPE)TRX_Read_Temperature(&tempReading);

          if(uhf_result != ADI_ADF7023_SUCCESS)
          {
                  TemperatureReadingErr++;
          }
          else
          {
                  TemperatureReadingSuccess++;
          }

#endif //TEMPERATURE_TEST

			//SetTestFlag(0,0);
#ifdef PATCH_802_15_4D

          /* if the 2 byte CRC used, then get the CRC status from lower layer */
          if ((TRX_crc_status() == false) /*&& (p_rx_frame->FCSLength == 2)*/)
          {
                  /* indicate higher layer about the packet reception with CRC error */
                  PD_Data_Indication_cb( pBuf, state_change_req, PHY_CRC_ERROR );
          }
          else
#endif
          {
                  /* Data received without CRC error(2 byte CRC).
                     For 4 byte CRC, the higher layer will verify about CRC failures */
                  PD_Data_Indication_cb( pBuf, state_change_req
#ifdef PATCH_802_15_4D
                  , PHY_SUCCESS
#endif

                  );
          }

          adi_int_EnterCriticalRegion();

        }

        adi_int_ExitCriticalRegion();
}


void PD_Data_Indication_cb( uint8_t *pBuf, bool state_change_req
#ifdef PATCH_802_15_4D
, u8 PHY_Status
#endif //PATCH_802_15_4D
)
{
    phy_rx_t *pdp;

    if(phy_trx_enable_status())
    {
#ifdef TELEC_TEST_HARNESS
        send_direct_pd_data_ind(pBuf, PHY_Status);
        if (!continuousRx_status())
        {
//            change_trx_state(&trxsm);
        }
        else
        {
            TRX_cont_Rx_15d4On();
        }
#endif
        return;
    }
    else
    {
        if(PHY_Status == PHY_CRC_ERROR)
        {
            TRX_Rx_15d4On();
            phy_pib.TRXState = PHY_RX_ON;
        }
        else
            adi_ADF7023_ReceiveIsrCallback ( rx_buf );
        return;
    }
}

#ifdef TELEC_TEST_HARNESS
void send_direct_pd_data_Telec_req(uint8_t CCA_Enable,uint16_t pd_data_len,uint8_t *pd_data,bool bUsePrevBuff)
{
  static phy_tx_t *phy_data;
  ulong temp = 0;
  uint16_t len;

  uint8_t CCATimerDuration15D4;
  int8_t CCAThreshHold15D4 = -80;

  PLME_get_request( phyCurrentChannel, &len, &temp );

  if(!bUsePrevBuff)
  {
    phy_data = (phy_tx_t *)bm_alloc(heap, sizeof(phy_tx_t) + 10 + pd_data_len);

    phy_data->PPDUCoding = false;
    phy_data->ModeSwitch =	false;
    phy_data->NewModeSUNPage = 	0x00;
    phy_data->ModeSwitchParameterEntry = 0x00;
    phy_data->psduLength = pd_data_len;
    phy_data->FCSLength = false;
    phy_data->TxChannel = temp & 0xff;
    phy_data->psdu = (((uint8_t *)&phy_data->reservedPHR) + 2);
    memcpy(phy_data->psdu, pd_data, phy_data->psduLength);
  }

  if ( pd_data[0] & 0x20 )
  {
		ack_required = 1;
    }
    else
    {
    	ack_required = 0;
    }
	PLME_Set_TRX_State( PHY_DEV_ON );

  CCAThreshHold15D4 = CCA_THRESHOLD;

  if(!get_T108_limitation_mode())
     CCATimerDuration15D4 = 5;
  else
      CCATimerDuration15D4 = 0;

	PD_Data_Request( NULL, phy_data
#ifdef PATCH_802_15_4D
							,CCA_Enable,
							(CCAThreshHold15D4 + 107),
                                                        CCATimerDuration15D4
#endif
							);

	//free_buf_request( phy_data );
    direct_data_ptr = (uint8_t *)phy_data;
}
#endif

void PD_Data_Confirmation_cb( void* pHandle,uchar status )
{
    /* Testing direct pd data req */
    if(phy_trx_enable_status())
    {
#ifdef TELEC_TEST_HARNESS
//    tx_packet_count++;
      send_direct_pd_data_conf(direct_data_ptr, status);
      if (ack_required && (status == PHY_SUCCESS))
      {
          TRX_set_15D4_rx_buffer();
          phy_pib.TRXState = PHY_RX_ON;
      }
      ack_required = 0;
#endif
      return;
    }
    else
    {
      sixlowpan_tx_in_progress = false;
#ifdef DISABLE_PATCH_802_15_4D
      adi_ADF7023_AsyncRx( );
      return;
#else
      TRX_Rx_15d4On();
      phy_pib.TRXState = PHY_RX_ON;
      return;
#endif
    }
}

/**
 *******************************************************************************
 ** \brief This function allocates a buffer depending on frame control of the
 **			received packet.
 ** \param recv_buf - First few bytes received..
 **
 ** \retval - starting location of the allocated buffer.
 ******************************************************************************/
uint8_t *mac_allocate_rx_buffer(uint8_t *recv_buf)
{
    return rx_buf;
}
/**
 *******************************************************************************
 ** \brief This function gets the phy_rx_t structure location
 **		in the allocated buffer for reception.
 ** \param buf - Reception buffer allocated.
 **
 ** \retval - Location of phy_rx_t structure, in allocated buffer.
 ******************************************************************************/
uint8_t *mac_rx_struct_location(uint8_t *buf)
{
    return rx_buf;
}

/**
 *******************************************************************************
 ** \brief This function gets the location of data reception.
 **		in the allocated buffer for reception.
 ** \param buf - Reception buffer allocated.
 ** \param recv_buf - First few bytes received.
 **
 ** \retval - Location for data reception.
 ******************************************************************************/
uint8_t *Get_mac_Reception_location(uint8_t *buf, uint8_t *recv_data)
{
    return rx_buf;
}

/*---------------------------------------------------------------------------*/
