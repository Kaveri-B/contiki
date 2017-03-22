/*
 * Analog Devices ADF7242 Low-Power IEEE 802.15.4 Transceiver
 *
 * Copyright 2009-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ADF7242_H
#define __ADF7242_H

typedef signed long                    S32;
typedef signed short                   S16;
typedef signed char                    S8;
typedef volatile signed long           VS32;
typedef volatile signed short          VS16;
typedef volatile signed char           VS8;
typedef unsigned long                  U32;
typedef unsigned short                 U16;
typedef unsigned char                  U8;
typedef unsigned long  const           UC32;  
typedef unsigned short const           UC16;
typedef unsigned char  const           UC8;
typedef volatile unsigned long         VU32;
typedef volatile unsigned short        VU16;
typedef volatile unsigned char         VU8;
typedef volatile unsigned long  const  VUC32;  
typedef volatile unsigned short const  VUC16;  
typedef volatile unsigned char  const  VUC8;   
typedef volatile U8  *                 U8P;
typedef volatile U16 *                 U16P;
typedef volatile U32 *                 U32P;
typedef volatile S8  *                 S8P;
typedef volatile S16 *                 S16P;
typedef volatile S32 *                 S32P;
typedef enum {FALSE = 0, TRUE = !FALSE} boolean;   
   
typedef enum PHY_State
{
  PHY_IDLE = 1,
  PHY_MEAS,
  PHY_RDY,
  PHY_RX,
  PHY_TX
}PHY_State_t;

typedef enum
{
  TRPS_UP = 0x00,
  TRPS_DOWN,
  TRPS_SHUTDOWN
}TRPSSTATE;

typedef enum 
{ 
  ISMBAND_863_870MHZ = 0,
  ISMBAND_896_901MHZ,
  ISMBAND_901_902MHZ,
  ISMBAND_902_928MHZ,
  ISMBAND_917_923MHZ,
  ISMBAND_928_960MHZ,
  ISMBAND_920_928MHZ,
  ISMBAND_950_958MHZ,
  ISMBAND_2400MHZ 
}ISMBAND;


struct adf7242_platform_data {

#define ADF_IEEE802154_HW_AACK          (1 << 1)
#define ADF_IEEE802154_AUTO_CSMA_CA     (1 << 2)
#define ADF_IEEE802154_PROMISCUOUS_MODE (1 << 3) /* no address filtering, turns off HW_AACK */
#define ADF_IEEE802154_REPORT_ACK       (1 << 4) /* Report Ack frames */

  int mode;

  /*
   * Specifies number of attempts to
   * retransmit unacknowledged
   * frames while in automatic CSMA-CA
   * Tx mode.
   */
  int max_frame_retries;

  /*
   * Specifies number of attempts to
   * repeat CSMA-CA algorithm prior to
   * cancellation of RC_TX command.
   * Valid range is 0 to 5;
   * 7: CSMA-CA algorithm is off
   */
  int max_cca_retries;

  /*
   * Specifies the maximum back-off
   * exponent used in the CSMA-CA
   * algorithm; valid range is 4 to 8
   *
   */
  int max_csma_be;

  /*
   * Specifies the minimum back-off
   * exponent used in the CSMA-CA
   * algorithm; valid range is 0 to
   * csma_max_be
   */
  int min_csma_be;

  /* SPI Bus and Device IDs */
  int bus, device;
};

typedef struct rf_statistics{
  uint32_t PHR_detect;
  uint32_t CRC_failure;
  uint32_t Rx_EOF;
  uint32_t Tx_EOF;
  uint32_t CCA_Failed;
  uint32_t pkt_Tx_time;
  uint32_t pkt_Rx_time;
  uint32_t cca_time;
  uint32_t Count_ACK_Missed;
  uint32_t re_init_count;
  int8_t  RSSI_val;
  uint32_t Tx_timeout_re_init_count;
    //@debug
  uint32_t tx_drop_count_for_full_buff;
  uint32_t tx_drop_count_for_full_nbr;
}rf_statistics_t;

ADI_SPI_RESULT SPI_Init(uint8_t DevNum,
              uint32_t clockFreq);

int SPI_Write(uint8_t *data, unsigned short bytesNumber,
             uint8_t *Prologue, uint16_t PrologueSize);

int SPI_Read(uint8_t *data, unsigned short bytesNumber,
             uint8_t *Prologue, uint16_t PrologueSize);

unsigned char ADF_SPI_XMIT(uint8_t *TxData, uint8_t *RxData, uint16_t DataLen, uint8_t *Prologue, uint16_t PrologueSize);

void ADF7242CallBack(void      *pCBParam,         /*!< Client supplied callback param */
                      uint32_t   Event,            /*!< Event ID specific to the Driver/Service */
                      void      *pArg);
#endif
