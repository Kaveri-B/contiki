/*
 * Analog Devices ADF7242 Low-Power IEEE 802.15.4 Transceiver
 *
 * Copyright 2009-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

/*
 * DEBUG LEVEL
 *     0       OFF
 *     1       INFO
 *     2       INFO + TRACE
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>   
#include <spi\adi_spi.h>
#include <gpio\adi_gpio.h>
#include <uart\adi_uart.h>
   
#include "adf7242.h"
#include "ram_lab_7242_2_0_IEEE15dot4_full_R6.h"
#include "radio.h"
#ifndef WIRESHARK_FEEDER
#include "contiki.h"
#include "net\netstack.h"
#include "net\packetbuf.h"
#endif
#include "RF_Module_API_Handler.h"

#include "ssdd_common/common_def.h"
#include "adi_spi_def_v1.h"

#define DEV_TYPE_HOST           1
#define DEV_TYPE_ROUTER         2
#define DEV_TYPE_EDGE_ROUTER    3
#define DEV_TYPE_NONE           4
#define DEV_TYPE                DEV_TYPE_NONE

#define MAX_POLL_LOOPS 1000000

/* All Registers */

#define REG_EXT_CTRL   0x100  /* RW External LNA/PA and internal PA control configuration bits */
#define REG_TX_FSK_TEST 0x101 /* RW TX FSK test mode configuration */
#define REG_CCA1       0x105  /* RW RSSI threshold for CCA */
#define REG_CCA2       0x106  /* RW CCA mode configuration */
#define REG_BUFFERCFG  0x107  /* RW RX_BUFFER overwrite control */
#define REG_PKT_CFG    0x108  /* RW FCS evaluation configuration */
#define REG_DELAYCFG0  0x109  /* RW RC_RX command to SFD or sync word search delay */
#define REG_DELAYCFG1  0x10A  /* RW RC_TX command to TX state */
#define REG_DELAYCFG2  0x10B  /* RW Mac delay extention */
#define REG_SYNC_WORD0 0x10C  /* RW sync word bits [7:0] of [23:0]  */
#define REG_SYNC_WORD1 0x10D  /* RW sync word bits [15:8] of [23:0]  */
#define REG_SYNC_WORD2 0x10E  /* RW sync word bits [23:16] of [23:0]  */
#define REG_SYNC_CONFIG        0x10F  /* RW sync word configuration */
#define REG_RC_CFG     0x13E  /* RW RX / TX packet configuration */
#define REG_RC_VAR44   0x13F  /* RW RESERVED */
#define REG_CH_FREQ0   0x300  /* RW Channel Frequency Settings - Low Byte */
#define REG_CH_FREQ1   0x301  /* RW Channel Frequency Settings - Middle Byte */
#define REG_CH_FREQ2   0x302  /* RW Channel Frequency Settings - 2 MSBs */
#define REG_TX_FD      0x304  /* RW TX Frequency Deviation Register */
#define REG_DM_CFG0    0x305  /* RW RX Discriminator BW Register */
#define REG_TX_M       0x306  /* RW TX Mode Register */
#define REG_RX_M       0x307  /* RW RX Mode Register */
#define REG_RRB                0x30C  /* R RSSI Readback Register */
#define REG_LRB                0x30D  /* R Link Quality Readback Register */
#define REG_DR0                0x30E  /* RW bits [15:8] of [15:0] for data rate setting */
#define REG_DR1                0x30F  /* RW bits [7:0] of [15:0] for data rate setting */
#define REG_PRAMPG     0x313  /* RW RESERVED */
#define REG_TXPB       0x314  /* RW TX Packet Storage Base Address */
#define REG_RXPB       0x315  /* RW RX Packet Storage Base Address */
#define REG_TMR_CFG0   0x316  /* RW Wake up Timer Configuration Register - High Byte */
#define REG_TMR_CFG1   0x317  /* RW Wake up Timer Configuration Register - Low Byte */
#define REG_TMR_RLD0   0x318  /* RW Wake up Timer Value Register - High Byte */
#define REG_TMR_RLD1   0x319  /* RW Wake up Timer Value Register - Low Byte */
#define REG_TMR_CTRL   0x31A  /* RW Wake up Timer Timeout flag */
#define REG_PD_AUX     0x31E  /* RW Battmon enable */
#define REG_GP_CFG     0x32C  /* RW GPIO Configuration */
#define REG_GP_OUT     0x32D  /* RW GPIO Configuration */
#define REG_GP_IN      0x32E  /* R GPIO Configuration */
#define REG_SYNT       0x335  /* RW bandwidth calibration timers */
#define REG_CAL_CFG    0x33D  /* RW Calibration Settings */
#define REG_PA_BIAS    0x36E  /* RW PA BIAS */
#define REG_SYNT_CAL   0x371  /* RW Oscillator and Doubler Configuration */
#define REG_IIRF_CFG   0x389  /* RW BB Filter Decimation Rate */
#define REG_CDR_CFG    0x38A  /* RW CDR kVCO */
#define REG_DM_CFG1    0x38B  /* RW Postdemodulator Filter */
#define REG_AGCSTAT    0x38E  /* R RXBB Ref Osc Calibration Engine Readback */
#define REG_RXCAL0     0x395  /* RW RX BB filter tuning, LSB */
#define REG_RXCAL1     0x396  /* RW RX BB filter tuning, MSB */
#define REG_RXFE_CFG   0x39B  /* RW RXBB Ref Osc & RXFE Calibration */
#define REG_PA_RR      0x3A7  /* RW Set PA ramp rate */
#define REG_PA_CFG     0x3A8  /* RW PA enable */
#define REG_EXTPA_CFG  0x3A9  /* RW External PA BIAS DAC */
#define REG_EXTPA_MSC  0x3AA  /* RW PA Bias Mode */
#define REG_ADC_RBK    0x3AE  /* R Readback temp */
#define REG_AGC_CFG1   0x3B2  /* RW GC Parameters */
#define REG_AGC_MAX    0x3B4  /* RW Slew rate  */
#define REG_AGC_CFG2   0x3B6  /* RW RSSI Parameters */
#define REG_AGC_CFG3   0x3B7  /* RW RSSI Parameters */
#define REG_AGC_CFG4   0x3B8  /* RW RSSI Parameters */
#define REG_AGC_CFG5   0x3B9  /* RW RSSI & NDEC Parameters */
#define REG_AGC_CFG6   0x3BA  /* RW NDEC Parameters */
#define REG_OCL_CFG1   0x3C4  /* RW OCL System Parameters */
#define REG_IRQ1_EN0   0x3C7  /* RW Interrupt Mask set bits  [7:0] of [15:0] for IRQ1 */
#define REG_IRQ1_EN1   0x3C8  /* RW Interrupt Mask set bits  [15:8] of [15:0] for IRQ1 */
#define REG_IRQ2_EN0   0x3C9  /* RW Interrupt Mask set bits  [7:0] of [15:0] for IRQ2 */
#define REG_IRQ2_EN1   0x3CA  /* RW Interrupt Mask set bits  [15:8] of [15:0] for IRQ2 */
#define REG_IRQ1_SRC0  0x3CB  /* RW Interrupt Source  bits  [7:0] of [15:0] for IRQ */
#define REG_IRQ1_SRC1  0x3CC  /* RW Interrupt Source bits  [15:8] of [15:0] for IRQ */
#define REG_OCL_BW0    0x3D2  /* RW OCL System Parameters */
#define REG_OCL_BW1    0x3D3  /* RW OCL System Parameters */
#define REG_OCL_BW2    0x3D4  /* RW OCL System Parameters */
#define REG_OCL_BW3    0x3D5  /* RW OCL System Parameters */
#define REG_OCL_BW4    0x3D6  /* RW OCL System Parameters */
#define REG_OCL_BWS    0x3D7  /* RW OCL System Parameters */
#define REG_OCL_CFG13  0x3E0  /* RW OCL System Parameters */
#define REG_GP_DRV     0x3E3  /* RW I/O pads Configuration and bg trim */
#define REG_BM_CFG     0x3E6  /* RW Battery Monitor Threshold Voltage setting */
#define REG_SFD_15_4   0x3F4  /* RW Option to set non standard SFD */
#define REG_AFC_CFG    0x3F7  /* RW AFC mode and polarity */
#define REG_AFC_KI_KP  0x3F8  /* RW AFC ki and kp */
#define REG_AFC_RANGE  0x3F9  /* RW AFC range */
#define REG_AFC_READ   0x3FA  /* RW Readback frequency error */

#define REG_PAN_ID0            0x112
#define REG_PAN_ID1            0x113
#define REG_SHORT_ADDR_0       0x114
#define REG_SHORT_ADDR_1       0x115
#define REG_IEEE_ADDR_0                0x116
#define REG_IEEE_ADDR_1                0x117
#define REG_IEEE_ADDR_2                0x118
#define REG_IEEE_ADDR_3                0x119
#define REG_IEEE_ADDR_4                0x11A
#define REG_IEEE_ADDR_5                0x11B
#define REG_IEEE_ADDR_6                0x11C
#define REG_IEEE_ADDR_7                0x11D
#define REG_FFILT_CFG          0x11E
#define REG_AUTO_CFG           0x11F
#define REG_AUTO_TX1           0x120
#define REG_AUTO_TX2           0x121
#define REG_AUTO_STATUS                0x122

/* REG_FFILT_CFG */
#define ACCEPT_BEACON_FRAMES   (1 << 0)
#define ACCEPT_DATA_FRAMES     (1 << 1)
#define ACCEPT_ACK_FRAMES      (1 << 2)
#define ACCEPT_MACCMD_FRAMES   (1 << 3)
#define ACCEPT_RESERVED_FRAMES (1 << 4)
#define ACCEPT_ALL_ADDRESS     (1 << 5)

/* REG_AUTO_CFG */
#define AUTO_ACK_FRAMEPEND     (1 << 0)
#define IS_PANCOORD            (1 << 1)
#define RX_AUTO_ACK_EN         (1 << 3)
#define CSMA_CA_RX_TURNAROUND  (1 << 4)

/* REG_AUTO_TX1 */
#define MAX_FRAME_RETRIES(x)   ((x) & 0xF)
#define MAX_CCA_RETRIES(x)     (((x) & 0x7) << 4)

/* REG_AUTO_TX2 */
#define CSMA_MAX_BE(x)         ((x) & 0xF)
#define CSMA_MIN_BE(x)         (((x) & 0xF) << 4)

#define CMD_SPI_NOP            0xFF /* No operation. Use for dummy writes */
#define CMD_SPI_PKT_WR         0x10 /* Write telegram to the Packet RAM starting from the TX packet base address pointer tx_packet_base */
#define CMD_SPI_PKT_RD         0x30 /* Read telegram from the Packet RAM starting from RX packet base address pointer rxpb.rx_packet_base */
#define CMD_SPI_MEM_WR(x)      (0x18 + (x >> 8)) /* Write data to MCR or Packet RAM sequentially */
#define CMD_SPI_MEM_RD(x)      (0x38 + (x >> 8)) /* Read data from MCR or Packet RAM sequentially */
#define CMD_SPI_MEMR_WR(x)     (0x08 + (x >> 8)) /* Write data to MCR or Packet RAM as random block */
#define CMD_SPI_MEMR_RD(x)     (0x28 + (x >> 8)) /* Read data from MCR or Packet RAM as random block */
#define CMD_SPI_PRAM_WR        0x1E /* Write data sequentially to current PRAM page selected */
#define CMD_SPI_PRAM_RD        0x3E /* Read data sequentially from current PRAM page selected */
#define CMD_RC_SLEEP           0xB1 /* Invoke transition of radio controller into SLEEP state */
#define CMD_RC_IDLE            0xB2 /* Invoke transition of radio controller into IDLE state */
#define CMD_RC_PHY_RDY         0xB3 /* Invoke transition of radio controller into PHY_RDY state */
#define CMD_RC_RX              0xB4 /* Invoke transition of radio controller into RX state */
#define CMD_RC_TX              0xB5 /* Invoke transition of radio controller into TX state */
#define CMD_RC_MEAS            0xB6 /* Invoke transition of radio controller into MEAS state */
#define CMD_RC_CCA             0xB7 /* Invoke Clear channel assessment */
#define CMD_RC_CSMACA          0xC1 /* initiates CSMA-CA channel access sequence and frame transmission */
#define CMD_RC_PC_RESET        0xC7 /* Reset the program counter of the firmware */
#define CMD_RC_RESET           0xC8 /* Reset the ADF7242 */

/* STATUS */

#define STAT_SPI_READY         (1 << 7)
#define STAT_IRQ_STATUS                (1 << 6)
#define STAT_RC_READY          (1 << 5)
#define STAT_CCA_RESULT                (1 << 4)
#define RC_STATUS_IDLE         1
#define RC_STATUS_MEAS         2
#define RC_STATUS_PHY_RDY      3
#define RC_STATUS_RX           4
#define RC_STATUS_TX           5
#define RC_STATUS_MASK         0xF

/* AUTO_STATUS */

#define SUCCESS                        0
#define SUCCESS_DATPEND                1
#define FAILURE_CSMACA         2
#define FAILURE_NOACK          3
#define AUTO_STATUS_MASK       0x7

#define PRAM_PAGESIZE          256

/* IRQ1 */

#define IRQ_CCA_COMPLETE       (1 << 0)
#define IRQ_SFD_RX             (1 << 1)
#define IRQ_SFD_TX             (1 << 2)
#define IRQ_RX_PKT_RCVD                (1 << 3)
#define IRQ_TX_PKT_SENT                (1 << 4)
#define IRQ_FRAME_VALID                (1 << 5)
#define IRQ_ADDRESS_VALID      (1 << 6)
#define IRQ_CSMA_CA            (1 << 7)

#define AUTO_TX_TURNAROUND     (1 << 3)
#define ADDON_EN               (1 << 4)

/* REG_EXTPA_MSC */
#define PA_PWR(x)   (((x) & 0xF) << 4)
#define EXTPA_BIAS_SRC    (1 << 3)
#define EXTPA_BIAS_MODE(x)  (((x) & 0x7) << 0)

/* REG_PA_CFG */
#define PA_BRIDGE_DBIAS(x)  (((x) & 0x1F) << 0)

/* REG_PA_BIAS */
#define PA_BIAS_CTRL(x)   (((x) & 0x1F) << 1)
#define REG_PA_BIAS_DFL   (1 << 0)

#define INT_RECEIVED 160

#ifdef RFMODULE_7242
#define ADF7242_SPI_DEVICE_NUM          (1)
#define SPI1_CS_0_PORTP1_MUX  ((uint32_t) ((uint32_t) 1<<18))
#else
/* Use SPI device 2 (SPIH), this is fixed because of board layout */
#define ADF7242_SPI_DEVICE_NUM          (2)
   
#define SPI2_CS_3_PORTP2_MUX  ((uint16_t) ((uint16_t) 2<<14))   
#endif

#define ADF_DEBUG       0
   
#define MAX_SYS_TICK_COUNT                  (104000)
#define TICK_DURATION                       (4000)
#define SYS_TICK_TO_ms_FACTOR               ((float)(1/CLOCK_FREQ_IN_MHZ))   

#define PACKET_RCV_TIME     (352) 
#define DBG(n, args ...) do { \
    if(ADF_DEBUG >= (n)) { \
      printf("DEBUG" # n ": " args); } \
} while(0)
/* ADF7242 status masks */
#define RC_PHY_RDY_MASK          (0x03)
#define RC_READY_MASK            (0x20)
#define RC_RX_MASK               (0x04)
#define RC_TX_MASK               (0x05)
/* ADF7242 802.15.4 status */
#define DS_15d4_STATE_EXTRACT_DATA   (0x04)
#define REG_802_15_4_STATE          0x312

struct adf7242_platform_data adf7242_platform_data = {
  .mode = (ADF_IEEE802154_HW_AACK | ADF_IEEE802154_AUTO_CSMA_CA | ADF_IEEE802154_REPORT_ACK),
  .max_frame_retries = 0,
  .max_cca_retries = 5,
  .max_csma_be = 5,
  .min_csma_be = 3,
  .bus = ADF7242_SPI_DEVICE_NUM,
  .device = 0,
};

typedef struct TxStateMachine {
  uint8_t ACK_RX_Done;
  uint8_t Tx_Seq_No;
  uint8_t Rx_Seq_No;
  uint8_t Ack_Req;
}TxStateMachine_t;

uint8_t ACK[] = {
  0x02,
  0x00,
  0x00,
};


volatile TxStateMachine_t TxState;

uint8_t bAckTxProgress = 0;

PHY_State_t mPHYState;

struct adf7242_platform_data *adf7242_pdata = (struct adf7242_platform_data *)&adf7242_platform_data;
void rf_spi_write_fast(uint8_t *pTxbuf, uint8_t txlen);
void adi_ADF7023_ReceiveIsrCallback ( );

uint32_t spidevicemem[(ADI_SPI_MEMORY_SIZE+3)/4];

static ADI_SPI_HANDLE hDevice;

static unsigned char tx_buf[255];
static unsigned char radio_buffer[256];
static bool tx_led, rx_led;

static uint32_t TickCounter = 0;
static uint32_t pkt_recv_err_timeout = 0;
static uint32_t pkt_transmit_err_timeout = 0;
static uint32_t pkt_Tx_start_time;
static uint32_t pkt_Rx_start_time;
static uint32_t pkt_cca_start_time;
volatile bool pkt_recv_err_timeout_set = false;  //debug-HP
volatile bool pkt_transmit_err_timeout_set = false;      //debug-HP

static volatile bool packet_pending;
static volatile int tx_status;
unsigned char adf7242_lqi;
uint8_t transmit_pwr;
rf_statistics_t rf_stats;
  
bool bSpiInitialized;
void Set_Ack_Done(uint8_t ack_done);
void Set_Rx_Seq_No(uint8_t seq_no);
void Set_Tx_Seq_No(uint8_t seq_no);
void Set_Ack_Required(uint8_t ack_req);
void SetLastAckFailureCount(void);

void Enable_Transceiver_Interrupts(void);
void Disable_Transceiver_Interrupts(void);

extern void GlowAllLEDs(void);
extern NodeType_t g_node_type;

extern uint8_t transmit_pwr;
extern uint16_t phy_current_channel;
extern uint32_t system_hibernate_duration;
extern uint32_t system_flexi_mode_sleep_duration;

#ifndef WIRESHARK_FEEDER
PROCESS(aducm3025_radio_process, "ADucM3025 radio driver");
#else
static void adi_packet_recvd ( void );
#endif

static uint8_t
adf7242_tx_irq(struct adf7242_platform_data *pdata)
{
  if(pdata->mode & ADF_IEEE802154_AUTO_CSMA_CA) {
    return IRQ_CSMA_CA;
  } else {
    return IRQ_TX_PKT_SENT;
  }
}

static int
adf7242_status(struct adf7242_platform_data *pdata, uint8_t *stat)
{
//  return SPI_Read(pdata->bus, pdata->device, stat, 1);
//  return SPI_Read(stat, 1, NULL, 0);
  return ADF_SPI_XMIT(NULL, stat, 1, NULL, 0);
}

static int
adf7242_wait_mask(struct adf7242_platform_data *pdata, uint8_t mask)
{
  uint8_t stat = 0;
  unsigned int cnt = 0;

  DBG(2, "%s :Enter\r\n", __func__);

  do {
    int ret = adf7242_status(pdata, &stat);
    if(ret < 0) {
      return ret;
    }
    cnt++;
  } while((stat & mask) != mask && (cnt < MAX_POLL_LOOPS));

  DBG(2, "%s :Exit loops=%d\r\n", __func__, cnt);

  return cnt == MAX_POLL_LOOPS ? -1 : 0;
}

static int
adf7242_wait_ready(struct adf7242_platform_data *pdata)
{
  DBG(2, "%s :Enter\r\n", __func__);
  int ret = adf7242_wait_mask(pdata, STAT_RC_READY);
  DBG(2, "%s :Exit\r\n", __func__);
  return ret;
}

static int
adf7242_wait_status(struct adf7242_platform_data *pdata, int status)
{
  uint8_t stat = 0;
  unsigned int cnt = 0;

  DBG(2, "%s :Enter\r\n", __func__);

  do {
    int ret = adf7242_status(pdata, &stat);
    if(ret < 0) {
      return ret;
    }
    stat &= RC_STATUS_MASK;
    cnt++;
  } while((stat != status) && (cnt < MAX_POLL_LOOPS));

  DBG(2, "%s :Exit loops=%d\r\n", __func__, cnt);

  return cnt == MAX_POLL_LOOPS ? -1 : 0;
}

static int
adf7242_write_fbuf(struct adf7242_platform_data *pdata,
                   uint8_t *data, uint8_t len)
{
  uint8_t buf[] = { CMD_SPI_PKT_WR, len + 2, };
  int ret;

  DBG(2, "%s :Enter\r\n", __func__);    

  ret = adf7242_wait_ready(pdata);
  if(ret < 0) {
    return ret;
  }

//  SPI_Set_CS(pdata->bus, pdata->device, 0);
//  ret = SPI_Write_NoCS(pdata->bus, pdata->device, buf, sizeof(buf));
//  if(!ret) {
//    ret = SPI_Write_NoCS(pdata->bus, pdata->device, data, len);
//  }
//  SPI_Set_CS(pdata->bus, pdata->device, 1);

  ADF_SPI_XMIT(data, NULL, len, buf, sizeof(buf));

  DBG(2, "%s :Exit\r\n", __func__);
  return ret;
}

static int
adf7242_read_fbuf(struct adf7242_platform_data *pdata,
                  uint8_t *data, uint8_t *len, uint8_t *lqi)
{
  uint8_t buf[] = { CMD_SPI_PKT_RD, CMD_SPI_NOP, };
  int ret;
  
//  buf[2] = *len;

  DBG(2, "%s :Enter\r\n", __func__);

  ret = adf7242_wait_ready(pdata);
  if(ret < 0) {
    return ret;
  }

//  SPI_Set_CS(pdata->bus, pdata->device, 0);
//  ret = SPI_Write_NoCS(pdata->bus, pdata->device, buf, sizeof(buf));
//  if(!ret) {
//    ret = SPI_Read_NoCS(pdata->bus, pdata->device, len, 1);
//  }
//  if(!ret) {
//    ret = SPI_Read_NoCS(pdata->bus, pdata->device, data, *len);
//  }
//  SPI_Set_CS(pdata->bus, pdata->device, 1);
  
//  SPI_Write(data, *len, buf, sizeof(buf));
  ADF_SPI_XMIT(NULL, data, *len, buf, sizeof(buf));

  if(!ret) {
    *lqi = data[*len - 1];
  }

  DBG(2, "%s :Exit\r\n", __func__);
  return ret;
}

static int
adf7242_read_reg(struct adf7242_platform_data *pdata,
                 uint16_t addr, uint8_t *data)
{
  uint8_t buf[] = {
    CMD_SPI_MEM_RD(addr), addr, CMD_SPI_NOP, CMD_SPI_NOP
  };
  uint8_t ReadValue[5];
  int ret;

  DBG(2, "%s :Enter\r\n", __func__);

  ret = adf7242_wait_ready(pdata);
  if(ret < 0) {
    return ret;
  }

//  SPI_Set_CS(pdata->bus, pdata->device, 0);
//  ret = SPI_Write_NoCS(pdata->bus, pdata->device, buf, sizeof(buf));
//  if(!ret) {
//    ret = SPI_Read_NoCS(pdata->bus, pdata->device, data, 1);
//  }
//  SPI_Set_CS(pdata->bus, pdata->device, 1);
//  SPI_Read(ReadValue, 4, buf, sizeof(buf));
  ADF_SPI_XMIT(buf, ReadValue, 4, NULL, 0);
  
  *data = ReadValue[3];

  DBG(2, "%s :Exit\r\n", __func__);
  return ret;
}

static int
adf7242_write_reg(struct adf7242_platform_data *pdata,
                  uint16_t addr, uint8_t data)
{
  uint8_t buf[] = { CMD_SPI_MEM_WR(addr), addr, data };
  int ret;

  DBG(2, "%s :Enter\r\n", __func__);

  ret = adf7242_wait_ready(pdata);
  if(ret < 0) {
    return ret;
  }

//  ret = SPI_Write(pdata->bus, pdata->device, buf, sizeof(buf));
//  SPI_Write(buf, sizeof(buf), NULL, 0);
//  rf_spi_write_fast(buf, sizeof(buf));
  ADF_SPI_XMIT(buf, NULL, sizeof(buf), NULL, 0);

  DBG(2, "%s :Exit REG 0x%hX, VAL 0x%hhX\r\n", __func__,
      addr, data);
  return ret;
}

static int
adf7242_cmd(struct adf7242_platform_data *pdata, uint8_t cmd)
{
  int ret;

  DBG(2, "%s :Enter CMD=0x%X\r\n", __func__, cmd);
  ret = adf7242_wait_ready(pdata);
  if(ret < 0) {
    return ret;
  }

//  ret = SPI_Write(pdata->bus, pdata->device, &cmd, 1);
//  SPI_Write(&cmd, 1, NULL, 0);
  ADF_SPI_XMIT(&cmd, NULL, 1, NULL, 0);

  DBG(2, "%s :Exit\r\n", __func__);
  return ret;
}

static void
adf7242_irqwork(void)
{
  
}

void
p1_handler(void)
{
  /* Ack the interrupt on the CPU side */
  adf7242_irqwork();
}

void
p2_handler(void)
{
  /* Ack the interrupt on the CPU side */
  adf7242_irqwork();
}

static int
adf7242_verify_firmware(struct adf7242_platform_data *pdata,
                        uint8_t *data, uint16_t len)
{
  int ret, i, j;
  unsigned int page;
  uint8_t cmd[] = { CMD_SPI_PRAM_RD, 0, CMD_SPI_NOP, };
  uint8_t buf[PRAM_PAGESIZE + 3];

  for(page = 0, i = len; i >= 0; i -= PRAM_PAGESIZE, page++) {
    unsigned short nb = i >= PRAM_PAGESIZE ? PRAM_PAGESIZE : i;
    ret = adf7242_write_reg(pdata, REG_PRAMPG, page);
    if(ret < 0) {
      goto err;
    }

//    SPI_Set_CS(pdata->bus, pdata->device, 0);
//    ret = SPI_Write_NoCS(pdata->bus, pdata->device,
//                         cmd, sizeof(cmd));
//    if(!ret) {
//      ret = SPI_Read_NoCS(pdata->bus, pdata->device,
//                          buf, sizeof(buf));
//    }
//    SPI_Set_CS(pdata->bus, pdata->device, 1);
    
//    SPI_Read(buf, sizeof(buf), cmd, sizeof(cmd));
    
    ADF_SPI_XMIT(NULL, buf, sizeof(buf) + 3, cmd, sizeof(cmd));
    
    
    if(ret) {
      goto err;
    }

    for(j = 0; j < nb; j++) {
      if(buf[j] != data[page * PRAM_PAGESIZE + j]) {
//        DBG fprintf(stderr, "ERROR: Expected 0x%02hhX, got 0x%02hhX\r\n",
//                data[page * PRAM_PAGESIZE + j],
//                buf[j]);
        ret = -1;
        goto err;
      }
    }
  }

  return 0;

err:
//  DBG fprintf(stderr, "Error while uploading firmware: %s\r\n",
//          strerror(-ret));
  return ret;
}

static int
adf7242_upload_firmware(struct adf7242_platform_data *pdata,
                        uint8_t *data, uint16_t len)
{
  int ret, i;
  unsigned int page;
  uint8_t cmd[] = { CMD_SPI_PRAM_WR, 0, };

  for(page = 0, i = len; i >= 0; i -= PRAM_PAGESIZE, page++) {
    unsigned short nb = i >= PRAM_PAGESIZE ? PRAM_PAGESIZE : i;

    ret = adf7242_write_reg(pdata, REG_PRAMPG, page);
    if(ret < 0) {
      goto err;
    }

//    SPI_Set_CS(pdata->bus, pdata->device, 0);
//    ret = SPI_Write_NoCS(pdata->bus, pdata->device,
//                         cmd, sizeof(cmd));
//    if(!ret) {
//      ret = SPI_Write_NoCS(pdata->bus, pdata->device,
//                           &data[page * PRAM_PAGESIZE], nb);
//    }
//    SPI_Set_CS(pdata->bus, pdata->device, 1);
    
//    SPI_Write(&data[page * PRAM_PAGESIZE], nb, cmd, sizeof(cmd));
    ADF_SPI_XMIT(&data[page * PRAM_PAGESIZE], NULL, nb, cmd, sizeof(cmd));
    
    if(ret) {
      goto err;
    }
  }

  DBG(1, "Firmware uploaded\r\n");
  return 0;

err:
//  DBG fprintf(stderr, "Error while uploading firmware: %s\r\n",
//          strerror(-ret));
  return ret;
}

static int
adf7242_reset(struct adf7242_platform_data *pdata)
{
  uint32_t i = 100000;
  uint8_t ResetCmd = CMD_RC_RESET;
  
  ADF_SPI_XMIT(&ResetCmd, NULL, 1, NULL, 0);
  
  /* delay of 2ms*/  
  while(i)
  {
    i--;
  }

  return adf7242_cmd(pdata, CMD_RC_IDLE);

}

static int
adf7242_hw_init(struct adf7242_platform_data *pdata)
{
  int ret;
  uint8_t reg, tmp;
  IRQn_Type eIrq;

  DBG(2, "%s :Enter\r\n", __func__);

  ret = adf7242_reset(pdata);
  if(ret < 0) {
    goto err;
  }

  /* Verify that what we read from SPI is actually correct,
   * by reading register which reset values are known. */
  ret = adf7242_read_reg(adf7242_pdata, REG_CH_FREQ0, &reg);
  if(!ret && reg != 128) {
    ret = -1;
  }
  if(ret < 0) {
    goto err;
  }
  ret = adf7242_read_reg(adf7242_pdata, REG_CH_FREQ1, &reg);
  if(!ret && reg != 169) {
    ret = -1;
  }
  if(ret < 0) {
    goto err;
  }
  ret = adf7242_read_reg(adf7242_pdata, REG_CH_FREQ2, &reg);
  if(!ret && reg != 3) {
    ret = -1;
  }
  if(ret < 0) {
    goto err;
  }

  ret = adf7242_write_reg(pdata, REG_PKT_CFG, (1 << 2));
  if(ret < 0) {
    goto err;
  }

  if(pdata->mode) {
    DBG(1, "Firmware size: %u bytes\r\n", sizeof(adf7242_firmware));
    
    adf7242_write_reg(pdata, REG_BUFFERCFG, 0x00); //default setting
    
    ret = adf7242_upload_firmware(pdata,
                                  (uint8_t *)adf7242_firmware, sizeof(adf7242_firmware));
    if(ret < 0) {
      goto err;
    }

//    ret = adf7242_verify_firmware(pdata,
//                                  (uint8_t *)adf7242_firmware, sizeof(adf7242_firmware));
//    if(ret < 0) {
//      goto err;
//    }

    ret = adf7242_write_reg(pdata, REG_FFILT_CFG,
                            (ACCEPT_DATA_FRAMES |
                            (pdata->mode & ADF_IEEE802154_PROMISCUOUS_MODE
                             ? ACCEPT_ALL_ADDRESS : 0)));
    if(ret < 0) {
      goto err;
    }

    ret = adf7242_write_reg(pdata, REG_AUTO_TX1,
                            MAX_FRAME_RETRIES(pdata->max_frame_retries) |
                            MAX_CCA_RETRIES(pdata->max_cca_retries));
    if(ret < 0) {
      goto err;
    }

    ret = adf7242_write_reg(pdata, REG_AUTO_TX2,
                            CSMA_MAX_BE(pdata->max_csma_be) |
                            CSMA_MIN_BE(pdata->min_csma_be));
    if(ret < 0) {
      goto err;
    }

    ret = adf7242_write_reg(pdata, REG_AUTO_CFG,
                            (pdata->mode & ADF_IEEE802154_HW_AACK ?
                             RX_AUTO_ACK_EN : 0));
    if(ret < 0) {
      goto err;
    }
    
  
    /* Write 0x8D to MCR Register 0x3FB and write 0xCA to MCR Register 0x3FC. */
    adf7242_write_reg(pdata, 0x3FB, 0x8D); 
  
    adf7242_read_reg(pdata, 0x3FB, &tmp);
  
    adf7242_write_reg(pdata, 0x3FC, 0xCA);
  
    adf7242_read_reg(pdata, 0x3FC, &tmp);    

    ret = adf7242_write_reg(pdata, REG_PKT_CFG, (ADDON_EN | (1 << 2)));
    if(ret < 0) {
      goto err;
    }
  }

  ret = adf7242_write_reg(pdata, REG_EXTPA_MSC, 0xF1);
  if(ret < 0) {
    goto err;
  }

  ret = adf7242_write_reg(pdata, REG_RXFE_CFG, 0x1D);
  if(ret < 0) {
    goto err;
  }

  /* Max Power */
  adf7242_read_reg(pdata, REG_PA_CFG, &tmp);
  tmp &= ~PA_BRIDGE_DBIAS(~0);
  tmp |= PA_BRIDGE_DBIAS(13);
  adf7242_write_reg(pdata, REG_PA_CFG, tmp);

  adf7242_read_reg(pdata, REG_PA_BIAS, &tmp);
  tmp &= ~PA_BIAS_CTRL(~0);
  tmp |= PA_BIAS_CTRL(55);
  adf7242_write_reg(pdata, REG_PA_BIAS, tmp);

  adf7242_read_reg(pdata, REG_EXTPA_MSC, &tmp);
  tmp &= ~PA_PWR(~0);
  tmp |= PA_PWR(15);
  ret = adf7242_write_reg(pdata, REG_EXTPA_MSC, tmp);

  ret = adf7242_write_reg(pdata, REG_IRQ1_EN0, 0);
  if(ret < 0) {
    goto err;
  }

  ret = adf7242_write_reg(pdata, REG_IRQ1_EN1,
                    IRQ_RX_PKT_RCVD | adf7242_tx_irq(pdata));
  if(ret < 0) {
    goto err;
  }

  ret = adf7242_write_reg(pdata, REG_IRQ1_SRC1, 0xFF);
  if(ret < 0) {
    goto err;
  }

  ret = adf7242_write_reg(pdata, REG_IRQ1_SRC0, 0xFF);
  if(ret < 0) {
    goto err;
  }

  ret = adf7242_cmd(pdata, CMD_RC_PHY_RDY);
  if(ret < 0) {
    goto err;
  }
#ifdef RFMODULE_7242
    #define GPIO_PORT   ADI_GPIO_PORT0
    #define GPIO_PIN    ADI_GPIO_PIN_15
#else	//#ifdef ADCM3029_EZKIT_USED
    #define GPIO_PORT   ADI_GPIO_PORT2
    #define GPIO_PIN    ADI_GPIO_PIN_3
#endif
   /* init the GPIO service */
  adi_gpio_InputEnable(GPIO_PORT, GPIO_PIN, true);
  
  adi_gpio_SetGroupInterruptPolarity(GPIO_PORT, GPIO_PIN);
  
  /* Enable pin interrupt on group interrupt A */
  adi_gpio_SetGroupInterruptPins(GPIO_PORT, SYS_GPIO_INTA_IRQn, GPIO_PIN);
  
  eIrq = SYS_GPIO_INTA_IRQn;
  /* Register the callback */
  adi_gpio_RegisterCallback (SYS_GPIO_INTA_IRQn, ADF7242CallBack, (void*)&eIrq);
  
  ADI_ENABLE_INT(SYS_GPIO_INTA_IRQn);
  
  DBG(1, "ADF7242 successfully initialized.\r\n");
  DBG(2, "%s :Exit\r\n", __func__);
  return 0;

err:
//  DBG fprintf(stderr, "Error initializing hardware: %s\r\n", strerror(-ret));
  return ret;
}

static int
adf7242_init(void)
{
  int ret;
  
#ifdef WIRESHARK_FEEDER  
  adf7242_pdata->mode |= ADF_IEEE802154_PROMISCUOUS_MODE;
#endif
    
  if(adf7242_pdata->mode & ADF_IEEE802154_PROMISCUOUS_MODE) {
    adf7242_pdata->mode &= ~ADF_IEEE802154_HW_AACK;
  }

  DBG(1, "Initializing SPI bus=%u device=%u\r\n",
      adf7242_pdata->bus, adf7242_pdata->device);

  /* MSB first, 2 MHz, idle at low level, serial output changes on
   * low to high front */
  if(g_node_type == NODE_6LN) {
    if(!bSpiInitialized) {
      ret = SPI_Init(adf7242_pdata->bus, 4000000);
      bSpiInitialized = true;
    }
    else {
     ret = 1;
    }
  }
  else {
    ret = SPI_Init(adf7242_pdata->bus, 4000000);
  }
 
  if(ret < 0) {
//  DBG    fprintf(stderr, "Error initializing SPI: %s\r\n",
//            strerror(-ret));
    return ret;
  }
#ifndef WIRESHARK_FEEDER  
  process_start(&aducm3025_radio_process, NULL);
#endif  
  
  LEDControl(0,1);
  LEDControl(1,1);
  LEDControl(2,1);
  
  ret = adf7242_hw_init(adf7242_pdata);
  
  uint8_t status = 0;
  
  adf7242_status(adf7242_pdata, &status);
  
  mPHYState = status & 0x0F;
  
  LEDControl(0,0);
  LEDControl(1,0);
  LEDControl(2,0);
  
  SysTick_Config(MAX_SYS_TICK_COUNT);
#if ((DEV_TYPE == DEV_TYPE_HOST) || (DEV_TYPE == DEV_TYPE_EDGE_ROUTER) || (DEV_TYPE == DEV_TYPE_ROUTER))
  adf7242_on();
#endif // ((DEV_TYPE == DEV_TYPE_HOST) || (DEV_TYPE == DEV_TYPE_EDGE_ROUTER) || (DEV_TYPE == DEV_TYPE_ROUTER))

  return ret ? ret : 1;
}

static int
adf7242_reinit(void)
{
  int ret;
  ADI_DISABLE_INT(SYS_GPIO_INTA_IRQn);
  
  ret = adf7242_hw_init(adf7242_pdata);
  
  uint8_t status = 0;
  
  adf7242_status(adf7242_pdata, &status);
  
  mPHYState = status & 0x0F;
  
  if(ret == -1)
  {
    GlowAllLEDs();
    while(1);
    return ret;
  }
   update_phy_params();

  return ret ? ret : 1;
}

/*
* @brief        This function returns whether TRX is busy in Tx/Rx.
* @params       None
* @returns      1, if TRX is busy in Tx/Rx.
*               0, if TRX is not busy.
*/
uint8_t  ADF7242_is_trx_busy(void)
{
  uint8_t rc_status;
  uint8_t rx_status;

  /* Read current state of TRX. */
  adf7242_status(adf7242_pdata, &rc_status);
  if ((rc_status & RC_TX_MASK) == RC_TX_MASK) {
    /* ADF7242 is busy in transmitting a packet. */
    return 1;
  }
  else if((rc_status & RC_RX_MASK) == RC_RX_MASK) {
    /* ADF7242 is in receiver mode check is it busy in receiving a packet.*/
    adf7242_read_reg(adf7242_pdata, REG_802_15_4_STATE, &rx_status);
    if((rx_status & DS_15d4_STATE_EXTRACT_DATA) == DS_15d4_STATE_EXTRACT_DATA){
      /* ADF7242 is busy in receiving a packet. */
      return 1;
    }
  }
  return 0;
}

static int
adf7242_send(const void *buf, unsigned short len)
{
#ifdef RADIO_TX_LED
  tx_led ^= 1;
  RADIO_TX_LED = tx_led;
#endif

  uint8_t lqi;
  uint8_t irq_status;
  
  /* This is to protect access of SPI from radio irq handler.*/
  Disable_Transceiver_Interrupts();
  /* Check whether the TRX busy or not. */
  if(ADF7242_is_trx_busy()){
      tx_status = RADIO_TX_COLLISION;
      Enable_Transceiver_Interrupts();
      return RADIO_TX_COLLISION;
  }
  /* If TRX is not busy then enter into PHY_RDY state to start tranmission process*/
  if(adf7242_pdata->mode & ADF_IEEE802154_AUTO_CSMA_CA){
    adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);
  }
  
  /* Check whether we received any rx_eof interrupt, If received then return to 
  handle received packet.*/
  adf7242_read_reg(adf7242_pdata, REG_IRQ1_SRC1, &irq_status);
  if((irq_status & IRQ_RX_PKT_RCVD)) {
    tx_status = RADIO_TX_COLLISION;
    Enable_Transceiver_Interrupts();
    return RADIO_TX_COLLISION;
  }
  Enable_Transceiver_Interrupts();
  
  tx_status = -1;
  
  DBG(1, "Sending packet with size %hu...\r\n", len);
  
  
  SetLastAckFailureCount();

  adf7242_write_fbuf(adf7242_pdata, (uint8_t *)buf, len);

  if(adf7242_pdata->mode & ADF_IEEE802154_AUTO_CSMA_CA) {
    
    if(pkt_Rx_start_time)
    {
      //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
      pkt_Rx_start_time = 0;
    }
    
    adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);
    adf7242_cmd(adf7242_pdata, CMD_RC_CSMACA);
    pkt_Tx_start_time = TickCounter;
  } else {
    uint8_t status = 0;
    
    if(pkt_Rx_start_time)
    {
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
      pkt_Rx_start_time = 0;
    }
    
    adf7242_cmd(adf7242_pdata, CMD_RC_TX);
    
    pkt_Tx_start_time = TickCounter;
  
    adf7242_status(adf7242_pdata, &status);
    
    mPHYState = status & 0x0F;
  }
  
  /* start timer for time required to transmit maximum packet size (2048 bytes) */
  pkt_transmit_err_timeout = PACKET_RCV_TIME + TickCounter;
  pkt_transmit_err_timeout_set = true;  //debug-HP
  
  /* Wait for the transfer to complete */
  while((tx_status < 0) && pkt_transmit_err_timeout_set) ;
  
  pkt_transmit_err_timeout_set = false; //debug-HP
   
  if(tx_status == RADIO_TX_OK)
  {
    if(TxState.Ack_Req)
    {
        volatile int timeout = 1000000;
        
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
  else if(tx_status == RADIO_TX_NOACK)
  {
    return RADIO_TX_NOACK;
  }

  return RADIO_TX_COLLISION;
}

static int
adf7242_prepare(const void *buf, unsigned short len)
{
  memcpy(tx_buf, buf, len < sizeof(tx_buf) ? len : sizeof(tx_buf));
  return RADIO_TX_OK;
}

static int
adf7242_transmit(unsigned short len)
{
  return adf7242_send(tx_buf, len);
}

static int
adf7242_read(void *buf, unsigned short buf_len)
{
  uint8_t lqi, len = 5;
  int ret;

#ifdef RADIO_RX_LED
  rx_led ^= 1;
  RADIO_RX_LED = rx_led;
#endif
  
  ret = adf7242_read_fbuf(adf7242_pdata, buf, &len, &lqi);
  
  len = *((uint8_t*)buf) + 1;

  ret = adf7242_read_fbuf(adf7242_pdata, buf, &len, &lqi);
  
  adf7242_cmd(adf7242_pdata, CMD_RC_RX);
  
  if(pkt_Rx_start_time)
  {
    //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
    rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
  }
  
  pkt_Rx_start_time = TickCounter;
  
  adf7242_lqi = lqi;
  
  rf_stats.RSSI_val = lqi;

#ifndef WIRESHARK_FEEDER  
  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rf_stats.RSSI_val);
#endif
  
  DBG(1, "Received a packet of %hhu bytes\r\n", len - 2);
  return ret < 0 ? ret : len - 2;
}

static int
adf7242_channel_clear(void)
{
  /* TODO */
  return 1;
}

static int
adf7242_receiving_packet(void)
{
  unsigned char status;
  int ret;

  ret = adf7242_status(adf7242_pdata, &status);
  if(ret < 0) {
    return ret;
  }

  return ((status & 0xf) == RC_STATUS_RX) && !(status & STAT_RC_READY);
}

static int
adf7242_pending_packet(void)
{
  int ret = (int)packet_pending;
  packet_pending = false;
  return ret;
}

static radio_result_t
adf7242_get_value(radio_param_t param,
                  radio_value_t *value)
{
  switch(param) {
  case RADIO_CONST_CHANNEL_MIN:
    *value = 11;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MAX:
    *value = 26;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CHANNEL: {
    uint32_t freq;
    uint8_t byte0, byte1, byte2, status;
    int ret;

    adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);
    do {
      ret = adf7242_status(adf7242_pdata, &status);
      if(ret < 0) {
        return RADIO_RESULT_ERROR;
      }
    } while(!(status & (RC_STATUS_PHY_RDY | RC_STATUS_IDLE)));

    if(adf7242_read_reg(adf7242_pdata, REG_CH_FREQ0, &byte0) < 0 ||
       adf7242_read_reg(adf7242_pdata,
                        REG_CH_FREQ1, &byte1) < 0 ||
       adf7242_read_reg(adf7242_pdata,
                        REG_CH_FREQ2, &byte2) < 0) {
      return RADIO_RESULT_ERROR;
    }

    freq = byte0 | ((uint32_t)byte1 << 8)
      | ((uint32_t)byte2 << 16);
    *value = (freq / 100UL - 2405UL) / 5UL + 11UL;

    adf7242_cmd(adf7242_pdata, CMD_RC_RX);
    
    if(pkt_Rx_start_time)
    {
      //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
    }
    
    pkt_Rx_start_time = TickCounter;
    
    return RADIO_RESULT_OK;
  }
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}

static radio_result_t
adf7242_set_value(radio_param_t param,
                  radio_value_t value)
{
  switch(param) {
  case RADIO_PARAM_CHANNEL: {
    uint32_t freq;
    uint8_t byte0, byte1, byte2;

    if(value < 11 || value > 26) {
      return RADIO_RESULT_INVALID_VALUE;
    }

    /* NOTE: I do this in three steps, otherwise it doesn't
     * calculate anything. Hurray for toolchain bugs. */
    freq = value - 11;
    freq = freq * 5UL + 2405UL;
    freq *= 100UL;

    byte0 = (uint8_t)freq;
    byte1 = freq >> 8;
    byte2 = freq >> 16;

    adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);

    if(adf7242_write_reg(adf7242_pdata, REG_CH_FREQ0, byte0) < 0 ||
       adf7242_write_reg(adf7242_pdata,
                         REG_CH_FREQ1, byte1) < 0 ||
       adf7242_write_reg(adf7242_pdata,
                         REG_CH_FREQ2, byte2) < 0) {
      return RADIO_RESULT_ERROR;
    }
    
    adf7242_on();
    
    DBG(1, "Switching to channel %u (%lu0 kHz)\r\n",
        (unsigned int)value, freq);
    return RADIO_RESULT_OK;
  }
  case RADIO_PARAM_PAN_ID: {
    uint16_t pan_id = value;

    adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);

    adf7242_write_reg(adf7242_pdata, REG_PAN_ID0, pan_id);
    adf7242_write_reg(adf7242_pdata, REG_PAN_ID1, pan_id >> 8);
    adf7242_cmd(adf7242_pdata, CMD_RC_RX);
    
    if(pkt_Rx_start_time)
    {
      //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
    }
    
    pkt_Rx_start_time = TickCounter;
    
    return RADIO_RESULT_OK;
  }
  case RADIO_PARAM_16BIT_ADDR: {
    uint16_t addr = value;

    adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);

    adf7242_write_reg(adf7242_pdata, REG_SHORT_ADDR_0, addr);
    adf7242_write_reg(adf7242_pdata, REG_SHORT_ADDR_1, addr >> 8);
    adf7242_cmd(adf7242_pdata, CMD_RC_RX);
    
    if(pkt_Rx_start_time)
    {
      //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
    }
    
    pkt_Rx_start_time = TickCounter;    
    
    return RADIO_RESULT_OK;
  }
  default:
    DBG(1, "%s tried to set param %u, but failed\r\n",
        __func__, param);
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}

int
adf7242_off(void)
{
  int ret = adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);
  uint8_t status = 0;
  
  adf7242_status(adf7242_pdata, &status);
  
  mPHYState = status & 0x0F;
  
  if(ret < 0) {
    return RADIO_RESULT_ERROR;
  } else {
    return RADIO_POWER_MODE_OFF;
  }
}

int RxErrCount = 0, RxCount = 0;

int
adf7242_on(void)
{
  int ret = adf7242_cmd(adf7242_pdata, CMD_RC_RX);
  uint8_t status = 0;
  uint16_t loopCount = 0;
  
  if(pkt_Rx_start_time)
  {
    //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
    rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
  }
  
  do
  {
    pkt_Rx_start_time = TickCounter;
    
    adf7242_status(adf7242_pdata, &status);
    
    mPHYState = status & 0x0F;
    
    loopCount++;
    
  }while((mPHYState != PHY_RX) && (loopCount < 10000));
  
  if(loopCount == 10000)
  {
    RxErrCount++;
  }
  
  if(ret < 0) {
    return RADIO_RESULT_ERROR;
  } else {
    return RADIO_POWER_MODE_ON;
  }
}

static radio_result_t
adf7242_get_object(radio_param_t param,
                   void *dest, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}

static radio_result_t
adf7242_set_object(radio_param_t param,
                   const void *src, size_t size)
{
  switch(param) {
  case RADIO_PARAM_64BIT_ADDR: {
    const char *addr = (const char *)src;

    adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);

    adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_0, addr[7]);
    adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_1, addr[6]);
    adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_2, addr[5]);
    adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_3, addr[4]);
    adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_4, addr[3]);
    adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_5, addr[2]);
    adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_6, addr[1]);
    adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_7, addr[0]);
    adf7242_cmd(adf7242_pdata, CMD_RC_RX);
    
    if(pkt_Rx_start_time)
    {
      //Each tick is 4ms and we were in PHY_RX for (TickCounter - pkt_Rx_start_time) ticks.
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
    }
    
    pkt_Rx_start_time = TickCounter;
    
    return RADIO_RESULT_OK;
  }
  default:
    DBG(1, "%s tried to set object %u, but failed\r\n",
        __func__, param);
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}

const struct radio_driver aducm3029_radio_driver = {
  .init = adf7242_init,
  .send = adf7242_send,
  .prepare = adf7242_prepare,
  .transmit = adf7242_transmit,
  .read = adf7242_read,
  .channel_clear = adf7242_channel_clear,
  .receiving_packet = adf7242_receiving_packet,
  .pending_packet = adf7242_pending_packet,
  .on = adf7242_on,
  .off = adf7242_off,
  .get_value = adf7242_get_value,
  .set_value = adf7242_set_value,
  .get_object = adf7242_get_object,
  .set_object = adf7242_set_object,
};

ADI_SPI_RESULT SPI_Init(uint8_t DevNum,
              uint32_t clockFreq)
{
  
  ADI_SPI_RESULT eResult = ADI_SPI_SUCCESS;
  ADI_GPIO_RESULT eResultGPIO = ADI_GPIO_SUCCESS;
   
  if(ADI_SPI_SUCCESS == eResult)
  {
    eResult = adi_spi_Open(DevNum, spidevicemem, ADI_SPI_MEMORY_SIZE, &hDevice);
  }
#if (ADI_SPI_CFG_ENABLE_STATIC_CONFIG_SUPPORT != 1)  
  if(ADI_SPI_SUCCESS == eResult)
  {
    eResult = adi_spi_SetBitrate(hDevice, clockFreq);
  }
  
  if(ADI_SPI_SUCCESS == eResult)
  {
    eResult = adi_spi_SetContinousMode(hDevice, true);
  }
  
  if(ADI_SPI_SUCCESS == eResult)
  {
    eResult = adi_spi_EnableDmaMode(hDevice, false);
  }
#endif  //(ADI_SPI_CFG_ENABLE_STATIC_CONFIG_SUPPORT != 1)  
  
  if(ADI_SPI_SUCCESS == eResult)
  {
#ifdef RFMODULE_7242
	eResult = adi_spi_SetChipSelect (hDevice, ADI_SPI_CS0);
#else
    eResult = adi_spi_SetChipSelect (hDevice, ADI_SPI_CS3);
#endif
  }
                
 return eResult;
}

unsigned char ADF_SPI_XMIT(uint8_t *TxData, uint8_t *RxData, uint16_t DataLen, uint8_t *Prologue, uint16_t PrologueSize)
{
  /* SPI transceiver instance */
  ADI_SPI_TRANSCEIVE_TYPE Transceiver;
  uint8_t nTemp = 0xFF;
  uint8_t size = 1;

  /* Initialize the transceiver */
  if(RxData)
  {
      Transceiver.pRxData           =   RxData;
      Transceiver.bRxIncrement      =   1;
  }
  else
  {
      Transceiver.pRxData           =   &nTemp;
      Transceiver.bRxIncrement      =   0;
  }

  if(TxData)
  {
      Transceiver.pTxData           =   TxData;
      Transceiver.bTxIncrement      =   1;
  }
  else
  {
      Transceiver.pTxData           =   &nTemp;
      Transceiver.bTxIncrement      =   0;
  }

  Transceiver.pPrologue             =   Prologue;
  Transceiver.PrologueSize          =   PrologueSize;
  Transceiver.DataSize              =   DataLen;
#ifdef RFMODULE_7242
  adi_spi_SetChipSelect (hDevice, ADI_SPI_CS0);
#else
  adi_spi_SetChipSelect (hDevice, ADI_SPI_CS3);
#endif
  /* Transmit the first sequence */
  hDevice->bBlockingMode = true;
  if(adi_spi_MasterRadioTx(hDevice, &Transceiver) != ADI_SPI_SUCCESS)
  {
      hDevice->bBlockingMode = false;
      return 1;
  }
  
  hDevice->bBlockingMode = false;
  return 0; 
}


//void rf_spi_write_fast(uint8_t *pTxbuf, uint8_t txlen)
//{
//  uint8_t              i;
//  uint8_t               byte;
//  
//  volatile  uint16_t nFifoStatus;
//  uint16_t nErrorStatus;
//  volatile uint16_t nStatus;
//
//  ADI_DISABLE_INT(SPI2_EVT_IRQn);
//  
//  pADI_SPI2->CTL &= (uint16_t)~((uint16_t)BITM_SPI_CTL_TIM | 
//                                (uint16_t)BITM_SPI_CTL_RFLUSH | 
//                                (uint16_t)BITM_SPI_CTL_TFLUSH);
//  
//  /* clear any residual */
//  //    nStatus = pADI_SPI2->STAT;
//  //    pADI_SPI2->STAT = nStatus;
//  
//  pADI_SPI2->CTL |= BITM_SPI_CTL_MASEN;
//  
//  /* Assert CS */
//  pADI_SPI2->CS_CTL = ADI_SPI_CS3;
//  
//  pADI_SPI2->CNT = txlen;
//  
//  // FIXME: maybe check FIFO status here? For now assume FIFO is empty
//  for (i = 0; i < txlen; i++) {
//    pADI_SPI2->TX = pTxbuf[i];
//  }
//  
//  byte = pADI_SPI2->RX;
//  
//  // nFifoStatus = pADI_SPI2->FIFO_STAT;
//  // nErrorStatus = pADI_SPI2->STAT;
//  
//  /* termination */
//  while (!(BITM_SPI_STAT_XFRDONE == (pADI_SPI2->STAT &BITM_SPI_STAT_XFRDONE)))
//    ;
//  
//  /* Clear XFRDONE */
//  pADI_SPI2->STAT = BITM_SPI_STAT_XFRDONE;
//  
//  /* must disable SPI here, otherwise we get and an unwanted tx fifo transfer 
//  after disabling CS */
//  pADI_SPI2->CTL &= (uint16_t)~(  (uint16_t)BITM_SPI_CTL_MASEN
//                                | (uint16_t)BITM_SPI_CTL_RFLUSH
//                                | (uint16_t)BITM_SPI_CTL_TFLUSH);
//  
//  /* Deassert CS */
//  pADI_SPI2->CS_CTL &= (uint16_t)~((uint16_t)ADI_SPI_CS3);
//  
//  NVIC_ClearPendingIRQ(SPI2_EVT_IRQn);
//  ADI_ENABLE_INT(SPI2_EVT_IRQn);
//  
//}
volatile uint8_t g_frame_valid = 0;
volatile uint8_t g_address_valid = 0;
void ADF7242CallBack(void      *pCBParam,         /*!< Client supplied callback param */
                      uint32_t   Event,            /*!< Event ID specific to the Driver/Service */
                      void      *pArg)
{
  int ret = RADIO_TX_OK;
  uint8_t irq, stat, status;
  
  while(1){ 
    status = 0;
    adf7242_status(adf7242_pdata, &status);
    
    mPHYState = status & 0x0F;
    
    irq = 0;
    adf7242_read_reg(adf7242_pdata, REG_IRQ1_SRC1, &irq);
    
    if(!irq){
      break;
    }
    
    /* Ack the interrupt on the module side */
    adf7242_write_reg(adf7242_pdata, REG_IRQ1_SRC1, irq);
    
    if(irq & IRQ_SFD_RX){
      g_frame_valid = 0;
      g_address_valid = 0;
    }
    if(irq & IRQ_FRAME_VALID){
      g_frame_valid = 1;
    }
    if(irq & IRQ_ADDRESS_VALID){
      g_address_valid = 1;
    }
    
    if((irq & IRQ_RX_PKT_RCVD) && g_frame_valid && g_address_valid) {
      g_frame_valid = 0;
      g_address_valid = 0;
      if(pkt_Rx_start_time)
      {
        rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
        pkt_Rx_start_time = 0;
      }
      
      /* Wait until ACK is processed */
      if((adf7242_pdata->mode & ADF_IEEE802154_HW_AACK) &&
         ((status & RC_STATUS_MASK) != RC_STATUS_PHY_RDY)) {
           adf7242_wait_status(adf7242_pdata, RC_STATUS_PHY_RDY);
         }
      rf_stats.Rx_EOF++;
      packet_pending = true;
#ifndef WIRESHARK_FEEDER
      adi_ADF7023_ReceiveIsrCallback();
#else
      adi_packet_recvd();
#endif
    }
    
    if(irq & adf7242_tx_irq(adf7242_pdata)) {
      adf7242_read_reg(adf7242_pdata, REG_AUTO_STATUS, &stat);
      stat &= AUTO_STATUS_MASK;
      
      rf_stats.pkt_Tx_time += ((TickCounter - pkt_Tx_start_time));
      pkt_Tx_start_time = 0;
      
      if(stat == FAILURE_NOACK) {
        ret = RADIO_TX_NOACK;
        rf_stats.Count_ACK_Missed++;
      } else if(stat == FAILURE_CSMACA) {
        ret = RADIO_TX_COLLISION;
        rf_stats.CCA_Failed++;
      } else if((stat == SUCCESS) || (stat == SUCCESS_DATPEND)){
        ret = RADIO_TX_OK;
        rf_stats.Tx_EOF++;
      }
      else {
        ret = RADIO_TX_ERR;
      }
      
      tx_status = ret;
      adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);
      
      do{
        adf7242_status(adf7242_pdata, &status);
        
        mPHYState = status & 0x0F;
      }while(mPHYState != PHY_RDY);
      
      adf7242_cmd(adf7242_pdata, CMD_RC_RX);
      
      do{
        adf7242_status(adf7242_pdata, &status);
        
        mPHYState = status & 0x0F;
      }while(mPHYState != PHY_RX);
      
      pkt_Rx_start_time = TickCounter;
    }
  }
}
#ifndef WIRESHARK_FEEDER
void adi_ADF7023_ReceiveIsrCallback ( )
{
  uint16_t len;
  uint8_t *pcktBuffPtr = (uint8_t *)packetbuf_dataptr();
  
  /* Toggle LED3 to indicate reception of a packet. */
  if(g_node_type != NODE_6LN){
    adi_gpio_Toggle(ADI_GPIO_PORT0,ADI_GPIO_PIN_13);
  }

  NETSTACK_RADIO.read(radio_buffer, 256);
  

  
  if(radio_buffer[0] > 2)
  {
    len = radio_buffer[0] - 2;
  
    memcpy(packetbuf_dataptr(), (radio_buffer + 1), len);
    
    if(len > 0) 
    {
      packetbuf_set_datalen(len);
      
      if(radio_buffer[1] & 0x02)
      {
        Set_Ack_Done(0x01);
        Set_Rx_Seq_No(radio_buffer[3]);
      }
      
      process_poll(&aducm3025_radio_process);
    }
  }
  else
  {
    len = 0; 
  }
}
#endif
/*---------------------------------------------------------------------------*/
#ifndef WIRESHARK_FEEDER
PROCESS_THREAD(aducm3025_radio_process, ev, data)
{
  int16_t len;
  
  PROCESS_BEGIN();
  
  while(1) {
    
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
  
    len = radio_buffer[0] - 2;
    
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
      
      packetbuf_set_datalen(len);
      NETSTACK_RDC.input();
      
#endif //WIRESHARK_FEEDER
    }
  }
  
  PROCESS_END();
}
#endif

ADI_SPI_HANDLE get_spi_handle()
{
  return hDevice;
}

/*----------------------------------------------------------------------------*/

/*!
 * @brief  Put ADF7242 in RC_SLEEP state.
 *
 * @param   None.
 *
 * @return  None.
 */
int8_t ADF_PwrMgt(uint8_t Mode)
{
  if(Mode == TRPS_DOWN)
  {
    if(pkt_Rx_start_time)
    {
      rf_stats.pkt_Rx_time += ((TickCounter - pkt_Rx_start_time));
      pkt_Rx_start_time = 0;
    }
    adf7242_cmd(adf7242_pdata, CMD_RC_IDLE);
    /*From any state by issuing RC_SLEEP command ADF7242 enters into sleep state*/
    adf7242_cmd(adf7242_pdata, CMD_RC_SLEEP);
    return 0;
  }
  else if(Mode == TRPS_UP)
  {
    //if (adi_gpio_DisableExIRQ(XINT_EVT0_IRQn)) {
    //}     
    adf7242_hw_init(adf7242_pdata);
    return 0;
  }
  
  return -1;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Wake up from sleep state
 *
 * @param   None.
 *
 * @return  None.
 */
void adf7242_wakeup_from_sleep(void)
{
//  /* As per ADF7242 data sheet: The host MCU can bring CS low at any time to 
//  wake the ADF7242 from the sleep state. */
//  /* As per AN-1082: PRAM block is volatile memory and must be reloaded each 
//  time the tranceiver is waking up from sleep state.*/
//    /* Initialize configured PHY mode */
//  ADF7242_PHY_init(TRX_config_params.phy_mode);
//  
//  /* Initialize channel frequency */
//  ADF7242_set_channel(TRX_config_params.channelFreq);
//     
//  /* Initialize TRX to ready state */
//  ADF7242_issue_command(CMD_RC_PHY_RDY);
//  ADF7242_wait_state(RC_PHY_RDY_MASK);
}

void SysTick_Handler(void)
{
  TickCounter++;
  if((pkt_transmit_err_timeout_set) && (pkt_transmit_err_timeout == TickCounter))
  {
    int init_status;
    tx_status = RADIO_TX_ERR;
    rf_stats.Tx_timeout_re_init_count++;
    do{
        init_status = adf7242_reinit();
      }while(init_status < 0);
    pkt_transmit_err_timeout_set = false;
  } 
}

/*---------------------------------------------------------------------------*/

void Enable_Transceiver_Interrupts( )
{
    NVIC_EnableIRQ(SYS_GPIO_INTA_IRQn);
}

/*---------------------------------------------------------------------------*/

void Disable_Transceiver_Interrupts( )
{
    NVIC_DisableIRQ(SYS_GPIO_INTA_IRQn);
}

/*---------------------------------------------------------------------------*/

#ifndef WIRESHARK_FEEDER

int send_ack(uint8_t seq_no)
{ 
  uint16_t payload_len = 3;
  ACK[2] = seq_no;
  bAckTxProgress = 1;
  
  return adf7242_send(ACK, 3);
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

uint32_t LastAckFailureCount = 0;

bool Get_Ack_Status()
{
  return(LastAckFailureCount != rf_stats.Count_ACK_Missed)?false:true;
}

void SetLastAckFailureCount()
{
  LastAckFailureCount = rf_stats.Count_ACK_Missed;
}

#else

void adf7242_get_channel(int *channel)
{
  adf7242_get_value(RADIO_PARAM_CHANNEL, channel);
}

void adf7242_set_channel(int channel)
{
  adf7242_set_value(RADIO_PARAM_CHANNEL, channel);
}

int ADF_RadioInit(unsigned char ProfileNo, int ISMBand, uint32_t Freq, uint8_t InitParm)
{
  adf7242_init();
}

void adi_packet_recvd ( )
{
  adf7242_read(radio_buffer, 256);
  
  adi_ADF7023_ReceiveIsrCallback(radio_buffer);
  
}
#endif

int Retireve_rf_statistics(unsigned char *buf, unsigned int Bufpos, int len)
{
  if(Is_StatisticsEnabled())
  {  
    buf[Bufpos++] = 'S'; // start of statistics

    Bufpos = AddLongToBuffer(rf_stats.Tx_EOF + rf_stats.Rx_EOF , buf, Bufpos, len);                             // Long 0 - 5
    Bufpos = AddLongToBuffer(rf_stats.CRC_failure + rf_stats.CCA_Failed, buf, Bufpos, len);                     // Long 1 - 5

    Bufpos = AddLongToBuffer(clock_seconds(), buf, Bufpos, len);                                                // Int 0 - 3
    Bufpos = AddLongToBuffer((rf_stats.pkt_Tx_time * 4) /1000 , buf, Bufpos, len);                              // Int 1 - 3
    Bufpos = AddLongToBuffer((rf_stats.pkt_Rx_time * 4) /1000 , buf, Bufpos, len);                              // Int 2 - 3
    Bufpos = AddLongToBuffer((rf_stats.cca_time  * 4)   /1000 , buf, Bufpos, len);                              // Int 2 - 3
    Bufpos = AddLongToBuffer(rf_stats.CCA_Failed, buf, Bufpos, len);                                            // Int 3 - 3
    //#ifdef ENABLE_LOW_POWER_MODE  
    Bufpos = AddLongToBuffer(system_hibernate_duration, buf, Bufpos, len);                                      // Int 3 - 3
    //#endif    
    Bufpos = AddLongToBuffer(system_flexi_mode_sleep_duration, buf, Bufpos, len);                               // Int 3 - 3

    //Active Up-Channel Index
    Bufpos = AddByteToBuffer(phy_current_channel,buf, Bufpos, len);                                             // Int 3 - 3

    //RSSI pack
    Bufpos = AddByteToBuffer(rf_stats.RSSI_val, buf, Bufpos, len);                                              // Byte 1 - 2  

    //Transmit Power
    Bufpos = AddByteToBuffer(transmit_pwr, buf, Bufpos, len);                                                   // Int 3 - 3

    //Rx Repeats Stat
    Bufpos = AddByteToBuffer(0, buf, Bufpos, len);                                                              // Int 3 - 3

    //CCA Treshold
    Bufpos = AddByteToBuffer(-70, buf, Bufpos, len);                                                            // Byte 1 - 2

    //BatVolt
    Bufpos = AddWordToBuffer(0, buf, Bufpos, len); 
  }
  
  return Bufpos;
}
//#ifdef UL_NODE
//const struct rdc_driver sicslowmac_driver = {
//  "sicslowmac",
//  init,
//  NULL,
//  NULL,
//  input_packet,
//  on,
//  off,
//  channel_check_interval
//};
//#endif