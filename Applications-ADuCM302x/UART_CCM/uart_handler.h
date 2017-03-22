
#ifndef __UART_HANDLER_H__
#define __UART_HANDLER_H__

#include <stdbool.h>

#include "contiki.h"
#include "net/ip/uip.h"
#include <spi/adi_spi.h>
#include "ADF7242.h"

#include <uart/adi_uart.h>
/*@}*/

                                                  /*
                    Boudrate divider for PCLK-26000000

+------------------------------------------------------------------------+
| CALCULATING UART DIV REGISTER VALUE FOR THE  INPUT CLOCK: 26000000     |
|------------------------------------------------------------------------|
|       BAUDRATE       DIV-C     DIV-M     DIV-N         OSR    DIFF     |
|------------------------------------------------------------------------|
|       00009600        0022      0003      1734        0003    0000     |
|------------------------------------------------------------------------|
|       00019200        0011      0003      1735        0003    0000     |
|------------------------------------------------------------------------|
|       00038400        0017      0001      0501        0003    0000     |
|------------------------------------------------------------------------|
|       00057600        0007      0002      0031        0003    0000     |
|------------------------------------------------------------------------|
|       00115200        0007      0002      0031        0002    0000     |
|------------------------------------------------------------------------|
|       00230400        0007      0002      0031        0001    0000     |
|------------------------------------------------------------------------|
|       00460800        0007      0002      0031        0000    0001     |
|------------------------------------------------------------------------|


*/

/* Select the boudrate divider for 57600 */
#define UART_DIV_C_9600         22
#define UART_DIV_C_19200        11
#define UART_DIV_C_38400        17
#define UART_DIV_C_57600        7
#define UART_DIV_C_115200       7
#define UART_DIV_C_230400       7
#define UART_DIV_C_460800       7

#define UART_DIV_M_9600         3
#define UART_DIV_M_19200        3
#define UART_DIV_M_38400        1
#define UART_DIV_M_57600        2
#define UART_DIV_M_115200       2
#define UART_DIV_M_230400       2
#define UART_DIV_M_460800       2

#define UART_DIV_N_9600         1734
#define UART_DIV_N_19200        1735
#define UART_DIV_N_38400        501
#define UART_DIV_N_57600        31
#define UART_DIV_N_115200       31
#define UART_DIV_N_230400       31
#define UART_DIV_N_460800       31

#define UART_OSR_9600           3
#define UART_OSR_19200          3
#define UART_OSR_38400          3
#define UART_OSR_57600          3
#define UART_OSR_115200         2
#define UART_OSR_230400         1
#define UART_OSR_460800         0

#define DEMOAPP_USER_INPUT_LENGTH        256
#define STATE_UART_LENGTH                1
#define STATE_CMD_PARAM                  2

#define DEFAULT_PAN_ID       0xABCD

#define ANodeVersion         "AD6LP02  svn_543"


#define DEFAULT_PHY                     0x20                                  // 100kbps and chan 0
#define PHY_NORM_CCA_LEVEL              -70                                   // Default -80dBm normally
#define PHY_NORM_MIN_TX_LEVEL           -85
#define PHY_NORM_REPEAT_NO              3
#define RESETSLOTFLAG                   0
#define INITAWAKESLOT                   1
#define CHAN_INACTIVE                   0xFF
#define NO_OF_PHY_CHANS                 10
#define DEFAULT_SLEEP_TIME              8                                     // Sleep slot = 0,125s => 1s default sleep period
#define UWCHAN_CYC_STARTPOS             0

#define PANID_DEFAULT                  0x1AD1
#define PANID_UNASSIGNED               0xFFFF

#define DEFAULT_PHY_PROFILE         2
#define DAEFULT_CHAN                868000000

#define INNER_ORBIT                 0                                         // top layer end of the network e.g. BS
#define OUTER_ORBIT                 15                                        // bottom of the network e.g. last endpoints

#define STANDARD_REG_WAIT_TIME          1500

PROCESS_NAME(uart_handler_process);

typedef struct {
   uint8_t      dtsn;
   uint8_t      sensor_node;
}UART_CONFIG_PARAMETERS;

typedef enum{
 HOP = 0x01,
 DTSN,
 SENSOR_NODE,
 PAN_ID,
 PREFIX,
 UART_READ_COMPLETE_STATE
}API_COMMANDS;


typedef enum{
UART_NOTSET = 0x01,
UART_CONFIG,
UART_TUNSLIP
}UARTMODE;

typedef enum RFModule_Interface_cb_tag {
  UART_CB,
  SPI_CB
}RFModule_Interface_cb_t;

extern uint16_t accept_dio_rank;
extern uint8_t sensor_end_node;
extern ADI_UART_HANDLE hUartDevice;

void ADI_UART_Receive_CB(uint8_t );
void UART_config_init(void);
void UARTCallback(void *pAppHandle, uint32_t nEvent, void *pArg);
void FlashCallback(void *pCBParam, uint32_t Event, void *pArg);

#define EUI64LEN                    			8                                         // Up to IPv6 supported
#define NO_OF_PHY_CHANS                 		10
#define FLASH_MEM_BLOCK_LENGTH                  128

#define DESCRIPT_1_STR_LEN                      30
#define MEM_LOC_CONFIG_PARAMS_IN_FLASH          0x800
#define MEM_LOC_IOT_CONFIG_PARAMS_IN_FLASH      0xA00
#define MEM_LOC_WEB_SERVER_PARAMS_IN_FLASH      0xA28
#define SIZE_OF_CONFIG_PARAMES                  0x180  //size Includes Confing Params,Run Params,URL

#define MEM_LOC_AD6L_RUNPARAMS					MEM_LOC_CONFIG_PARAMS_IN_FLASH
#define MEM_LOC_AD6L_PARAMS						MEM_LOC_CONFIG_PARAMS_IN_FLASH + 0x100
#define MEM_LOC_NODE_DESCRIPT1	                0x880

#define ADINET_DEVICE_ID                		0x01062007 //It is not Flash Memory Location and this code is expecting by WSN Tool

#define SRD_BUFFER_SIZE             			128

#define NO_OF_UPCHANS               			9

#define MEM_LOC_URL			        			MEM_LOC_CONFIG_PARAMS_IN_FLASH + 0x80
#define URL_STR_LEN                             80

/* Memory required by the driver for DMA mode of operation */
#define ADI_UART_MEMORY_SIZE    				(ADI_UART_BIDIR_MEMORY_SIZE)

#define  PRINT_REPORT

#define SIZE_OF_TX_BUFFER  26

#define SIZE_OF_RX_BUFFER  26

#define UART_DEVICE_NUM 0

typedef enum { NF_UNCONFIGURED = 0,
               NF_ENDPOINT = 1,
               NF_ROUTER = 2,
               NF_CENTRALPOINT = 3,
               NF_SNIFFER = 4,
               NF_UNKNOWN = 5,
               NF_6LN = 6,
               NF_6LR = 7,
               NF_6LBR =8,
               NF_6LSNIFFER = 9,

      }NODEFUNCTION;
typedef enum { CONFIG_OKAY = 0, CONFIG_NOT_OKAY } CONFIGSTATE;

typedef enum
{
  ANS_ASLEEP = 0x00,
  ANS_AWAKE,
  ANS_GOING_TO_SLEEP,
  ANS_STAY_LISTENING,
  ANS_WAKING_UP,
  ANS_UPSTREAM_DATA,
  ANS_DOWNSTREAM_DATA,
  ANS_SELFORGANISING,
  ANS_HW_PROBLEM,
  ANS_NOT_CONFIG,
  ANS_LOST_CONTACT
}ADINETSTATES;

typedef enum { AS_NOTREG = 0, AS_NOTATTACHED, AS_ATTACHED }ATTACHEDSTATES;

typedef enum { CS_SENDRDY = 0x01, CS_MESSAGE, CS_CHECKREPLY, CS_TIMEOUT, CS_SENDING }COMMSTAGES;
typedef enum { EXSTART = 0x01, EXTALKING, EXPREDECT, EXLISTEN, EXMONITOR, EXPAUSE, EXFINISH }EXCHANGESTATES;
typedef enum { MODE_A = 0x00, MODE_B, MODE_C, MODE_D, MODE_E, MODE_F }MODE; // Don't change the order of this enum, 08.01.2014 Mode E added for sniffer
typedef enum { COS_RSTSTATE = 0x00, COS_USDATA = 0x01, COS_UPMODE = 0x02, COS_HB = 0x03}COORDSTATES;





#define UART_BUFFER_SIZE 	150
#define UART_HEADER_LEN		4

typedef enum
{
  UARTRDY = 0x00,
  UARTRXING,
  UARTRXERR,
  UARTTXING,
  UARTTXFIN,
  UARTRXFIN,
  UARTRXCRLF,
  UARTWU
}UARTSTATES;

#define UART_CRC_LEN		2


#define SYNCHBYTE0POS 0
#define SYNCHBYTE1POS 1
#define UARTLENPOS    2

#define SYNCHBYTE0    0x4A
#define SYNCHBYTE1    0x43
#define CR            0x0D
#define LF            0x0A

#define RADIONET_LLH_LEN  9
#define MB_MSGHEADLEN     17

/*----- All configuration parameters are store in this structure ------*/
/*---- Do not change the member arrangements in this structure!!! -----*/
typedef struct
{
  int32_t         SleepTime_ms;                                             // Amount of time the node sleeps in multiples of 125ms
  unsigned long   DeviceType;
  uint8_t         AdrAry[EUI64LEN];
  uint8_t         BindAdrAry[EUI64LEN];
  uint8_t         UpChanNoAry[NO_OF_UPCHANS];                               // Physical channels
  uint8_t         PANID[2];
  uint8_t         Orbit;
  uint8_t         RepeatLimit;                                             // Max. amount of times a message can be repeated
  uint8_t         Reserved1;
  ISMBAND         ISMBand;
  uint8_t         DWChanNo;
  uint8_t         DWChanPHY;
  uint8_t         UPChanPHY;
  uint8_t         RepeatsTillLost;                                          // No of repeats before node reconfigures itself
  uint8_t         XXXXXXXXXXXX;
  uint8_t         MaxTxPwr;                                                 //
  int8_t          CCA_Lim;                                                  // Max. noise chan can have before changed
  int8_t          MinRSS;                                                   // Min signal strength to reduce tx power
  NODEFUNCTION    NodeFunct;
  uint8_t         MainOpts;
  uint8_t         SecOpts;
  uint8_t         OptAry[2];
  uint8_t         ServerIPAdr[4];
  uint8_t         MyIPAdr[4];
  uint8_t         OtherIPAdr[4];
  uint8_t         EncryptionWord[16];
  uint32_t        CheckSum;
}ANODECONFIGSTATE;

typedef struct
{
  uint8_t IPType;                       //specifies the type of IP whether its static or dervied from DHCP
  uint8_t EnableDtls;                   // Flag indicating DTLS security is Enabled/Disabled
  uint8_t ReservedBytes[2];             // Reserved 2 bytes for future
  uint32_t WebTransferProtocol;          // Specifies the type of protocol being used
  uint8_t IPAddr[8];                    // Holds the IP address of teh device
  uint8_t SubnetMask[8];                // Holds Syubnet mask of the device
  uint8_t DefaultGatewayAddr[8];        // Specifies the gateway address
  uint8_t DNSServerAddr[8];             // Specifies the DNS server address
  char ServerAddr[40];                // Holds the server address
  char ServerPort[8];                  // Server port
  char EndpointName[16];                // Name of the Endpoint
  char PskIdentity[16];                 // PSK identifier for DTLS
  char PskKey[16];                      // PSK key for DTLS
  uint32_t CheckSum;
}ANODEIOTCONFIGSTATE;

typedef struct
{
  uint8_t IPAddr[8];                    // Holds the IP address of teh device
  uint8_t SubnetMask[8];                // Holds Syubnet mask of the device
  uint8_t DefaultGatewayAddr[8];        // Specifies the gateway address
  uint8_t DNSServerAddr[8];             // Specifies the DNS server address
}ANODEIOTRUNSTATE;
/*----------------- All system variables are stored in this structure --------*/
typedef struct
{
  unsigned long         ActFreq;             //CheckSum
  int32_t               SleepTime_ms_ctr;    // Amount time the node sleeps in 10ms steps
  int32_t               SleepTime_ms;
  int32_t                RegWaitTime_ms;
  uint32_t                NW_SynTime;
  uint8_t                xxActPHYProfile;
  uint8_t               HWState;             // Transceiver okay?
  CONFIGSTATE           ConfigState;
  uint8_t               ActUpConnNo;
  uint8_t               TxPwr;
  uint8_t               AckTxPwr;
  uint8_t               RunOptions;
  //S8                    CCA_Lim;
  //S8                    MinRSSILim;
  uint16_t               LocExchID;          // Identify the local loop
  MODE                  Mode;
  unsigned char         NoUWChannels;
  unsigned char         UWChanTxCycCntr;
  unsigned char         UWChanRxCycCntr;
  unsigned char         AutoSendFlag;
  long                  ISMChannels[NO_OF_PHY_CHANS];
  unsigned char         MuteFlag;
  unsigned char         RepeatCntr;          // Counts the no. of times a message is repeated
  unsigned int          InitStateFlag;
  unsigned int          SlotCntr;
  int                   SlotCntrTO;          // Count the number of slots for a TO (ca. 30ms in most cases)
  unsigned char         TOFlag;
  //unsigned char         SlotInitFlag;
  unsigned int          StatTxInterval;
  unsigned int          StatCCAInterval;
  unsigned int          StatPhaseInterval;
  unsigned int          StatMsgDuration;
  unsigned int          StatTxDuration;
  unsigned int          StatCCADuration;
  uint32_t                StatMsgCntr;
  uint32_t                StatBadMsgCntr;
  unsigned int          StatMsgNotSent;
  unsigned char         StatRxRepeats;
  ADINETSTATES          State;
  ATTACHEDSTATES        AttachedState;
  unsigned char         CCATries;
  COORDSTATES           CoOrdState_A;
  COMMSTAGES            CommStage;
  int8_t                RSSI_PHY_on;
  int8_t                RSSI_PackRx;
  EXCHANGESTATES        ExchangeState;
  uint32_t               RandomNo;
  uint8_t                E2EAckFlag;
  uint8_t                MsgCntr;
  uint16_t               PanId;
  uint16_t               RxPanId;
  uint8_t                MasterRepeatCntr;
  uint8_t               WakeUpNotification;
  int                   BatVolt;
}ANODERUNSTATE;

unsigned char*  UARTTxPayloadBuffer(void);
unsigned char GetByteFromBuffer(char Index, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd);
unsigned int AddWordToBuffer(int IntVal, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd);
unsigned int GetNextBufPos(unsigned int Pos, unsigned char *Buf);
uint8_t UARTTx(uint8_t *tx_buff, uint16_t buff_len);
void Send_ReceivedData2Uart(unsigned char *buf,unsigned int Bufpos, char len);
void InitUART();
bool_t uart_tx_is_in_progress(void);
void SaveConfigtoFlash(void);
void ANode_SetConfig(CONFIGSTATE ConfigState);
unsigned char FlashErase(long block_start, long block_size);
void cmd_hal_write(void);
void adi_cmd_SubmitRxBuffer(uint8_t cb_from, uint8_t *rx_buff, uint16_t buf_len);
void config_init(bool readConfigFlash );
void update_phy_params(void);
bool Node_IsSleepy();

#endif //__UART_HANDLER_H__