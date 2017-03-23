/***********************************************************************************************************************

Copyright(c) 2014 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.

***********************************************************************************************************************/

/**
 * \file
 * 		Main implementation file for uart handler code
 * \author
 *    Analog Devices
 */

#include <uart/adi_uart.h>
#include "contiki.h"
#include "uart_handler.h"
#include "string.h"
#include <flash/adi_flash.h>
#include "net/ip/uip.h"
#include "net/ip/uipopt.h"

//#include "ADSensors.h"
#include "common.h"
#include "hif_utility.h"
#include "WSN_UI_Handler.h"
#include "RF_Module_API_Handler.h"
#include "phy.h"

#define DEV_TYPE_HOST           1
#define DEV_TYPE_ROUTER         2
#define DEV_TYPE_EDGE_ROUTER    3
#define DEV_TYPE_NONE           4
#define DEV_TYPE                DEV_TYPE_NONE
   
uint8_t mConfigFlashMemory[SIZE_OF_CONFIG_PARAMES];
uint8_t mIotConfigFlashMemory[SIZE_OF_CONFIG_PARAMES];

uint8_t ConfiguredKey[16] =
  {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};

extern uip_802154_longaddr ieee_802154_extended_addr;
extern uip_ipaddr_t prefix_configured;
extern uint16_t pan_id_configured;
extern uint8_t sensor_end_node;
extern uip_lladdr_t pck_originator_lladdr;
extern uint16_t accept_dio_rank;
extern uint16_t mac_dst_pan_id;
extern uint16_t mac_src_pan_id;
extern uint16_t mac_pan_id;

UART_CONFIG_PARAMETERS *uart_app_info = (UART_CONFIG_PARAMETERS*)&mConfigFlashMemory[0];
extern ADI_FEE_HANDLE hFlashDevice;
extern void ADI_UART_ENABLE_TX_EXT(ADI_UART_HANDLE const hDevice);
/*---------------------------------------------------------------------------*/

PROCESS(uart_handler_process, "Uart Handler Process");
extern ADI_UART_HANDLE hUartDevice;

extern hif_t hif;
extern NodeType_t g_node_type;

uint16_t hif_bytes_to_drop;
uint8_t g_rcvd_cmd_frame_type;
uint8_t g_rcvd_cmd_frame_interface_type;
static uart_tx_data_t* mp_send_data;
#pragma location="never_retained_ram"
uint8_t uart_tx_buffer[MAX_BUFF_SIZE];

ANODECONFIGSTATE *mConfigParams = (ANODECONFIGSTATE*)&mConfigFlashMemory[256];
ANODERUNSTATE mRunParams;
ANODEIOTRUNSTATE mRunIotParams;

ANODEIOTCONFIGSTATE *mIotConfigParams = (ANODEIOTCONFIGSTATE*)&mIotConfigFlashMemory[0];

#pragma location="never_retained_ram"
unsigned char 	RxBuffer[UART_BUFFER_SIZE + 1];
#pragma location="never_retained_ram"
unsigned char 	TxBuffer[UART_BUFFER_SIZE + 1];
char            UARTRxBufPos = 0;
char            UARTTxBufPos = 0;
char 	        mTxLen;
char            mRespFlag;

/*-------------------------------------------*/
/* Return the length of the package received */
/*-------------------------------------------*/
unsigned int    UARTRxPayloadLen(void) { return ((unsigned int)RxBuffer[UARTLENPOS]- UART_HEADER_LEN - UART_CRC_LEN); }
unsigned int    UARTTxBufferEnd(void) { return (UART_BUFFER_SIZE - UART_HEADER_LEN - UART_CRC_LEN); }

unsigned char*  UARTTxPayloadBuffer(void) { return(TxBuffer + UART_START_OF_PAYLOAD); }

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])


void Check_Uart_State( void );
bool reinit_uart = false;
/*----------------------------------------------------------------------------*/

/*
* @brief        Get the long value at position in the Index prameter
*/
unsigned char GetByteFromBuffer(char Index, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd)
{
  unsigned int TypePos, IndexCnt = 0, n;
  TypePos = BufPos;
  while(TypePos < (BufEnd - 1))       // Data can't be closer to the end as this
  {
    if(Buffer[TypePos] == BYTE_MARK)
    {
      if(IndexCnt == Index) return Buffer[TypePos + 1];
      IndexCnt++;
    }
    n = GetNextBufPos(TypePos, Buffer);
    if(n == 0) return 0;	// Error condition if position not changed
    TypePos += n;
  }
  return 0;
}
/*----------------------------------------------------------------------------*/

/*
* @brief        Add an int to the buffer that will be transmitted. The string is
*       composed of as itenfifications byte with 'i', followed by 2 bytes. The
*       no. of bytes added is returned.
*/
unsigned int AddWordToBuffer(int IntVal, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd)
{
  int nVal;
  char *Chptr;
  if(BufPos > BufEnd - 3) return(BufPos); 			// Not added
  nVal = IntVal;
  Chptr = (char*)&nVal;
  Buffer[BufPos++] = INT_MARK;
  Buffer[BufPos++] = *Chptr++;
  Buffer[BufPos++] = *Chptr;
  return(BufPos);
}

/*--------------------------------------------------------------------------*/
/* Get the position modifier depending on the data type found in the buffer */
/*--------------------------------------------------------------------------*/
unsigned int GetNextBufPos(unsigned int Pos, unsigned char *Buf)
    {
    switch (Buf[Pos])
        {
        case LONG_MARK: return LONGOBJLEN;
        case STR_MARK: return (STROBJLEN + Buf[Pos + 1]);
        case BYTE_MARK: return BYTEOJLEN;
        case INT_MARK: return INTOBJLEN;
        case SINGLE_MARK: return SINGLEOBJLEN;
        case BYTEARY_MARK: return (BYTEARYOBJLEN + Buf[Pos + 1]);
        case UNIT_MARK: return UNITOBJLEN;
        default: return 0;		// Fault condition
        }
    }

/*------------------------------------------------------*/
/* Get the long value at position in the Index prameter */
/*------------------------------------------------------*/
unsigned int GetStringFromBuffer(char Index, unsigned char *DestBuf, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd)
    {
    unsigned int TypePos, IndexCnt = 0, n;
    TypePos = BufPos;
    while(TypePos < (BufEnd - 5))        				// Data can't be closer to the end as this
        {
        if(Buffer[TypePos] == STR_MARK)						// The correct long value found
            {
            if(IndexCnt == Index)
                {
                for(n=0; n<Buffer[TypePos + 1]; n++) *DestBuf++ = Buffer[TypePos + 2 + n];
                return n;
                }
            else IndexCnt++;
            }
        n = GetNextBufPos(TypePos, Buffer);
        if(n == 0) return 0;							// Error condition if position not changed
        TypePos += n;
        }
    return 0;
    }

/*---------------------------------------------------------------------------------*/
/* Add a char to the buffer that will be transmitted. The string is composed of an */
/* itenfifications byte with 'c', followed by 1 byte.                              */
/*---------------------------------------------------------------------------------*/
unsigned int AddByteToBuffer(char ChVal, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd)
    {
    if(BufPos > BufEnd - 2) return(BufPos); 			// Not added
    Buffer[BufPos++] = BYTE_MARK;
    Buffer[BufPos++] = ChVal;
    return(BufPos);										// No error, return buffer position
    }



/*-----------------------------------------------------------------------------------*/
/* Add a string to the buffer that will be transmitted. The string is composed of an */
/* itenfifications byte with 's', followed by a byte with the length of the string.  */
/*-----------------------------------------------------------------------------------*/
unsigned int AddStringToBuffer(char *StrVal, unsigned int Len, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd)
    {
    if((BufPos > BufEnd - Len - 3) || (Len == 0)) return(BufPos); 	// Not added
    Buffer[BufPos++] = STR_MARK;
    Buffer[BufPos++] = Len + 1;
    memcpy((char *)&Buffer[BufPos], StrVal, Len);
    Buffer[BufPos + Len] = 0;
    BufPos = BufPos + Len + 1;
    return(BufPos);
    }

/*-----------------------------------------------------------------------------------*/
/* Add a string to the buffer that will be transmitted. The string is composed of an */
/* itenfifications byte with 's', followed by a byte with the length of the string.  */
/*-----------------------------------------------------------------------------------*/
unsigned int AddByteAryToBuffer(unsigned char *Ary, unsigned int Len, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd)
    {
    if((BufPos > BufEnd - Len - 3) || (Len == 0)) return(BufPos); 	// Not added
    Buffer[BufPos++] = BYTEARY_MARK;
    Buffer[BufPos++] = Len;
    memcpy((char *)&Buffer[BufPos], Ary, Len);
    BufPos += Len;
    return(BufPos);
    }

#if ((DEV_TYPE == DEV_TYPE_HOST))
/*------------------------------------------------------*/
/* Get the long value at position in the Index prameter */
/*------------------------------------------------------*/
long GetLongFromBuffer(char Index, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd)
    {
    unsigned int TypePos, IndexCnt = 0, n;
    char *Chptr;
    long lVal;
    TypePos = BufPos;
    while(TypePos < (BufEnd - 2))        			// Data can't be closer to the end as this
        {
        if(Buffer[TypePos] == LONG_MARK)					// The correct long value found
            {
            if(IndexCnt == Index)
                {
                Chptr = (char*)&lVal;
                for(n=0; n<4; n++) *(Chptr + n) = Buffer[TypePos + n + 1];
                return lVal;
                }
            else IndexCnt++;
            }
        n = GetNextBufPos(TypePos, Buffer);
        if(n == 0) return 0;						// Error condition if position not changed
        TypePos += n;
        }
    return 0;
    }

/*-----------------------------------------------------------------------------------*/
/* Add a value of type single float to a position in the transmit buffer for sending */
/* add the preamble, in this case 'f' to identify the buffer position as a single    */
/* float. Obviously there is an over head of 1 byte, meaning 5 bytes are sent.		 */
/*-----------------------------------------------------------------------------------*/
unsigned int AddSingleToBuffer(float FltVal, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd)
    {
    char *ChPtr;
    unsigned int n;
    float lVal;
    lVal = FltVal;
    ChPtr = (char *)&lVal;
    if((BufPos + SINGLEOBJLEN) > BufEnd) return(BufPos); 		// Not added
    Buffer[BufPos] = SINGLE_MARK;
    for(n=1; n<5; n++) Buffer[BufPos + n] = *(ChPtr++);
    return (BufPos + SINGLEOBJLEN);
    }

/*-------------------------------------------------------------------------------------*/
/* Add a long int to the buffer that will be transmitted. The string is composed of an */
/* itenfifications byte with 'l', followed by 4 bytes.		                       */
/*-------------------------------------------------------------------------------------*/
unsigned int AddLongToBuffer(long lVal, unsigned char *Buffer, unsigned int BufPos, unsigned int BufEnd)
    {
    long nVal;
    unsigned int n;
    char *ChPtr;
    nVal = lVal;
    ChPtr = (char*)&nVal;
    if((BufPos + LONGOBJLEN) > BufEnd) return(BufPos); // Error condition buffer too small for string
    Buffer[BufPos] = LONG_MARK;
    for(n=1; n<5; n++) Buffer[BufPos + n] = *(ChPtr++);
    return (BufPos + LONGOBJLEN);
    }

#endif //((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))

/*-------------------------------------------------------------------------------------*/
/* U32 FlashWrite(U32 *block_start,U32 offset_into_block,U32 count,U32 const *buffer)  */
/* where: block_start        Start of the block to write to			       */
/*        offset_into_block  Offset in bytes into the block to write to		       */
/*		  count              Number of bytes to write			       */
/*        buffer             Src buffer of bytes to write			       */
/*        boolean  TRUE if successful, else false	                               */
/*-------------------------------------------------------------------------------------*/
unsigned char FlashWrite(long block_start, long count, long* buffer)
{
  ADI_FEE_RESULT eResult = ADI_FEE_SUCCESS;
  eResult = adi_fee_SubmitTxBuffer(hFlashDevice,block_start,(void*)buffer,count);
  void* pBuffer;
  adi_fee_GetTxBuffer(hFlashDevice, &pBuffer);
  return eResult;
}

/*-----------------------------------------------------------*/
/* U32 FlashErase(void *    block_start,U32  block_size)     */
/*  where: Erase the block starting at block_start	     */
/*         block_start  Start address of block to be erased  */
/*         block_size   Size of block to be erased.	     */
/*         boolean  TRUE if successful, else false	     */
/*-----------------------------------------------------------*/
unsigned char FlashErase(long block_start, long block_size)
{
  uint32_t nStartPage, nEndPage;
  ADI_FEE_RESULT eResult = ADI_FEE_SUCCESS;
  adi_fee_GetPageNumber(hFlashDevice, block_start, &nStartPage);
  adi_fee_GetPageNumber(hFlashDevice, block_start+block_size-1, &nEndPage);
  eResult = adi_fee_PageErase(hFlashDevice,nStartPage,nEndPage);
  return eResult;
}


unsigned char ANode_GetTXPwr(void) { return mRunParams.TxPwr; }
void ANode_SetConfig(CONFIGSTATE ConfigState) { mRunParams.ConfigState = ConfigState; }


int ANode_Version(unsigned char* Buffer, int Bufpos, int MaxLen) { return AddStringToBuffer((char *)ANodeVersion, strlen(ANodeVersion), Buffer, Bufpos, MaxLen); }

unsigned char ANode_GetAutoSend(void) {return mRunParams.AutoSendFlag;}
bool Node_IsSleepy() { return ((mConfigParams->MainOpts & 0x20)); }
void Node_SetSleepy( bool mode)
{
  if (mode) mConfigParams->MainOpts |= (0x20);
  else mConfigParams->MainOpts &= ~0x20;
}
#define SLEEP_ENABLE_MASK               0x20
#define MAC_SECURITY_ENABLE_MASK        0x40
bool Node_IsMacSecurityEnabled() { return ((mConfigParams->MainOpts & 0x40)); }
#define ENABLE_STATISTICS_MASK          0x80
void Node_EnableStatistics(bool mode)
{
    if (mode) mConfigParams->MainOpts |= (0x80);
    else mConfigParams->MainOpts &= ~0x80;
}
bool Is_StatisticsEnabled()
{
 return ((mConfigParams->MainOpts & 0x80));
}

/* Function will be used by the MAC security to obtain key */
uint8_t* ANode_GetConfigKey()
{
  return &(mConfigParams->EncryptionWord[0]);
}

void config_init(bool readConfigFlash )
{
  U32 n, CheckSum;
  unsigned char * StoreArea = (unsigned char*)MEM_LOC_AD6L_PARAMS;
  unsigned char * IotConfigArea = (unsigned char*)MEM_LOC_IOT_CONFIG_PARAMS_IN_FLASH;

  if(readConfigFlash)
  {
    for(n=0; n<(sizeof(ANODEIOTCONFIGSTATE)); n++) *((unsigned char*)mIotConfigParams + n) = *(IotConfigArea + n);
    CheckSum = mIotConfigParams->CheckSum;
    for(n=0; n<(sizeof(ANODEIOTCONFIGSTATE) - 4); n++) CheckSum -= *((unsigned char*)mIotConfigParams + n);
    if(CheckSum != 0)
    {
      memset(mIotConfigParams,'\0',(sizeof(ANODEIOTCONFIGSTATE)-4));
      mIotConfigParams->IPType = 0x01; // DHCP Enabled
      mIotConfigParams->EnableDtls = 0x01; // DTLS Enabled
      memcpy(mIotConfigParams->ServerAddr,"52.1.231.165",strlen("52.1.231.165"));
      if (g_node_type == NODE_6LN)
      {
        mIotConfigParams->DefaultGatewayAddr[0] = 0x00;
        mIotConfigParams->DefaultGatewayAddr[1] = 0x05;
        mIotConfigParams->DefaultGatewayAddr[2] = 0xF7;
        mIotConfigParams->DefaultGatewayAddr[3] = 0xFE;
        mIotConfigParams->DefaultGatewayAddr[4] = 0xFF;
        mIotConfigParams->DefaultGatewayAddr[5] = 0x00;
        mIotConfigParams->DefaultGatewayAddr[6] = 0x00;
        mIotConfigParams->DefaultGatewayAddr[7] = 0x02;
      }
      memcpy(mIotConfigParams->EndpointName,"ADIV12",strlen("ADIV12"));
      memcpy(mIotConfigParams->PskIdentity,"Client_identity",strlen("Client_identity"));
      memcpy(mIotConfigParams->PskKey,"secretPSK",strlen("secretPSK"));
      memcpy(mIotConfigParams->ServerPort,"5684",strlen("5684"));
      mIotConfigParams->WebTransferProtocol = 0x01;
      ANode_SetConfig(CONFIG_OKAY);
    }

    for(n=0; n<sizeof(ANODECONFIGSTATE); n++) *((unsigned char*)mConfigParams + n) = *(StoreArea + n);
    CheckSum = mConfigParams->CheckSum;
    for(n=0; n<(sizeof(ANODECONFIGSTATE) - 4); n++) CheckSum -= *((unsigned char*)mConfigParams + n);

    if(CheckSum != 0)
    {
  #ifndef WIRESHARK_FEEDER
      if(g_node_type == NODE_6LBR) {
        ieee_802154_extended_addr.addr[7] = 0x01;
      }
      else if (g_node_type == NODE_6LR) {
        ieee_802154_extended_addr.addr[7] = 0x02;
      }
      else if (g_node_type == NODE_6LN) {
        ieee_802154_extended_addr.addr[7] = 0x03;
      }

  #else
  #ifdef WIRESHARK_FEEDER
      ieee_802154_extended_addr.addr[7] = 0x04;
  #else
      ieee_802154_extended_addr.addr[7] = 0x05;
  #endif    //WIRESHARK_FEEDER
  #endif    // !WIRESHARK_FEEDER

      /*This is first time , so Write the default values into the memory*/
      memcpy(&mConfigParams->AdrAry, &ieee_802154_extended_addr, sizeof(uip_802154_longaddr));
      memcpy(&mConfigParams->BindAdrAry, &prefix_configured, sizeof(mConfigParams->BindAdrAry));
      memcpy(&mConfigParams->PANID, &pan_id_configured, sizeof(pan_id_configured));
      mConfigParams->Orbit = 1;
      mConfigParams->ISMBand = ISMBAND_2400MHZ;
      mConfigParams->DWChanNo = 11;
      mConfigParams->DWChanPHY = 0;



      mConfigParams->CheckSum = 0;
      mConfigParams->MainOpts &= ~MAC_SECURITY_ENABLE_MASK;
#if ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
      mConfigParams->MainOpts &= ~SLEEP_ENABLE_MASK;
#endif
      memcpy(&mConfigParams->EncryptionWord,&ConfiguredKey[0],16);
      for(n=0; n<(sizeof(ANODECONFIGSTATE) - 4); n++) mConfigParams->CheckSum += *((unsigned char*)mConfigParams + n);

      FlashErase(MEM_LOC_AD6L_PARAMS, 1);
      //FlashWrite((long)StoreArea,sizeof(mConfigParams),(long*)&mConfigParams);
      FlashWrite(MEM_LOC_CONFIG_PARAMS_IN_FLASH,SIZE_OF_CONFIG_PARAMES,(long*)mConfigFlashMemory);
      uart_app_info->dtsn = 1;
      uart_app_info->sensor_node = 0;
      FlashErase(MEM_LOC_AD6L_RUNPARAMS, 1);
     // FlashWrite(MEM_LOC_AD6L_RUNPARAMS,sizeof(uart_app_info),(long*)&uart_app_info);
      FlashWrite(MEM_LOC_CONFIG_PARAMS_IN_FLASH,SIZE_OF_CONFIG_PARAMES,(long*)mConfigFlashMemory);
    }
    memcpy(&mac_src_pan_id, &mConfigParams->PANID, sizeof(mac_src_pan_id));
    memcpy(&mac_dst_pan_id, &mConfigParams->PANID, sizeof(mac_dst_pan_id));
    memcpy(&mac_pan_id, &mConfigParams->PANID , sizeof(mac_pan_id));
//    memset(mIotConfigParams,'\0',sizeof(mIotConfigParams));
//    mIotConfigParams->IPType = 0x01; // DHCP Enabled
//    mIotConfigParams->EnableDtls = 0x01; // DTLS Enabled
//    memcpy(mIotConfigParams->PskIdentity,"Client_identity",strlen("Client_identity"));
//    memcpy(mIotConfigParams->PskKey,"secretPSK",strlen("secretPSK"));
//    memcpy(mIotConfigParams->ServerPort,"5684",strlen("5684"));
//    mIotConfigParams->WebTransferProtocol = 0x01;
    ANode_SetConfig(CONFIG_OKAY);

    //get the runParms
    StoreArea = (unsigned char*)MEM_LOC_AD6L_RUNPARAMS;
    for(n=0; n<sizeof(UART_CONFIG_PARAMETERS); n++) *((unsigned char*)uart_app_info + n) = *(StoreArea + n);

    StoreArea = (unsigned char*)MEM_LOC_URL;
    for(n=0; n<128; n++) *((unsigned char*)&mConfigFlashMemory[128] + n) = *(StoreArea + n);

  #ifndef WIRESHARK_FEEDER
    if(g_node_type == NODE_6LBR) {
      mConfigParams->NodeFunct         = NF_6LBR; //NF_CENTRALPOINT;
    }
    else if(g_node_type == NODE_6LR) {
      mConfigParams->NodeFunct         = NF_6LR ;// NF_ROUTER;
    }
    else if(g_node_type == NODE_6LN) {
      mConfigParams->NodeFunct         = NF_6LN ;//NF_ENDPOINT;
    }

  #else
  #ifdef WIRESHARK_FEEDER
      mConfigParams->NodeFunct         = NF_6LSNIFFER ;// NF_SNIFFER;
  #else
      mConfigParams->NodeFunct         = NF_UNCONFIGURED;
  #endif    //WIRESHARK_FEEDER
  #endif    // !WIRESHARK_FEEDER
  }

  /* copy the read values into relevant global varialbles*/
  memcpy(&ieee_802154_extended_addr, &mConfigParams->AdrAry, sizeof(uip_802154_longaddr));
  //hop_to_join();
  sensor_end_node = uart_app_info->sensor_node;
  memcpy(&prefix_configured, &mConfigParams->BindAdrAry, sizeof(mConfigParams->BindAdrAry));
  memcpy(&pan_id_configured, &mConfigParams->PANID, sizeof(pan_id_configured));

}

/*----------------------------------------------------------------------------*/


extern uint8_t transmit_pwr;

void update_phy_params(void)
{
  uint16_t pan_id = ((mConfigParams->PANID[1] << 8) | mConfigParams->PANID[0]);
  phy_set_panId(pan_id);
  phy_set_ExtendedAddress(mConfigParams->AdrAry);
  phy_set_channel(mConfigParams->DWChanNo, mConfigParams->ISMBand, mConfigParams->DWChanPHY);
}

void Derive_LL_Address ( uint8_t * ipaddr, uint8_t * lladdr )
{
    memcpy(lladdr, &ipaddr[8], sizeof (uip_lladdr));
    lladdr[0] ^= 0x02;
}

#ifndef WIRESHARK_FEEDER
void Send_ReceivedData2Uart(unsigned char *buf,unsigned int Bufpos, char len)
{
    U8 *TxDataBuf;
    int16_t Port = 1035;
    TxDataBuf = UARTTxPayloadBuffer();

    *(TxDataBuf+Bufpos)= RADIONET_LLH_LEN + MB_MSGHEADLEN + len; Bufpos++;  //LLH_PACKLEN_POS

    *(TxDataBuf+Bufpos)=0xFF; Bufpos++;  //LLH_OPTS_POS
    *(TxDataBuf+Bufpos)=0xFF; Bufpos++;  //LLH_SRCORBIT_POS
    *(TxDataBuf+Bufpos)=0xFF; Bufpos++;  //LLH_TYPEREPEAT_POS
    *(TxDataBuf+Bufpos)=0xFF; Bufpos++;  //LLH_SIGLEVEL_POS

    //LLH_PANID_POS
    memcpy((TxDataBuf+Bufpos),&mConfigParams->PANID,2); Bufpos+=2;

    //LLH_EXCH_ID_POS
    *(TxDataBuf+Bufpos)=0x56; Bufpos++;
    *(TxDataBuf+Bufpos)=0x78; Bufpos++;

    //MB_MSG_INDEX
    *(TxDataBuf+Bufpos)=0xAA; Bufpos++;
    *(TxDataBuf+Bufpos)=0xBB; Bufpos++;

    //MB_ORBIT_POS
    *(TxDataBuf+Bufpos)=(UIP_TTL - UIP_IP_BUF->ttl)+1; Bufpos++;                //SCH-Changed hops to orbit number

    // MAC address of the sender
    Derive_LL_Address((uint8_t *)&UIP_IP_BUF->srcipaddr,(TxDataBuf+Bufpos)); Bufpos+=8;

    //SyncTime
    *(TxDataBuf+Bufpos)=0xFF; Bufpos++;
    *(TxDataBuf+Bufpos)=0xFF; Bufpos++;
    *(TxDataBuf+Bufpos)=0xFF; Bufpos++;
    *(TxDataBuf+Bufpos)=0xFF; Bufpos++;

    //MB_PORT_POS
     memcpy((TxDataBuf+Bufpos),&Port,2); Bufpos+=2;

    // copy received message to uart buffer
    memcpy((TxDataBuf+Bufpos),buf,len); Bufpos+=len;

    if(g_rcvd_cmd_frame_interface_type == UART_CB) {
      if(g_rcvd_cmd_frame_type == FRAME_TYPE_RF_MODULE){
        TxBuffer[RFMODULE_FRAME_CMD_POS] = UART_SUBCMD_SRDBUFFER;
#if ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
        RF_Module_FrameTx(Bufpos);
#endif
      }
    }
}
#endif /* WIRESHARK_FEEDER      */


uint8_t get_current_dtsn()
{
  return uart_app_info->dtsn == 0?++uart_app_info->dtsn:uart_app_info->dtsn;
}

void increament_and_store_dtsn()
{
  uart_app_info->dtsn++;
  FlashErase(MEM_LOC_AD6L_RUNPARAMS, 1);
  //FlashWrite(MEM_LOC_AD6L_RUNPARAMS,sizeof(uart_app_info),(long*)&uart_app_info);
  FlashWrite(MEM_LOC_CONFIG_PARAMS_IN_FLASH,SIZE_OF_CONFIG_PARAMES,(long*)mConfigFlashMemory);
}

void store_prefix()
{
  memcpy(&mConfigParams->BindAdrAry, &prefix_configured, sizeof(mConfigParams->BindAdrAry));
  mConfigParams->CheckSum = 0;
  for(int n=0; n<(sizeof(ANODECONFIGSTATE) - 4); n++) mConfigParams->CheckSum += *((unsigned char*)mConfigParams + n);
  FlashErase(MEM_LOC_AD6L_PARAMS, 1);
  //FlashWrite(MEM_LOC_AD6L_PARAMS,sizeof(mConfigParams),(long*)&mConfigParams);
  FlashWrite(MEM_LOC_CONFIG_PARAMS_IN_FLASH,SIZE_OF_CONFIG_PARAMES,(long*)mConfigFlashMemory);
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
  return pan_id_configured;
}

uint16_t get_dst_pan_id(void)
{
  return pan_id_configured;
}

void SaveConfigtoFlash(void)
{
    mConfigParams->CheckSum = 0;
    for(int n=0; n<(sizeof(ANODECONFIGSTATE) - 4); n++) mConfigParams->CheckSum += *((unsigned char*)mConfigParams + n);
    FlashErase(MEM_LOC_AD6L_PARAMS, 1);
    FlashWrite(MEM_LOC_CONFIG_PARAMS_IN_FLASH,SIZE_OF_CONFIG_PARAMES,(long*)mConfigFlashMemory);
}

uint8_t GetOrbitNumber()
{
  return mConfigParams->Orbit;
}

/*---------------------------------------------------------------------------*/

/* Print a string to the terminal */
static void PRINT_C(char* string) {

  int16_t size_l = 0;
  size_l = strlen(string);
  send_uart_frame((uint8_t *)string, size_l);
}

/* The __write and the __read functions are used by stdout to redirect printf
* and scanf to the hyperterminal */
size_t __write(int handle, const unsigned char* buf, size_t bufsize) {


  for(int i = 0; i < bufsize; i++) {

    if(buf[i] == '\n')
      PRINT_C("\r\n");
  }
  return bufsize;
}

size_t __read(int handle, unsigned char* buf, size_t bufsize) {

  for(int i = 0; i < bufsize; i++)
    buf[i] = ECHO();

  return bufsize;
}
/*----------------------------------------------------------------------------*/

/*
* @brief        Writes the given bytes into UART. This can be used before system
*               reset.
*/
uint8_t UARTTx(uint8_t *tx_buff, uint16_t buff_len)
{
  while((pADI_UART0->COMLSR &((uint16_t) BITM_UART_COMLSR_TEMT)) == 0 );
  for(int i = 0; i < buff_len; i++) {
    pADI_UART0->COMTX = tx_buff[i];
    while((pADI_UART0->COMLSR &((uint16_t) BITM_UART_COMLSR_TEMT)) == 0 );
  }
  return 0;
}
/*----------------------------------------------------------------------------*/

int uart_hal_write(uart_tx_data_t* p_data)
{
  if (!p_data->tx_in_progress) {
    ADI_DISABLE_INT(UART_EVT_IRQn);
    mp_send_data = p_data;
    mp_send_data->tx_in_progress = true;
    //adi_uart_EnableTx(hUartDevice, true);
    ADI_UART_ENABLE_TX_EXT(hUartDevice);
    ADI_ENABLE_INT(UART_EVT_IRQn);
  }
  return 0;
}

bool_t uart_tx_is_in_progress(void){
  return send_data.tx_in_progress;
}
/*----------------------------------------------------------------------------*/

void uart_hal_tx_cb(void)
{
  uart_write(hUartDevice, mp_send_data->data[mp_send_data->tx_rd_index]);

  mp_send_data->tx_rd_index++;
  mp_send_data->tx_rd_index  %= MAX_BUFF_SIZE;

  if(mp_send_data->tx_wr_index == mp_send_data->tx_rd_index) {
    adi_uart_EnableTx(hUartDevice, false);
    mp_send_data->tx_in_progress = false;
    mp_send_data->tx_wr_index = 0;
    mp_send_data->tx_rd_index = 0;

  }
}
/*----------------------------------------------------------------------------*/

void CommonUARTHandler(void)
{
#if ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
  if(g_rcvd_cmd_frame_type == FRAME_TYPE_RF_MODULE){
    RF_Module_API_Handler(RxBuffer);
  }
#endif
}
/*----------------------------------------------------------------------------*/

PROCESS_THREAD(uart_handler_process, ev, data)
{

  PROCESS_BEGIN();

  while(1)
  {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    CommonUARTHandler();
  }
  PROCESS_END();
}
/*----------------------------------------------------------------------------*/
void CommonCommandHandler(uint8_t cb_from);


void UARTCallback( void *pAppHandle, uint32_t nEvent, void *pArg)
{
  /* CASEOF (event type) */
  switch (nEvent){
    /* CASE (buffer processed) */
  case ADI_UART_EVENT_TX_BUFFER_PROCESSED:
    break;

  case ADI_UART_EVENT_RX_BUFFER_PROCESSED:
    CommonCommandHandler(UART_CB);
    break;

  default:
    reinit_uart = true;
    break;
  }
}

//@debug
    bool_t RFM_SPIRxSubmit = false;
    uint8_t *RFM_SPIRxSubmit_buff;
    uint16_t RFM_SPIRxSubmit_buff_len;

void adi_cmd_SubmitRxBuffer(uint8_t cb_from, uint8_t *rx_buff, uint16_t buf_len)
{
  if(cb_from == UART_CB){
    if( adi_uart_SubmitRxBuffer(hUartDevice, rx_buff, buf_len) != ADI_UART_SUCCESS){
      reinit_uart = true;
    }
  }
#if  ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
  else if(cb_from == SPI_CB) {
    //SpiSlaveRxSubmitBuffer(rx_buff, buf_len);
//    RFM_SPIRxSubmit = true;
    RFM_SPIRxSubmit_buff = rx_buff;
    RFM_SPIRxSubmit_buff_len = buf_len;
  }
#endif
}

void send_cmd_frame_ack(uint8_t cb_from, uint8_t status)
{
  if(cb_from == UART_CB) {
  send_uart_frame_ack(status);
  }
#if  ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
  else if(cb_from == SPI_CB) {
    send_spi_frame_ack(status);
  }
#endif
}

void spi_hal_write(uart_tx_data_t* p_data){

  if(!p_data->tx_in_progress){
#if ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
    //SpiSlaveTx(&p_data->data[p_data->tx_rd_index], p_data->tx_wr_index);
#endif
  }
}

void cmd_hal_write(void)
{
  if(g_rcvd_cmd_frame_interface_type == UART_CB){
    uart_hal_write(&send_data);
  }
  else if(g_rcvd_cmd_frame_interface_type == SPI_CB){
    spi_hal_write(&send_data);
  }
}

uint8_t count = 0;
void CommonCommandHandler(uint8_t cb_from)
{
  uint8_t PayloadSize;
  uint8_t HdrSize = 0x03;
  static uint16_t len;
  uint8_t crc;

  switch (hif.rx_state){
  case RX_INIT:
    if((RxBuffer[0] == SYNCHBYTE0) || (RxBuffer[0] == SOF)){
      hif.rx_state = RX_SOF_1;
      /*reading 2nd byte of SOF*/
      adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[1], 1);
      /* Start guard timer */
      //guard_timer_start(5000);
    }
    else {
#ifdef TELEC_TEST_HARNESS
        if(!hifToTelec(RxBuffer[0]))
        {
            hif.rx_state = TELEC_TEST;
        }
#endif
      /* If received byte is not a SOF, then start receiving next byte from the
      begining of the buffer*/
      adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[0], 1);
    }
    break;

#ifdef TELEC_TEST_HARNESS
case TELEC_TEST:
    if(hifToTelec(RxBuffer[0]))
    {
        hif.rx_state = RX_INIT;
    }
    adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[0], 1);
    break;
#endif

  case RX_SOF_1:
    if((RxBuffer[0] == SYNCHBYTE0) && (RxBuffer[1] == SYNCHBYTE1)){
      hif.uart_frame_type = FRAME_TYPE_UI;
      hif.rx_state = READING_HDR;
      adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[2], 1);
    }
    else if((RxBuffer[0] == SOF) && (RxBuffer[1] == SOF_1)){
      hif.uart_frame_type = FRAME_TYPE_RF_MODULE;
      hif.rx_state = READING_HDR;
      adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[2], 2);
    }
    else{
      /*If the 2nd byte of SOF is mismatched, then reinit the rx state machine*/
//      guard_timer_disable();
//      hif.rx_state = RX_INIT;
//      adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[0], 1);
    }
    break;

  case READING_HDR:
    if(hif.uart_frame_type == FRAME_TYPE_UI){
      len = RxBuffer[2];
      if(len > UART_BUFFER_SIZE){
        /* If the frame length is greater than the Rx buffer size, then drop the
        frame.*/
        PayloadSize = len - HdrSize;
        hif_bytes_to_drop = PayloadSize;
        adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[0], 1);
        hif.rx_state = READ_ERR_BYTES;
        return;
      }
      hif.rx_state = READING_PLD;
      PayloadSize = len - HdrSize;
      adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[3], PayloadSize);
    }
    if(hif.uart_frame_type == FRAME_TYPE_RF_MODULE) {
      len =  ((0x00FF & (uint16_t)RxBuffer[3]) << 8) |
        (0x00FF & (uint16_t)RxBuffer[2]);
      if(len > UART_BUFFER_SIZE){
        /* If the frame length is greater than the Rx buffer size, then drop the
        frame.*/
        guard_timer_disable();
        hif_bytes_to_drop = len + 1;
        adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[0], 1);
        hif.rx_state = READ_ERR_BYTES;
        return;
      }
      hif.rx_state = READING_PLD;
      adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[4], len + 1);
    }
    break;

  case READING_PLD:
    /* Received the complete frame, stop the guard timer */
    guard_timer_disable();
    if(hif.uart_frame_type == FRAME_TYPE_UI){
      hif.rx_state = RX_INIT;
      adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[0], 1);
      g_rcvd_cmd_frame_type = FRAME_TYPE_UI;
      g_rcvd_cmd_frame_interface_type = cb_from;
      process_post(&uart_handler_process, PROCESS_EVENT_POLL, NULL);
    }
    if(hif.uart_frame_type == FRAME_TYPE_RF_MODULE){
      crc = Generate_Checksum( &RxBuffer[4], len );
      if ( crc == RxBuffer[len + 4 ] ){
        if (ACK_PACKET == RxBuffer[4]){
          return;
        }
        if (PRINT_PACKET == RxBuffer[4]){
          return;
        }
        if (PRINT_HEX_PACKET == RxBuffer[4]){
          return;
        }
        g_rcvd_cmd_frame_type = FRAME_TYPE_RF_MODULE;
         g_rcvd_cmd_frame_interface_type = cb_from;
        process_post(&uart_handler_process, PROCESS_EVENT_POLL, NULL);
        send_cmd_frame_ack(cb_from, UART_DATA_SUCCESS);
      }
      else {
        send_cmd_frame_ack(cb_from, UART_DATA_CRC_FAILED);
      }
      //Send data after reception completes
      if (tx_data_pending) {
        cmd_hal_write();
        tx_data_pending = 0;
      }
    }
    break;

  case READ_ERR_BYTES:
    hif_bytes_to_drop--;
    if(hif_bytes_to_drop == 0){
      hif.rx_state = RX_INIT;
      //Send data after reception completes
      if (tx_data_pending) {
        cmd_hal_write();
        tx_data_pending = 0;
      }
    }
    adi_cmd_SubmitRxBuffer(cb_from, &RxBuffer[0], 1);
    break;

  default:
    break;
  }
}
/*----------------------------------------------------------------------------*/

void Check_Uart_State( void )
{
  if(reinit_uart == true) {
    reinit_uart = false;
    guard_timer_disable();
    adi_uart_Close(hUartDevice);
    InitUART();
  }
}
