/***********************************************************************************************************************

Copyright (c) 2013 - 2015, Analog Devices, Inc.  All rights reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.

***********************************************************************************************************************/

//#include "utils/inc/common.h"
#include "common.h"
#include "utest_support.h"
#include <int/adi_int.h>
#include <flash/adi_flash.h>
#include <string.h>

#ifdef UTEST_SUPPORT

/* storage for stamps */
utu_stamp_t utu_stamp[MAX_STAMPS];
volatile int utu_index;
const static uint8_t log_valid_string[4] = {'L','O','G','S'};


/* labels for printing */
static char* label[] = {
    "NONE",
    "T-INIT  ",
    "T-IDLE  ",
    "T-TX_BCN",
    "T-BCAST ",
    "T-W_BCN ",
    "T-TX    ",
    "T-W_ACK ",
    "T-TX_ACK",
    "T-CFP   ",
    "T-OFF   ",
    "T-RSSI  ",
    "SI-NONE ",
    "SO-NONE ",
    "SI-BCN  ",
    "SO-BCN  ",
    "SI-CAP  ",
    "SO-CAP  ",
    "SI-CFP  ",
    "SO-CFP  ",
    "SI-INACT",
    "SO-INACT",
    "N-NONE  ",
    "N-ED    ",
    "N-ACTPAS",
    "N-ORPHAN",
    "Y-STOP  ",
    "Y-SEARCH",
    "Y-TRACK ",
    "P-IDLE  ",
    "P-ACTIVE",
    "P-PASS  ",
    "R-IDLE  ",
    "R-W_BCN ",
    "R-W_CRA ",
    "R-PIB   ",
    "C-IDLE  ",
    "C-DELAY ",
    "C-CCA   ",
    "C-ISSUCC",
    "C-ISFAIL",
    "A-rx_on ",
    "A-sample",
    "A-stop  ",
    "A-off   ",
    "L-IDLE  ",
    "L-BOFF  ",
    "L-ACTIVE",
    "L-SUSP  ",
    "L-WARMUP",
    "L-DELAY ",
    "L-SAMPLA",
    "L-SAMPLB",
    "D-NONE  ",
    "D-QUEUED",
    "D-PENDIN",
    "D-ARRIVD",
    "P-ACTIVE",
    "P-SNOOZE",
    "P-SLEEP ",
    "P-DSLEEP",
    "A-DISASC",
    "A-PENDIN",
    "A-ASSOC ",
    "I-ISR   ",
    "I-PHY   ",
    "I-SPI   ",
    "I-12M   ",
    "I-32K   ",
    "12M-KICK",
    "TX-ON   ",
    "TX-OFF  ",
    "RX-START",
    "RX-IND  ",
    "RX-BCN  ",
    "RX-DATA ",
    "RX-CMD  ",
    "RX-ACK  ",
    "RX-ERR  ",
    "spi_on  ",
    "spi_off ",
    "alarm   ",
    "realign ",
    "action  ",
    "usr1    ",
    "usr2    ",
    "usr3    ",
    "_ErrW_  ",
    "_ErrO_  ",
    "_ErrK_  ",
    "_ErrR_  ",
    "_ErrC_  ",
    "_ErrP_  ",
    "_ErrPHY_",
    "check   ",
    "tc-pkt-tx-trig",
    "tc-tx-win-started",
    "tc-cca-trigg",
    "tc-cca-on",
    "tc-cca-stopped",
    "tc-rssi-start",
    "tc-rssi-end",
    "tc-cca-done",
    "tc-1-ms-expiry",
    "tc-10ms-exp"
};


/* print stamps */
void utu_print( void )
{
    int i, j;
    long z0 = 0, delta = 0;

    z0 = utu_stamp[utu_index].time;

    /* print timestamps calculating time offset */
    for( i = utu_index; i < utu_index + MAX_STAMPS; i++ )
    {
        j = i & ( ( 1 << MAX_STAMPS_ORDER ) - 1 );

        /* don't print if empty */
        if( utu_stamp[j].label == UTUL_NONE ) continue;

        /* calculate offset */
        delta = utu_stamp[j].time - z0;

        /* set next reference */
        z0 = utu_stamp[j].time;

        /* print timestamp
        debug(("\r\n%3d / %s / %9lu / %6ld symb %4ld tick / %ld",
               j,
               label[ utu_stamp[j].label ],
               utu_stamp[j].time,
               (long) (delta / HWTIMER_SYMBOL_LENGTH),
               (long) (delta % HWTIMER_SYMBOL_LENGTH),
               (long) utu_stamp[j].data)); */
    }
}


/* pack unit test timestamps */
uint16_t utu_pack( void *obj, uint8_t *dst )
{
    memcpy( (void *) dst, (void *) &utu_index, sizeof( utu_index ) );
    dst += sizeof( utu_index );
    memcpy( (void *) dst, utu_stamp, sizeof( utu_stamp ) );
    return sizeof( utu_index ) + sizeof( utu_stamp );
}


/* unpack unit test timestamps */
void utu_unpack( void *obj, uint8_t *src )
{
    memcpy( (void *) &utu_index, (void *) src, sizeof( utu_index ) );
    src += sizeof( utu_index );
    memcpy( utu_stamp, (void *) src, sizeof( utu_stamp ) );
}

uint8_t
Write_log_To_Flash(void)
 {
  #define LOG_STABLE_STORAGE_SEC_1 (118 * 1024)
  #define LOG_STABLE_STORAGE_SEC_2 (120 * 1024)
  #define FLASH_ALIGN_TO_4(x)  (x += x%4)
  
//    uint32_t  pnPage, size = 0;
//    uint32_t fptr = LOG_STABLE_STORAGE_SEC_2;
//    uint32_t page_start = LOG_STABLE_STORAGE_SEC_2;
//    uint8_t flash_write_status;
//    uint8_t i = 0;
//    
//    size = sizeof(utu_stamp[MAX_STAMPS]);
//    size += sizeof(log_valid_string) + (2 * sizeof(uint32_t));
//    size /= 512;
//                
//    if((flash_write_status = adi_Flash_GetPageNumber (page_start,&pnPage)) !=
//    ADI_FLASH_SUCCESS )
//    {
//      return flash_write_status;
//    }
//        
//    for (i = 0; i<= size; i++) 
//    {
//      if(ADI_FLASH_SUCCESS != (flash_write_status = adi_Flash_ErasePage(pnPage + i))) 
//      {
//        return flash_write_status; //use size
//      }
//    }
//
//    if(ADI_FLASH_SUCCESS !=
//      (flash_write_status = adi_Flash_Write(fptr, (uint8_t *)&utu_stamp, sizeof(utu_stamp[MAX_STAMPS])))) 
//    {
//        return flash_write_status;
//    }
//    
//    fptr += sizeof(utu_stamp[MAX_STAMPS]);
//    FLASH_ALIGN_TO_4(fptr);
//   
//    fptr = LOG_STABLE_STORAGE_SEC_2;
//    if(ADI_FLASH_SUCCESS !=
//      (flash_write_status = adi_Flash_Write (fptr, (uint8_t *)log_valid_string, sizeof(log_valid_string)))) {
//      return flash_write_status;
//    }
//     return flash_write_status;
        
  }

void utu_timestamp( ts_lable_t l, unsigned long d ){
        utu_stamp[utu_index].time = clock_time(); \
        utu_stamp[utu_index].data = ( d ); \
        utu_stamp[utu_index].label = (ts_lable_t)( l ); \
        utu_stamp[utu_index].diff = utu_stamp[utu_index].time - utu_stamp[(utu_index)?utu_index-1:(MAX_STAMPS-1)].time;\
        utu_index++; \
        utu_index = utu_index & ( ( 1 << MAX_STAMPS_ORDER ) - 1 ); 
}

/* initialise stamps */
void utu_initialise( void)
{
    int i;
    utu_index = 0;
    for( i = 0; i < MAX_STAMPS ; i++ )
    {
        utu_stamp[i].time = 0;
        utu_stamp[i].data = 0;
        utu_stamp[i].label = UTUL_NONE;
    }
}
#endif//#ifdef UTEST_SUPPORT


