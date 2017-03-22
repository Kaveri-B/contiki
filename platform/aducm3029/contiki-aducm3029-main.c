/*
Copyright (c) 2013 - 2015, Analog Devices, Inc.  All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted 
(subject to the limitations in the disclaimer below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation 
  and/or other materials provided with the distribution.
* Neither the name of Analog Devices, Inc. nor the names of its contributors 
  may be used to endorse or promote products derived from this software without 
  specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
* \file
* 		Main implementation file of Contiki Port for ADucRF101
* \author
*    Analog Devices
*/

#include <stdio.h>
#include <string.h>

#include <tmr/adi_tmr.h>
#include <wdt/adi_wdt.h>
#include <gpio/adi_gpio.h>
#include <uart/adi_uart.h>
#include <spi/adi_spi.h>
#include <flash/adi_flash.h>
#include <adi_pwr_config.h>
#include "net/ip/uip.h"
#include "contiki-conf.h"
#include "contiki-net.h"

#include "utest_support.h"
#include "sys/stimer.h"
#include <uart/adi_uart.h>
#include "uart_handler.h"
#include "startup.h"             /* Obtain defintion of WEAK */
#include "hif_utility.h"
#include "RF_Module_API_Handler.h"

#define DEV_TYPE_HOST           1
#define DEV_TYPE_ROUTER         2
#define DEV_TYPE_EDGE_ROUTER    3
#define DEV_TYPE_NONE           4
#define DEV_TYPE                DEV_TYPE_NONE
/* Handle for UART device */
#pragma data_alignment=4
ADI_UART_HANDLE          hUartDevice;

/* Memory for  UART driver */
#pragma data_alignment=4
static uint8_t UartDeviceMem[ADI_UART_MEMORY_SIZE];

#pragma data_alignment=4
ADI_FEE_HANDLE hFlashDevice;

#pragma data_alignment=4
static uint8_t FlashDeviceMem[ADI_FEE_MEMORY_SIZE];

uip_ipaddr_t prefix_configured = {0x00,0x00,0x00,0x00,0x00,0x00,0xaa,0xaa};

uint16_t pan_id_configured = {0xaaaa};
uint16_t accept_dio_rank = 0;
uint8_t sensor_end_node = 0;
uip_802154_longaddr ieee_802154_extended_addr = {0x00, 0x05, 0xf7, 0xfe, 0xff,0x00,0x00,01};

#ifdef  LALG_LAYER_ON
#include "LALG_layer.h"
 
 
#endif
/* Enum variable to hold the device type. */
#if (DEV_TYPE == DEV_TYPE_EDGE_ROUTER)
NodeType_t g_node_type = NODE_6LBR;
#elif (DEV_TYPE == DEV_TYPE_ROUTER)
NodeType_t g_node_type = NODE_6LR;
#elif (DEV_TYPE == DEV_TYPE_HOST)
NodeType_t g_node_type = NODE_6LN;
#else
NodeType_t g_node_type = NODE_6LBR;
#endif

RPL_MOP_Type_t g_RPL_MOP_type = RPL_MOP_TYPE_NON_STORING;
/* Host application interface structure. */
extern hif_t hif;

typedef enum 
{ 
  TURN_OFF = 0x00,
  TURN_ON	
} LED_OnOffState;


#define LED3  0   //used in main.c
#define LED4  1  //used in UDP Client.c and UDP Server.c  
#define LED5  2  //Not used
#define LED1  3

#define WDT_PERIODIC_RESET_DURATION     30
void LEDControl(int WhichLED, LED_OnOffState State);

uint8_t WdtMemory[ADI_WDT_MEMORY_SIZE];

ADI_WDT_HANDLE hWDTDevice=NULL;

extern void Check_Uart_State( void );

void Enable_WDT()
{


  if (adi_wdt_Open(ADI_WDT_DEVID_0, &WdtMemory, sizeof(WdtMemory),&hWDTDevice)) {
    printf("adi_wdt_Open failed\n");
  }


  if( adi_wdt_Enable(hWDTDevice, true) != ADI_WDT_SUCCESS )
  {
    printf("adi_wdt_Enable failed\n");
  }

}


void Disable_WDT()
{


  if (adi_wdt_Open(ADI_WDT_DEVID_0, &WdtMemory, sizeof(WdtMemory),&hWDTDevice)) {
    printf("adi_wdt_Open failed\n");
  }

  if( adi_wdt_Enable(hWDTDevice, false) != ADI_WDT_SUCCESS )
  {
    printf("adi_wdt_Enable failed\n");
  }
  
  if( adi_wdt_Close(hWDTDevice) != ADI_WDT_SUCCESS)
  {
    printf("adi_wdt_Close failed\n");
  }
}

void InitLEDs(void)
{
    ADI_GPIO_RESULT eGpioResult;
    static uint8_t gpioMemory[ADI_GPIO_MEMORY_SIZE];
    volatile int i, j, k;
    
    /* init the GPIO service */
    if(eGpioResult = adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE))
    {
        return;
    }
    
    /* set GPIO output LED 4 and 5 */
    if(eGpioResult = adi_gpio_OutputEnable(ADI_GPIO_PORT1, (ADI_GPIO_PIN_12 | ADI_GPIO_PIN_13), true))
    {
        return;
    }
    
    /* set GPIO output LED 3 */
    if(eGpioResult = adi_gpio_OutputEnable(ADI_GPIO_PORT0, ADI_GPIO_PIN_13, true))
    {
        return;
    }
    
    for(i = 0;i < 250;i++)
    {
      for(j = 0;j < 25;j++)
      {
        for(k = 0;k < 2;k++)
        {
           k++;
           k--;
        }
      }
    }
    
    LEDControl(LED3,TURN_OFF);
    LEDControl(LED4,TURN_OFF);
    LEDControl(LED5,TURN_OFF);
}

/*-----------------------------------------------*/
/*              Turn the LEDs ON/OFF             */
/*-----------------------------------------------*/
void LEDControl(int WhichLED, LED_OnOffState State)
{
    if(WhichLED == LED3)
    {
      if(State == TURN_ON)
      { 
        adi_gpio_SetLow(ADI_GPIO_PORT0, ADI_GPIO_PIN_13);
      }
      else
      { 
        adi_gpio_SetHigh(ADI_GPIO_PORT0, ADI_GPIO_PIN_13);        
      }
    }
    else if(WhichLED == LED4)                                                     	
    {                                                                       
      if(State == TURN_ON)
      {
        adi_gpio_SetLow( ADI_GPIO_PORT1, ADI_GPIO_PIN_12);
      }
      else
      {
        adi_gpio_SetHigh( ADI_GPIO_PORT1, ADI_GPIO_PIN_12);
      } 
    }
    else if(WhichLED == LED5)                                            
    {
      if(State == TURN_ON)
      {
        adi_gpio_SetLow( ADI_GPIO_PORT1, ADI_GPIO_PIN_13);
      }
      else
      {
        adi_gpio_SetHigh( ADI_GPIO_PORT1, ADI_GPIO_PIN_13);
      }
    }

    else if(WhichLED == LED1)
    {
      if(State == TURN_ON)
      {
        adi_gpio_SetLow( ADI_GPIO_PORT1, ADI_GPIO_PIN_13);
      }
      else
      {
        adi_gpio_SetHigh( ADI_GPIO_PORT1, ADI_GPIO_PIN_13);
      }
    }
}

extern unsigned char 	RxBuffer[];

void InitUART()
{
  /* Variable for storing the return code from UART device */
  ADI_UART_RESULT  eUartResult;

 adi_uart_Open(UART_DEVICE_NUM,ADI_UART_DIR_BIDIRECTION,UartDeviceMem,ADI_UART_MEMORY_SIZE,&hUartDevice);

#if (ADI_UART_CFG_ENABLE_STATIC_CONFIG_SUPPORT != 1)
  /* Configure  UART device with NO-PARITY, ONE STOP BIT and 8bit word length. */
  adi_uart_SetConfiguration(hUartDevice,
                                              ADI_UART_NO_PARITY,
                                              ADI_UART_ONE_AND_HALF_TWO_STOPBITS,
                                              ADI_UART_WORDLEN_8BITS);
  
  /* Baud rate div values are calcuated for PCLK 26Mhz. Please use the
  host utility UartDivCalculator.exe provided with the installer"
  */
  adi_uart_ConfigBaudRate(hUartDevice,
                                            UART_DIV_C_460800,
                                            UART_DIV_M_460800,
                                            UART_DIV_N_460800,
                                            UART_OSR_460800);
#endif

  /* Enable the Data flow for Rx */
  adi_uart_EnableRx(hUartDevice,true);
  /*Initialize the UART transmission structure.*/
  RFM_cmd_tx_struct_init();
  /* Initialize Host interface state.*/
  hif.rx_state = RX_INIT;
  adi_uart_RegisterCallback(hUartDevice,UARTCallback, &hif);
  adi_uart_SubmitRxBuffer(hUartDevice,RxBuffer,1);
}

#if  ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
void Init_RFM_SPI(void){
  //ConfigureSlaveSPI(ADI_SPI_DEVID_0,ADI_SPI_CS0);
  RFM_cmd_tx_struct_init();
//  hif.uart_frame_type = FRAME_TYPE_RF_MODULE;
//  hif.rx_state = READING_PLD;
//  SpiSlaveRxSubmitBuffer(RxBuffer, 1);
  hif.rx_state = RX_INIT;
  //SpiSlaveRxSubmitBuffer(RxBuffer,1);
}
#endif

void
init_hw(void)
{
  SystemInit();

  adi_initpinmux();

//  Enable_WDT();
  Disable_WDT();

  adi_pwr_Init();

#if (ADI_PWR_CFG_ENABLE_STATIC_CONFIG_SUPPORT != 1)
  if(ADI_PWR_SUCCESS !=(adi_pwr_SetLFClockMux(ADI_CLOCK_MUX_LFCLK_LFXTAL)))
  {

  }

  if(ADI_PWR_SUCCESS !=(adi_pwr_EnableClockSource(ADI_CLOCK_SOURCE_LFXTAL,true)))
  {

  }  
  
  adi_pwr_SetClockDivider(ADI_CLOCK_HCLK,1);
  
  adi_pwr_SetClockDivider(ADI_CLOCK_PCLK,1);
#endif

  InitLEDs();
  
  InitUART();

#if ((DEV_TYPE != DEV_TYPE_HOST) && (DEV_TYPE != DEV_TYPE_EDGE_ROUTER) && (DEV_TYPE != DEV_TYPE_ROUTER))
  Init_RFM_SPI();
#endif

#ifndef LLSEC_DISABLE
  crypto_init();
#endif  
  
  adi_fee_Open(0, FlashDeviceMem, sizeof(FlashDeviceMem), &hFlashDevice);
  NVIC_SetPriority(SPI2_EVT_IRQn,0);

}

/*-------------------------------------------------------------------------*/

void
contiki_os_net_init(void)
{
  
  /* Set addresses BEFORE starting tcpip process */
  /* Set the link local address. */
  
  NVIC_SetPriority(UART_EVT_IRQn, (NVIC_GetPriority(SPI2_EVT_IRQn) + 1));
  NVIC_SetPriority(SYS_GPIO_INTA_IRQn, (NVIC_GetPriority(UART_EVT_IRQn) + 1));
  NVIC_SetPriority(TMR0_EVT_IRQn, (NVIC_GetPriority(SYS_GPIO_INTA_IRQn) + 1));
  NVIC_SetPriority(XINT_EVT0_IRQn, (NVIC_GetPriority(TMR0_EVT_IRQn) + 1));
  NVIC_SetPriority(XINT_EVT1_IRQn, (NVIC_GetPriority(XINT_EVT0_IRQn)));
  NVIC_SetPriority(XINT_EVT2_IRQn, (NVIC_GetPriority(XINT_EVT0_IRQn)));
  NVIC_SetPriority(XINT_EVT3_IRQn, (NVIC_GetPriority(XINT_EVT0_IRQn)));
  NVIC_SetPriority(RTC1_EVT_IRQn, 1);
#ifndef UL_NODE
  memcpy(&uip_lladdr.addr, &ieee_802154_extended_addr, sizeof (uip_lladdr));
#endif
  linkaddr_set_node_addr((linkaddr_t *)&ieee_802154_extended_addr);
  
  /* Clock */
  tmr_service_init();

  //use MAC address to seed
  random_init(ieee_802154_extended_addr.addr[0] + \
              ieee_802154_extended_addr.addr[1] + \
              ieee_802154_extended_addr.addr[2] + \
              ieee_802154_extended_addr.addr[3] + \
              ieee_802154_extended_addr.addr[4] + \
              ieee_802154_extended_addr.addr[5] + \
              ieee_802154_extended_addr.addr[6] + \
              ieee_802154_extended_addr.addr[7]);
  
  /* Initialize process subsystem */
  process_init();
  
  /* etimers must be started before ctimer_init */
  process_start(&etimer_process, NULL);
  
  ctimer_init();
  
  queuebuf_init();
  
#if defined(WIRESHARK_FEEDER) || defined(FOREN6_FEEDER)
  /* Only Init the ADucM3025 driver*/
  NETSTACK_RADIO.init();
#else

  /* Start Network */
  netstack_init();
#endif

#if ((DEV_TYPE == DEV_TYPE_HOST) || (DEV_TYPE == DEV_TYPE_EDGE_ROUTER) || (DEV_TYPE == DEV_TYPE_ROUTER))
  update_phy_params();

#ifndef UL_NODE
  /* Start tcpip process */
  process_start(&tcpip_process, NULL);

  #if (defined(ETHERNET_6LBR) && (DEV_TYPE == DEV_TYPE_EDGE_ROUTER))
  ip64_init();
  #endif
#endif
#endif

  #ifndef UL_NODE

      #ifndef WIRESHARK_FEEDER
        #if ((DEV_TYPE == DEV_TYPE_HOST) || (DEV_TYPE == DEV_TYPE_EDGE_ROUTER))
        autostart_start(autostart_processes);
        #endif
      #endif
  #endif
}



/*-------------------------------------------------------------------------*/
/*------------------------- Main Scheduler loop----------------------------*/
/*-------------------------------------------------------------------------*/
PROCESS_NAME(border_router_process);
PROCESS_NAME(uart_handler_process);
PROCESS_NAME(LWM2M_packet_process);

#ifdef LALG_LAYER_ON
PROCESS_NAME(lalg_packet_process);
#endif
#if(RESTCOAP)
PROCESS_NAME(rest_engine_process);

#endif
extern ADI_SPI_HANDLE hSpiSlaveDev;

static struct timer wdt_reset_timer;
 int
main(void)
{
  init_hw();

  config_init(true);

#ifdef ENABLE_LOW_POWER_MODE
  if(Node_IsSleepy())
      Low_Power_Mode_Init();
#endif /* ENABLE_LOW_POWER_MODE */

  contiki_os_net_init();
  utu_initialise();

#if(RESTCOAP)
  process_start(&rest_engine_process, NULL);

#elif(SLIPRADIO)
  process_start(&adapter_init, NULL);
  process_start(&slip_process, NULL);
#else
#if (DEV_TYPE == DEV_TYPE_EDGE_ROUTER)
  process_start (&border_router_process, NULL);
#endif
#endif
#if (LALG_LAYER_ON)
process_start (&lalg_packet_process, NULL);

#endif
#if(UL_NODE)
  process_start (&LWM2M_packet_process, NULL);
#endif
  process_start(&uart_handler_process, NULL);


  timer_set(&wdt_reset_timer, WDT_PERIODIC_RESET_DURATION * CLOCK_SECOND);


  while(1) {
    process_run();
    if(timer_expired(&wdt_reset_timer))
    {
      LEDControl(LED5,TURN_ON);
      adi_wdt_ResetTimer(hWDTDevice);
      LEDControl(LED5,TURN_OFF);
      timer_set(&wdt_reset_timer, WDT_PERIODIC_RESET_DURATION * CLOCK_SECOND);
    }
    Check_Uart_State();
  }
}

void watchdog_periodic(void)
{
  
}

 void GlowAllLEDs(void)
 {
   LEDControl(LED3,TURN_ON);
   LEDControl(LED4,TURN_ON);
   LEDControl(LED5,TURN_ON);
   
 }
 