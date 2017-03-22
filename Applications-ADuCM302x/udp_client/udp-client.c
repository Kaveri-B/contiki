/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#ifndef ADF7242
  #include "ADF7023.h"
#else
  #include <spi/adi_spi.h>
  #include "ADF7242.h"
#endif
#include <gpio/adi_gpio.h>
#include "rpl/rpl.h"
#include "contiki-conf.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-nd6.h"
#include "net/ipv6/sicslowpan.h"
#include "net/ip/uip-udp-packet.h"
#include "sys/ctimer.h"

#include "net/rpl/rpl.h"

#include "ADSensors.h"
#include "common.h"
#include "sensor-common.h"
#include "lpm.h"
#include "uart_handler.h"
/*---------------------------------------------------------------------------*/
/* Definitions */
/*---------------------------------------------------------------------------*/

#define UDP_SERVER_PORT	1035
#define MCAST_SINK_UDP_PORT 3001 /* Host byte order */

/*---------------------------------------------------------------------------*/
/* Enums */
/*---------------------------------------------------------------------------*/
#define MAX_PAYLOAD_LEN		 (127)

/*---------------------------------------------------------------------------*/
/* Static Variables */
/*---------------------------------------------------------------------------*/

static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *sink_conn;
extern uint32_t current_sleep_duration;




uint8_t udp_connected;

extern uint8_t RxBuffer[];
/*---------------------------------------------------------------------------*/
/* Static Function Declarations */
/*---------------------------------------------------------------------------*/

/*****************************************************************************/
void send_packet(void *);
static uint32_t ExtractSleepDurationTime();
/*****************************************************************************/

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
#ifndef WITH_COAPS
AUTOSTART_PROCESSES(&udp_client_process,NULL);
#endif
/*---------------------------------------------------------------------------*/
/*****************************************************************************/
static void
tcpip_handler(void)
{
  char *RxDataBuf;
  char ByteVal1;
  rpl_dag_t* currentDag;

  if(uip_newdata()) 
  {
    RxDataBuf = (char *)uip_appdata + 13;
    ByteVal1 = GetByteFromBuffer(0, RxDataBuf, 0, 128);
    if(ByteVal1 == 1)
    {
      ByteVal1 = GetByteFromBuffer(0, RxDataBuf, 2, 128);
      if(ByteVal1 == 0)
      {
        LEDControl(LED3,TURN_OFF);
      }
      else if (ByteVal1 == 1)
      {
        LEDControl(LED3,TURN_ON);        
      }
    }
    else if(ByteVal1 == 2)
    {
      if(NULL != rpl_get_default_instance())
        rpl_free_instance(rpl_get_default_instance()); 
    }
    else if(ByteVal1 == 3)
    {
      NVIC_SystemReset();
    }    
    if(ByteVal1 == 4)
    {
      current_sleep_duration = GetLongFromBuffer(0, RxDataBuf, 2, 128)/1000;
      if(current_sleep_duration == 0)
        current_sleep_duration = SLEEP_DURATION;
    }
    else if(ByteVal1 == 5)
    {
        Low_Power_Mode_Init();  
        Node_SetSleepy(true);
    }
    else if(ByteVal1 == 6)
    {
      ByteVal1 = GetByteFromBuffer(0, RxDataBuf, 2, 128);
      if(ByteVal1 == 0)
      {
        Node_EnableStatistics(false);
      }
      else if (ByteVal1 == 1)
      {
        Node_EnableStatistics(true);        
      }
    }  
    else if(ByteVal1 == 9)
    {
        SaveConfigtoFlash();
    }
  }
}
void
send_packet(void *ptr)
{  
  char buf[MAX_PAYLOAD_LEN];
  int  len = 0;
  static int LocFailCount = 0;
  rpl_dag_t* currentDag;

  if(!udp_connected)
    return;
  
  if(Node_IsSleepy())
  {
    if(!Get_Ack_Status())
    {
      LocFailCount++;
      if(LocFailCount >= 4)
      { 
        if(NULL != rpl_get_default_instance())
          rpl_free_instance(rpl_get_default_instance()); 
        LocFailCount = 0;
        udp_connected = 0;
        return;
      }
    }
    else
    {
      LocFailCount = 0;
    }
  }
#ifndef SLIM_HOST 
  /* Disable TRX irq while reading sensor data. Because same SPI is used to read 
  sensor data and TRX, which is resulting in SPI hang */
  Disable_Transceiver_Interrupts();
  adsAPI_Read_ADSensorData();
  //Enable TRX irq after reading sensor data
  Enable_Transceiver_Interrupts();
  
  len = Add_SensData2Payload(buf,0, MAX_PAYLOAD_LEN);
#endif


  currentDag  = rpl_get_any_dag();
  
  uip_udp_packet_sendto(client_conn, buf, len,
                        &currentDag->dag_id, UIP_HTONS(UDP_SERVER_PORT)); 
  
  
}

static void led_on(void)
{
  adi_gpio_SetLow( ADI_GPIO_PORT1,ADI_GPIO_PIN_12);
}

static void led_off(void)
{
  adi_gpio_SetHigh( ADI_GPIO_PORT1,ADI_GPIO_PIN_12);
}


/*****************************************************************************/
uint32_t ADI_App_Get_Sleep_Duration ( void )
{
  return current_sleep_duration;
}
volatile uint32_t dbg_udp_connected_false_count = 0;
static void  router_callback(int event, uip_ipaddr_t *route, uip_ipaddr_t *ipaddr, 
                                                                  int numroutes) 
{ 
   if(event == UIP_DS6_NOTIFICATION_DEFRT_ADD)
   { 
     if(udp_connected == TRUE)
       return;
     led_on();
      
      //sleepy nodes don't need downward routing
     if(Node_IsSleepy())
        rpl_set_mode(RPL_MODE_FEATHER);
        
     udp_connected = TRUE;
     process_post_synch(&udp_client_process, 0, NULL);    
   }
   if(event == UIP_DS6_NOTIFICATION_DEFRT_RM)
   {
     led_off();
     udp_connected = FALSE;
     dbg_udp_connected_false_count++;
   }
   
}
static struct timer send_timer;
uint32_t ADI_LPM_Sleep( ADI_LOW_POWER_MODE_TYPE mode,
                       ADI_WAKEUP_EVENT_TYPE event,
                       uint32_t sleep_duration )
{
  switch ( event )
  {
    case ADI_WAKEUP_EVENT_EXT_IRQ0:
    case ADI_WAKEUP_EVENT_EXT_IRQ1:
    case ADI_WAKEUP_EVENT_EXT_IRQ2:
    case ADI_WAKEUP_EVENT_EXT_IRQ3:
    case ADI_WAKEUP_EVENT_WAKEUP_TIMER:         
      Hibernate_Device(event,sleep_duration);
      break;
    default:
      return ADI_API_INVALID_PARAM;
  }
  return ADI_API_SUCCESS;
}

extern volatile uint8_t interrupt_source;
uint8_t timer_started = 0;
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct uip_ds6_notification n;
  uip_ipaddr_t addr;
  uip_ds6_maddr_t *rv;  
  
  static struct etimer et;
  PROCESS_BEGIN();
  
#ifndef SLIM_HOST  
  adsAPI_ADSensInit();
#endif    
  
  if(Node_IsSleepy())
    Low_Power_Mode_Init();
  
  uip_ds6_notification_add(&n, router_callback);
  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(UDP_SERVER_PORT));

  /*
   * IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
   */
  uip_ip6addr(&addr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  rv = uip_ds6_maddr_add(&addr);

  sink_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(sink_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));
  
  PROCESS_WAIT_EVENT();
    
  while(1) 
  {
       
    while(!udp_connected)
    {
      PROCESS_PAUSE();
    }

    if(!Node_IsSleepy())
    {
      if(!timer_started)
      {  
        send_packet(NULL);
        etimer_set(&et, current_sleep_duration * CLOCK_SECOND);
        timer_started = 1;
      }
      PROCESS_YIELD();

      if(ev == tcpip_event)
      {
        tcpip_handler();
      }         
      else
      {    
        timer_started = 0;
      }
    }
    if(Node_IsSleepy())    
    {
      send_packet(NULL);
      timer_set(&send_timer, current_sleep_duration * CLOCK_SECOND);
      while(LPM_Ready_To_Sleep_Check() != STACK_READY)
      {
        if(timer_expired(&send_timer))
        {
          send_packet(NULL);
          timer_set(&send_timer, current_sleep_duration * CLOCK_SECOND);
        }
        PROCESS_PAUSE();     
      }
      led_off();
      
  #ifdef WAKEUP_PERIODIC
      ADI_LPM_Sleep(ADI_LOW_POWER_MODE_PERIODIC,ADI_WAKEUP_EVENT_WAKEUP_TIMER,ExtractSleepDurationTime());
  #elif defined (WAKEUP_SENSOR_EVENT)
      ADI_LPM_Sleep(ADI_LOW_POWER_MODE_PERIODIC,ADI_WAKEUP_EVENT_EXT_IRQ0,ExtractSleepDurationTime());
  #elif defined (WAKEUP_UART_EVENT)
      ADI_LPM_Sleep(ADI_LOW_POWER_MODE_PERIODIC,ADI_WAKEUP_EVENT_EXT_IRQ1,ExtractSleepDurationTime());
  #else
      ADI_LPM_Sleep(ADI_LOW_POWER_MODE_PERIODIC,ADI_WAKEUP_EVENT_EXT_IRQ2,ExtractSleepDurationTime());
  #endif
      
      if(interrupt_source !=  RTC1_EVT_IRQn)
      {  
        if((interrupt_source ==  XINT_EVT3_IRQn))
        {   
#ifndef SLIM_HOST        
            CollectData();
#endif          
        }      
        else if((interrupt_source ==  XINT_EVT1_IRQn))
        {   
          InitUART();
          Node_SetSleepy(false);
          rpl_set_mode(RPL_MODE_LEAF);
        }
      }  
      led_on();
    }
  }  
  PROCESS_END();
}

static uint32_t ExtractSleepDurationTime()
{
  uint32_t MinimumExpiryTime;
  uint32_t temp1;
  uint32_t temp2;


  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
  
  while(nbr != NULL) 
  {
    temp1 = stimer_remaining(&nbr->reachable)*1000;
    temp2 = stimer_remaining(&nbr->sendns)*1000;
    
    MinimumExpiryTime = (temp1 < temp2)?temp1:temp2;
    
    nbr = nbr_table_next(ds6_neighbors, nbr);
 }
  MinimumExpiryTime = (MinimumExpiryTime < (current_sleep_duration*1000))?
                       MinimumExpiryTime:(current_sleep_duration*1000);
  if( MinimumExpiryTime < 1000 )
    MinimumExpiryTime = 1000;
  return (MinimumExpiryTime/1000);//In Seconds
}


/*---------------------------------------------------------------------------*/


