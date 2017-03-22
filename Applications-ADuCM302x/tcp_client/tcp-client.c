
#include <stdio.h> 
#include "contiki-net.h" 
#include "net/rpl/rpl.h"
#include "net/ipv6/uip-icmp6.h"
#include "net/rpl/rpl-private.h"
#include "ADF7023.h"
#include "ADSensors.h"

#include <gpio/adi_gpio.h>


#include "lpm.h"

#define MAX_PAYLOAD_LEN		 (127)


static struct psock ps; 
static struct psock* psocket;
static uint8_t buffer[256]; 
uint32_t current_sleep_duration = SLEEP_DURATION;

static struct stimer send_packet_timer;
static struct ctimer backoff_timer;
static void led_on(void);
static void led_off(void);
static int tcp_send(void *ptr);
static uint8_t bFlag = TRUE;
uint8_t tcp_connected;
static struct etimer subtimer;

PROCESS(example_client_tcp, "Example protosocket client");
AUTOSTART_PROCESSES(&example_client_tcp,NULL);



static void send_packet(void *ptr)
{  
  if(bFlag == TRUE)
  {  
    bFlag  = FALSE;    
    tcp_send(NULL);
    stimer_set(&send_packet_timer,5);
  }
  else
  {    
    if(stimer_expired(&send_packet_timer))
    {      
      bFlag  = TRUE;
    }  
  }
}
static int tcp_send(void *ptr)
{
  char out_buf[MAX_PAYLOAD_LEN];
  int  len = 0;
  static int LocFailCount = 0;
  rpl_dag_t* currentDag;
  
  if(NULL == uip_ds6_defrt_choose()) //NUD has failed wait for alternate parent selection
  {
    led_off(); 
#ifndef ENABLE_LOW_POWER_MODE  
    ctimer_set(&backoff_timer, current_sleep_duration * 1000, send_packet, NULL);
#endif
    return 0;
  }
  else
  {
    led_on();
  }
  #ifndef SLIM_HOST  
    adsAPI_Read_ADSensorData();
  #endif  
  len = Add_SensData2Payload(out_buf,0, MAX_PAYLOAD_LEN); 
  PSOCK_INIT(&ps, buffer, sizeof(buffer)); 
  psocket = &ps;
  PSOCK_BEGIN(psocket);  
  PSOCK_SEND(psocket,(uint8_t*)out_buf,len);
  PSOCK_END(psocket);
}
PROCESS_THREAD(example_client_tcp, ev, data)
{
   rpl_dag_t* currentDag;

   PROCESS_BEGIN();
   
   #ifndef SLIM_HOST  
   adsAPI_ADSensInit();
   #endif
   while(1)
   {
     while(!tcp_connected)
     {  
       PROCESS_PAUSE();
     }
     {
         currentDag  = rpl_get_any_dag();
         tcp_connect(&currentDag->dag_id, UIP_HTONS(1010), NULL);
         do 
         { 
           PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event); 
           if(uip_connected() || uip_poll())
            send_packet(NULL);
         } while(!(uip_closed() || uip_aborted() || uip_timedout())); 
     }
   }
   PROCESS_END(); 
}
/*****************************************************************************/
static void led_on(void)
{
  adi_gpio_SetLow( ADI_GPIO_PORT1,ADI_GPIO_PIN_12);
}
/*****************************************************************************/

/*****************************************************************************/
static void led_off(void)
{
  adi_gpio_SetHigh( ADI_GPIO_PORT1,ADI_GPIO_PIN_12);
}



 

