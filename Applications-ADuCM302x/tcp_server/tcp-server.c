

/*
 * This is a small example of how to write a TCP server using
 * Contiki's protosockets. It is a simple server that accepts one line
 * of text from the TCP connection, and echoes back the first 10 bytes
 * of the string, and then closes the connection.
 *
 * The server only handles one connection at a time.
 *
 */

#include <string.h>

/*
 * We include "contiki-net.h" to get all network definitions and
 * declarations.
 */
#include "contiki-net.h"

#include <gpio/adi_gpio.h>


/*
 * We define one protosocket since we've decided to only handle one
 * connection at a time. If we want to be able to handle more than one
 * connection at a time, each parallell connection needs its own
 * protosocket.
 */
static struct psock ps;

/*
 * We must have somewhere to put incoming data, and we use a 10 byte
 * buffer for this purpose.
 */
static char buffer[256];
PROCESS(example_psock_server_process, "Example protosocket server");
AUTOSTART_PROCESSES(&example_psock_server_process,NULL);

/*---------------------------------------------------------------------------*/
void tcpip_handler(void)
{
  char *str;
  char len = 0;
  len = uip_datalen();
    
  if(uip_newdata()) {
    str = (char*)uip_appdata;
    str[len] = '\0';
    //PRINTF("DATA recv '%s'\n", str);
 
    if(ANode_GetAutoSend())
      Send_ReceivedData2Uart(str,0,len);
  }
}

/*---------------------------------------------------------------------------*/
/*
 * We declare the process.
 */
//PROCESS(example_psock_server_process, "Example protosocket server");
/*---------------------------------------------------------------------------*/
/*
 * The definition of the process.
 */
PROCESS_THREAD(example_psock_server_process, ev, data)
{
  /*
   * The process begins here.
   */
  PROCESS_BEGIN();

  /*
   * We start with setting up a listening TCP port. Note how we're
   * using the UIP_HTONS() macro to convert the port number (1010) to
   * network byte order as required by the tcp_listen() function.
   */
  tcp_listen(UIP_HTONS(1010));

  /*
   * We loop for ever, accepting new connections.
   */
  while(1) 
  {
    PROCESS_YIELD();
    if(ev == tcpip_event) 
    {
      tcpip_handler();
    }
  }
  /*
   * We must always declare the end of a process.
   */
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

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
/*****************************************************************************/

