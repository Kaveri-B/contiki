#ifdef APP_UART

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "common.h"
#include "uip.h"

#include "contiki-conf.h"   // to access the common UART input structure

#include "flash.h"
#include "uart_handler.h" 

const static uint8_t valid_string[8] = {'U','A','R','T','C','O','N','F'};

static uint8_t cmd_state = STATE_UART_LENGTH, uart_length;
static uint8_t user_input_cur_data_index = 0;
uint8_t user_input_buf[DEMOAPP_USER_INPUT_LENGTH];
uint8_t i=0;

struct ctimer uart_monitor;
void adi_uart_monitor( void );
void Process_UART_Rx_Data(void);

UART_CONFIG_PARAMETERS uart_app_info;


void
adi_uart_monitor()
{
  ctimer_stop( &uart_monitor );
  user_input_cur_data_index = 0;
  cmd_state = STATE_UART_LENGTH;
  printf("command receive failed resetting UART \n");
}


/*****************************************************************************/

static uint8_t
App_Write_To_Flash(void)
{
#define STABLE_STORAGE_SEC_1 (120 * 1024)
#define STABLE_STORAGE_SEC_2 (122 * 1024)
#define FLASH_ALIGN_TO_4(x)  (x += x%4)

	uint32_t  pnPage, size = 0;
	uint32_t fptr = STABLE_STORAGE_SEC_2;
	uint32_t page_start = STABLE_STORAGE_SEC_2;
        uint8_t flash_write_status;

	size = sizeof(UART_CONFIG_PARAMETERS);
	size += sizeof(valid_string) + (2 * sizeof(uint32_t));

	size /= 512;

	#if size > 3
		#error "Size of list exceeds the flash size provided,2k"
	#endif

  if((flash_write_status = adi_Flash_GetPageNumber (page_start,&pnPage)) !=
      ADI_FLASH_SUCCESS ) {
    return flash_write_status;
  }

  for (i = 0; i<= size; i++) {
    if(ADI_FLASH_SUCCESS != (flash_write_status = adi_Flash_ErasePage(pnPage + i))) {
      return flash_write_status; //use size
    }
  }

  fptr += sizeof(valid_string);
  if(ADI_FLASH_SUCCESS !=
    (flash_write_status = adi_Flash_Write (fptr, (uint8_t *)&uart_app_info, sizeof(UART_CONFIG_PARAMETERS)))) {
    return flash_write_status;
  }
  fptr += sizeof(UART_CONFIG_PARAMETERS);
  FLASH_ALIGN_TO_4(fptr);

  fptr = STABLE_STORAGE_SEC_2;
  if(ADI_FLASH_SUCCESS !=
    (flash_write_status = adi_Flash_Write (fptr, (uint8_t *)valid_string, sizeof(valid_string)))) {
    return flash_write_status;
  }

	return flash_write_status;
}

/*****************************************************************************/


void Process_UART_Rx_Data(void)
{
  uint8_t flash_write_status = ADI_FlASH_ERR_UNKNOWN_ERROR;
  uint8_t reverse[2]= {0x00};
  
  switch(user_input_buf[0])
  {
   case PAN_ID:
   
     /*copy received pan ID in reverse order to local array*/
      for(i =0; i<sizeof(reverse); i++) 
      {
        reverse[i] = user_input_buf[sizeof(uint16_t)-i];
      }
     
      /* copy pan id freom local array to api_conf array*/
       memcpy(&uart_app_info.pan_id,&reverse[0], sizeof(uint16_t));
       
   break;
    
   case IEEE_ADDRESS:
       /*Copy the entire ieee address */
       for(i=1; i<9; i++)
       {
           uart_app_info.ieee_id[i-1]= user_input_buf[i];
       }
    break;
  
   case UART_READ_COMPLETE_STATE:
        /*Write the configured value to Flash*/
        flash_write_status = App_Write_To_Flash();
        if ( flash_write_status != ADI_FLASH_SUCCESS )
        {  
          /* handle error condition*/
        }
   break; 
    
   default:
   
   break;
  }
   
}


void ADI_UART_Receive_CB ( uint8_t uart_data )
{
  switch(cmd_state) /*handlers state*/
  {
  case STATE_UART_LENGTH:
    ctimer_set( &uart_monitor, 2 * CLOCK_SECOND,(void(*)(void *)) adi_uart_monitor, NULL );
    uart_length = uart_data;
    cmd_state = STATE_CMD_PARAM;
    break;
    
  case STATE_CMD_PARAM:
    uart_length --;
    user_input_buf[user_input_cur_data_index] = uart_data;
    if( uart_length == 0) 
    {
      cmd_state = STATE_UART_LENGTH;
      printf("command received  \n");
      Process_UART_Rx_Data();
      user_input_cur_data_index = 0;
      ctimer_stop( &uart_monitor );
    }
    else 
    {
      user_input_cur_data_index ++;
    }
    break;
    
  default:
    break;
  }
}


static uint8_t
App_Read_From_Flash(UART_CONFIG_PARAMETERS * app_info_stored)
{
#define STABLE_STORAGE_SEC_1 (120 * 1024)
#define STABLE_STORAGE_SEC_2 (122 * 1024)
#define FLASH_ALIGN_TO_4(x)  (x += x%4)

	uint32_t  pnPage, size = 0;
	uint32_t page_start = STABLE_STORAGE_SEC_2;
        uint8_t valid_string_stored[8];
        uint8_t status = ADI_FlASH_ERR_UNKNOWN_ERROR;

	size = sizeof(UART_CONFIG_PARAMETERS);
	size += sizeof(valid_string) + (2 * sizeof(uint32_t));

	size /= 512;

	#if size > 3
		#error "Size of list exceeds the flash size provided,2k"
	#endif

  if( (status = adi_Flash_GetPageNumber (page_start,&pnPage)) != ADI_FLASH_SUCCESS ) {
    return status;
  }

  memcpy(valid_string_stored, (uint8_t *)page_start, sizeof(valid_string_stored));
  if ( !memcmp(valid_string_stored, valid_string, sizeof(valid_string_stored)) )
  {
    /* Memory Matched. Got the required configuration. Break. */
    memcpy((uint8_t *)app_info_stored,
           (uint8_t *)(page_start + sizeof(valid_string_stored)),
           sizeof(UART_CONFIG_PARAMETERS));

    status = ADI_FLASH_SUCCESS;
  }
  else
  {
    status = ADI_FLASH_ERR_NO_VALID_DATA;
  }

	return status;
}


/*****************************************************************************/

extern uip_802154_longaddr ieee_802154_extended_addr;
/* Default ieee value */
uip_802154_longaddr DEFAULT_MAC_ADDRESS =  {0xff,0xbb, 0xbb,0xbb,0xbb,0xbb,0xbb,0xbb};
//uint16_t mac_dst_pan_id, mac_src_pan_id ;


void
UART_config_init()
{   
  /* Read memory for earlier written values */ 
   if ( App_Read_From_Flash(&uart_app_info) != ADI_FLASH_SUCCESS )
   {
     /*This is first time , so Write the default values into the memory*/
      if ( App_Write_To_Flash() != ADI_FLASH_SUCCESS )
      {
        /* Handle error condition if write fails*/
      }
   }   
  
  /* copy the read values into relevant global varialbles*/ 
   memcpy(&ieee_802154_extended_addr, &uart_app_info.ieee_id, sizeof(uip_802154_longaddr));
  }

#endif /* APP_UART */