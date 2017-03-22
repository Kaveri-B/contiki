/*!*****************************************************************************
 * @file:    uart_hal.c
 * @brief:   UART Callback for TX and RX Events.
 * @details: 
 * @version: $Revision:
 * @date:    $Date:
 -----------------------------------------------------------------------------
Copyright (c) 2010-2014 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-
INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF
CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/
#include <string.h>
#include "common.h"
#include <drivers/uart/adi_uart.h>
#include "uart_hal.h"
#include "core_cm3.h"
#include "adi_uart_config.h"

/*
** ============================================================================
** Private Macro definitions
** ============================================================================
*/
/**< default maximum buffer size*/
 #define UART_DEVICE_NUM 0 
/* Memory required by the driver for DMA mode of operation */
#define ADI_UART_MEMORY_SIZE    (ADI_UART_BIDIR_MEMORY_SIZE)

/*
** ============================================================================
** Public Variable Definitions
** ============================================================================
*/

uart_hal_data_t uart_hal_info;
 
uint16_t requested_tx_byte_count = 0;

static tx_data_t* mp_send_data;

/* Memory for  UART driver */
#pragma data_alignment=4
static uint8_t UartDeviceMem[ADI_UART_MEMORY_SIZE];

#pragma data_alignment=4
static ADI_UART_HANDLE          hDevice;


/*-----------------------------------------------------------------------------
  UART device driver internal functions
*----------------------------------------------------------------------------*/
void uart_write(ADI_UART_HANDLE  *const   pDevice, uint8_t data);
void hif_handle_rcv_data(uint8_t byte, uint8_t skip_circular_buf);
uint8_t uart_read(ADI_UART_HANDLE  *const   hDevice);
void ADI_UART_ENABLE_RX_EXT(ADI_UART_HANDLE const hDevice);
void ADI_UART_ENABLE_TX_EXT(ADI_UART_HANDLE const hDevice);
void update_comm_type(uint8_t type);
 
/*-----------------------------------------------------------------------------
  UART inline functions
*----------------------------------------------------------------------------*/

/*!
* @brief                 UART Initialization
*
* @param[in]   devID     Enumerated UART device identifier. This id is used to open the
*                        correct instance of the peripheral. ADI_UART_DEVID_0 is the only
*                        allowed value for the systems with a single UART.
* @param[out]  pHandle   Points to the location where the device handle will be stored upon
*                        successful initialization.
*
* @param[in]   pInitData Pointer to the #ADI_UART_INIT_DATA stucture. Application uses this structure to
*                        supply initialization parameters to the init function. If NULL is passed then
*                        UART driver operates in polling mode by polling the status bits. With valid receive
*                        and transmit buffers the driver operates in interrupt mode with internal buffering.
*
* @return      Status
*                        - #ADI_UART_SUCCESS                     upon success
*                        - #ADI_UART_ERR_INVALID_DEVID       [D] if invalid device id is passed
*                        - #ADI_UART_ERR_ALREADY_INITIALIZED [D] if the uart is already initialized
*                        - #ADI_UART_ERR_INVALID_BUFFER      [D] pInitData contains invalid buffer(s)
*
* @details
*                        Initializes and configures UART. Upon successful initialization a handle
*                        to the device is stored in the supplied pHandle.Applications must use this
*                        handle value in all subsequent api calls to this device. Device initialization
*                        includes configuring the GPIO pins, setting up UART with default baudrate,word
*                        length, parity and stop bits. Two buffers one for receive and one for transmit
*                        were also initialized.
*
*                        Init call sets up UART with default initialization values.Default UART values are
*                        baud rate -> 9600, word length -> 8, parity -> none, stop bits -> 1.
*                        Default initialization values for UART are controlled through the following macros
*                        in the UART header file.
*
*                        - #ADI_UART_COMLCR_INITIALIZER   configure word length and stop bits.
*                        - #ADI_UART_BAUD_9600            default baud rate
*
* @note                  Init call does not enable the UART device. Once configuration and other setup is complete
*                        applications has to explicitly enable UART via adi_UART_Enable call. Once UART is enabled
*                        changing the UART configuration such as baudrate, wordlength, operating mode (interrupt,poll,
*                        or dma) may result in un-desired behavior. Applications must disable UART before changing UART
*                        configuration and enable UART after setting the new configuration.
*
* @sa                    adi_UART_UnInit
* @sa                    adi_UART_Enable
*/
void uart_hal_init()
{
    ADI_UART_RESULT  eUartResult;
    /* Open the UART device.Data transfer is bidirectional with NORMAL mode by default.  */
    eUartResult = adi_uart_Open(UART_DEVICE_NUM, 
                                ADI_UART_DIR_BIDIRECTION,
                                UartDeviceMem,
                                ADI_UART_MEMORY_SIZE,
                                &hDevice);

    /* Configure  UART device with NO-PARITY, ONE STOP BIT and 8bit word length. */
    if(eUartResult == ADI_UART_SUCCESS)
        eUartResult = adi_uart_SetConfiguration(hDevice,
                                                ADI_UART_NO_PARITY,
                                                ADI_UART_ONE_STOPBIT,
                                                ADI_UART_WORDLEN_8BITS);

    /* Baud rate div values are calcuated for PCLK 26Mhz. Please use the
       host utility UartDivCalculator.exe provided with the installer"
    */
    if(eUartResult == ADI_UART_SUCCESS)
        eUartResult = adi_uart_ConfigBaudRate(hDevice,
                                                UART_DIV_C_460800,
                                                UART_DIV_M_460800,
                                                UART_DIV_N_460800,
                                                UART_OSR_460800);       
    
}
void uart_hal_close(void)
{
    adi_uart_Close(hDevice);
}
/* This function enables the callback for RX event
   callback is invoked after receiving requested no. of bytes 
*/ 							
int uart_hal_read(uint8_t* p_data, uint32_t len )
{    
    ADI_UART_ENABLE_RX_EXT(hDevice);
    
    return 0;
}

/* This function writes the UART bytes in interrupt mode.
   So the buffer passed should retain its contents till all the bytes are Tx'ed 
   It can be cleared after the TX Complete Event(in the callback function).
*/
int uart_hal_write(tx_data_t* p_data, uint32_t len)
{     
    if (!p_data->tx_in_progress)
    {
        ADI_DISABLE_INT(UART_EVT_IRQn);        
        
        mp_send_data = p_data;
    
        mp_send_data->tx_in_progress = true;

        ADI_UART_ENABLE_TX_EXT(hDevice);
    
        ADI_ENABLE_INT(UART_EVT_IRQn);
    }
    
    return 0;
    
}
int uart_hal_register_callback(call_back cb, void* param)
{
    uart_hal_info.cb = cb;
    uart_hal_info.param = param;
    return 0;
}

void uart_hal_rx_cb(void)
{    
    uint8_t data_byte = uart_read(hDevice);
    hif_handle_rcv_data(data_byte, false);
    update_comm_type(0);   
}

void uart_hal_tx_cb(void)
{
    uart_write(hDevice, mp_send_data->data[mp_send_data->tx_rd_index]);
    
    mp_send_data->tx_rd_index++; 
    mp_send_data->tx_rd_index  %= MAX_HIF_BUFF_SIZE;
    
    if(mp_send_data->tx_wr_index == mp_send_data->tx_rd_index)
    {
        adi_uart_EnableTx(hDevice, false);
        uart_hal_info.cb( NULL_POINTER, TX_COMPLETE_EVENT, uart_hal_info.param );        
        
    }
}

bool_t check_uart_tx_status(void)
{
    return mp_send_data->tx_in_progress;
}
