#ifndef _UART_HAL_
#define _UART_HAL_

#include <adi_types.h>

#ifdef __cplusplus
extern "C" {
#endif 

  
/*****************************************************************************
 * @enum Transceiver_events
 *    Enumeration to indicate the transceiver state
 *****************************************************************************/

/*
                    Baudrate divider for PCLK-26000000 

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

/**
 *****************************************************************************
 * @struct Structure for UART Data
 *****************************************************************************/
typedef struct uart_hal_data_tag
{
	call_back cb;
	void* param;
}uart_hal_data_t;

extern tx_data_t send_data;

extern void uart_hal_init(void);

extern void uart_hal_close(void);

extern int uart_hal_read(uint8_t* p_data, uint32_t len );

int uart_hal_write(tx_data_t*, uint32_t);

extern int uart_hal_register_callback(call_back cb, void* param);    

void hif_hal_rx_low_priority_cb(void);

#ifdef __cplusplus
}
#endif
#endif /* _UART_HAL_ */