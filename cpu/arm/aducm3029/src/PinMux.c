/*
 **
 ** Source file generated on November 9, 2015 at 12:13:30.	
 **
 ** Copyright (C) 2015 Analog Devices Inc., All Rights Reserved.
 **
 ** This file is generated automatically based upon the options selected in 
 ** the Pin Multiplexing configuration editor. Changes to the Pin Multiplexing
 ** configuration should be made by changing the appropriate options rather
 ** than editing this file.
 **
 ** Selected Peripherals
 ** --------------------
 ** SPI2 (SCLK, MISO, MOSI, CS_0, CS_1, CS_2, CS_3)
 ** I2C0 (SCL0, SDA0)
 ** UART0 (Tx, Rx, SOUT_EN)
 **
 ** GPIO (unavailable)
 ** ------------------
 ** P0_04, P0_05, P0_09, P0_10, P0_11, P0_12, P1_02, P1_03, P1_04, P1_05, P2_07, P2_10
 */

#include "device.h"
#include <gpio/adi_gpio.h>

#define SPI0_SCLK_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<0))
#define SPI0_MOSI_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<2))
#define SPI0_MISO_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<4))
#define SPI0_CS_0_PORTP0_MUX  ((uint32_t) ((uint16_t) 1<<6))

#define SPI1_SCLK_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<12))
#define SPI1_MISO_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<14))
#define SPI1_MOSI_PORTP1_MUX  ((uint32_t) ((uint32_t) 1<<16))
#define SPI1_CS_0_PORTP1_MUX  ((uint32_t) ((uint32_t) 1<<18))
#define SPI2_SCLK_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<4))
#define SPI2_MISO_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<6))
#define SPI2_MOSI_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<8))
#define SPI2_CS_2_PORTP2_MUX  ((uint32_t) ((uint32_t) 2<<20))
#define SPI2_CS_3_PORTP2_MUX  ((uint16_t) ((uint16_t) 2<<14))
#define UART0_TX_PORTP0_MUX  ((uint32_t) ((uint32_t) 1<<20))
#define UART0_RX_PORTP0_MUX  ((uint32_t) ((uint32_t) 1<<22))
#define UART0_SOUT_EN_PORTP0_MUX  ((uint32_t) ((uint32_t) 3<<24))
#define I2C0_SCL0_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<8))
#define I2C0_SDA0_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<10))



int32_t adi_initpinmux(void);

/*
 * Initialize the Port Control MUX Registers
 */
int32_t adi_initpinmux(void) {
    /* Port Control MUX registers */
    /* Port Control MUX registers */
    *((volatile uint32_t *)REG_GPIO0_CFG) =  I2C0_SCL0_PORTP0_MUX
     | I2C0_SDA0_PORTP0_MUX | UART0_TX_PORTP0_MUX | UART0_RX_PORTP0_MUX
     | UART0_SOUT_EN_PORTP0_MUX | SPI0_SCLK_PORTP0_MUX | SPI0_MOSI_PORTP0_MUX | SPI0_MISO_PORTP0_MUX | SPI0_CS_0_PORTP0_MUX;
#ifdef RFMODULE_7242
	*((volatile uint32_t *)REG_GPIO1_CFG) = SPI1_SCLK_PORTP1_MUX | SPI1_MISO_PORTP1_MUX
     | SPI1_MOSI_PORTP1_MUX | SPI1_CS_0_PORTP1_MUX;
#else
    *((volatile uint32_t *)REG_GPIO1_CFG) = SPI2_SCLK_PORTP1_MUX | SPI2_MISO_PORTP1_MUX
     | SPI2_MOSI_PORTP1_MUX;// | SPI0_CS_1_PORTP1_MUX;
    *((volatile uint32_t *)REG_GPIO2_CFG) = SPI2_CS_2_PORTP2_MUX | SPI2_CS_3_PORTP2_MUX;// | SPI0_CS_2_PORTP2_MUX;
#endif
    return 0;
}

void disable_CS3()
{
#ifdef RFMODULE_7242
    *((volatile uint32_t *)REG_GPIO1_CFG) &= ~SPI1_CS_0_PORTP1_MUX;
	*((volatile uint32_t *)REG_GPIO1_CFG) &= ~SPI1_MISO_PORTP1_MUX;
	adi_gpio_OutputEnable(ADI_GPIO_PORT1, ADI_GPIO_PIN_9, true);	//SPI1_CS0
    adi_gpio_InputEnable(ADI_GPIO_PORT1, ADI_GPIO_PIN_8, true);		//SPI1_MISO
#else
    *((volatile uint32_t *)REG_GPIO2_CFG) &= ~(SPI2_CS_3_PORTP2_MUX);
    *((volatile uint32_t *)REG_GPIO1_CFG) &= ~SPI2_MISO_PORTP1_MUX;
    adi_gpio_OutputEnable(ADI_GPIO_PORT2, ADI_GPIO_PIN_7, true);
    adi_gpio_InputEnable(ADI_GPIO_PORT1, ADI_GPIO_PIN_4, true);   
#endif
}

