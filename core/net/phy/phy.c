/***********************************************************************************************************************

Copyright(c) 2014 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.

***********************************************************************************************************************/
#include <stdint.h>
#include "device.h"
#include <stdbool.h>
#include "net/netstack.h"
#include <spi\adi_spi.h>
#include "adf7242.h"
#include "phy.h"

const phy_config_t phy_config[8][3] = {
  {
    {863125000, 200000, 1.0,  50},
    {863225000, 400000, 1.0, 100},
    {0,         0,      0,     0}
  },
  {
    {896012500, 12500, 0.5, 10 },
    {896025000, 12500, 0.5, 20 },
    {896050000, 12500, 0.5, 40 }
  },
  {
    {901012500, 12500, 0.5, 10 },
    {901025000, 12500, 0.5, 20 },
    {901050000, 12500, 0.5, 40 }
  },
  {
    {902200000, 200000, 1.0,  50},
    {902400000, 400000, 0.5, 150},
    {902400000, 400000, 0.5, 200}
  },
  {
    {917100000, 200000, 1.0, 50 },
    {917300000, 400000, 0.5, 150},
    {917300000, 400000, 0.5, 200}
  },
  {
    {928012500, 12500, 0.5, 10 },
    {928025000, 12500, 0.5, 20 },
    {928050000, 12500, 0.5, 40 }
  },
  {
    {920600000, 200000, 1.0, 50 },
    {920900000, 400000, 1.0, 100},
    {920800000, 600000, 1.0, 200}
  },
  {
    {951000000, 200000, 1.0, 50 },
    {951100000, 400000, 1.0, 100},
    {951200000, 600000, 1.0, 200}
  }
};

uint16_t phy_current_channel  = 0;

uint8_t phy_set_channel(uint16_t ChannelNum, ISMBAND FreqBand, uint8_t PhyMode )
{
  uint8_t result = 0;
  #ifndef WIRESHARK_FEEDER
    NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, ChannelNum);
  #else
    adf7242_set_channel(ChannelNum);
  #endif


  phy_current_channel = ChannelNum;

  return (result);
}

uint16_t phy_get_channel(void)
{
  int channel;
  #ifndef WIRESHARK_FEEDER
    NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &channel);
  #else
    adf7242_get_channel(&channel);
  #endif
  return channel;
}

void phy_set_panId(uint16_t PanId)
{
  #ifndef WIRESHARK_FEEDER
    NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, PanId);
  #endif
}

void phy_set_ExtendedAddress(uint8_t *ExtendedAddress)
{
  #ifndef WIRESHARK_FEEDER
    NETSTACK_RADIO.set_object(RADIO_PARAM_64BIT_ADDR, ExtendedAddress, 8);
  #endif
}



