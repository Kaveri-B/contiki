/***********************************************************************************************************************

Copyright(c) 2014 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.

***********************************************************************************************************************/

typedef struct {
  uint32_t      CenterFreq;
  uint32_t      ChannelSpacing;
  float         ModulationIndex;
  uint8_t       DataRate;
}phy_config_t;

typedef enum  {
  IFBW100KHZ = 0,
  IFBW150KHZ,
  IFBW200KHZ,
  IFBW300KHZ
} If_Filter_BW_t;

#define IFBW_FIELD                                      (0x3 << 6)
#define RADIO_CFG_9                                      0x115

uint8_t phy_set_channel(uint16_t ChannelNum, ISMBAND FreqBand, uint8_t PhyMode );
uint8_t phy_set_datarate(ISMBAND FreqBand, uint8_t PhyMode );
void phy_set_modulation_index(ISMBAND FreqBand, uint8_t PhyMode );  
void phy_set_panId(uint16_t PanId);
void phy_set_ExtendedAddress(uint8_t *ExtendedAddress);
