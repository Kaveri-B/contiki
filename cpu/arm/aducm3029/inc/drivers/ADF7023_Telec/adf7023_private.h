/*!
 *****************************************************************************
 * @file:    uhf_private.h
 * @brief:   UHF Device Driver Private Implementations for ADuCRFxxx
 * @version: $Revision: 11204 $
 * @date:    $Date: 2011-08-31 16:54:08 -0400 (Wed, 31 Aug 2011) $
 *----------------------------------------------------------------------------
 *
 Copyright (c) 2010 Analog Devices, Inc.

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

 THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
 PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

 
#define MAKE_UINT16(lsb,msb)                     (u16)(lsb) | (((u16)(msb)) << 8)
#define MAKE_ULONG(byte1,byte2,byte3,byte4)      (u32)(byte1) | (((u32)(byte2)) << 8) | (((u32)(byte3)) << 16) | (((u32)(byte4)) << 24)
#define XTRCT_BITS(byte,numbits,offset)          (byte  & (((0x1 << numbits)-1)<< offset))
#define XTRCT_VALUE(byte,numbits,offset)         ((byte >> offset) & ((0x1 << numbits)-1))
#define SET_BITS(byte,numbits,offset,value)      ((byte & ~(((0x1 << numbits)-1) << offset)) | (value << offset))


// interrupt_mask_0
#define interrupt_mask_0_num_wakeups         (0x1 << 7) // Enable the interrupt for when the number_of_wakeups exceeds the number_of_wakeups_irq_threshold of the firmware timer
#define interrupt_mask_0_swm_rssi_det        (0x1 << 6) // Interrupt indicating that the RSSI threshold has been exceeded
#define interrupt_mask_0_aes_done            (0x1 << 5) // Interrupt when an AES operation is completed
#define interrupt_mask_0_tx_eof              (0x1 << 4) // End of Frame interrupt after packet transmission
#define interrupt_mask_0_address_match       (0x1 << 3) // Interrupt on receive address match
#define interrupt_mask_0_crc_correct         (0x1 << 2) // Interrupt on reception of valid CRC
#define interrupt_mask_0_syncbyte_detect     (0x1 << 1) // Interrupt on reception of valid sync byte pattern
#define interrupt_mask_0_premable_detect     (0x1 << 0) // Interrupt on receive preamble qualification


// interrupt_mask_1
#define interrupt_mask_1_battery_alarm       (0x1 << 7)  // Interrupt on Battery voltage user defined threshold
#define interrupt_mask_1_rc_ready            (0x1 << 6)  // Interrupt on radio controller command finished
#define interrupt_mask_1_Reserved0           (0x1 << 5)  //
#define interrupt_mask_1_wuc_timeout         (0x1 << 4)  // Interrupt when the wuc timer reaches 0x0000
#define interrupt_mask_1_Reserved1           (0x1 << 3)  //
#define interrupt_mask_1_Reserved2           (0x1 << 2)  //
#define interrupt_mask_1_spi_ready           (0x1 << 1)  // Interrupt on SPI ready for access
#define interrupt_mask_1_rc_error            (0x1 << 0)  // Interrupt on Radio Controller error


// packet_length_control


// Offset value in bytes that is added to the received packet
// length field value (in variable length packet mode)
// so the communications processor  knows the correct number
// of bytes to read. Useful in legacy systems where the
// definition of the packet length may or may not include CRC.
#define packet_length_control_length_offset_numbits     (3)
#define packet_length_control_length_offset_offset      (0)
#define packet_length_control_length_offset_minus4      (0x0 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_minus3      (0x1 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_minus2      (0x2 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_minus1      (0x3 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_minus0      (0x4 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_plus1       (0x5 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_plus2       (0x6 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_plus3       (0x7 << packet_length_control_length_offset_offset)

// Enables the sport mode interface for transmit and receive data.
#define packet_length_control_sport_mode_numbits        (2)
#define packet_length_control_sport_mode_offset         (3)
#define packet_length_control_sport_mode_disabled       (0x0 << packet_length_control_sport_mode_offset)
#define packet_length_control_sport_mode_sport_preamble (0x1 << packet_length_control_sport_mode_offset)
#define packet_length_control_sport_mode_sport_sync     (0x2 << packet_length_control_sport_mode_offset)
#define packet_length_control_sport_mode_unused         (0x3 << packet_length_control_sport_mode_offset)


// 1: Append CRC in transmit mode. Check CRC in receive mode.
#define packet_length_control_crc_en_no                 (0x0 << 5)
#define packet_length_control_crc_en_yes                (0x1 << 5)
#define CRC_EN  (1<<5)

// 1: Fixed packet length mode. Fixed packet length in tx and rx,
// given by  packet_lenght_max.
// 0: Variable packet length mode. In rx mode packet length is
// given by first byte in Packet RAM. In tx mode the packet length is given by packet_lenght_max.
#define packet_length_control_packet_len_variable       (0x0 << 6)
#define packet_length_control_packet_len_fixed          (0x1 << 6)
#define PACKET_LENGTH_FIXED (1<<6)

// Over the air arrangement of each transmitted Packet RAM byte.
// Byte transmitted either MSB or LSB first. The same setting should be used on the Tx and Rx side of the link.
//1: Data byte MSB first
//0: Data byte LSB first
#define packet_length_control_data_byte_lsb             (0x0 << 7)
#define packet_length_control_data_byte_msb             (0x1 << 7)
#define PACKET_ENDIAN                                   (0x1 << 7)


// Sets the sync word length in bits. 24 bits is the maximum.
// Note that the sync word matching length can be any value
// up to 24 bits, but the transmitted sync word pattern is
// a multiple of 8 bits. Hence, for non-byte-length sync words,
// the transmitted sync pattern should be filled out with the
// preamble pattern.
#define sync_control_sync_word_length_numbits            (5)
#define sync_control_sync_word_length_offset             (0)
#define sync_control_sync_word_length_0                  (0  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_1                  (1  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_2                  (2  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_3                  (3  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_4                  (4  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_5                  (5  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_6                  (6  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_7                  (7  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_8                  (8  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_9                  (9  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_10                 (10 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_11                 (11 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_12                 (12 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_13                 (13 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_14                 (14 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_15                 (15 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_16                 (16 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_17                 (17 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_18                 (18 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_19                 (19 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_20                 (20 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_21                 (21 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_22                 (22 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_23                 (23 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_24                 (24 << sync_control_sync_word_length_offset)

// Sets the sync word error tolerance in bits.
#define sync_control_sync_error_tol_numbits              (2)
#define sync_control_sync_error_tol_offset               (6)
#define sync_control_sync_error_tol_0_errors_allowed     (0 << sync_control_sync_error_tol_offset)
#define sync_control_sync_error_tol_1_errors_allowed     (1 << sync_control_sync_error_tol_offset)
#define sync_control_sync_error_tol_2_errors_allowed     (2 << sync_control_sync_error_tol_offset)
#define sync_control_sync_error_tol_3_errors_allowed     (3 << sync_control_sync_error_tol_offset)

// Symbol length
#define symbol_mode_symbol_length_8_bit                  (0 << 0)
#define symbol_mode_symbol_length_10_bit                 (1 << 0)
// Data whitening and de-whitening
#define symbol_mode_data_whitening_disabled              (0 << 3)
#define symbol_mode_data_whitening_enabled               (1 << 3)
// : 8b/10b encoding and decoding
#define symbol_mode_eight_ten_encoding_disabled          (0 << 4 )
#define symbol_mode_eight_ten_encoding_enabled           (1 << 4 )
// Programmable CRC
#define symbol_mode_prog_crc_en_disabled                 (0 << 5)
#define symbol_mode_prog_crc_en_enabled                  (1 << 5)
// Manchester encoding and decoding
#define symbol_mode_manchester_enc_disabled              (0 << 6)
#define symbol_mode_manchester_enc_enabled               (1 << 6)


#define preamble_match_qual_disabled                     (0x0 << 0)
#define preamble_match_11_in_24_win                      (0x1 << 0)
#define preamble_match_10_in_24_win                      (0x2 << 0)
#define preamble_match_9_in_24_win                       (0x3 << 0)
#define preamble_match_8_in_24_win                       (0x4 << 0)
#define preamble_match_7_in_24_win                       (0x5 << 0)
#define preamble_match_6_in_24_win                       (0x6 << 0)
#define preamble_match_5_in_24_win                       (0x7 << 0)
#define preamble_match_4_in_24_win                       (0x8 << 0)
#define preamble_match_3_in_24_win                       (0x9 << 0)
#define preamble_match_2_in_24_win                       (0xA << 0)
#define preamble_match_1_in_24_win                       (0xB << 0)
#define preamble_match_0_in_24_win                       (0xC << 0)

// Smart wake mode enable
#define mode_control_swm_en_disabled                        (0x0 << 7)
#define mode_control_swm_en_enabled                         (0x1 << 7)
// IF Filter calibration enable
#define mode_control_bb_cal_disabled                        (0x0 << 6)
#define mode_control_bb_cal_enabled                         (0x1 << 6)
//  RSSI qualify in low power mode enable
#define mode_control_swm_rssi_qual_disabled                 (0x0 << 5)
#define mode_control_swm_rssi_qual_enabled                  (0x1 << 5)
// Fast PHY_TX to PHY_RX switching enable
#define mode_control_tx_auto_turnaround_disabled            (0x0 << 4)
#define mode_control_tx_auto_turnaround_enabled             (0x1 << 4)
// Fast PHY_RX to PHY_TX switching enable
#define mode_control_rx_auto_turnaround_disabled            (0x0 << 3)
#define mode_control_rx_auto_turnaround_enabled             (0x1 << 3)
// Use the custom synthesizer lock time defined in register 0x3E and 0x3F
#define mode_control_custom_trx_synth_lock_time_en_disabled (0x0 << 2)
#define mode_control_custom_trx_synth_lock_time_en_enabled  (0x1 << 2)
// External LNA enable signal on ATB4 is enabled.
#define mode_control_ext_lna_en_disabled                    (0x0 << 1)
#define mode_control_ext_lna_en_enabled                     (0x1 << 1)
// External PA enable signal on ATB3 is enabled.
#define mode_control_ext_pa_en_disabled                     (0x0 << 0)
#define mode_control_ext_pa_en_enabled                      (0x1 << 0)


#define radio_cfg_11_afc_kp_2_numbits                   (4)
#define radio_cfg_11_afc_kp_2_offset                    (4)
#define radio_cfg_11_afc_kp_2_power_0                   (0  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_1                   (1  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_2                   (2  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_3                   (3  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_4                   (4  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_5                   (5  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_6                   (6  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_7                   (7  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_8                   (8  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_9                   (9  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_10                  (10 << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_11                  (11 << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_12                  (12 << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_13                  (13 << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_14                  (14 << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_15                  (15 << radio_cfg_11_afc_kp_2_offset)


#define radio_cfg_11_afc_ki_2_numbits                   (4)
#define radio_cfg_11_afc_ki_2_offset                    (0)
#define radio_cfg_11_afc_ki_2_power_0                   (0  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_1                   (1  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_2                   (2  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_3                   (3  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_4                   (4  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_5                   (5  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_6                   (6  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_7                   (7  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_8                   (8  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_9                   (9  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_10                  (10 << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_11                  (11 << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_12                  (12 << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_13                  (13 << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_14                  (14 << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_15                  (15 << radio_cfg_11_afc_ki_2_offset)

// Sets the AFC mode.
#define radio_cfg_10_afc_lock_mode_numbits              (2)
#define radio_cfg_10_afc_lock_mode_offset               (0)
#define radio_cfg_10_afc_lock_mode_free_running         (0x0  << radio_cfg_10_afc_lock_mode_offset)
#define radio_cfg_10_afc_lock_mode_disabled             (0x1  << radio_cfg_10_afc_lock_mode_offset)
#define radio_cfg_10_afc_lock_mode_paused               (0x2  << radio_cfg_10_afc_lock_mode_offset)
#define radio_cfg_10_afc_lock_mode_lock_after_preamble  (0x3  << radio_cfg_10_afc_lock_mode_offset)

// Sets which AFC scheme to use.
#define radio_cfg_10_afc_scheme_numbits                 (2)
#define radio_cfg_10_afc_scheme_offset                  (2)
#define radio_cfg_10_afc_scheme_mode_1                  (0x1  << radio_cfg_10_afc_scheme_offset)
#define radio_cfg_10_afc_scheme_fixed_value             (0x2  << radio_cfg_10_afc_scheme_offset)

// Sets the AFC polarity.
#define radio_cfg_10_afc_polarity_numbits                (1)
#define radio_cfg_10_afc_polarity_offset                 (4)
#define radio_cfg_10_afc_polarity_fixed_value            (0x0  << radio_cfg_10_afc_polarity_offset)
#define radio_cfg_10_afc_polarity_invert                (0x0  << 4)
#define radio_cfg_10_afc_polarity_default               (0x1  << 4)


// Sets the receiver demodulation scheme.
#define radio_cfg_9_demod_scheme_numbits                (3)
#define radio_cfg_9_demod_scheme_offset                 (0)
#define radio_cfg_9_demod_scheme_2FSK_GFSK_MSK_GMSK     (0x0  << radio_cfg_9_demod_scheme_offset)
#define radio_cfg_9_demod_scheme_RESERVED               (0x1  << radio_cfg_9_demod_scheme_offset)
//#define radio_cfg_9_demod_scheme_OOK                    (0x2  << radio_cfg_9_demod_scheme_offset)
#define DEMOD_FIELD                                     (0x7)

// Sets the transmitter modulation.
#define radio_cfg_9_mod_scheme_numbits                  (3)
#define radio_cfg_9_mod_scheme_offset                   (3)
#define radio_cfg_9_mod_scheme_2_level_FSK              (0x0  << radio_cfg_9_mod_scheme_offset)
#define radio_cfg_9_mod_scheme_2_level_GFSK             (0x1  << radio_cfg_9_mod_scheme_offset)
//#define radio_cfg_9_mod_scheme_OOK                      (0x2  << radio_cfg_9_mod_scheme_offset)
#define radio_cfg_9_mod_scheme_carrier_only             (0x3  << radio_cfg_9_mod_scheme_offset)
#define MOD_FIELD                                       (0x7 << 3)

// Sets the receiver IF filter bandwidth. Note thatex_addr_mask setting an IF
// filter bandwidth of 200kHz automatically changes the receiver
// IF frequency from 200kHz to 300kHz.
#define radio_cfg_9_ifbw_numbits                        (2)
#define radio_cfg_9_ifbw_offset                         (6)
#define radio_cfg_9_ifbw_100kHz                         (0x0  << radio_cfg_9_ifbw_offset)
#define radio_cfg_9_ifbw_150kHz                         (0x1  << radio_cfg_9_ifbw_offset)
#define radio_cfg_9_ifbw_200kHz                         (0x2  << radio_cfg_9_ifbw_offset)
#define radio_cfg_9_ifbw_300kHz                         (0x3  << radio_cfg_9_ifbw_offset)
#define IFBW_FIELD                                      (0x3 << 6)


// Single or differential PA
#define radio_cfg_8_pa_single_diff_sel_single_ended     (0x0  << 7)
#define radio_cfg_8_pa_single_diff_sel_differential     (0x1  << 7)

// Sets the PA power
#define radio_cfg_8_pa_power_numbits                    (4)
#define radio_cfg_8_pa_power_offset                     (3)

#define radio_cfg_8_pa_power_setting_3                  (0x0 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_7                  (0x1 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_11                 (0x2 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_15                 (0x3 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_19                 (0x4 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_23                 (0x5 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_27                 (0x6 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_31                 (0x7 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_35                 (0x8 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_39                 (0x9 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_43                 (0xA << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_47                 (0xB << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_51                 (0xC << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_55                 (0xD << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_59                 (0xE << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_63                 (0xF << radio_cfg_8_pa_power_offset)
#define PA_POWER                                        (0xF << 3)

//this will set the default PA_POWER_MCR register value
//It is better if this matches the default radio_cfg_8_pa_power_setting
//so those are set consistently by default at init time.
#define DEFAULT_PA_MCR_REG_SETTING						63

// Sets the PA ramp rate. The PA will ramp at the programmed rate
// until it reaches the level indicated by the pa_power setting.
// The ramp rate is dependent on the programmed data rate.
#define radio_cfg_8_pa_ramp_numbits                     (3)
#define radio_cfg_8_pa_ramp_offset                      (0)
#define radio_cfg_8_pa_ramp_ramp_off                    (0x0 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_256                         (0x1 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_128                         (0x2 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_64                          (0x3 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_32                          (0x4 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_16                          (0x5 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_8                           (0x6 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_4                           (0x7 << radio_cfg_8_pa_ramp_offset)
#define PA_RAMP                                         0x7

// AGC lock mode
#define radio_cfg_7_agc_lock_mode_numbits               (2)
#define radio_cfg_7_agc_lock_mode_offset                (6)
#define radio_cfg_7_agc_lock_mode_free_running          (0x0  << radio_cfg_7_agc_lock_mode_offset)
#define radio_cfg_7_agc_lock_mode_manual                (0x1  << radio_cfg_7_agc_lock_mode_offset)
#define radio_cfg_7_agc_lock_mode_hold                  (0x2  << radio_cfg_7_agc_lock_mode_offset)
#define radio_cfg_7_agc_lock_mode_lock_after_preamble   (0x3  << radio_cfg_7_agc_lock_mode_offset)

#define radio_cfg_7_synth_lut_control_numbits             (2)
#define radio_cfg_7_synth_lut_control_offset              (4)
#define radio_cfg_7_synth_lut_control_predef_rx_predef_tx (0x0  << radio_cfg_7_synth_lut_control_offset)
#define radio_cfg_7_synth_lut_control_custom_rx_predef_tx (0x1  << radio_cfg_7_synth_lut_control_offset)
#define radio_cfg_7_synth_lut_control_predef_rx_custom_tx (0x2  << radio_cfg_7_synth_lut_control_offset)
#define radio_cfg_7_synth_lut_control_custom_rx_custom_tx (0x3  << radio_cfg_7_synth_lut_control_offset)

// This holds the values for lf_r2pi_res_code and lf_r2pl_res_code
// which control the value of the second pole resistor of the PLL loop filter.
// In conjunction with setting  synth_lut_control =1 or =3
// this setting allows the receiver PLL loop bandwidth
// to be changed to optimize the receiver local oscillator phase noise.
#define radio_cfg_7_synth_lut_config_1_numbits          (4)
#define radio_cfg_7_synth_lut_config_1_offset           (0)

//bit 4 of POWERDOWN_RX
#define ADC_PD_N			(0x10)	
//bit 1 of POWERDOWN_AUX register
#define TEMPMON_PD_EN		(0x2)
//bits 3:2 of ADC_CONFIG_LOW register
#define ADC_REF_CHSEL		(0xC)
//connect ADC to temp sensor
#define ADC_TO_TEMP			(0x2 << 2)
//connect ADC to RSSI
#define ADC_TO_RSSI			(0x0 << 2)


#define TEMP_CORRECTION		-54.0
/*
 ** EOF
*/
