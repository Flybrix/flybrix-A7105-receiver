/*
 *  This file is part of A7105-uart, a UART interface to the A7105 wireless
 *  tranceiver.
 *  Copyright (C) 2015 J.Deitmerg <mowfask@gmail.com>
 *
 *  A7105-uart is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  A7105-uart is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with A7105-uart.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef A7105_H
#define A7015_H

#include "common.h"


#define A7105_SLEEP      0x80
#define A7105_IDLE       0x90
#define A7105_STANDBY    0xA0
#define A7105_PLL        0xB0
#define A7105_RX         0xC0
#define A7105_TX         0xD0
#define A7105_RST_WRPTR  0xE0
#define A7105_RST_RDPTR  0xF0

// registers
#define A7105_00_MODE          0x00
#define A7105_01_MODE_CONTROL  0x01
#define A7105_02_CALC          0x02
#define A7105_03_FIFOI         0x03
#define A7105_04_FIFOII        0x04
#define A7105_05_FIFO_DATA     0x05
#define A7105_06_ID_DATA       0x06
#define A7105_07_RC_OSC_I      0x07
#define A7105_08_RC_OSC_II     0x08
#define A7105_09_RC_OSC_III    0x09
#define A7105_0A_CK0_PIN       0x0A
#define A7105_0B_GPIO1_PIN1    0x0B
#define A7105_0C_GPIO2_PIN_II  0x0C
#define A7105_0D_CLOCK         0x0D
#define A7105_0E_DATA_RATE     0x0E
#define A7105_0F_PLL_I         0x0F
#define A7105_10_PLL_II        0x10
#define A7105_11_PLL_III       0x11
#define A7105_12_PLL_IV        0x12
#define A7105_13_PLL_V         0x13
#define A7105_14_TX_I          0x14
#define A7105_15_TX_II         0x15
#define A7105_16_DELAY_I       0x16
#define A7105_17_DELAY_II      0x17
#define A7105_18_RX            0x18
#define A7105_19_RX_GAIN_I     0x19
#define A7105_1A_RX_GAIN_II    0x1A
#define A7105_1B_RX_GAIN_III   0x1B
#define A7105_1C_RX_GAIN_IV    0x1C
#define A7105_1D_RSSI_THOLD    0x1D
#define A7105_1E_ADC           0x1E
#define A7105_1F_CODE_I        0x1F
#define A7105_20_CODE_II       0x20
#define A7105_21_CODE_III      0x21
#define A7105_22_IF_CALIB_I    0x22
#define A7105_23_IF_CALIB_II   0x23
#define A7105_24_VCO_CURCAL    0x24
#define A7105_25_VCO_SBCAL_I   0x25
#define A7105_26_VCO_SBCAL_II  0x26
#define A7105_27_BATTERY_DET   0x27
#define A7105_28_TX_TEST       0x28
#define A7105_29_RX_DEM_TEST_I 0x29
#define A7105_2A_RX_DEM_TEST_II 0x2A
#define A7105_2B_CPC           0x2B
#define A7105_2C_XTAL_TEST     0x2C
#define A7105_2D_PLL_TEST      0x2D
#define A7105_2E_VCO_TEST_I    0x2E
#define A7105_2F_VCO_TEST_II   0x2F
#define A7105_30_IFAT          0x30
#define A7105_31_RSCALE        0x31
#define A7105_32_FILTER_TEST   0x32

#define A7105_MODE_TRER_MASK  (uint8_t)(1 << 0) // TRX is enabled



enum A7105_reg {
    A7105_reg_mode = 0x00,
    A7105_reg_mode_control,   //0x01
    A7105_reg_calib,          //0x02
    A7105_reg_FIFOI,          //0x03
    A7105_reg_FIFOII,         //0x04
    A7105_reg_FIFO_data,      //0x05
    A7105_reg_ID,             //0x06
    A7105_reg_OSCI,           //0x07
    A7105_reg_OSCII,          //0x08
    A7105_reg_OSCIII,         //0x09
    A7105_reg_CKO,            //0x0a
    A7105_reg_GPIOI,          //0x0b
    A7105_reg_GPIOII,         //0x0c
    A7105_reg_clock,          //0x0d
    A7105_reg_data_rate,      //0x0e
    A7105_reg_channel,        //0x0f
    A7105_reg_PLLII,          //0x10
    A7105_reg_PLLIII,         //0x11
    A7105_reg_PLLIV,          //0x12
    A7105_reg_PLLV,           //0x13
    A7105_reg_TXI,            //0x14
    A7105_reg_TXII,           //0x15
    A7105_reg_delayI,         //0x16
    A7105_reg_delayII,        //0x17
    A7105_reg_RX,             //0x18
    A7105_reg_RX_gainI,       //0x19
    A7105_reg_RX_gainII,      //0x1a
    A7105_reg_RX_gainIII,     //0x1b
    A7105_reg_RX_gainIV,      //0x1c
    A7105_reg_RSSI_thres,     //0x1d
    A7105_reg_ADC,            //0x1e
    A7105_reg_codeI,          //0x1f
    A7105_reg_codeII,         //0x20
    A7105_reg_codeIII,        //0x21
    A7105_reg_IF_calibI,      //0x22
    A7105_reg_IF_calibII,     //0x23
    A7105_reg_VCO_c_calib,    //0x24
    A7105_reg_VCO_sb_calibI,  //0x25
    A7105_reg_VCO_sb_calibII, //0x26
    A7105_reg_battery,        //0x27
    A7105_reg_TX_test,        //0x28
    A7105_reg_RX_testI,       //0x29
    A7105_reg_RX_testII,      //0x2a
    A7105_reg_CPC,            //0x2b
    A7105_reg_crystal_test,   //0x2c
    A7105_reg_PLL_test,       //0x2d
    A7105_reg_VCO_testI,      //0x2e
    A7105_reg_VCO_testII,     //0x2f
    A7105_reg_IFAT,           //0x30
    A7105_reg_R_scale,        //0x31
    A7105_reg_filter_test     //0x32
};

enum A7105_strobe {
    A7105_strobe_sleep    = 0x80,
    A7105_strobe_idle     = 0x90,
    A7105_strobe_standby  = 0xa0,
    A7105_strobe_PLL      = 0xb0,
    A7105_strobe_RX       = 0xc0,
    A7105_strobe_TX       = 0xd0,
    A7105_strobe_TX_reset = 0xe0,
    A7105_strobe_RX_reset = 0xf0
};

enum A7105_mode{
    master,
    slave
};

// This can't easily be configured dynamically. 1 for 500kHz, 0 for 250kHz
#define CHNL_WIDTH 1

void A7105_reset(void);

/* A7105_calib: Perform 3 calibrations as in chapter 15 of datasheet.
 *              Should be performed when everything is set up (to the point
 *              that a channel is selected).
 * Returns:
 *  0       on success
 *  The ored combination of the following values:
 *    0x01  if VCO bank calibration took more than 1000us
 *    0x02  if VCO bank calibration was not successful
 *    0x04  if VCO current calibration took more than 1000us
 *    0x08  if VCO current calibration was not successful
 *    0x10  if IF filter bank calibration took more than 1000us
 *    0x20  if IF filter bank calibration was not successful
 */
uint8_t A7105_calib(void);

void A7105_init(void);

/* ID read and write functions:
 * Read or write the 32 bit ID. The most significant bit is read/written
 * first. With the recommended values for byte 0 the ID pattern should
 * therefore be 0x5******* or 0xA*******.
 */
uint32_t A7105_ID_read(void);
void A7105_ID_write(uint32_t ID);

void A7105_set_channel(uint8_t);
void A7105_set_mode(enum A7105_mode);

uint8_t A7105_receive_byte(void);
void A7105_send_byte(uint8_t data);

void A7105_ReadPayload(uint8_t * _packet, uint8_t len);
void A7105_WritePayload(uint8_t * _packet, uint8_t len);


#endif
