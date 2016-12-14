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


#include "A7105.h"
#include "A7105_spi.h"
void A7105_reset(void)
{
    SPI_reg_write(A7105_reg_mode, 0x00);
}

uint8_t A7105_calib(void)
{
    uint8_t retval = 0;
    uint8_t calibreg;

    // Assume MFBS == 0 after reset
    SPI_single_write(A7105_strobe_PLL);
    // Start filter bank (FBC, bit 0), VCO current (VCC, bit 2) and VCO
    // bank (VBC, bit 1) calibrations all at the same time.
    // Leave VTH and VTL at recommended values.
    SPI_reg_write(A7105_reg_calib, 0x07);

    // We could poll the end of the calibration and use a timer to determine
    // if it takes too long. That's not easily portable, so we waste some
    // time instead.
    _delay_us(1000);

    calibreg = SPI_reg_read(A7105_reg_calib);

    // Check calibration timeouts
    if(testbit(calibreg, 0))
    {
        // IF filter bank calibration took too long
        retval |= 0x10;
    }
    if(testbit(calibreg, 1))
    {
        // VCO bank calibration took too long
        retval |= 0x01;
    }
    if(testbit(calibreg, 2))
    {
        // VCO current calibration took too long
        retval |= 0x04;
    }

    // Check calibration success
    // FBCF bit: Indicates calib failure
    if(testbit(SPI_reg_read(A7105_reg_IF_calibI), 4))
    {
        // IF calib not successful
        retval |= 0x20;
    }

    // VBCF bit: Indicates calib failure
    if(testbit(SPI_reg_read(A7105_reg_VCO_sb_calibI), 3))
    {
        // VCO bank calibration not successful
        retval |= 0x02;
    }
    // FVCC bit: Indicates calib failure
    if(testbit(SPI_reg_read(A7105_reg_VCO_c_calib), 4))
    {
        // VCO current calib not successful
        retval |= 0x08;
    }

    SPI_single_write(A7105_strobe_standby);

    return(retval);
}

void A7105_init(void)
{
    A7105_reset();
    // After reset 16MHz crystal and 500Kbps data rate are already
    // configured. The default base frequency is then 2400.001454 MHz.
    // A channel width of 500KHz is recommended. 16MHz/(4*500KHz)-1 = 7,
    // leave all the other bits as they are.
    SPI_reg_write(A7105_reg_PLLII, 0xf0 | (7 << 1));
    // Note: To stay in the 2.4 to 2.483 GHz ISM band, only channels 1
    // through 166 should be used (with "Auto IF Exchange" enabled, master
    // sends on 1-166, slave sends on 0-165).

    /* Current understanding of the datasheet (concerning Auto IF):
     * Master sends on channel n, sets ULS=0 to receive on channel n-1.
     * Slave then has to send on channel n-1, but sets ULS=1 to receive on
     * channel n-1+1=n. Besides the bad english, the datasheet is also
     * not clear about what to do if the channel width is not 500KHz, as
     * ULS always shifts by 500KHz.
     */

    // Demodulator DC estimation average mode: 0x1 is recommended.
    // Preamble pattern detection length: 0x2 recommended for 500Kbps
    SPI_reg_write(A7105_reg_codeII, 0x4 | (0x1 << 4) | 0x2);
    // Demodulator DC estimation mode: Average and hold (0x2)
    SPI_reg_write(A7105_reg_RX_testI, 0x07 | (0x2 << 5));
    // Enable auto IF offset (shift frequency while receiving) and FIFO
    // mode
    SPI_reg_write(A7105_reg_mode_control, (1 << 5) | (1 << 1));
    // Set BPF bandwidth to 500KHz (bits 6 and 5 shall always be set to 1)
    // Although it always reads 0, we assume 500KHz is already selected
    // after reset.
    // SPI_reg_write(A7105_reg_RX, (0x3 << 5) | 0x02);

    // Disable FIFO extension (FPM = 0) and segmentation (PSA = 0)
    SPI_reg_write(A7105_reg_FIFOII, 0);

    // Set FIFO size to 1 (for testing only)
    SPI_reg_write(A7105_reg_FIFOI, 0);
}

uint32_t A7105_ID_read(void)
{
    uint32_t ID;
    uint8_t idbytes[4];

    // We could read this straight into ID, but the result would depend on
    // the endiness of the architecture.
    SPI_reg_multi_read(A7105_reg_ID, idbytes, 4);

    ID  = (uint32_t) idbytes[0] << 24;
    ID |= (uint32_t) idbytes[1] << 16;
    ID |= (uint32_t) idbytes[2] << 8;
    ID |= (uint32_t) idbytes[3] << 0;

    return(ID);
}

void A7105_ID_write(uint32_t ID)
{
    uint8_t idbytes[4];

    idbytes[0] = ID >> 24;
    idbytes[1] = ID >> 16;
    idbytes[2] = ID >> 8;
    idbytes[3] = ID >> 0;

    SPI_reg_multi_write(A7105_reg_ID, idbytes, 4);
}

void A7105_set_channel(uint8_t chnl)
{
    SPI_reg_write(A7105_reg_channel, chnl);
}

void A7105_set_mode(enum A7105_mode mode)
{
    uint8_t rxreg;

    // Register 0x18 always reads 0x00, so we have to write the channel
    // width again.

    // RXSM0 and RXSM1 shall always be set to 1
    // DMG shall always be set to 0
    rxreg = 0x3 << 5;

    if(CHNL_WIDTH)
    {
        setbit(rxreg, 1);
    }

    if(mode == master)
    {
        clearbit(rxreg, 0);
    }
    else
    {
        setbit(rxreg, 0);
    }

    SPI_reg_write(A7105_reg_RX, rxreg);
}

uint8_t A7105_receive_byte(void)
{
    uint8_t data;

    SPI_single_write(A7105_strobe_PLL);
    SPI_single_write(A7105_strobe_RX_reset);
    SPI_single_write(A7105_strobe_RX);

    _delay_ms(100);

    data = SPI_reg_read(A7105_reg_FIFO_data);
    SPI_single_write(A7105_strobe_standby);

    return(data);
}

void A7105_send_byte(uint8_t data)
{
    SPI_single_write(A7105_strobe_PLL);
    SPI_single_write(A7105_strobe_TX_reset);
    SPI_reg_write(A7105_reg_FIFO_data, data);

    SPI_single_write(A7105_strobe_TX);
    _delay_ms(100);
    SPI_single_write(A7105_strobe_standby);
}



