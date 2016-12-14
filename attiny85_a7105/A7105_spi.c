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

/* This file contains a software implementation (bitbang FTW) of the 3-wire
 * SPI used by the A7105.
 */

#include "A7105_spi.h"

static void SPI_bus_write(uint8_t data)
// Writes the byte data to the SPI bus.
{
    int8_t i;

    // Make MISO/MOSI output only for the duration of this function
    setbit(SPI_IO_DDR, SPI_IO_DDP);

    // Ugly bitbanging ahead! To make this as fast as possible, the bit
    // shift from the current data bit to the IO bit is dynamically
    // calculated. As negative shifts are not allowed, we have to do this
    // in two loops (and switch the shifting direction in the second).
    // Note, however, that the second loop will be optimized out, if
    // SPI_IO_PP equals 0 (see A7015_SPI.h).
    // Splitting the binary operations into understandable instructions
    // would probably make this slow, as all the variables are volatile.
    for(i = 7; i >= SPI_IO_PP; --i)
    {
        SPI_IO_PORT = (SPI_IO_PORT & ~(1 << SPI_IO_PP)) | // clear IO bit
                      ((data & (1 << i))    // find current address bit
                       >> (i - SPI_IO_PP)); // and shift if from position i
                                            // to the IO bit position
        SPI_CLK_HIGH();
        SPI_CLK_LOW();
    }
    if(SPI_IO_PP > 0)
    {
        for(i = SPI_IO_PP - 1; i >= 0; --i)
        {
            SPI_IO_PORT =
                (SPI_IO_PORT & ~(1 << SPI_IO_PP)) | // clear IO bit
                ((data & (1 << i))    // find current address bit
                << (SPI_IO_PP - i));  // and shift if from position i
                                      // to the IO bit position
            SPI_CLK_HIGH();
            SPI_CLK_LOW();
        }
    }

    clearbit(SPI_IO_DDR, SPI_IO_DDP);
}

static uint8_t SPI_bus_read(void)
// Reads a single byte from the SPI bus.
{
    int8_t i;
    uint8_t data = 0;

    // Assume MISO/MOSI is input (SPI_bus_write should leave it as that)

    // Analogous to SPI_bus_write()
    for(i = 7; i >= SPI_IO_PP; --i)
    {
        data |= (SPI_IO_PIN & (1 << SPI_IO_PP)) //slect IO bit
                << (i - SPI_IO_PP);         //and shift it to position i

        SPI_CLK_HIGH();
        SPI_CLK_LOW();
    }
    if(SPI_IO_PP > 0)
    {
        for(i = SPI_IO_PP-1; i >= 0; --i)
        {
            data |= (SPI_IO_PIN & (1 << SPI_IO_PP)) //slect IO bit
                    >> (SPI_IO_PP - i);     //and shift it to position i

            // We're pulsing the clock once too often, but that shouldn't
            // matter.
            SPI_CLK_HIGH();
            SPI_CLK_LOW();
        }
    }

    return(data);
}

void SPI_single_write(uint8_t data)
{
    SPI_CSELECT();
    SPI_bus_write(data);
    SPI_CDESELECT();
}

void SPI_reg_write(uint8_t reg, uint8_t data)
{
    // Address byte[7]: Control registers (0) or strobe command(1)
    // Address byte[6]: Write to (0) or read from (1) control register
    reg &= 0x3f;

    SPI_CSELECT();
    SPI_bus_write(reg);
    SPI_bus_write(data);
    SPI_CDESELECT();
}

uint8_t SPI_reg_read(uint8_t reg)
{
    uint8_t data;

    // Don't forget register read bit (bit 6 -> 0x40)
    reg &= 0x3f;
    reg |= 0x40;

    SPI_CSELECT();
    SPI_bus_write(reg);
    data = SPI_bus_read();
    SPI_CDESELECT();

    return(data);
}

void SPI_reg_multi_write(uint8_t reg, uint8_t *buf, uint8_t len)
/* Write len bytes from buf into register reg.
 */
{
    uint8_t i;

    reg &= 0x3f;
    SPI_CSELECT();
    SPI_bus_write(reg);

    for(i = 0; i < len; ++i)
    {
        SPI_bus_write(buf[i]);
    }

    SPI_CDESELECT();
}

void SPI_reg_multi_read(uint8_t reg, uint8_t *buf, uint8_t len)
/* Read len bytes from register reg into buf
 */
{
    uint8_t i;

    // Don't forget register read bit (bit 6 -> 0x40)
    reg &= 0x3f;
    reg |= 0x40;
    SPI_CSELECT();
    SPI_bus_write(reg);

    for(i = 0; i < len; ++i)
    {
        buf[i] = SPI_bus_read();
    }

    SPI_CDESELECT();

}

void SPI_init(void)
{
    // CS and CLK as outputs
    setbit(SPI_CS_DDR, SPI_CS_DDP);
    setbit(SPI_CLK_DDR, SPI_CLK_DDP);
    // MOSI/MISO shall be input as often as possible
    clearbit(SPI_IO_DDR, SPI_IO_DDP);

    // CS high by default
    SPI_CDESELECT();
    // CLK low by default
    SPI_CLK_LOW();
}

#define A7105_RST_WRPTR  0xE0
void A7105_WritePayload(uint8_t * _packet, uint8_t len)
{
    uint8_t i;

    SPI_CSELECT();
    SPI_bus_write(A7105_RST_WRPTR);
    SPI_bus_write(0x05);

    for(i = 0; i < len; ++i)
    {
        SPI_bus_write(_packet[i]);
    }

    SPI_CDESELECT();
}

void A7105_ReadPayload(uint8_t * _packet, uint8_t len)
{
    uint8_t i;

    // Don't forget register read bit (bit 6 -> 0x40)
    SPI_CSELECT();
    SPI_bus_write(0x45);

    for(i = 0; i < len; ++i)
    {
        _packet[i] = SPI_bus_read();
    }

    SPI_CDESELECT();
}
