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

#ifndef A7105_SPI_H
#define A7105_SPI_H

#include "common.h"

#define SPI_CS_PORT PORTB
#define SPI_CS_PP   PB2
#define SPI_CS_DDR  DDRB
#define SPI_CS_DDP  DDB2

#define SPI_CLK_PORT PORTB
#define SPI_CLK_PP   PB1
#define SPI_CLK_DDR  DDRB
#define SPI_CLK_DDP  DDB1

// The A7105 by default uses the same pin for MISO and MOSI. We'll do that
// as well. Note that choosing PB0 (== 0!) makes the soft SPI a bit more
// efficient.
#define SPI_IO_PORT PORTB
#define SPI_IO_PIN  PINB
#define SPI_IO_PP   PB0
#define SPI_IO_DDR  DDRB
#define SPI_IO_DDP  DDB0

#define SPI_CSELECT() clearbit(SPI_CS_PORT, SPI_CS_PP)
#define SPI_CDESELECT() setbit(SPI_CS_PORT, SPI_CS_PP)

#define SPI_CLK_HIGH() setbit(SPI_CLK_PORT, SPI_CLK_PP)
#define SPI_CLK_LOW() clearbit(SPI_CLK_PORT, SPI_CLK_PP)

void SPI_init(void);
void SPI_single_write(uint8_t data);
void SPI_reg_write(uint8_t addr, uint8_t data);
uint8_t SPI_reg_read(uint8_t addr);
void SPI_reg_multi_read(uint8_t reg, uint8_t *buf, uint8_t len);
void SPI_reg_multi_write(uint8_t reg, uint8_t *buf, uint8_t len);

#endif
