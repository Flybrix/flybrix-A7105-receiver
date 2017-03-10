#ifndef COMMON_H
#define COMMON_H

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>

//bit operations
#define setbit(byte, bit) ((byte) |= ((1) << (bit)))
#define clearbit(byte, bit) ((byte) &= ~((1) << (bit)))
#define togglebit(byte, bit) ((byte) ^= ((1) << (bit)))
#define testbit(byte, bit) (((byte) >> (bit)) & (1))    //returns 1 or 0

#endif
