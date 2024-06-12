/*
 * DataProc.h
 *
 * Functions for data converting
 * 
 * Author: Mike Cherkes, UR4UEM
 *
 */ 

#ifndef DATAPROC_H_
#define DATAPROC_H_

#include <stdint.h>
#include <stdbool.h>

// convert "8-bit + sign" value to "regular" 8-bit integer (optimization needed)
static int8_t dp_i9to8(const uint8_t uint8, const bool negative) {
    if(negative) {
        if((uint8 & 0b10000000) == 0b10000000) { // fit into 7 bit?
            return (int8_t) uint8; // return as is
        } else {
            return 0b10000000; // return bottom value of signed 8-bit int
        }
    } else {
        if((uint8 & 0b10000000) == 0) { // fit into 7 bit?
            return (int8_t) uint8; // return as is
        } else {
            return 0b01111111; // return top value of signed 8-bit int
        }
    }
}

// Reverse bits of the byte
static uint8_t dp_reverse_byte(uint8_t x) {
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xAA);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xCC);
    x = ((x >> 4) & 0x0F) | ((x << 4) & 0xF0);
    return x;
}

#endif /* DATAPROC_H_ */
