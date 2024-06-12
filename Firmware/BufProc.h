/*
 * BufProc.h
 *
 * Functions for work with data buffers
 *  
 * Author: Mike Cherkes, UR4UEM
 *
 */ 

#ifndef BUFPROC_H_
#define BUFPROC_H_

#include <stdint.h>
#include <stdbool.h>

// Get lower byte from buffer and shift rest of bytes left
// !!! NO ACTUAL BUFFER LENGTH CHECKED !!!
static uint8_t buf_get_byte(volatile uint8_t *const buffer, volatile uint8_t *const buffer_size) {
    const uint8_t result = buffer[0]; // make a copy of the first byte
    
    --(*buffer_size); // decrease size
    
    // shift rest of the buffer left
    for(uint8_t i = 0; i < *buffer_size; ++i) {
        buffer[i] = buffer[i + 1];
    }

    return result;
}

// Append byte to the buffer end
// Free buffer space is checked before action
static bool buf_set_byte(const uint8_t byte, volatile uint8_t *const buffer, volatile uint8_t *const buffer_size, const uint8_t max_buffer_size) {
    if(*buffer_size < max_buffer_size) {
        buffer[(*buffer_size)++] = byte; // write byte and increase size
        return true; // all ok
    } else {
        return false; // no room
    }
}

#endif /* BUFPROC_H_ */
