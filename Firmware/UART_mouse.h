/*
 * UART_mouse.h
 *
 * Serial mouse emulation driver
 * 
 * Author: Mike Cherkes, UR4UEM
 *
 */ 

#ifndef UART_MOUSE_H_
#define UART_MOUSE_H_

#include <stdint.h>
#include <stdbool.h>

#include "Defines.h"
#include "UART.h"

static inline void uart_mouse_init() {
    uart_init(); // init serial port driver

    DDRB &= ~(1 << DDB1); // set pin 6 (PB1) as input
    PORTB |= (1 << PB1); // enable pull-up on pin 6 (PB1)
}

// stop communicating (usually if PC terminal becomes inactive)
static inline void uart_mouse_stop() {
    uart_abort();
}

// check for "Logitech" jumper
static inline bool uart_mode_switch_closed(void) {
    return ((PINB & (1 << PINB1)) == 0); // pin 6 (PB1)
}

// form and transmit "standart" (without extensions) serial mouse packet
static void uart_mouse_packet(const int8_t DX, const int8_t DY, const bool left, const bool right) {
    uart_tx((1 << 6) | ((left ? 1 : 0) << 5) | ((right ? 1 : 0) << 4) | ((((uint8_t) DY) >> 6) << 2) | ((((uint8_t) DX) >> 6) << 0));
    uart_tx((0 << 6) | (((uint8_t) DX) & 0b00111111));
    uart_tx((0 << 6) | (((uint8_t) DY) & 0b00111111));
}

// transmit "mouse" marker
static inline void uart_mouse_tx_id(void) {
    uart_tx('M'); // "Mouse"
}

// transmit buttons count (optional)
static inline void uart_mouse_tx_ext_id(void) {
    #if defined(BUTTONS) && BUTTONS >= 2 && BUTTONS <= 3
    uart_tx('0' + BUTTONS); // send buttons count
    #endif
}

// transmit complete mouse event
static void uart_mouse_tx_event(const int8_t DX, const int8_t DY, const bool left, const bool right, const bool middle) {
    static bool prev_middle = false;

    if(uart_tx_room() >= 4) { // only if enough space available 
        if(uart_mode_switch_closed()) { // "Logitech" protocol
            uart_mouse_packet(DX, DY, left, right); // standart packet
            if(middle) { // if middle button pressed
                uart_tx(0x20); // send Logitech-style extension for middle button
            }
        } else { // "Microsoft" protocol
            if(middle == prev_middle) { // if no middle button state changed
                uart_mouse_packet(DX, DY, left, right); // send standart packet
            } else {
                uart_mouse_packet(0, 0, false, false); // send Microsoft-style extension for middle button
            }
        }

        prev_middle = middle; // save current state of middle button
    }
}

#endif /* UART_MOUSE_H_ */
