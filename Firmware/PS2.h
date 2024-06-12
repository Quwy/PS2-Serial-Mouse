/*
 * PS2.h
 *
 * PS/2 bus driver (USI-based)
 * 
 * Author: Mike Cherkes, UR4UEM
 *
 */ 

#ifndef PS2_H_
#define PS2_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

#include "Defines.h"
#include "BufProc.h"
#include "DataProc.h"

#define PS2_TX_BUFFER_SIZE 3 // will totally enough because maximum 3 bytes send per whole session
#define PS2_RX_BUFFER_SIZE (3 * 4) // 4 mouse reports

#define PS2_BUS_TIMEOUT 2 // approx, ms

typedef enum {
    PS2_STAGE_IDLE,
    PS2_STAGE_TX_PULL_SCL_DOWN,
    PS2_STAGE_TX_PULL_SDA_DOWN,
    PS2_STAGE_TX_RELEASE_SCL,
    PS2_STAGE_TX_WAIT_SYNC,
    PS2_STAGE_TX_FIRST_PART,
    PS2_STAGE_TX_LAST_PART,
    PS2_STAGE_TX_WAIT_SDA_LOW,
    PS2_STAGE_TX_WAIT_SCL_LOW,
    PS2_STAGE_TX_WAIT_BUS_RELEASE,
    PS2_STAGE_RX_FIRST_PART,
    PS2_STAGE_RX_LAST_PART
} ps2_stage_t;

typedef enum {
    PS2_DELAY_UNIT_US,
    PS2_DELAY_UNIT_MS
} ps2_delay_unit_t;

static volatile ps2_stage_t ps2_stage = PS2_STAGE_IDLE; // Current machine stage
static volatile uint8_t ps2_tx_buffer[PS2_TX_BUFFER_SIZE]; // TX buffer
static volatile uint8_t ps2_rx_buffer[PS2_RX_BUFFER_SIZE]; // RX buffer
static volatile uint8_t ps2_tx_buffer_size = 0; // Actual size of TX data
static volatile uint8_t ps2_rx_buffer_size = 0; // Actual size of RX data
static uint8_t ps2_current_byte; // buffer for current byte processed (RX or TX)
static bool ps2_current_byte_parity; // buffer for current byte parity (lower bit, TX only)

static inline void ps2_init(void) {
    // USI init
    USIDR = 0xFF; // high bit must be "1" in idle state for leave SDA released, but set all bits to "1" for future left shift actions
    USICR = (1 << USIWM1) | // enable USI in two wire mode (clock disabled for idle mode), since this SDA and SCL pins operates as open-collector (SDA = USIDR[7] && PB0)
            (1 << USIOIE) | // enable Counter Overflow Interrupt (USI_OVF_vect)
            (1 << USISIE); // enable TWI/I2C start condition interrupt (only for discard all corresponding unusual actions, because can't to completely disable start condition detector in two-wire mode)

    // Pins init
    PORTB |= (1 << PB0) | (1 << PB2); // release pins 5 (PB0/SDA), 7 (PB2/SCL), if they was pulled down
    DDRB |= (1 << DDB0) | (1 << DDB2); // set pins 5 (PB0/SDA), 7 (PB2/SCL) as outputs (with open-collector in the two-wire USI mode)

    // Timer1 init
    TCCR1 = (1 << CTC1); // generate TIMER1_COMPA_vect interrupt on compare match with OCR1A
    TIMSK |= (1 << OCIE1A); // enable TIMER1_COMPA_vect overflow Interrupt

    // Interrupts init
    PCMSK |= (1 << PCINT0) | (1 << PCINT2); // Enable pins 5 (PB0/SDA), 7 (PB2/SCL) change interrupt
}

// start delay timer (delay parameter must be greater than 0)
static void ps2_start_timer(const ps2_delay_unit_t units, const uint8_t delay) {
    uint8_t div;

    if(units == PS2_DELAY_UNIT_US) {
        div = (1 << CS10) | (1 << CS12); // divider 16 for 1 usec interval
    } else {
        div = (1 << CS10) | (1 << CS11) | (1 << CS12) | (1 << CS13); // divider 16384 for ~1 msec interval
    }

    TCNT1 = 0; // reset timer counter
    OCR1A = delay - 1; // set end value for timer
    TCCR1 = (TCCR1 & ~((1 << CS10) | (1 << CS11) | (1 << CS12) | (1 << CS13))) | div; // set divider and start timer
}

// stop delay timer
static inline void ps2_stop_timer(void) {
    TCCR1 &= ~(1 << CS10) & ~(1 << CS11) & ~(1 << CS12) & ~(1 << CS13); // stop timer
}

// start transmitting
static void ps2_start_byte_tx(void) {
    if(ps2_stage == PS2_STAGE_IDLE) { // start TX only if idle
        ps2_stage = PS2_STAGE_TX_PULL_SCL_DOWN; // first stage to form TX start condition
        ps2_start_timer(PS2_DELAY_UNIT_US, 10); // keep some interval after last byte, if it was
    }
}

// start recieving
static void ps2_start_byte_rx(void) {
    ps2_stage = PS2_STAGE_RX_FIRST_PART; // first stage begins
    USISR = (USISR & 0b11110000) | (0b00001111 & (16 - 11)); // 11 edges (rising edge of Start bit clock, and 10 edges for 5 data bits DATA0-DATA4)
    USICR |= (1 << USICS0) | (1 << USICS1); // enable USI external clock, falling edge
}

// stop byte TX or RX
static void ps2_stop_byte(const ps2_stage_t new_stage) {
    ps2_stage = new_stage;
    USICR &= ~(1 << USICS0) & ~(1 << USICS1); // disable USI clock
    USIDR = 0xFF; // high bit must be "1" in idle state for leave SDA released, but set all bits to "1" for future left shift actions
    PORTB |= (1 << PB0) | (1 << PB2); // release pins 5 (PB0/SDA), 7 (PB2/SCL), if they was pulled down
}

// 
static void ps2_process_tx(void) {
    if(ps2_tx_buffer_size > 0) { // if TX buffer is not empty
        ps2_start_byte_tx(); // start transmitting
    }
}

// transmit one byte (or place it to TX buffer if bus busy)
static void ps2_tx(const uint8_t byte) {
    buf_set_byte(byte, ps2_tx_buffer, &ps2_tx_buffer_size, PS2_TX_BUFFER_SIZE);
    ps2_start_byte_tx();
}

// local PCINT0 handler, must be called from ISR(PCINT0_vect)
static inline void ps2_PCINT0(const uint8_t prev_PINB) {
    switch(ps2_stage) {
        case PS2_STAGE_IDLE: // in idle state check all signals for start condition
            if((prev_PINB & (1 << PINB2)) != 0 && (PINB & (1 << PINB2)) == 0 && // previous SCL (PB2) state was "high" and current state becomes "low" (falling edge of clock signal)
               (PINB & (1 << PINB0)) == 0) { // SDA (PB0) already low
                ps2_start_byte_rx(); // starting receive
                ps2_start_timer(PS2_DELAY_UNIT_MS, PS2_BUS_TIMEOUT); // start bus timeout countdown
            }
            break;

        case PS2_STAGE_TX_WAIT_SYNC: // first TX stage: waiting for clock signal
            if((prev_PINB & (1 << PINB2)) != 0 && (PINB & (1 << PINB2)) == 0) { // previous SCL (PB2) state was "high" and current state becomes "low" (falling edge of clock signal)
                ps2_stage = PS2_STAGE_TX_FIRST_PART; // sending first part of data
                USIDR = (ps2_current_byte | 0b00000111); // high part of the data byte passed "as is", lower bytes always "high" to release SDA after part will complete and register shifts left
                USISR = (USISR & 0b11110000) | (0b00001111 & (16 - 10)); // 10 edges (for 5 data bits DATA0-DATA4)
                PORTB |= (1 << PB0); // release SDA (let it driven by USIDR[7])
                USICR |= (1 << USICS1); // enable USI external clock, rising edge
            }
            break;

        case PS2_STAGE_TX_WAIT_SDA_LOW: // main TX stage completed, waiting for SDA becomes low as first ACK sign
            if((prev_PINB & (1 << PINB0)) != 0 && (PINB & (1 << PINB0)) == 0) { // SDA becomes low, go to the next stage
                ps2_stage = PS2_STAGE_TX_WAIT_SCL_LOW; // now must wait for SCL becomes low
            }
            break;

        case PS2_STAGE_TX_WAIT_SCL_LOW: // main TX stage completed, SDA already low, waiting for SCL becomes low as second ACK sign
            if((prev_PINB & (1 << PINB2)) != 0 && (PINB & (1 << PINB2)) == 0) { // SCL becomes low, go to the next stage
                ps2_stage = PS2_STAGE_TX_WAIT_BUS_RELEASE; // now must wait for whole bus release as sign of idle state
            }
            break;

        case PS2_STAGE_TX_WAIT_BUS_RELEASE: // SDA and SCL are low, now just wait for they released as sign of bus idle state
            if((PINB & (1 << PINB0)) != 0 && (PINB & (1 << PINB2)) != 0) { // only if both SDA and ACL are high
                ps2_stage = PS2_STAGE_IDLE; // all completed
                ps2_stop_timer(); // stop bus timeout timer
                ps2_process_tx(); // try to send next byte
            }
            break;

        default: // another stages processed in the USI_OVF_vect and TIMER1_COMPA_vect handlers
            break;
    }

    sei(); // enable interrupts
}

// get received bytes count
static inline uint8_t ps2_rx_count(void) {
    return ps2_rx_buffer_size;
}

// get one byte from RX buffer
// !!! NO ACTUAL BUFFER LENGTH CHECKED !!!
// !!! CALL uart_rx_count() BEFORE !!!
static inline uint8_t ps2_rx(void) {
    return buf_get_byte(ps2_rx_buffer, &ps2_rx_buffer_size);
}

// calculate Odd parity bit for byte
static bool ps2_calc_parity(const uint8_t byte) {
    bool odd_bit = true;

    for(uint8_t i = 0; i < 8; ++i) {
        if((byte & (1 << i)) != 0) {
            odd_bit = !odd_bit;
        }
    }

    return odd_bit;
}

// abort all PS/2 communications (usually after PC terminal becomes inactive)
static void ps2_abort(void) {
    if(ps2_stage != PS2_STAGE_IDLE) {
        ps2_stop_byte(PS2_STAGE_IDLE); // stop USI and reset stage
        ps2_stop_timer(); // stop bus timeout timer
        ps2_rx_buffer_size = 0; // clear RX buffer
        ps2_tx_buffer_size = 0; // clear TX buffer
    }
}

// TWI/I2C start condition interrupt
ISR(USI_START_vect) {
    USISR |= (1 << USISIF); // reset start condition flag -> discard all corresponding bus locks
}

// USI counter overflow interrupt
ISR(USI_OVF_vect) {
    USISR |= (1 << USIOIF); // clear interrupt flag
    cli(); // disable interrupts to avoid timings violation

    switch(ps2_stage) {
        case PS2_STAGE_TX_FIRST_PART: // first TX stage completed
            ps2_stage = PS2_STAGE_TX_LAST_PART; // ready for next stage
            USIDR = (ps2_current_byte << 5) | (ps2_current_byte_parity ? 0b00010000 : 0b00000000) | (0b00001111); // lower part of the data byte, Parity bit, "1111" for release SDA line after completion
            USISR |= (0b00001111 & (16 - 8)); // 8 edges (6 edges for 3 data bits DATA5-DATA7, 2 edges for Parity)
            break;

        case PS2_STAGE_TX_LAST_PART: // last TX stage completed
            ps2_stop_timer(); // stop bus timeout timer
            ps2_stop_byte(PS2_STAGE_TX_WAIT_SDA_LOW); // stop transmitting
            break;

        case PS2_STAGE_RX_FIRST_PART: // first RX stage completed
            ps2_stage = PS2_STAGE_RX_LAST_PART; // set next stage
            ps2_current_byte = (USIDR << 3); // save first 5 bits as high part of the byte
            USIDR = 0xFF; // make sure SDA line will released
            USISR |= (0b00001111 & (16 - 10)); // 10 edges (6 edges for 3 data bits DATA5-DATA7, 2 edges for Parity, 2 edges for Stop)
            break;

        case PS2_STAGE_RX_LAST_PART: // last RX stage completed
            ps2_stop_timer(); // stop bus timeout timer
            if((USIDR & 0b00000001) != 0) { // if stop bit is "high"
                ps2_current_byte |= (USIDR >> 2) & 0b00000111; // save last 3 bits as low part of the byte (Parity and Stop bits are shifted out)
                if(ps2_calc_parity(ps2_current_byte) == ((USIDR & 0b00000010) != 0)) { // check parity
                    buf_set_byte(dp_reverse_byte(ps2_current_byte), ps2_rx_buffer, &ps2_rx_buffer_size, PS2_RX_BUFFER_SIZE); // add byte to the RX buffer
                }
            }
            ps2_stop_byte(PS2_STAGE_IDLE); // stop receiving
            break;

        default: // unknown stage, must not be here, resetting state
            ps2_stop_byte(PS2_STAGE_IDLE); // stop any transferring
            ps2_stop_timer(); // stop bus timeout timer
    }

    sei(); // enable interrupts
    ps2_process_tx(); // send next byte if exists
}

// delay timer interrupt handler
ISR(TIMER1_COMPA_vect) {
    cli(); // disable interrupts
    ps2_stop_timer(); // stop timer

    switch(ps2_stage) {
        case PS2_STAGE_TX_PULL_SCL_DOWN: // time to pull SCL down
            ps2_stage = PS2_STAGE_TX_PULL_SDA_DOWN; // set next stage
            ps2_start_timer(PS2_DELAY_UNIT_US, 100); // set timer for handle next stage
            PORTB &= ~(1 << PB2); // pull SCL down
            break;

        case PS2_STAGE_TX_PULL_SDA_DOWN: // time to pull SDA down
            ps2_stage = PS2_STAGE_TX_RELEASE_SCL; // set next stage
            ps2_start_timer(PS2_DELAY_UNIT_US, 10); // set timer for handle next stage
            PORTB &= ~(1 << PB0); // pull SDA down
            break;

        case PS2_STAGE_TX_RELEASE_SCL: // time to release SCL
            ps2_stage = PS2_STAGE_TX_WAIT_SYNC; // next stage: wait for clock signal
            PORTB |= (1 << PB2); // release SCL
            ps2_current_byte = dp_reverse_byte(buf_get_byte(ps2_tx_buffer, &ps2_tx_buffer_size)); // extract next byte from TX buffer
            ps2_current_byte_parity = ps2_calc_parity(ps2_current_byte); // calculate parity bit (will transmit it later without a calculation delay)
            ps2_start_timer(PS2_DELAY_UNIT_MS, PS2_BUS_TIMEOUT); // start bus timeout countdown
            break;

        default: // another stages can be here only if bus timeout expired
            ps2_stop_byte(PS2_STAGE_IDLE); // stop any transferring
            // timer already stopped in top of handler
            ps2_process_tx(); // try to send next byte, if exists
    }

    sei(); // enable interrupts
}

#endif /* PS2_H_ */
