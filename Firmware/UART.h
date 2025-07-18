/*
 * UART.h
 *
 * UART simplified (TX-only) driver
 * 
 * Author: Mike Cherkes, UR4UEM
 *
 */ 

#ifndef UART_H_
#define UART_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

#include "Defines.h"
#include "BufProc.h"

#define UART_TX_BUFFER_SIZE (4 * 3) // 3 packets of 3-button mouse or 4 packets of 2-button mouse

#ifdef UART_HI_BPS
  #ifndef UART_19200_BPS
    #define UART_BPS_CORRECTION (-1) // measured on the real hardware
  #else
    #define UART_BPS_CORRECTION (-2) // measured on the real hardware
  #endif
#else
  #define UART_BPS_CORRECTION (+8) // measured on the real hardware
#endif

typedef enum {
    UART_STAGE_IDLE,
    UART_STAGE_TX_START_BIT,
    UART_STAGE_TX_DATA,
    UART_STAGE_TX_STOP_BIT,
    UART_STAGE_TX_FINISH // second stop bit (as recommended)
} uart_stage_t;

static volatile uint8_t uart_current_byte; // current transmitted byte
static volatile uint8_t uart_current_bit; // current transmitted bit
static volatile uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE]; // TX buffer
static volatile uint8_t uart_tx_buffer_size = 0; // actual size of TX data
static volatile uart_stage_t uart_stage = UART_STAGE_IDLE; // protocol stage

static inline void uart_init(void) {
    // Pins init
    PORTB &= ~(1 << PB4); // disable pull-up for pin 3 (PB4/RTS)
    DDRB &= ~(1 << DDB4); // set pin 3 (PB4/RTS) as input
    #ifdef RESET_AS_DTR
    PORTB &= ~(1 << PB5); // disable pull-up for pin 1 (PB5/DTR)
    DDRB &= ~(1 << DDB5); // set pin 1 (PB5/DTR) as input
    #endif
    PORTB |= (1 << PB3); // set pin 2 (PB3/RXD) to "high" idle state
    DDRB |= (1 << DDB3); // set pin 2 (PB3/RXD) as output

    // Timer0 init
    TCCR0A |= (1 << WGM01); // set Timer0 CTC mode: top value is OCR0A
    TIMSK |= (1 << OCIE0A); // enable compare match interrupt with OCR0A (TIMER0_COMPA_vect)

    // Interrupts init
    PCMSK |= (1 << PCINT4); // enable pin 3 (PB4/RTS) change interrupt
    #ifdef RESET_AS_DTR
    PCMSK |= (1 << PCINT5); // enable pin 1 (PB5/RESET/DTR) change interrupt
    #endif
}

// start UART timer
static void uart_start_timer(void) {
    TCNT0 = 0; // reset timer counter
    #ifdef UART_HI_BPS
    #ifndef UART_19200_BPS
    OCR0A = 208 + UART_BPS_CORRECTION; // set top value for 9600 bps (2 000 000 / 9 600) = 208 (plus correction from the real device sample)
    #else
    OCR0A = 104 + UART_BPS_CORRECTION; // set top value for 19200 bps (2 000 000 / 19200) = 104 (plus correction from the real device sample)
    #endif
    TCCR0B |= (1 << CS01); // start timer with prescaler /8 (16 000 000 / 8 = 2 000 000 increments/sec)
    #else
    OCR0A = 208 + UART_BPS_CORRECTION; // set top value for 1200 bps (250 000 / 1200) = 208 (plus correction from the real device sample)
    TCCR0B |= (1 << CS00) | (1 << CS01); // start timer with prescaler /64 (16 000 000 / 64 = 250 000 increments/sec)
    #endif
}

// stop UART timer
static void uart_stop_timer(void) {
    TCCR0B &= ~(1 << CS00) & ~(1 << CS01) & ~(1 << CS02); // stop timer clock
}

// check for PC ready to communicate with mouse
static inline bool uart_host_active(const uint8_t PORT) {
    #ifdef RESET_AS_DTR
    return ((PORT & ((1 << PB4) | (1 << PB5))) == 0); // check for "low" on pins 3 (PB4/RTS), 1 (PB5/RESET/DTR), equialent of ((PORT & (1 << PB4)) == 0) && ((PORT & (1 << PB5)) == 0)
    #else
    return ((PORT & (1 << PB4)) == 0); // check for "low" on pin 3 (PB4/RTS)
    #endif
}

// start transmitting
static void uart_start_byte_tx() {
    if(uart_stage == UART_STAGE_IDLE && uart_host_active(PINB)) { // start TX only if idle and PC terminal is active
        uart_stage = UART_STAGE_TX_START_BIT; // first stage to form TX start condition
        uart_current_byte = buf_get_byte(uart_tx_buffer, &uart_tx_buffer_size); // extract next byte from TX buffer
        uart_current_bit = 0; // low bit first
        uart_start_timer(); // start transmitting
    }
}

// interface function to transmit a byte
static void uart_tx(const uint8_t byte) {
    buf_set_byte(byte, uart_tx_buffer, &uart_tx_buffer_size, UART_TX_BUFFER_SIZE); // extract next byte from buffer
    uart_start_byte_tx(); // start transmitting
}

// set state of TX data line
static void uart_set_tx(const bool state) {
    if(state) {
        PORTB |= (1 << PB3); // pin 2 (PB3/TX) to "high"
    } else {
        PORTB &= ~(1 << PB3); // pin 2 (PB3/TX) to "low"
    }
}

// check for bytes in the buffer and start TX if any
static void uart_process_tx(void) {
    if(uart_tx_buffer_size > 0) { // if buffer not empty
        uart_start_byte_tx();
    }
}

// abort any transmitting and clear buffers (usually if terminal becomes inactive)
static void uart_abort(void) {
    uart_stage = UART_STAGE_IDLE; // transmission aborted
    uart_stop_timer(); // stop UART clock timer
    uart_set_tx(true); // set TX line idle state
    uart_tx_buffer_size = 0; // clear TX buffer
}

// check for available space in the TX buffer
static inline uint8_t uart_tx_room(void) {
    return UART_TX_BUFFER_SIZE - uart_tx_buffer_size;
}

// UART clock timer handler
ISR(TIMER0_COMPA_vect) {
    switch(uart_stage) {
        case UART_STAGE_TX_START_BIT: // generate start bit delay
            uart_stage = UART_STAGE_TX_DATA; // next stage after delay finished
            uart_set_tx(false); // start bit always "low"
            break;

        case UART_STAGE_TX_DATA: // data stage
            uart_set_tx((uart_current_byte & (1 << uart_current_bit)) != 0); // set TX line according to current bit
            if(++uart_current_bit == 7) { // if last (7-th) bit was populated on TX line
                uart_stage = UART_STAGE_TX_STOP_BIT; // set next stage
            }
            break;

        case UART_STAGE_TX_STOP_BIT: // generate first stop bit delay
            uart_stage = UART_STAGE_TX_FINISH; // set next stage
            uart_set_tx(true); // stop bit always "high"
            break;

        case UART_STAGE_TX_FINISH: // second stop bit finished
        default: // if any unexpected state passed
            uart_stage = UART_STAGE_IDLE; // transmission finished
            uart_set_tx(true); // set TX line idle state
            uart_stop_timer(); // stop UART clock timer
            uart_process_tx(); // check buffer for next byte and start transmitting if any
    }
}

#endif /* UART_H_ */
