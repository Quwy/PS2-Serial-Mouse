/*
 * main.c
 *
 * Main module
 * 
 * Author: Mike Cherkes, UR4UEM
 *
 */ 

#include "Defines.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <stdbool.h>

#if defined(START_MARKER)
#include <util/delay.h>
#endif

#include "PS2_mouse.h"
#include "UART_mouse.h"

#define INITIAL_PINB ((1 << PB0) | (1 << PB2) | (1 << PB4)) // initial state of pins on PORTB (initial value assumes high levels on the SDA ans SCL as bus idle state, high on RTS as no terminal connected)

typedef enum {
    TERMINAL_STATE_INACTIVE,
    TERMINAL_STATE_PENDING,
    TERMINAL_STATE_ACTIVE
} terminal_state_t;

static volatile terminal_state_t terminal_state;
static volatile bool terminal_tx_ext_id = false;

// init watchdog
static inline void wdt_init(void) {
    WDTCR = (1 << WDCE) | (1 << WDE); // watchdog preparing
    WDTCR = (1 << WDIE) | WDTO_60MS; // setup watchdog to interrupt mode every 60 ms
}

// init GPIO system
static inline void gpio_init(void) {
    
    #ifdef START_MARKER // start marker for logic analizer trigger
    DDRB |= (1 << DDB3);
    _delay_us(10);
    DDRB &= ~(1 << DDB3);
    #endif

    GIMSK |= (1 << PCIE); // Enable pin changes interrupt
}

// init MCU
static inline void mcu_init(void) {
    sleep_enable(); // prepare sleep mode
    set_sleep_mode(SLEEP_MODE_IDLE); // SLEEP_MODE_IDLE allows to handle all necessary events
    sei(); // enable interrupts

    #ifdef PROTEUS // Proteus incorrectly simulates PLL clock (set 64 MHz instead of 16 MHz)
    CLKPR |= (1 << CLKPCE); // enable system clock prescaler
    CLKPR = (1 << CLKPS1);  // set prescaler for /4
    #endif
}

// pin change event local handler
static inline void main_PCINT0(const uint8_t prev_PINB) {
    if(uart_host_active(prev_PINB) && !uart_host_active(PINB)) { // if terminal ready pins changed state from "ready" to "not ready"
        switch(terminal_state) {
            case TERMINAL_STATE_ACTIVE: // if terminal was active, stopping all and deactivate terminal state
                uart_mouse_stop();
                ps2_mouse_stop(); // reset mouse and stop PS2 communication
                terminal_tx_ext_id = false;
                terminal_state = TERMINAL_STATE_INACTIVE;
                break;

            case TERMINAL_STATE_PENDING: // if terminal in pending state, just reset state to inactive
                terminal_state = TERMINAL_STATE_INACTIVE;
                break;

            default: // nothing to do if terminal already inactive
                break;
        }
    }
}

// perform periodic checks and tasks
static inline void main_timer_process(void) {
    switch(terminal_state) {
        case TERMINAL_STATE_PENDING: // if terminal state remains pending whole WDT timer delay, set terminal ready state
            terminal_state = TERMINAL_STATE_ACTIVE;
            ps2_mouse_start(); // init PS/2 mouse and start communication
            break;

        case TERMINAL_STATE_INACTIVE: // if terminal state is inactive
            if(uart_host_active(PINB)) { // check pins
                terminal_state = TERMINAL_STATE_PENDING; // set sdtate to pending if pins in the active state
            }
            break;

        default: // if terminal is active or unknown state, do nothing
            break;
    }

    if(terminal_tx_ext_id && terminal_state == TERMINAL_STATE_ACTIVE) {
        uart_mouse_tx_ext_id(); // send extended id if terminal is active and primary id already sent
        terminal_tx_ext_id = false;
    }
}


// main function
int main(void) {
    const ps2_mouse_state_t *ps2_mouse_state;
    
    gpio_init();
    mcu_init();
    uart_mouse_init();
    ps2_mouse_init(&ps2_mouse_state); // init PS2 mouse and get mouse data pointer
    wdt_init();
    
    #ifndef DUMMY_TEST_MODE

    terminal_state = (uart_host_active(PINB) ? TERMINAL_STATE_PENDING : TERMINAL_STATE_INACTIVE); // initial terminal state

    // main event loop
    while(true) {
        sleep_cpu(); // always try to avoid free-running main loop, use sleep mode if no current tasks

        ps2_mouse_process(terminal_state != TERMINAL_STATE_ACTIVE); // process mouse events if PC terminal is active
        
        if(ps2_mouse_responce_ok()) { // if init mouse response is "OK"
            uart_mouse_tx_id(); // send primary mouse id to port
            terminal_tx_ext_id = true; // reload external id send flag
        }

        if(ps2_mouse_was_updated()) { // if mouse data updated
            uart_mouse_tx_event(ps2_mouse_state->DX, ps2_mouse_state->DY, ps2_mouse_state->left, ps2_mouse_state->right, ps2_mouse_state->middle); // sent UART packet with ioncoming data
        }
    }

    #else

    #ifndef UART_9600_BPS
    ps2_tx(0xF3); // set report rate
    ps2_tx(20); // in 1200 bps mode only 20 reports per second fit into UART bandwidth
    #endif

    ps2_tx(0xF4); // start auto reporting

    // main event loop
    while(true) {
        sleep_cpu();

        // repeat all PS/2 outgoing data to the UART
        while(ps2_rx_count() > 0) {
            uart_tx(ps2_rx());
        }
    }

    #endif
}

// pin change interrupt handler (general for all modules)
ISR(PCINT0_vect) {
    static uint8_t prev_PINB = INITIAL_PINB;

    GIFR |= (1 << PCIF); // clear interrupt flag

    ps2_PCINT0(prev_PINB); // call PS/2 pin change handler
    main_PCINT0(prev_PINB); // call main pin change handler

    prev_PINB = PINB; // save prior port state
}

// watchdog interrupt handler
ISR(WDT_vect) {
    WDTCR |= (1 << WDIE); // important to renew WDIE bit in each call (reset will performed if no WDIE bit at next watchdog "tick")

    #ifndef DUMMY_TEST_MODE
    main_timer_process(); // UART scheduled events dispatcher
    #endif
}
