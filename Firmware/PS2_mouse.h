/*
 * PS2_mouse.h
 *
 * PS/2 mouse driver
 * 
 * Author: Mike Cherkes, UR4UEM
 *
 */ 

#ifndef PS2_MOUSE_H_
#define PS2_MOUSE_H_

#include <stdint.h>
#include <stdbool.h>

#include "Defines.h"
#include "PS2.h"
#include "DataProc.h"

#define PS2_MOUSE_CURRENT_CODE_NULL 0x00

typedef struct {
    int8_t DX, DY;
    bool middle, right, left;
} ps2_mouse_state_t;

typedef struct {
    bool updated;
    ps2_mouse_state_t state;
} ps2_mouse_internal_state_t;

static uint8_t ps2_mouse_current_code = PS2_MOUSE_CURRENT_CODE_NULL;
static bool ps2_mouse_ok = false;
static ps2_mouse_internal_state_t ps2_mouse_state = {.updated = false, 
                                                     .state = {.DX = 0, 
                                                               .DY = 0, 
                                                               .middle = false, 
                                                               .right = false, 
                                                               .left = false}
                                                    };

static inline void ps2_mouse_init(const ps2_mouse_state_t **const state) {
    ps2_init(); // init PS/2 bus driver
    *state = &ps2_mouse_state.state; // return current mouse state buffer for quick access from upper layer
}

static void ps2_mouse_stop(void) {
    ps2_abort(); // stop all pending transfers and clear all buffers
    ps2_tx(0xFF); // reset mouse command
    ps2_mouse_ok = false;
}

static void ps2_mouse_start(void) {
    ps2_abort(); // clear all buffers before start
    
    #ifdef UART_HI_BPS
    #ifndef UART_19200_BPS
    //ps2_tx(0xF3); // set report rate command
    //ps2_tx(90); // in the 9600-bps mode ?? reports per second fit into UART bandwidth without drops
    #else
    //ps2_tx(0xF3); // set report rate command
    //ps2_tx(180); // in the 19200-bps mode 180 reports per second fit into UART bandwidth without drops
    #endif
    #else
    ps2_tx(0xF3); // set report rate command
    ps2_tx(20); // in the 1200-bps mode only 20 reports per second fit into UART bandwidth without drops
    #endif
    
    ps2_tx(0xF4); // start auto reporting
}

// check for last mouse response and reset flag
static bool ps2_mouse_responce_ok(void) {
    if(ps2_mouse_ok) {
        ps2_mouse_ok = false;
        return true;
    } else {
        return false;
    }
}

// check for mouse data updated and reset flag
static bool ps2_mouse_was_updated(void) {
    if(ps2_mouse_state.updated) {
        ps2_mouse_state.updated = false;
        return true;
    } else {
        return false;
    }
}

// mouse events queue dispatcher
static void ps2_mouse_process(const bool ignore_all) {
    if(ps2_rx_count() > 0) { // in data exists
        if(ignore_all) { // if ignore mode (usually if PC terminal is inactive)
            while(ps2_rx_count() > 0) { // retrieve all bytes to null
                ps2_rx();
            }
        } else {
            if(ps2_mouse_current_code == PS2_MOUSE_CURRENT_CODE_NULL) { // if no command pending
                ps2_mouse_current_code = ps2_rx(); // retrieve next byte
            }

            switch(ps2_mouse_current_code) {
                case PS2_MOUSE_CURRENT_CODE_NULL: // null char, do nothing
                    break;

                case 0xFA: // "OK" answer from mouse
                    ps2_mouse_ok = true;
                    ps2_mouse_current_code = PS2_MOUSE_CURRENT_CODE_NULL;
                    break;

                default:
                    if((ps2_mouse_current_code & (1 << 3)) == 0) { // in the first byte of PS/2 packet 3-th bit always "high"
                        ps2_mouse_current_code = PS2_MOUSE_CURRENT_CODE_NULL; // skip unknown command
                    } else if(ps2_rx_count() >= 2) { // if whole command present in the RX buffer, extract data
                        ps2_mouse_state.state.middle = ((ps2_mouse_current_code & (1 << 2)) != 0);
                        ps2_mouse_state.state.right = ((ps2_mouse_current_code & (1 << 1)) != 0);
                        ps2_mouse_state.state.left = ((ps2_mouse_current_code & (1 << 0)) != 0);
                        ps2_mouse_state.state.DX = dp_i9to8(ps2_rx(), (ps2_mouse_current_code & (1 << 4)) != 0);
                        ps2_mouse_state.state.DY = -dp_i9to8(ps2_rx(), (ps2_mouse_current_code & (1 << 5)) != 0); // Y-axis is inverted in the serial protocol
                        ps2_mouse_state.updated = true; // set "updated" flag
                        ps2_mouse_current_code = PS2_MOUSE_CURRENT_CODE_NULL; // command completed
                    }
                    break;
            }
        }
    }
}

#endif /* PS2_MOUSE_H_ */
