/*
 * Defines.h
 *
 * Project defines
 * 
 * Author: Mike Cherkes, UR4UEM
 *
 */ 

#ifndef DEFINES_H_
#define DEFINES_H_

/* VERSION */
#define VERSION "1.00b" // not used in the code, just text tag

/* UART SECTION */
//#define UART_9600_BPS // define for 9600 bps, otherwise 1200 bps wiil used. note what 9600 bps is not common and will not work with most mouse drivers
#define BUTTONS     3 // assume 3-button mouse (no risk if 2-button mouse attached)
//#define RESET_AS_DTR // define if pin 1 (RESET/PB5) used as GPIO for DTR line input

/* FUSES SECTION */
// these values not used in the code, but will useful to configure microcontroller
#ifdef RESET_AS_DTR
    #define efuse 0x01
    #define lfuse 0xc1
    #define hfuse 0x7f
#else
    #define efuse 0x01
    #define lfuse 0xc1
    #define hfuse 0xdf
#endif

/* DEBUG SECTION */
//#define DUMMY_TEST_MODE // just repeat all incoming data from PS/2 to UART (high bit truncated)
//#define START_MARKER    // generate 10 uS marker on pin 2 (PB3/TX) at startup for logic analizer trigger

#endif /* DEFINES_H_ */
