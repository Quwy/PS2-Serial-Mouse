# PS2 to Serial mouse adapter Firmware

This folder contain Firmware sources for the adapter.

To build please use Microchip Studio 7.0 or later with AVR/GNU C Compiler 7.3.0 or more modern. Previous versions of compiler can reach microcontroller's flash size.

Please see [Defines.h](https://github.com/Quwy/PS2-Serial-Mouse/blob/main/Firmware/Defines.h) file for build options before make.

## FUSES

### RTS-only fuses

efuse 0x01
lfuse 0xc1
hfuse 0xdf

#### RTS-DTR fuses

efuse 0x01
lfuse 0xc1
hfuse 0x7f
