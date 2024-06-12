# PS2 to Serial mouse adapter electronics

Schematic diagrams and PCB layouts for both PS/2 and USB adapter variants.

Please note differences of RTS-only and RTS-DTR implementations.

RTS-only implementation listen only RTS line of the RS232C port to detect init/reset signal from the mouse driver. The DTR line is ignored. This mean not full compliance with Serial Mouse specs, but enough to most existing mouse drivers. But it does give an opportunity to preserve pin 1 of microcontroller as default RESET function and use simple and cheap ISP programmers (such as [USBASP](https://www.google.com/search?q=usbasp)) to flash IC.

In this mode resistor R4 must NOT be installed on PCB, and there is recommended to install resistor R1 as RESET pull-up.

RTS-DTR implementation as opposed to RTS-only listen both RTS and DTR signals and will compatible with any mouse driver. But this is achieved at the cost of switching pin 1 to GPIO mode and lose microcontroller's ISP function. Since fuses will flashed, only high voltage serial programming tool can be used to write the IC (including reset fuses to default state). So be very careful with fuses for RTS-DTR mode.

For RTS-DTR mode resistor R4 (0-10 Ohm) must be installed to PCB and R1 must NOT be installed.

Everything else is same in both modes.
