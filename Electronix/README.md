# PS2 to Serial mouse adapter electronics

Schematic diagrams and PCB layouts for both PS/2 and USB adapter variants.

## PS/2 socket

![PS/2 Diagramm](https://github.com/Quwy/PS2-Serial-Mouse/blob/main/Electronix/Output/ps2_com_ps2.png)

## USB socket

![USB Diagramm](https://github.com/Quwy/PS2-Serial-Mouse/blob/main/Electronix/Output/ps2_com_usb.png)

## Differences between RTS-only and RTS-DTR implementations.

**RTS-only** implementation listen only RTS line of the RS232C port to detect init/reset signal from the mouse driver. The DTR line is ignored. This mean not full compliance with Serial Mouse specs, but enough to most existing mouse drivers. But it does give an opportunity to preserve pin 1 of microcontroller as default RESET function and use simple and cheap ISP programmers (such as [USBASP](https://www.google.com/search?q=usbasp)) to flash IC.

In this mode resistor R4 must **NOT** be installed on PCB, and there is recommended to install resistor R1 as RESET pull-up.

**RTS-DTR** implementation as opposed to RTS-only listen both RTS and DTR signals and will compatible with any mouse driver. But this is achieved at the cost of switching pin 1 to GPIO mode and lose microcontroller's ISP function. Since fuses will flashed, only high voltage serial programming tool can be used to write the IC (including reset fuses to default state). So be very careful with fuses for RTS-DTR mode.

For RTS-DTR mode resistor R4 (0-10 Ohm or jumper wire) must be installed to PCB and R1 must **NOT** be installed.

Everything else is same in both modes.

## PCB layout features

In the both PS/2 and USB layouts socket shield is used as signal ground trace for connect mouse GND pin to adapter ground. For this reason only metal sockets must be used. Especially it concerns a PS/2 which wide available in the whole-plastic variant without an metal shield.
