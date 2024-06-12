# PS2-Serial-Mouse
Active adapter for connect PS/2 mouse to the RS232C serial port.

![Adapters photo](https://github.com/Quwy/PS2-Serial-Mouse/blob/main/InfoPage/devs.jpg?raw=true)

With this adapter you don't need to search and buy rare and expensive old serial mouse for your retro computer. Any mouse with PS/2 interface and most of simple USB-mouses will operate as Microsoft or Logitech serial mouse.

* Only 16 parts including PCB and sockets.
* Small device size.
* Simple one-layer one-side PCB.
* No SMD components.
* Common and cheap microcontroller.
* Selectable 3-button mouse protocol (Microsoft or Logitech).
* Open-source electronics and firmware.

Adapter is based on ATtiny25 microcontroller, but PCB layout and firmware source are fully compatible with whole 25/45/85 series without any changes.

There are two variants of PCB layout: with PS/2 and USB mouse socket. Both they have one-layer one-side design without VIA covering and fully ready even for manual tracing.
![PCBs](https://github.com/Quwy/PS2-Serial-Mouse/blob/main/InfoPage/PCBs.jpg?raw=true)
They are fully identical except an mouse socket type.

Note what USB version does not provide true USB host. The USB socket is only used as built-in PS/2-to-USB passive adapter.

![USB Adapter](https://github.com/Quwy/PS2-Serial-Mouse/blob/main/InfoPage/dev_usb_ps2_adapter.jpg?raw=true)

Only universal USB+PS/2 mouse will work!

![Gaming VS Simple](https://github.com/Quwy/PS2-Serial-Mouse/blob/main/InfoPage/gaming_simple.webp?raw=true)
Usually it is a simple office mouse with two buttons and one clickable wheel.

Jumper is used to switch 3-button mouse protocol extensions between Microsoft and Logitech style. The 1-2 position (close to the mouse socket) mean Microsoft, 2-3 respectively Logitech. Jumper is hotplug-ready.

Since RS232C port does not provide power supply enougt to feed most of PS/2 mouses, the standart "floppy-style" 4-pin MOLEX plug is used as power source (+5V line only).
