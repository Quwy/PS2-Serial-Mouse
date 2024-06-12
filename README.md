# PS2-Serial-Mouse
Active adapter for connect PS/2 mouse to RS232C serial port.

![Adapters photo](https://github.com/Quwy/PS2-Serial-Mouse/blob/main/InfoPage/devs.jpg?raw=true)

With this adapter you don't need to search and buy rare and expensive old serial mouse for your retro computer. Any mouse with PS/2 interface and most of simple USB-mouses will operate as Microsoft or Logitech serial mouse.

* Only 11 parts (excluding PCB and sockets).
* Small device size.
* Simple one-layer one-side PCB.
* Common and cheap microcontroller.
* Selectable 3-button mouse protocol (Microsoft or Logitech).

Adapter is based on ATtiny25 microcontroller, but PCB and firmware source are fully compatible with whole 25/45/85 series without any changes.

There is two variants of PCB layout: with PS/2 and USB socket.
![PCBs](https://github.com/Quwy/PS2-Serial-Mouse/blob/main/InfoPage/PCBs.jpg?raw=true)
They are fully identical except mouse socket type.

Note what USB version does not provide true USB host. The USB socket used only as inbound PS/2-to-USB passive adapter. Only universal USB+PS/2 mouses will work!

Jumper is used to switch 3-button mouse protocol extensions between Microsoft and Logitech style. The 1-2 position (see schematic diagramm) mean Microsoft, 2-3 correspondely Logitech. Jumper is hotplug-ready.
