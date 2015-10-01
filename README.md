Eversolar Inverter Monitor
==========================

This is a device for monitoring Eversolar/Zeversolar inverters and
feeding the acquired data into a server that will do "something" with
it.

It requires the UIPEthernet arduino library (from
https://github.com/ntruchsess/arduino_uip). In order to reduce SRAM
usage, I changed utility/uipethernet-conf.h in that library to disable
DHCP, reduce the maximum number of TCP and UDP connections, and reduce
the number of packets that can be buffered in a TCP stream. You may or
may not need to do the same.

It also required the AltSoftSerial library (from
http://www.pjrc.com/teensy/td_libs_AltSoftSerial.html) for debug
output. At some point I intend to make this optional.

The Makefile is set up to use Arduino-Makefile (from
https://github.com/sudar/Arduino-Makefile). It expects it to be in
../Arduino-Makefile so if you have it elsewhere you will need to
update the Makefile to reference wherever you have it installed.

If you use the schematic and/or board in eagle/ then the fuse bits you
will need to set on your ATmega328 are: low=0xf7 high=0xd9 ext=0x04
