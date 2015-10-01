ARDUINO_DIR	= /Applications/Arduino.app/Contents/Java
ARDMK_DIR	= ../Arduino-Makefile
AVR_TOOLS_DIR	= /opt/local
MONITOR_PORT	= /dev/tty.usbmodem1d11
VENDOR		= arduino
BOARD_TAG	= uno
MCU		= atmega328
F_CPU		= 20000000
ARCHITECTURE	= avr
ISP_PORT	= /dev/tty.usbmodem1d11
ISP_PROG	= avrisp
AVRDUDE_OPTS	= -v -B 5
RESET_CMD	= /usr/bin/true


DEBUGFLAGS = 
CFLAGS += $(DEBUGFLAGS)
CXXFLAGS += $(DEBUGFLAGS) -fno-threadsafe-statics

# include my base makefile
include ../Arduino-Makefile/Arduino.mk

credentials.h:	credentials.h_example
	cp credentials.h_example credentials.h
