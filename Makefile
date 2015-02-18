BOARD_TAG = atmega644
ARDUINO_LIBS =

CFLAGS:=
CXXFLAGS:=
unexport CFLAGS
unexport CXXFLAGS

MONITOR_PORT := /dev/ttyUSB1

include /usr/share/arduino/Arduino.mk
