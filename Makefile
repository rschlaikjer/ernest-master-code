BOARD_TAG = ethernet
ARDUINO_LIBS =

CFLAGS:=
CXXFLAGS:=
unexport CFLAGS
unexport CXXFLAGS

MONITOR_PORT := /dev/ttyUSB0

include /usr/share/arduino/Arduino.mk
