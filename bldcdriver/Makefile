BOARD_TAG    = teensy31
ARDUINO_LIBS =
BOARD_SUB    = 96

PROJECT_DIR  = .

CPPFLAGS += -I$(PROJECT_DIR)/msg
CPPFLAGS += -I$(PROJECT_DIR)/nanopb
CPPFLAGS += -I$(PROJECT_DIR)/PacketSerial/src
CXXFLAGS += -std=c++11
CXXFLAGS += -DPB_WITHOUT_64BIT
CFLAGS += -DPB_WITHOUT_64BIT

include /opt/Arduino-Makefile/Teensy.mk
