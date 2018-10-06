Bldc Driver Firmware
--------------------

This is the code for my brushless motor driver.

Dependencies:
* NanoPB (https://github.com/nanopb/nanopb)
* Arduino-Makefile (https://github.com/sudar/Arduino-Makefile)
* PacketSerial (https://github.com/bakercp/PacketSerial)

To compile:
* Get Arduino Makefile.
* Get NanoPB. I assume its binary distribution was simply extracted to some folder. Point the environment variable NANOPB to the nanopb install dir such that $NANOPB/generator-bin/protoc is the compiler that comes with nanopb.
* Clone PacketSerial as submodule ($git submodule update PacketSerial (?))
* $make  or $make upload
