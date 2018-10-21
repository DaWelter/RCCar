BLDC Driver Firmware
--------------------

This is the code for my brushless motor driver. It is based on Arduino & Teensy.

## Dependencies
* _Teensyduino_ (https://www.pjrc.com/teensy/teensyduino.html)
* _NanoPB_ (https://github.com/nanopb/nanopb)
* _Arduino-Makefile_ (https://github.com/sudar/Arduino-Makefile)
* _PacketSerial_ (https://github.com/bakercp/PacketSerial)

## Building
* Get Arduino Makefile.
* Get NanoPB. I assume its binary distribution was extracted somewhere. Point the environment variable NANOPB to the nanopb install dir such that ```NANOPB/generator-bin/protoc``` is the compiler that comes with nanopb.
* Clone PacketSerial as submodule ```git submodule update PacketSerial```
* ```make```  or ```make upload```
