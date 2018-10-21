# RC Car Code and Schematics

This repository contains code and materials for my RC model car project. See [project page](https://hackaday.io/haxx).

## Structure of this repo
* app: The software for the command station and the vehicle, with the exception of ...
* bldcdriver: Firmware for my motor driver
* kicad: KiCad project files with the circuit schematics.
* packages: Common code in form of some python packages

## Dependencies

* _pygame_ - For joystick support
* _wifibroadcast_ - To send packets analog transmission style
* _pyqtgraph_ - Graphing
* _vext.pyqt4_ - Use system QT in a virtual env
* _posix_ipc_ - Communication with wifibroadcast
* [_cobs-python_](https://github.com/cmcqueen/cobs-python) - Message framing
* [_crc8_](https://github.com/niccokunzmann/crc8) or crc8cy - Error checking
* _protobuf_ - Message serialization
* _cython_ (0.28 from Feb. 2018 or later) - For crc8cy
* _boost-python_ - For h264decoder
* _libav_ - Video decoding
* [_minimu9-ahrs_](https://github.com/DavidEGrayson/minimu9-ahrs) - IMU readings for Raspberry Pi
* _pyquaternion_ - Coordinate transformation for IMU readings
* _socat_ - Communication via sockets
