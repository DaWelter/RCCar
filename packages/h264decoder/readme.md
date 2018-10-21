H264 Decoder Python Module
==========================

The aim of this project is to provide a simple decoder for video
captured by the Raspberry Pi camera. At the time of this writing I only
need H264 decoding, since a H264 stream is what the RPi software 
delivers. Furthermore, flexibility to incorporate the decoder in larger
python programs in various ways is desirable.
The code might also serve as example for libav and boost python usage.

Files
-----
* `h264decoder.hpp`, `h264decoder.cpp` and `h264decoder_python.cpp` contain the module code.
* Other source files are tests and demos.
The examples folder contains a script called `displayh264.py` which acts as a very simple player. It can read raw h264 streams from a file if given the filename, or if given -, it will read from stdin. It needs matplotlib as additional dependency.


Requirements
------------
* libav
* boost python


Todo
----
* Fix broken sample code, that is except displayh264 which should work.


License
-------
The code is published under the Mozilla Public License v. 2.0. 
