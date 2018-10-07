To make the code work on the RCCar one must install the rccarcommon package.
Execute setup.py via pip: 
$ sudo pip install .
For development one can have symlinks created:
$ sudo pip install -e .

The same holds true for my protocobs package.

Dependencies:
* pygame
* wifibroadcast
* pyqtgraph
* vext.pyqt4 (if using virtualenv)
* posix_ipc
* cobs-python (https://github.com/cmcqueen/cobs-python)
* crc8 (https://github.com/niccokunzmann/crc8) or crc8cy
* protobuf
* cython (0.28 from Feb. 2018 or later) for crc8cy.
* boost-python
* libav
* minimu9-ahrs (https://github.com/DavidEGrayson/minimu9-ahrs)
* pyquaternion

Non-python dependencies:
* socat

Configuration:
* 'denyinterfaces wlan1' added to dhcpcd.conf to prevent dhcp client from essing with the wifi
* 'deny-interfaces=wlan1 added to avahi-daemon.conf
* Shutting off the wifi initially on boot seems to be important. I got errors when switching to monitor mode, otherwise.

Todo:
* Consider QFlightInstruments (https://sourceforge.net/projects/qfi/)
