#!/bin/bash
source /etc/rccar.conf

# For testing, send data with $socat - udp-sendto:localhost:5600, for instance.
socat UDP-LISTEN:5600,reuseaddr,fork - | tx -p 1 -b $BLOCK_SIZE_TEL -r $FECS_TEL -f $PACKET_LENGTH_TEL $CAR_WIFI
