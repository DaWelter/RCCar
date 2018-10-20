#!/bin/bash
# For testing, send data with $socat - udp-sendto:localhost:5600, for instance.
socat UDP-LISTEN:5600,reuseaddr,fork - | tx -p 2 -b $BLOCK_SIZE_TEL -r $FECS_TEL -f $PACKET_LENGTH_TEL -x $TRANSMISSION_COUNT_CTRL $STATION_WIFI
