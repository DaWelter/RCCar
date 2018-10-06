#!/bin/bash
rx -p 0 -b $BLOCK_SIZE -r $FECS -f $PACKET_LENGTH $STATION_WIFI | socat - udp-send:localhost:5700
