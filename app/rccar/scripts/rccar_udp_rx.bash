#!/bin/bash
source /etc/rccar.conf

rx -p 2 -b $BLOCK_SIZE_TEL -r $FECS_TEL -f $PACKET_LENGTH_TEL $CAR_WIFI | socat - udp-send:localhost:5700
