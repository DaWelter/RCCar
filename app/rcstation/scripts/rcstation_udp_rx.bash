#!/bin/bash
rx -p 1 -b $BLOCK_SIZE_TEL -r $FECS_TEL -f $PACKET_LENGTH_TEL $STATION_WIFI | socat - udp-send:localhost:5500
