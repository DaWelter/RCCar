#!/bin/bash
source /etc/rccar.conf

raspivid --nopreview --awb auto -ih -t 0 -w $WIDTH -h $HEIGHT -fps $FPS -b $BITRATE -n -g $KEYFRAMEINTERVAL -pf high -o - | tx -b $BLOCK_SIZE -r $FECS -f $PACKET_LENGTH -x $TRANSMISSION_COUNT $CAR_WIFI
