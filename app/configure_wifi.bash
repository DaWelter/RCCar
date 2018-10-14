#!/bin/bash
set -e

if [ -z "${BASH}" ]
then
    echo "Please run this script with the BASH shell\n" >&2
    exit 1
fi

SLEEPTIME=0.5
SLEEPTIMEB=0.6

function monitor_mode_on {
#  echo "$1 power on ..."
#  iwconfig $1 txpower on
#  sleep $SLEEPTIME
  echo "$1 down ..."
  ifconfig $1 down
  sleep $SLEEPTIME
  echo "monitor mode ..."
  iw dev $1 set monitor otherbss fcsfail
  sleep $SLEEPTIME
  rfkill unblock all
  sleep $SLEEPTIME
  echo "$1 up ..."
  ifconfig $1 up
  sleep $SLEEPTIME
  echo "channel $2 ..."
  iwconfig $1 channel 1  # directly changing to the target channel does not work!
  iwconfig $1 channel $2
  sleep $SLEEPTIMEB
}


function monitor_mode_off {
  echo "$1 down ..."
  ifconfig $1 down
  sleep $SLEEPTIME
  echo "managed mode ..."
  iw dev $1 set type managed
#  sleep $SLEEPTIME
#  echo "$1 up ..."
#  ifconfig $1 up
#  sleep $SLEEPTIME
#  echo "$1 power off ..."
#  iwconfig $1 txpower off
#  sleep $SLEEPTIMEB
}


function do_shutoff {
  echo "$1 down ..."
  ifconfig $1 down
  echo "$1 power off ..."
  iwconfig $1 txpower off
}


if [ $0 == $BASH_SOURCE ] ; then
  WIFI=$1
  MODE=$2
  CHN=$3
  case $MODE in
    managed)
      monitor_mode_off $WIFI
    ;;
    monitor)
      monitor_mode_on $WIFI $CHN
    ;;
    shutoff)
      do_shutoff $WIFI
    ;;
    *)
      do_shutoff
      echo "Error: $MODE"
      exit 1
    ;;
  esac
fi
