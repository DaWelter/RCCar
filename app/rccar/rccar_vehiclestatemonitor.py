#!/usr/bin/python
import pigpio
import time
import sys
import subprocess
import signal
import threading

from rccarcommon.misc import get_wifi_mode
from rccarcommon.misc import load_settings


def hasBooted():
    try:
      output = subprocess.check_output('runlevel')
      a, b = output.strip('\n \t').split(' ')
      return int(b) in (3,5)
    except:
      pass
    return False

def isRCCarTargetActive():
    try:
      output = subprocess.check_output("systemctl list-units --no-legend rccar.target".split())
      return output.startswith('rccar.target loaded active active')
    except:
      return False

settings = load_settings()
pi = pigpio.pi()
pinWifiState = 26 # GPIO numbering 
# the other LED is on GPIO 13
pi.set_mode(pinWifiState, pigpio.OUTPUT)

stopevent = threading.Event()
mutex = threading.Lock()
blinkconfig = 0, 100

def setLED1(level):
  pi.write(pinWifiState, level)

def blinkLED1(counts_off, counts_total):
  global blinkconfig
  with mutex:
    blinkconfig = counts_off, counts_total
  # pi.set_PWM_frequency(pinWifiState, freq)
  # pi.set_PWM_dutycycle(pinWifiState, int(dutycycle*255.99))

def blink_loop():
  def get_counts():
    with mutex:
      return blinkconfig
  while 1:
    counter = 0
    setLED1(0)
    while counter < get_counts()[0]:
      time.sleep(0.01)
      counter += 1
    setLED1(1)
    while counter < get_counts()[1]:
      time.sleep(0.01)
      counter += 1

def update_state():
  # Checking the wifi state is redundant with the systemd target check. However, I want to make sure everything is okay.
  mode = get_wifi_mode(settings['CAR_WIFI'])
  booted = hasBooted()
  rccartarget = isRCCarTargetActive()
  return (mode, booted, rccartarget)


def set_blink_config(systemstate):
  mode, booted, rccartarget = systemstate
  if not booted:
    blinkLED1(100, 100)
  elif mode == 'monitor' and rccartarget:
    blinkLED1(50, 100)
  elif mode == 'managed' and not rccartarget:
    blinkLED1(0, 100)
  else:
    blinkLED1(8, 15)


def update_state_loop():
  while not stopevent.isSet():
    time.sleep(0.25)
    set_blink_config(update_state())


update_thread = threading.Thread(target = update_state_loop)
update_thread.start()

def cleanup(signum, frame):
  stopevent.set()
  update_thread.join()
  setLED1(0)
  sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

blink_loop()