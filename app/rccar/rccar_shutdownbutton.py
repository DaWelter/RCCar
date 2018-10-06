#!/usr/bin/python

import pigpio
import time
import sys
import os
import subprocess
import signal
pi = pigpio.pi()

pinShutdown = 05 # GPIO numbering


def shutdown():
  print ('Shutdown Button Pressed!')
  subprocess.check_call('sudo shutdown now'.split())


def cleanup(signum, frame):
  sys.exit(0)


class Debounce(object):
  ''' When called, call the callback the first time. Afterwards don't relay calls for 'ignore_time' seconds.'''
  def __init__(self, cb, ignore_time):
    self.t = None
    self.cb = cb
    self.ignore_time = ignore_time
  
  def __call__(self, *args):
    t = time.time()
    if self.t is None or t > self.t + self.ignore_time:
      self.cb(*args)
      self.t = t


def cb(pin, level, tick):
  if pin == pinShutdown:
    shutdown()

def main():
  pi.set_mode(pinShutdown, pigpio.INPUT)
  pi.set_pull_up_down(pinShutdown, pigpio.PUD_UP)

  signal.signal(signal.SIGINT, cleanup)
  signal.signal(signal.SIGTERM, cleanup)
  
  time.sleep(0.1)
  pi.callback(pinShutdown, pigpio.FALLING_EDGE, Debounce(cb, 0.5))

  # main thread goes to sleep forever
  while 1:
    time.sleep(60)
      

if __name__ == '__main__':
  main()
