#!/usr/bin/env python2

import os
import sys
import numpy as np

import h264decoder

import matplotlib.pyplot as pyplot


img = None
fig, ax = pyplot.subplots(1,1)

def display(framedata):
  global img, fig, ax
  (frame, w, h, ls) = framedata
  if frame is not None:
    print 'frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls)
    frame = np.fromstring(frame, dtype = np.ubyte, count = len(frame), sep = '')
    frame = frame.reshape((h, ls/3, 3))
    frame = frame[:,:w,:]
    
    if not img:
      img = ax.imshow(frame)
      pyplot.show(block = False)
    else:
      img.set_data(frame)
      pyplot.draw()
    pyplot.pause(0.001)

def run_decode_frame(decoder, data_in):
  while len(data_in):
    framedata, nread = decoder.decode_frame(data_in)
    data_in = data_in[nread:]
    display(framedata)  

def run_decode(decoder, data_in):
  framedatas = decoder.decode(data_in)
  for framedata in framedatas:
    display(framedata)


input_file = sys.stdin if  sys.argv[1]=='-' else open(sys.argv[1], 'rb')
decoder = h264decoder.H264Decoder()
while 1:
  data_in = input_file.read(1024)
  if not data_in:
    break
  run_decode(decoder, data_in)
