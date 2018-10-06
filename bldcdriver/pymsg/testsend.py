#!/usr/bin/env python2
import argparse
import serial
import sys
import motor_pb2
import google.protobuf.message
import time
import cobs_pb_coding

SET_SPEED = 10

the_msg = motor_pb2.MotorControlMsg()

parser = argparse.ArgumentParser(description = 'listen for and send data on a serial port')
parser.add_argument('port')
parser.add_argument('bauds')
parseResult = parser.parse_args(sys.argv[1:])

ser = serial.Serial(parseResult.port, baudrate=parseResult.bauds)


def on_new_frame(msg):
  kind = 'STATUS' if msg.HasField('status') else ('SCOPE' if msg.HasField('scope_phase_current') else 'UNKOWN')
  print kind + ' MSG RECV = ' + str(msg)

motor_msg_decoder = cobs_pb_coding.ProtobufDecoder(motor_pb2.MotorReportContainerMsg, on_new_frame)


t = time.time()
while 1:
  buf = b''
  while ser.inWaiting():
    c = ser.read()
    buf += c
  #if buf:
    #print ("All received=", list(map(ord, buf)))
  if buf:
    for c in buf:
      # Pure python ... 5ms for the scope message to be decoded
      motor_msg_decoder.putc_and_maybe_decode(c)

  if time.time() - t > 0.1:
    the_msg.speed = SET_SPEED
    frame = cobs_pb_coding.encode(the_msg)
    #ser.write('garbage')
    ser.write(frame)
    #ser.write('moregarbage')
    t = time.time()
