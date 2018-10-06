from __future__ import print_function
from cobs import cobs
try:
  from crc8cy import crc8
except ImportError:
  print(__file__,"falls back to import pure python crc8")
  from crc8 import crc8
import google.protobuf.message
import sys

""" Encoding/Decoding of protobuf messages using the COBS byte stuffing method.
"""

__all__ = ['encode', 'ProtobufDecoder']

class CobsFrameDecoder(object):
  def __init__(self, on_decoded, max_buffer_size = 4096):
    self.buf = b''
    self.on_decoded = on_decoded
    self.MAX_BUF_SIZE = max_buffer_size

  def putc_and_maybe_decode(self, data):
    assert type(data) == bytes
    for c in data:
      if sys.version_info[0]>=3:
        c = bytes([c])
      if c == b'\0':  # This is not c!
        try:
          decoded = cobs.decode(self.buf)
        except cobs.DecodeError:
          pass
        else:
          if len(decoded) > 1:
            cs = decoded[-1:]
            decoded = decoded[:-1]
            computed_cs = crc8(decoded).digest()
            #print('Decoded', list(map(ord, decoded)), 'CS computed', ord(computed_cs), 'CS', ord(cs))
            if computed_cs == cs:
              self.on_decoded(decoded)
        self.buf = b''
      else:
        if len(self.buf)>self.MAX_BUF_SIZE:
          self.buf = b''
        self.buf += c



class ProtobufDecoder():
  """
    Provides the `putc_and_maybe_decode` function. Give it a char, and if a protobuf message
    could be decoded, it will call your callback `on_decoded` with the message.
  """
  def __init__(self, message_type, on_decoded, max_buffer_size = 4096):
    self.cobs_frame_dec = CobsFrameDecoder(self._on_frame_decoded, max_buffer_size)
    self.on_decoded = on_decoded
    self.putc_and_maybe_decode = self.cobs_frame_dec.putc_and_maybe_decode
    self.message_type = message_type

  def _on_frame_decoded(self, frame):
    try:
      m = self.message_type()
      m.ParseFromString(frame)
    except google.protobuf.message.DecodeError:
      pass
    else:
      self.on_decoded(m)



def encode(msg):
  """
    Return cobs encoded serialization of protobuf message `msg`.
  """
  buf = msg.SerializeToString()
  # print ('Serialized = ', map(lambda x: hex(ord(x)), buf), len(buf))
  cs = crc8(buf).digest()
  buf = buf + cs
  # print ('With CS = ', cs, buf)
  frame = cobs.encode(buf)
  # The leading 0 is needed to clear potential garbage on the receiver side that
  # might have been received before this message. Only then will our frame be
  # decoded correctly.
  frame = b'\0' + frame + b'\0'
  # print ('Cobs Encoded = ', map(lambda x: hex(ord(x)), frame))
  return frame


