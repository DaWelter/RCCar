# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 22:59:47 2016

@author: mwelter
"""
import time
import copy
import sys
import Queue
import select
import string
import subprocess
import shlex
import os
import re

def sgn(x):
  return 1 if x>=0 else -1

def clamp(x, a, b):
  return min(max(x, a), b)

# might be tempted to use time.clock but this would be wrong
# since it only measures the time spent in python code, not
# the actual runtime of the program (?!)
class ElapsedSeconds(object):
  def __init__(self):
    self.t = time.time()
  def __call__(self, reset = False):
    t1 = time.time()
    t0 = self.t
    if reset:
      self.t = t1
    return t1 - t0
  def reset(self):
    self.t = time.time()
    

class Struct(dict):
    __getattr__ = dict.__getitem__
    def __setattr__(self,k,v): dict.__setitem__(self,k,v)
    def __deepcopy__(self, memo):
      return Struct(copy.deepcopy(dict(self)))
    def updated(self, other):
      dict.update(self, other)
      return self
    def copy(self):
      return Struct(dict.copy(self))
    def __getstate__(self):
      return dict(self)
    def __setstate__(self, d):
      dict.update(self, d)


__ignore_exceptions = True

if __ignore_exceptions:
  def SafeOutput(dev, s):
      try:
        dev.write(s)
      except:
        pass

  def SafeCall(fun):
    try:
      fun()
    except Exception, e:
      SafeOutput(sys.stderr, str(e)+'\n')

else:
  def SafeOutput(dev, s):
    dev.write(s)

  def SafeCall(fun):
    fun()


def queue_put_nowait(queue, item):
  try:
    queue.put_nowait(item)
  except Queue.Full:
    # might be full. Then try to remove the first element.
    try:
      queue.get_nowait()
    except Queue.Empty: # however another thread might have just removed an item when this thread also tried to remove an item, emptying the queue
      pass
    try:
      queue.put_nowait(item) # since this thread is the only one writing, the queue should now have room left
    except Queue.Full:  # but this thread might actually not the only one writing ....
      return 
      


def file_read(f, size, timeout):
  fds, _, _ = select.select([f.fileno()], [], [], timeout)
  if not fds:
    return None
  try:
    return os.read(f.fileno(), size)
  except IOError:
    return None


def enable_non_blocking_mode(f):
  import fcntl
  import os
  fd = f.fileno()
  fl = fcntl.fcntl(fd, fcntl.F_GETFL)
  fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)  


def substitute_command_sequence(strOrSequence, settings):
  if isinstance(strOrSequence, str):
    lexer = shlex.shlex(strOrSequence)
    lexer.whitespace_split = True
    lexer.commenters = ''
    lexer.punctuation_chars = ''
    strOrSequence = [ str(token).strip('"').strip("'") for token in lexer ]
  return map(lambda s: string.Template(s).substitute(settings),  strOrSequence)


def get_wifi_mode(dev):
  try:
    lines = subprocess.check_output(('iw dev %s info' % dev).split())
    lines = lines.split('\n')
    for l in lines:
      l = l.strip()
      if l.startswith('type'):
        l = l.split(' ')
        return l[1]
  except:
    pass
  return 'error'


def load_settings():
  '''
    Read config file which is like a .ini file from windows, except that there shall be no [section] directives.
    Assignments like
    KEY = "value"
    are supported, as well as # comments.
    Values may or may not be put in quotation marks. However,
    KEY = foo bar
    with the newline in between is an error.
  '''
  out = {}
  def process_line(tokens):
    '''Process tokens to find the key, value pair of the setting. Put k,v into out dict.'''
    if not tokens:
      return
    key, assign, val = tokens
    if assign != '=':
      raise ValueError("Bad token '%s', expected assignment." % assign)
    # Strip quotation marks. shlex does not do it automatically.
    if (val.startswith("'") and val.endswith("'")) or\
       (val.startswith('"') and val.endswith('"')):
      val = val[1:-1]
    out[key] = val
  # Read line by line.
  with open('/etc/rccar.conf','r') as f:
    for line in f.readlines():
      try:
        process_line(list(shlex.shlex(line)))
      except:
        print "Warning: failed to read line '%s' of rccar.conf" % line.strip()
  return out