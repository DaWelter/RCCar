# -*- coding: utf-8 -*-
"""
Created on Thu Apr 14 11:58:12 2016

@author: mwelter
"""
import os
import Queue
import threading



class _BaseThread(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
  
  def wait(self):
    self.join()


class Recorder(_BaseThread):
  import stat
  PERMISSIONS = stat.S_IWOTH | stat.S_IROTH | stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP | stat.S_IWGRP
  
  def __init__(self, recording_stop_cb):
    _BaseThread.__init__(self)
    '''Set up everything for recoding the video to a file.
       Will not use an existing file. Instead, will append a numeric postfix
       counting up to 100, until an unused filename is found. If not 
       an Exception is raised.'''
    self.filename_pattern = 'recording%02i.h264'
    self.queue = Queue.Queue(maxsize = 1024)
    self.gogogo = True
    self.filename = self._determine_filename()
    self.timeout = 0.2
    self.buffer = []
    self.buffer_size = 0
    self.WRITE_BUFFER_SIZE = 100 * 1024
    self._recording_stop_cb = recording_stop_cb
  
  def _determine_filename(self):
    for i in xrange(100):
      fn = self.filename_pattern % i
      if not os.path.isfile(fn):
        return fn
    raise ValueError('looked from filenames %s to %s but all of these files exist! Aborting.' % (self.filename_pattern % 0, self.filename_pattern % 99))

  def _process(self, data, force_write = False):
    #print 'processing %i bytes, queue size = %i' % (len(data), self.queue.qsize())
    self.buffer.append(data)
    self.buffer_size += len(data)
    if (self.buffer_size > self.WRITE_BUFFER_SIZE) or force_write:
      #print 'queue size %i buffer size %i' %  (self.queue.qsize(), self.buffer_size)
      buffer = ''.join(self.buffer)
      with open(self.filename, 'ab', buffering = 0) as f:
        f.write(buffer)
      os.chmod(self.filename, self.PERMISSIONS)
      self.buffer = []
      self.buffer_size = 0

  def _main_loop(self):
    while 1:
      if not self.gogogo: break
      try:
        data = self.queue.get(timeout = self.timeout)
        self.queue.task_done()
      except Queue.Empty:
        continue
      if not self.gogogo: break
      self._process(data)
    self._process('', force_write = True)

  def put_video_packet(self, data):
    self.queue.put_nowait(data)
    #queue_put(self.queue, data)

  def run(self):
    print 'RECORDER RUNNING (%s)' % self.filename
    try:
      self._main_loop()
    finally:
      self._recording_stop_cb()
    print 'RECORDING STOP'




if __name__ == '__main__':
  import time
  import subprocess
  
  def stop_cb():
    print 'stopped'
  
  s = '../raspberrypi/simulateRaspivid -b 1000000 -f 1008 -i testingclip.h264'.split(' ')   
  raspivid = subprocess.Popen(s, stdout=subprocess.PIPE, bufsize = 0)
  f = raspivid.stdout
  time.sleep(0.1)

  recorder = Recorder(stop_cb)
  recorder.start()

  t = time.time()
  while (time.time() -  t < 10.):
    data = f.read(1024)
    recorder.put_video_packet(data)

  raspivid.kill()

  print 'waiting for queue'
  recorder.queue.join()
  print 'stopping thread ...'
  recorder.gogogo = False
  recorder.wait()
  print 'halt'
