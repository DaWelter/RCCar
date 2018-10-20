#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 16 21:41:08 2016

@author: mwelter
"""
import posix_ipc
import ctypes
import sys
import os

#from wbcutil import *  # for settings, and possibly other things


MAX_PENUMBRA_INTERFACES = 8
#typedef struct {
  #uint32_t received_packet_cnt;
  #uint32_t wrong_crc_cnt;
  #int8_t current_signal_dbm;
#} wifi_adapter_rx_status_t;
class wifi_adapter_rx_status_t(ctypes.Structure):
  _pack_ = 1
  _fields_ = [('received_packet_cnt', ctypes.c_uint32),
              ('wrong_crc_cnt', ctypes.c_uint32),
              ('sequence_number', ctypes.c_uint32),
              ('current_signal_dbm', ctypes.c_byte)]


#typedef struct {
  #time_t last_update;
  #uint32_t received_block_cnt;
  #uint32_t damaged_block_cnt;
  #uint32_t tx_restart_cnt;

  #uint32_t wifi_adapter_cnt;
  #wifi_adapter_rx_status_t adapter[MAX_PENUMBRA_INTERFACES];
#} wifibroadcast_rx_status_t;
class wifibroadcast_rx_status_t(ctypes.Structure):
  _pack_ = 1
  _fields_= [('last_update', ctypes.c_uint64),
               ('received_block_cnt', ctypes.c_uint32),
               ('damaged_block_cnt', ctypes.c_uint32),
               ('tx_restart_cnt', ctypes.c_uint32),
               ('wifi_adapter_cnt', ctypes.c_uint32),
               ('wifi_adapter_rx_status', wifi_adapter_rx_status_t * MAX_PENUMBRA_INTERFACES)]

rxshm = None
def readshm(name):
  global rxshm
  if rxshm is None:
    try:
      rxshm = posix_ipc.SharedMemory(name, flags = 0, read_only = True)    
    except:
      #print 'Warning: could not open wifibroadcast shared memory block'
      return None
  os.lseek(rxshm.fd, 0, os.SEEK_SET)
  mem = os.read(rxshm.fd, ctypes.sizeof(wifibroadcast_rx_status_t))
  #for i in mem:
  #  print ord(i),
  #print '\n',
  #print len(mem), ctypes.sizeof(wifibroadcast_rx_status_t)
  return wifibroadcast_rx_status_t.from_buffer_copy(mem)

def print_wifibroadcast_rx_status_t(stat):
  print 'last_update', stat.last_update
  print 'received_block_cnt', stat.received_block_cnt
  print 'damaged_block_cnt', stat.damaged_block_cnt
  print 'tx_restart_cnt', stat.tx_restart_cnt
  print 'wifi_adapter_cnt', stat.wifi_adapter_cnt
  for i in xrange(stat.wifi_adapter_cnt):
    print 'adapter %i:' % i
    print '  received_packet_cnt', stat.wifi_adapter_rx_status[i].received_packet_cnt
    print '  wrong_crc_cnt', stat.wifi_adapter_rx_status[i].wrong_crc_cnt
    print '  sequence_number', stat.wifi_adapter_rx_status[i].sequence_number
    print '  current_signal_dbm', stat.wifi_adapter_rx_status[i].current_signal_dbm


# TODO: This is really a 'struct' or 'blob'. Name it as such.
class Status(object):
  def __init__(self, cstruct, names):
    for name in names:
      setattr(self, name, int(getattr(cstruct, name)))
  
def _pythonize(cst):
  members = 'last_update received_block_cnt damaged_block_cnt tx_restart_cnt'.split(' ')
  adapter_members = 'received_packet_cnt wrong_crc_cnt sequence_number current_signal_dbm'.split(' ')
  status = Status(cst, members)
  status.adapters = adapters = []
  for i in xrange(cst.wifi_adapter_cnt):
    adapter = Status(cst.wifi_adapter_rx_status[i], adapter_members)
    adapters.append(adapter)
  return status

def status(port_num = 0):
  cst = readshm('/wifibroadcast_rx_status_%i' % port_num)
  if not cst:
    return None
  else:
    return _pythonize(cst)


if __name__ == '__main__':
  info = readshm('/wifibroadcast_rx_status_1')
  print_wifibroadcast_rx_status_t(info)