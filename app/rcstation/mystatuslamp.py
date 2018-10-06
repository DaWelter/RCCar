# -*- coding: utf-8 -*-
"""
Created on Sun Apr 10 13:42:47 2016

@author: mwelter
"""
from PyQt4 import QtCore, QtGui


class MyStatusLamp(QtGui.QWidget):
  def __init__(self, queue, parent = None):
    QtGui.QWidget.__init__(self, parent)
    self._state = False             # boolean, either on or off
    self._timeout = QtCore.QTimer(self)   # will disable the lamp
    self._timeout.timeout.connect(self.turn_off)
  
  @property
  def auto_off_delay_millis(self):
    return self._timeout.interval()

  # property documentation
  # https://docs.python.org/2/library/functions.html#property
  # set until automatic shutoff of the lamp
  @auto_off_delay_millis.setter
  def auto_off_delay_millis(self, t):
    if t is not None:
      self._timeout.setInterval(t)
      self._timeout.start()
    else:
      self._timeout.stop()

  
  @QtCore.pyqtSlot()
  def turn_on(self):
    self._timeout.start()
    if self._state != True:
      self._state = True
      self.update()
   
  @QtCore.pyqtSlot()
  def turn_off(self):
    if self._state != False:
      self._timeout.stop()
      self._state = False
      self.update()
   
  def paintEvent(self, event):
    p = QtGui.QPainter(self)
    p.setClipRegion(event.region())
    color = QtCore.Qt.yellow if self._state else QtCore.Qt.black
    p.fillRect(self.rect(),color)