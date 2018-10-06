# -*- coding: utf-8 -*-
"""
Created on Sun Apr 10 13:42:47 2016

@author: mwelter
"""
from PyQt4 import QtCore, QtGui


class MyVideoWidget(QtGui.QWidget):
  def __init__(self, parent = None):
    QtGui.QWidget.__init__(self, parent)
    self.frame_img = None

  def new_frame(self, frame_img):
    self.frame_img = frame_img
    self.update()
  
  def paintEvent(self, event):
    p = QtGui.QPainter(self)
    if self.frame_img:
      region = event.region()
      p.setClipRegion(region)  # first set clipping area as given by redraw event      
      frame, img = self.frame_img
      # get relevant rectangles
      self_rect = self.rect()
      img_rect = img.rect()  # [0, width]x[0, height]
      refresh_rect = event.rect()
      
      centering_offset = (self_rect.size() - img_rect.size())/2  # offset from the widget boundary to the bottom left of the image to center it in the widget
      centering_offset = QtCore.QPoint(centering_offset.width(), centering_offset.height()) # convert to point
      img_rect.translate(centering_offset)  # representation of the image region in widget coordinate space
      img_rect_in_widget = img_rect.intersected(refresh_rect)  # intersect with area that should be redrawn
      img_rect.translate(-centering_offset) # translate back to image space

      p.drawImage(centering_offset, img, img_rect) # draw section of the image with the given offset to make it centered
      
      region = region.subtracted(QtGui.QRegion(img_rect_in_widget))  # remove drawn image region from the region to be refreshed.
      brush = QtGui.QBrush(QtCore.Qt.blue)
      for rect in region.rects():  # draw the area surrounding the image in a different color. And only draw on the refreshed area.
        p.fillRect(rect,brush)
    else:
      p.setPen(QtCore.Qt.red)
      p.fillRect(self.rect(),QtGui.QBrush(QtCore.Qt.blue))
      p.drawText(self.rect(), QtCore.Qt.AlignCenter, "TEST")