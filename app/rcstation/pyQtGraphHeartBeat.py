import time
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np

#class RunningBuffer(object):
#  NUM_POINTS = 1000
#  def __init__(self):
#    self.data  = np.zeros(RunningBuffer.NUM_POINTS, np.float32)
#    self.ptr   = 0
#  
#  def update(self, value):
#    data, ptr = self.data, self.ptr
#    data[ptr] = value
#    self.ptr = (ptr+1) % len(data)
#
#  def __len__(self):
#    return len(self.data)

class WipeGraphTimer(object):
  def __init__(self, tRange):
    self.tBegin = time.time()
    self.tRange = tRange

  def elapsed(self, t):
    return t - self.tBegin
  
  def update(self, t):
    if t > self.tBegin + self.tRange:
      self.tBegin += self.tRange
      return True
    return False

class WipeGraphBuffer(object):
  NUM_POINTS = 5000
  def __init__(self, timer, numChannels):
    shape = (WipeGraphBuffer.NUM_POINTS,numChannels+1)
    self.data  = np.zeros(shape, np.float32)
    self.prev  = np.zeros_like(self.data)
    self.ptr   = 0
    self.prevLen = 0
    self.needRecomputeIndices = True
    self.timer = timer
    
  def insert(self, t, values):
    dt = self.timer.elapsed(t)
    if self.ptr>0 and dt < self.data[self.ptr-1,0]:  # wrap around detected (tBegin must have increased)
      self.swap_()
    data, ptr = self.data, self.ptr
    if ptr < len(data):
      data[ptr,:] = (dt,)+tuple(values)
      self.ptr = ptr + 1
    self.needRecomputeIndices = True

  def getLeftRightParts(self):
    '''left part is the most recent data, then comes the wiper line then comes older data on the right hand side'''
    if self.needRecomputeIndices:
      iTnow = self.ptr
      iTend = iTnow+1
      while iTend < len(self.data) and self.prev[iTend,0] < self.data[iTnow,0]:
        iTend += 1
      self.needRecomputeIndices = False
      self.iTnow = iTnow
      self.iTend = iTend
    return self.data[:self.iTnow,...], self.prev[self.iTend:self.prevLen,...]

  def swap_(self):
    self.data, self.prev = self.prev, self.data
    self.prevLen = self.ptr
    self.ptr = 0


class WipePlot(object):
  def __init__(self, graph, timer, plotargs, plotkwargs):
    self.p1 = graph.plot(*plotargs, **plotkwargs)
    plotkwargs.pop('name', None)
    self.p2 = graph.plot(*plotargs, **plotkwargs)
    self.buffer  = WipeGraphBuffer(timer, 1)

  def updatePlot(self):
    l, r = self.buffer.getLeftRightParts()
    self.p1.setData(l[:,0], l[:,1])
    self.p2.setData(r[:,0], r[:,1])

  def insert(self, t, data):
    self.buffer.insert(t, (data,))



class WipeGraph(object):
  def __init__(self, tRange, *args, **kwargs):
    self.graph = graph = pg.PlotItem(*args, **kwargs)
    if isinstance(tRange, WipeGraphTimer):
      self.timer = tRange
    else:
      self.timer = WipeGraphTimer(tRange)
    self.vline = vline = pg.InfiniteLine(None, 90, movable = False, pen = 0.5)
    graph.addItem(vline)
    graph.hideAxis('bottom')
    vline = pg.InfiniteLine(0., 0, movable = False, pen = 0.5)
    graph.addItem(vline)
    graph.setRange(xRange = (0., self.timer.tRange))
    graph.enableAutoRange('xy', False)
    graph.setMouseEnabled(x=False, y=False)
    self.wipePlots = []
    for name in ['addLegend', 'setYRange']:
      setattr(self, name, getattr(self.graph, name))

  def add_hline(self, y, pen):
    hline = pg.InfiniteLine((0, y), 0, movable = False, pen = pen)
    self.graph.addItem(hline)

  def addPlot(self, *args, **kwargs):
    p = WipePlot(self.graph, self.timer, args, kwargs)
    self.wipePlots.append(p)
    return p

  def updatePlots(self, t):
    self.timer.update(t)
    dt = self.timer.elapsed(t)
    self.vline.setValue(dt)
    for p in self.wipePlots:
      p.updatePlot()


class MyLegend(pg.LegendItem):
  def __init__(self, *args, **kwargs):
    pg.LegendItem.__init__(self, *args, **kwargs)
    
  def paint(self, p, *args):
      p.setPen(pg.functions.mkPen(255,255,255,100))
      #p.setBrush(fn.mkBrush(100,100,100,50))
      #p.drawRect(self.boundingRect())

  @staticmethod
  def addTo(plotItem, itemPos, parentPos, offset, size=None):
    plotItem.legend = l = MyLegend(size, None)
    l.setParentItem(plotItem.vb)
    l.anchor(itemPos, parentPos, offset)
    return l
