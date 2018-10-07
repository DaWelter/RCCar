#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import threading
import time
import Queue
import traceback
import signal
import collections
import numpy as np
import socket

import pygame
from PyQt4 import QtCore, QtGui, uic
import pyqtgraph

import rccarcommon
from rccarcommon import telemetry
from rccarcommon import misc

import wifibroadcastinfo
import h264decoder
import pyQtGraphHeartBeat
import inputfilter
import recordermodule


WbcInfo = collections.namedtuple('WbcInfo', 'signalStrengthDbm error')
def getWbcInfo():
  status = wifibroadcastinfo.status()
  if status:
    assert len(status.adapters) == 1
    return WbcInfo(
      signalStrengthDbm = float(status.adapters[0].current_signal_dbm),
      error = False)
  else:
    return WbcInfo(
      signalStrengthDbm=-float('inf'),
      error = True
    )

#updating the gui from a thread: 
# http://stackoverflow.com/questions/10694971/pyqt-update-gui
# how to use a timer to update stuff:
# http://stackoverflow.com/questions/23786340/use-qthread-to-periodically-update-a-qtablewidget-pyqt
#documentation of signals and slots in python
# documentation: http://pyqt.sourceforge.net/Docs/PyQt4/new_style_signals_slots.html

def convert_scope_phase_current_message(msg):
  ''' (Oscillo)'scope type data is received in counts of the ADC output.
      Hence it needs conversion to physical units, which is done here.
      We also like to have a numpy array instead of protobuf message.
  '''
  data = np.asarray(msg.values).astype(np.float32)
  CURRENT_SENSE_GAIN = 64
  CURRENT_VREF = 1.2
  CURRENT_SENSE_RESOLUTION = 8
  CURRENT_SENSE_RESISTOR = 0.01
  CURRENT_COUNTS_TO_AMP = CURRENT_VREF / (
    CURRENT_SENSE_GAIN * CURRENT_SENSE_RESISTOR * (1 << CURRENT_SENSE_RESOLUTION))
  data *= CURRENT_COUNTS_TO_AMP
  period = msg.period * 1.e-3
  return (period, data)


MSG_VIDEOFRAME_QUEUED = 2
MSG_TELEMETRY_QUEUED = 3


class BaseThread(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
  
  def wait(self):
    self.join()



class VideoThread(BaseThread):
  def __init__(self, receiver):
    BaseThread.__init__(self)
    self.gogogo = True
    self.receiver = receiver
    self.daemon = True
    self.h264decoder = h264decoder.H264Decoder()

  def run(self):
    while self.gogogo:
      data_in = self.receiver.receive_video_raw()
      if data_in is not None:
        if self.receiver.recorder:
          self.receiver.recorder.put_video_packet(data_in)
        frames = self.h264decoder.decode(data_in)
        if frames:
          (frame, w, h, ls) = frames[-1]
          img = QtGui.QImage(frame, w, h, ls, QtGui.QImage.Format_RGB888)        
          misc.queue_put_nowait(self.receiver.videoqueue, (frame, img))
          self.receiver.post_message(MSG_VIDEOFRAME_QUEUED)
    print('video thread stop')



DATA_REMOTE_MEASURED = 1
DATA_REMOTE_PHASE_CURRENT_SCOPE = 2
DATA_WBC_INFO = 3


class Telemetry1Thread(BaseThread):  
  def __init__(self, receiver):
    BaseThread.__init__(self)
    self.gogogo = True
    self.receiver = receiver
    self.daemon = True
    self.decoder = telemetry.CarReportDecoder(self.onDecode)

  def onDecode(self, msg):
    if msg.HasField('status'):
      self.enqueue(DATA_REMOTE_MEASURED, msg.status)
    elif msg.HasField('scope_phase_current'):
      self.enqueue(DATA_REMOTE_PHASE_CURRENT_SCOPE,
                   convert_scope_phase_current_message(msg.scope_phase_current))

  def enqueue(self, kind, stats):
    misc.queue_put_nowait(self.receiver.telemetryqueue, (kind, stats))
    self.receiver.post_message(MSG_TELEMETRY_QUEUED)

  def run(self):
    while self.gogogo:
      data_in = self.receiver.receive_telemetry_raw()
      if data_in is not None:
        #print('TEL_REC:', str(data_in))
        self.decoder.putc_and_maybe_decode(data_in)
    print('telemetry 1 thread stop')


class Telemetry2Thread(BaseThread):
  def __init__(self, receiver):
    BaseThread.__init__(self)
    self.gogogo = True
    self.receiver = receiver
    self.daemon = True

  def enqueue(self, stats):
    misc.queue_put_nowait(self.receiver.telemetryqueue, (DATA_WBC_INFO, stats))
    self.receiver.post_message(MSG_TELEMETRY_QUEUED)
  
  def run(self):
    while self.gogogo:
      stats = getWbcInfo()
      self.enqueue(stats)
      time.sleep(0.2)
    print('telemetry 2 thread stop')




class PyGameThread(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    self.gogogo = True
    self.daemon = True
  
  def run(self):
    # NOTE: if no events from joystick received: check if joystick is initialized (Joystick init()  called)
    # Apparently we need to do this in order to get state updates for the joystick
    while self.gogogo:
      pygame.event.wait()

  def stop(self):
    pygame.event.post(pygame.event.Event(pygame.QUIT))
    self.gogogo = False
    self.join()    




class ReceiverState(QtCore.QObject):
  trigger_post_message = QtCore.pyqtSignal(int)

  # attack of the blob class! :->
  def __init__(self):
    self.MAX_READ_SIZE = 1024*1024*10 # 10 Megs. Sufficient of a video packet?
    QtCore.QObject.__init__(self)
    self.videoqueue        = Queue.Queue(maxsize = 2)
    self.telemetryqueue    = Queue.Queue(maxsize = 5)
    self.videothread       = VideoThread(self)
    self.telemetry1thread  = Telemetry1Thread(self)
    self.telemetry2thread  = Telemetry2Thread(self)
    self.recorder          = None
    self.command           = inputfilter.SteeringControl()

  def start(self):
    self.telemetrypipe, self.videopipe, self.commandpipe = self._open_sockets()
    self._start_threads()

  def close(self, *args):
    self._stop_threads()
    self.telemetrypipe.close()
    self.videopipe.close()
    self.commandpipe.close()

  def send_command_raw(self, data):
    self.commandpipe.send(data)

  def receive_video_raw(self):
    try:
      data, addr = self.videopipe.recvfrom(self.MAX_READ_SIZE)
      return data
    except socket.timeout as e:
      return None

  def receive_telemetry_raw(self):
    try:
      data, addr = self.telemetrypipe.recvfrom(self.MAX_READ_SIZE)
      return data
    except socket.timeout as e:
      return None

  def post_message(self, msg):
    self.trigger_post_message.emit(msg)

  def _start_threads(self):
    for t in [self.videothread, self.telemetry1thread, self.telemetry2thread]:
      t.start()

  def _stop_threads(self):
    for t in [self.videothread, self.telemetry1thread, self.telemetry2thread]:
      t.gogogo = False
      t.wait()
    if self.recorder:
      self.recorder.gogogo = False
      self.recorder.join()

  def _open_sockets(self):
    # Messages come in here ...
    sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rx.bind(('', 5500))
    sock_rx.settimeout(0.5)
    # Video comes in here ...
    sock_rx_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rx_video.bind(('', 5700))
    sock_rx_video.settimeout(0.5)
    # Messages go out this way ...
    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_tx.connect(('127.0.0.1', 5600))
    return sock_rx, sock_rx_video, sock_tx



# NOTE: screen size of laptop is 1366 x 768
class ControlMainWindow(QtGui.QMainWindow):
  SUPER = QtGui.QMainWindow
  def __init__(self, parent = None):
    self.SUPER.__init__(self, parent)
    self.ui = uic.loadUi(os.path.join(os.path.dirname(__file__), 'wbcrx.ui'), self)

    # configure bits of the UI
    self.ui.comm_indicator.auto_off_delay_millis = 100

    # setup graph views
    self.list_of_graphs = []
    tRange = 10.
    self.graphFwd = g = self.graphing_add_wipegraph(0, 0, tRange, 'Fwd', (-1., 1.))
    self.graphing_add_legend(g)
    self.plotGears       = g.addPlot(pen = '666', name = 'gears')
    self.plotFwdSet      = g.addPlot(pen = "a00", name = 'set')
    self.plotFwdRemote   = g.addPlot(pen = "00f", name = 'remote')

    self.graphRight = g = self.graphing_add_wipegraph(1, 0, self.graphFwd.timer, 'Right', (-1., 1.))
    self.plotRightRemote = g.addPlot(pen = "00f", name = 'remote')
    self.plotRightSet  = g.addPlot(pen = "a00", name = 'set')

    self.graphCurrent = g = self.graphing_add_wipegraph(2, 0, self.graphFwd.timer, 'I', (-2., 2.))
    self.plotCurrent  = g.addPlot(name = 'bat')
    
    self.graphVoltage = g = self.graphing_add_wipegraph(3, 0, self.graphFwd.timer, 'V', (-1., 14.))
    MIN_VOLTAGE = 1.1 * 8.
    MAX_VOLTAGE = 1.4 * 8.
    g.add_hline(MIN_VOLTAGE, pyqtgraph.functions.mkPen(255,0,0,200))
    g.add_hline(MAX_VOLTAGE, pyqtgraph.functions.mkPen(128,128,128,200))    
    self.plotVoltage  = g.addPlot(name = 'bat', pen = 'fff')

    self.graphRpm = g = self.graphing_add_wipegraph(4, 0, self.graphFwd.timer, 'RPM', (-5000, 5000))
    self.plotRpmRemote = g.addPlot(pen = '00f', name = 'remote')

    self.graphCurrentScope = g = self.graphing_add_graph(5, 0, (0., 1.), 'Amps', (-1., 1.))
    self.plotCurrentScope =  pyqtgraph.PlotDataItem(pen = 'fff')
    g.addItem(self.plotCurrentScope)

    self.update_tick_timer = t = QtCore.QTimer(self)
    t.setInterval(20)
    t.timeout.connect(self.update_tick)
    t.start()


  def graphing_add_legend(self, g):
    '''for building the gui'''
    pyQtGraphHeartBeat.MyLegend.addTo(g.graph, (1.,0.,), (1.,0.), (-1.,1.), size = (5, 5))

  def graphing_add_graph(self, row, col, tRange, title, yRange):
    graph = pyqtgraph.PlotItem(title = title)
    graph.enableAutoRange('x', False)
    graph.enableAutoRange('y', False)
    graph.setYRange(*yRange)
    graph.setXRange(*tRange)
    graph.setMouseEnabled(x=False, y=False)
    self.ui.graphing.addItem(graph, row  = row, col = col)
    return graph

  def graphing_add_wipegraph(self, row, col, tRange, title, yRange):
    '''for building the gui'''
    graph = pyQtGraphHeartBeat.WipeGraph(tRange, title=title)
    self.ui.graphing.addItem(graph.graph, row  = row, col = col)
    graph.setYRange(*yRange)
    self.list_of_graphs.append(graph)
    return graph


  @QtCore.pyqtSlot(int)
  def handle_message(self, msg):
    if msg == MSG_VIDEOFRAME_QUEUED:
      self.on_videoframe_ready()
    if msg == MSG_TELEMETRY_QUEUED:
      self.on_telemetry_queued()


  # @QtCore.pyqtSlot()
  # def on_recording_stop(self):
  #   self.ui.recording_btn.setChecked(False)
  #   self.ui.recording_btn.update()


  @QtCore.pyqtSlot()
  def on_videoframe_ready(self):
    self.blink_comms_indicator()
    try:
      frame_img = self.receiver.videoqueue.get_nowait()
    except Queue.Empty:
      return
    self.ui.videowidget.new_frame(frame_img)


  @QtCore.pyqtSlot()
  def on_telemetry_queued(self):
    try:
      kind, stats = self.receiver.telemetryqueue.get_nowait()
    except Queue.Empty:
      return
    if kind == DATA_REMOTE_MEASURED:
      self.blink_comms_indicator()
      self.update_ui_telemetry(stats)
    elif kind == DATA_REMOTE_PHASE_CURRENT_SCOPE:
      # Not that important so don't blink here.
      self.update_ui_scope(stats)
    elif kind == DATA_WBC_INFO:
      self.update_ui_wbc_info(stats)
    else:
      return


  def update_ui_wbc_info(self, stats):
    txt = 'ERR' if stats.error else \
        ('%.0fdbm' % stats.signalStrengthDbm)
    self.ui.lbl_linkquality.setText(txt)


  def blink_comms_indicator(self):
    self.ui.comm_indicator.turn_on()


  @QtCore.pyqtSlot()
  def recording_button_clicked(self, on):
    #print 'CLICK', on
    self.receiver.toggle_recording(on)


  # NOTE: data display appears extra jittery because packets of full telemetry
  # info come in clusters of several within a very short time. (wifibroadcast).
  # Using normal networking, samples arrive more evenly.
  def update_ui_telemetry(self, data):
    assert type(data) is rccarcommon.telemetry.CarStatusMsg
    self.update_ui_plots(data)
    self.update_ui_labels(data)


  def update_ui_plots(self, data):
    t = time.time() # TODO: Don't use local time. Would be better to use time stamp on the remote vehicle or something like that.
    plots    = [ self.plotVoltage, self.plotCurrent, self.plotRpmRemote ]
    MILLIAMP_TO_AMP = 1.e-3
    stateVec = [ data.voltage, MILLIAMP_TO_AMP*data.phase_current, data.speed ]
    for p, v in zip(plots, stateVec):
      p.insert(t, v)


  def update_ui_labels(self, data):
    self.ui.lbl_systemload.setText(
      u'%.0f%%' % (100*data.system_load))
    self.ui.lbl_heading.setText(
      u'%.0f°' % data.euler_h)
    self.ui.lbl_pitch.setText(
      u'%.0f°' % data.euler_p)
    self.ui.lbl_bank.setText(
      u'%.0f°' % data.euler_b)

  def update_ui_scope(self, data):
    period, values = data
    p = self.plotCurrentScope
    g = self.graphCurrentScope
    r = getattr(self, 'rangeGraphCurrentScope', 1.)
    maxval = np.amax(np.abs(values))
    maxval = max(maxval, r)
    g.setYRange(-maxval, maxval)
    g.setXRange(0., period)
    self.rangeGraphCurrentScope = maxval
    p.setData(np.linspace(0., period, len(values)), values)


  def commanded_to_graphing(self, t, commanded):
    (fwd, right, cam), input_raw, gears = commanded
    plots = [ self.plotFwdSet, self.plotRightSet, self.plotGears ]
    for p, v in zip(plots, [fwd, right, gears]):
      p.insert(t, v)


  def keyPressEvent(self, e):
#    print 'key press', e.key()
    if self.receiver.command.keyPressEvent(e):
        pass
    elif e.key() == QtCore.Qt.Key_Q and e.modifiers() == QtCore.Qt.ControlModifier:
        self.close()
    elif e.key() == QtCore.Qt.Key_F11:
      if self.isFullScreen():
        self.showNormal()
      else:
        self.showFullScreen()
    else: 
      self.SUPER.keyPressEvent(self, e)


  def keyReleaseEvent(self, e):
#    print 'key release', e.key()
    if self.receiver.command.keyReleaseEvent(e):
      pass
    elif e.key() == QtCore.Qt.Key_Q and e.modifiers() == QtCore.Qt.ControlModifier:
      pass
    elif e.key() == QtCore.Qt.Key_F11:
      pass
    else:
      self.SUPER.keyReleaseEvent(self, e)


  def update_input(self, t):
    self.receiver.command.update()
    self.commanded_to_graphing(
      t,
      (self.receiver.command.output,
       self.receiver.command.rawOutput,
       self.receiver.command.gears)
    )

  # TODO: move to thread so the command stream is not interrupted when the UI blocks.
  def send_command(self):
    fwd, right, cam = self.receiver.command.output
    msg = telemetry.CarControlContainerMsg(
      steer = telemetry.CarSteerMsg(
        speed = int(fwd * 100),
        right = right,
        cam_right = cam
      )
    )
    data = telemetry.encode(msg)
    self.receiver.send_command_raw(data)

  
  def update_tick(self):
    '''periodically update plots, poll joystick, and maybe do other things ...'''
    t = time.time()
    self.update_input(t)
    for g in self.list_of_graphs:
      g.updatePlots(t)
    self.send_command()


  def closeEvent(self, ev):
    '''proper way to cleanup when the app closes'''
    #http://pyqt.sourceforge.net/Docs/PyQt4/qwidget.html#closeEvent
    print 'closing ...'
    self.update_tick_timer.stop()
    print 'close'
    return self.SUPER.closeEvent(self, ev)



# TODO: exception handler that kills subprocesses no matter what!!
# great tip from 
# http://stackoverflow.com/questions/1235349/python-how-can-i-handle-any-unhandled-exception-in-an-alternative-way
def handle_exception(exc_type, exc_value, exc_traceback):
  """ handle all exceptions """

  ## KeyboardInterrupt is a special case.
  ## We don't raise the error dialog when it occurs.
  
  print 'CUSTOM EXCEPTION HANDLER!!'
#  if issubclass(exc_type, KeyboardInterrupt):
#    QtGui.QApplication.exit(0)
#    return

  filename, line, _, _ = traceback.extract_tb( exc_traceback ).pop()
  filename = os.path.basename( filename )
  
  #error    = "%s: %s" % ( exc_type.__name__, exc_value )
#  QtGui.QMessageBox.critical(None,"Error",
#    "<html>A critical error has occured.<br/> "
#  + "<b>%s</b><br/><br/>" % error
#  + "It occurred at <b>line %d</b> of file <b>%s</b>.<br/>" % (line, filename)
#  + "</html>")

  print "Closed due to an error. This is the full error report:"
  print
  print "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
  QtGui.QApplication.exit(1)

# install handler for exceptions
sys.excepthook = handle_exception


def sigint_handler(signum, frame):
  # the proper way to terminate a pyqt app
  QtGui.QApplication.exit(0)
  

class Application(QtGui.QApplication):
  def __init__(self, args):
    QtGui.QApplication.__init__(self, args)
    self.aboutToQuit.connect(self.onAboutToQuit)

    pygame.init()
    pygame.joystick.init()
    self.pygame_thread = PyGameThread()
    self.pygame_thread.start()
    
    # now start receiver, transmitter and all the other systems
    self.receiver = ReceiverState()
    self.receiver.start()

    self.win = ControlMainWindow()
    self.win.receiver = self.receiver

    self.receiver.trigger_post_message.connect(self.win.handle_message)

    self.win.show()


  def onAboutToQuit(self):
    print 'onAboutToQuit'
    # in case of CTRL-C the window is still visible and must be closed
    # in order for the cleanup routines to run.
    if self.win.isVisible():  
      self.win.close()
    self.receiver.close()
    self.pygame_thread.stop()


if __name__ == '__main__':
  app = Application(sys.argv)
  signal.signal(signal.SIGINT, sigint_handler)
  exec_result = app.exec_()
  sys.exit(exec_result)
