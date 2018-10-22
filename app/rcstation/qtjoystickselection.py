import os
import sys
import collections
import functools
import json

import pygame

from PyQt4 import QtCore, QtGui, uic

#import joystick


class Joystick(object):
  def __init__(self, ident, pygameStick):
    self.ident    = ident
    self.name     = pygameStick.get_name()
  
  def __str__(self):
    return self.name


class UnassignedJoystick(object):
  def __init__(self):
    self.ident = (None, 0)
    self.name  = "Unassigned"

  def __str__(self):
    return self.name

unassignedJoystick = UnassignedJoystick()

JoystickEvent = collections.namedtuple("JoystickEvent", "dev axis button value".split())

class Joysticks(object):
  def __init__(self):
    self.eventlist = []
    self.numberToStick = []
    for i in range(pygame.joystick.get_count()):
      pygamestick = pygame.joystick.Joystick(i)
      pygamestick.init()
      joystick = Joystick((pygamestick.get_name(), 0), pygamestick)
      self.numberToStick.append(joystick)

  def canHandlePygameEvent(self, ev):
    return ev.type in (pygame.JOYBUTTONDOWN, 
                       pygame.JOYBUTTONUP,
                       pygame.JOYAXISMOTION)

  def poll(self):
    self.eventlist, ev = [], self.eventlist # swap
    return ev
  
  def transformEvent(self, ev):
    if ev.type == pygame.JOYBUTTONDOWN:
      return JoystickEvent(self.numberToStick[ev.joy], None, ev.button, 1)
    if ev.type == pygame.JOYBUTTONUP:
      return JoystickEvent(self.numberToStick[ev.joy], None, ev.button, 0)
    if ev.type == pygame.JOYAXISMOTION:
      return JoystickEvent(self.numberToStick[ev.joy], ev.axis, None, ev.value)
    raise Exception("Unkown event type")

  def handlePygameEvent(self, ev):
    self.eventlist.append(self.transformEvent(ev)) 


def gameThingsEventLoop(joysticks):
  pygame.event.pump()
  for ev in pygame.event.get():       
    if joysticks.canHandlePygameEvent(ev):
      joysticks.handlePygameEvent(ev)


class InputDeviceSelection(object):
  def __init__(self, dev = None, axis = None, button = None):
    self.axis = axis
    self.dev = unassignedJoystick if dev is None else dev
    self.button = button
  
  def update(self, ev):
    if abs(ev.value) > 0.5:
      self.axis = ev.axis
      self.dev  = ev.dev
      self.button = ev.button
  
  def __str__(self):
    axis_or_button = ("Axis %i" % self.axis) if self.axis is not None else (("Button %i" % self.button) if self.button is not None else "??")
    return "%s: %s" % (self.dev, axis_or_button)

  def toSerializationDict(self):
    return dict(
      joystick = self.dev.ident,
      axis = self.axis,
      button = self.button
    )

  @staticmethod
  def fromSerializationDict(settings, identToJoystickMap):
    return InputDeviceSelection(
      dev = identToJoystickMap[tuple(settings['joystick'])],
      axis = settings['axis'],
      button = settings['button'])


class Command(object):
  def __init__(self, name):
    self.inputDeviceSelection = InputDeviceSelection()
    self.name = name
    self.widgetButton = None
    self.widgetLabel  = None
  
  def updateDeviceLabel(self):
    self.widgetLabel.setText(str(self.inputDeviceSelection))
  
  def updateInputDeviceSelection(self, ev):
    self.inputDeviceSelection.update(ev)
    self.updateDeviceLabel()
  
  def createWidgets(self, parent):
    self.widgetLabel = w1 = QtGui.QLabel(parent)
    self.widgetButton = w2 = QtGui.QPushButton(parent)
    w2.setText(self.name)
    w2.setCheckable(True)
    self.updateDeviceLabel()

  def addToDict(self, settings):
    settings[self.name] = self.inputDeviceSelection.toSerializationDict()


if __name__ == '__main__':
  class MainWindow(QtGui.QMainWindow):
    SUPER = QtGui.QMainWindow
    def __init__(self, parent = None):
      self.SUPER.__init__(self, parent)
      
      self.commands = []
      for name in "fwdAxis steerAxis slower faster camAxis abort".split():
        self.commands.append(Command(name))
      
      self.centralWidget = QtGui.QWidget(self)
      self.setCentralWidget(self.centralWidget)
      layout = QtGui.QGridLayout()
      self.centralWidget.setLayout(layout)
      
      for i, cmd in enumerate(self.commands):
        cmd.createWidgets(self.centralWidget)
        layout.addWidget(cmd.widgetButton, i, 0)
        layout.addWidget(cmd.widgetLabel, i, 1)
        cmd.widgetButton.clicked.connect(
          functools.partial(self._commandSelected, cmd = cmd))
      
      self.saveButton = QtGui.QPushButton(self.centralWidget)
      layout.addWidget(self.saveButton, len(self.commands), 0, 1, 2)
      self.saveButton.clicked.connect(self._saveSettings)
      self.saveButton.setText("Save")
      
      self.joysticks = Joysticks()
      
      self.polltimer = QtCore.QTimer()
      self.polltimer.timeout.connect(self._pollSticks)
      self.polltimer.setInterval(10)
      self.polltimer.start()
      
      self.currentCommand = None
    
    def _commandSelected(self, on, cmd):
      for othercmd in self.commands:
        if othercmd is not cmd:
          othercmd.widgetButton.setChecked(False)
      self.currentCommand = cmd if on else None
    
    def _updateStickSelection(self, ev):
      if self.currentCommand is not None:
        self.currentCommand.updateInputDeviceSelection(ev)
    
    def _pollSticks(self):
      gameThingsEventLoop(self.joysticks)
      events = self.joysticks.poll()
      for ev in events:
        self._updateStickSelection(ev)
    
    def _saveSettingsToFilename(self, fn):
      settings = dict()
      for cmd in self.commands:
        cmd.addToDict(settings)
      with open(fn, 'w') as f:
        json.dump(settings, f)
        f.write('\n')
    
    def _saveSettings(self, _):
      fileName = QtGui.QFileDialog.getSaveFileName(self, "Save Settings", os.path.join(os.environ["HOME"],'.config'), "JSON File (*.json)") 
      if fileName:
        self._saveSettingsToFilename(fileName)



  def joystickSelectionMain():
    pygame.init()
    pygame.joystick.init()
    app = QtGui.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
  
  joystickSelectionMain()