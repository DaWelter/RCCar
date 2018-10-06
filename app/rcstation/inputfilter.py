# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 22:57:27 2016

@author: mwelter
"""
import time
import json
import os
import collections
import pygame

try:
  from PyQt5 import QtCore
except:
  print ("%s: Import of PyQt5 failed. Fallback to version 4." % __file__)
  from PyQt4 import QtCore

from qtjoystickselection import unassignedJoystick, InputDeviceSelection
from rccarcommon.misc import sgn


class ConstantState(object):
  def __init__(self, c):
    self.state = c

  def updateS(self, dt):
    pass

class TimeContinuousSignalFilter(object):
  def __init__(self):
    self.source = None
    
  def updateS(self, dt):
    self.source.updateS(dt)
    self.update(dt, self.source.state)


class TimeContinuousSignalFilter2(object):
  def __init__(self):
    self.source1 = None
    self.source2 = None
  
  def updateS(self, dt):
    self.source1.updateS(dt)
    self.source2.updateS(dt)
    self.update(dt, self.source1.state, self.source2.state)
    

class FunctionMapFilter(TimeContinuousSignalFilter):
  def __init__(self, fun):
    TimeContinuousSignalFilter.__init__(self)
    self.fun = fun
    self.state = 0.
  
  def update(self, dt, f):
    self.state = self.fun(f)



class BinaryFunctionMapFilter(TimeContinuousSignalFilter2):
  def __init__(self, fun):
    TimeContinuousSignalFilter2.__init__(self)
    self.fun = fun
    self.state = 0.
  
  def update(self, dt, f1, f2):
    self.state = self.fun(f1, f2)


def chain(instance, *sources):
  if len(sources) > 1:
    for i, source in enumerate(sources):
      setattr(instance, 'source%i' % (i+1), source)
  else:
    instance.source = sources[0]
  return instance


class SecondOrderFilter(TimeContinuousSignalFilter):
  '''driven spring-damper system; numerically integrated,
     
     First Argument is dampending coefficient. Note: If no spring
     constant is provided, lower values mean slower response.
     
     Spring Coefficient is by default adjusted so that the output
     of a step input reaches the final value in the least amount of
     time without overshoot.
  '''
  #STEADY_STATE_FWD_MAX = 0.90
  def __init__(self, coeffD, springyness = 1.):
    TimeContinuousSignalFilter.__init__(self)
    self.state   = 0. # position
    self.v       = 0. # speed
    self.coeffD = coeffD
    self.coeffP = self.coeffD**2 / 4. * springyness
    self.source = None

  def update(self, dt, f):
    v, y = self.v, self.state
    cD, cP = self.coeffD, self.coeffP
    # euler integration
    yNew = y + v * dt
    vNew = v + dt * ((f - y)*cP - cD * v)
    self.state = yNew
    self.v     = vNew


class ForwardControlIntegrator(TimeContinuousSignalFilter2):
  def __init__(self):  
    TimeContinuousSignalFilter2.__init__(self)
    self.state = 0.
    self.lastRate = 0.
    self.fwdBounds   = -1., 1.
    self.coeffFwdRate = 2.

  def update(self, dt, f, fnofilt):
    if abs(f) < 1.e-3:
      f = 0.
    if self.state > -1.e-3 and fnofilt > 0. and (self.fwdLastRate) < 1.e-3:
      self.fwdBounds = (0., 1.)
    elif self.state < 1.e-3 and fnofilt < 0. and abs(self.fwdLastRate) < 1.e-3:
      self.fwdBounds = (-1., 0.)
    self.state = self.state + dt * f**3 * self.coeffFwdRate
    self.state = min(max(self.state, self.fwdBounds[0]), self.fwdBounds[1])
    self.fwdLastRate = fnofilt


class DiscretValueSwitch(TimeContinuousSignalFilter):
  def __init__(self, values, debounceTime, threshold, hysteresis):
    TimeContinuousSignalFilter.__init__(self)
    assert len(values) > 0
    self.maxIndex = len(values)-1
    self.values = values
    self.currentIndex = 0
    self.state = values[0]
    self.sinceLastChange = 0.
    self.threshold = threshold
    self.hysteresis = hysteresis
    self.debounceTime = debounceTime
    self.isNeutral = True
  
  def update(self, dt, f):
    self.sinceLastChange += dt
    if self.isNeutral:
      if f > self.threshold + self.hysteresis and \
        self.sinceLastChange > self.debounceTime and \
        self.currentIndex < self.maxIndex:
          self.currentIndex += 1
          self.state = self.values[self.currentIndex]
          self.sinceLastChange = 0.
          self.isNeutral = False
      if f < -self.threshold - self.hysteresis and \
        self.sinceLastChange > self.debounceTime and \
        self.currentIndex > 0:
          self.currentIndex -= 1
          self.state = self.values[self.currentIndex]
          self.sinceLastChange = 0.
          self.isNeutral = False
    else:
      if abs(f) < self.threshold - self.hysteresis:
        self.isNeutral = True
      

###############################################################################
### Joystick input handling
###############################################################################
class JoystickInputAxisFilter(object):
  def __init__(self, stick, axis, multiplier = 1.):
    self.stick = stick
    self.axis  = axis
    self.state = 0.
    self.multiplier = multiplier
    self.stick.init()
  
  def updateS(self, dt):
    self.state = self.multiplier*self.stick.get_axis(self.axis)
    #NOTE: Gamepad axis values reach max their limits then the physical stick is moved to only 50% of max deflection.


class JoystickInputButtonFilter(object):
  def __init__(self, stick, button, multiplier = 1.):
    self.stick = stick
    self.button  = button
    self.state = 0.
    self.multiplier = multiplier
    self.stick.init()
  
  def updateS(self, dt):
    self.state = self.multiplier*self.stick.get_button(self.button)



def CreateIdentToPygameStickDict():
  byName = collections.defaultdict(list)
  for i in range(pygame.joystick.get_count()):
    js = pygame.joystick.Joystick(i)
    byName[js.get_name()].append(js)
  joysticks = {}
  for name, l in byName.items():
    for i, js in enumerate(l):
      joysticks[(name, i)] = js
  joysticks[unassignedJoystick.ident] = unassignedJoystick
  return joysticks


def LoadJoystickConfig(filename):
  try:
    with open(filename, 'r') as f:
       settings = json.load(f)
  except IOError:
    return None
  identToPygameStick = CreateIdentToPygameStickDict()
  commandToInputFilter = {}
  for command, commandSettings in settings.items():
    try:
      deviceSelect = InputDeviceSelection.fromSerializationDict(commandSettings, identToPygameStick)
    except KeyError:
      print "Warning: Joystick %s not available!" % (str(commandSettings))
      continue
    if deviceSelect.dev is unassignedJoystick:
      print "Warning: Command %s is unassigned!" % (command)
      continue
    print "Command %s assigned to %s" % (command, str(deviceSelect))
    if deviceSelect.axis is not None:
      jsinput = JoystickInputAxisFilter(deviceSelect.dev, deviceSelect.axis)
    elif deviceSelect.button is not None:
      jsinput = JoystickInputButtonFilter(deviceSelect.dev, deviceSelect.button)
    else:
      raise Exception("Command %s uses neither axis nor buttons of %s" % (command, str(deviceSelect.dev)))
    commandToInputFilter[command] = jsinput
  return commandToInputFilter


###############################################################################
### Keyboard input handling
###############################################################################
class KeyboardInputFilter(object):
  def __init__(self, upkey, downkey):
    self.upkey = upkey
    self.downkey = downkey
    self.state = 0.
    self.keystate = dict()
  
  def _filter_event_by_key_state(self, e):
    '''
      If we get key down event we only want to process it if we think that the key is not already down.
      Vice versa for key up events.
    '''
    key = e.key()
    state = self._to_key_state(e)
    previous_state = self.keystate.get(key, 0)
    return state != previous_state

  def _to_key_state(self, e):
    '''
      KeyPress -> 1, meaning key is pressed
      KeyRelease -> 0, key not pressed
    '''
    state = {
      QtCore.QEvent.KeyPress : 1,
      QtCore.QEvent.KeyRelease : 0
    }[e.type()]
    return state

  def willThisHandle(self, e):
    '''
      check for actual keys pressed and sane pressed/released state.
    '''
    u, d = self.upkey, self.downkey
    return (e.type() in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease)) \
      and (e.key() in (u,d))  \
      and self._filter_event_by_key_state(e)

  def keyPressEvent(self, e):
    '''
      record state
    '''
    u, d = self.upkey, self.downkey
    assert e.key() in (u, d)
    if not e.isAutoRepeat():
      self.keystate[e.key()] = self._to_key_state(e)
      self.state += 1. if e.key()==u else -1.
      #print 'state %s/%s = %s' % (u, d, self.state)
  
  def keyReleaseEvent(self, e):
    '''
      record state
    '''
    u, d = self.upkey, self.downkey
    assert e.key() in (u, d)
    if not e.isAutoRepeat():
      self.keystate[e.key()] = self._to_key_state(e)
      self.state -= 1. if e.key()==u else -1.
      #print 'state %s/%s = %s' % (u, d, self.state)
  
  def updateS(self, dt):
    pass  
  
# NOTE: you think the joystick has a deadzone. Well the linux joystick driver
# implements its own deadzone. To change it, run jstest-gtk. 
# See also: https://wiki.archlinux.org/index.php/Gamepad#Setting_up_deadzones_and_calibration
class SteeringControl(object):
  def __init__(self):
    self.eventHandlers = []
    self.lastTime = None
    if not self.setupJoystickControl():
      self.setupKeyboardControl()
  
  def setupKeyboardControl(self):
    self.rawFwd = kbFwd     = KeyboardInputFilter(QtCore.Qt.Key_Up, QtCore.Qt.Key_Down)
    # create pipeline nodes
    fwdScale = FunctionMapFilter(lambda x: x * 1.)
    fwdFilt  = SecondOrderFilter(50.)
    fwdInt   = ForwardControlIntegrator()
    fwdFilt2 = FunctionMapFilter(lambda x: sgn(x)*pow(abs(x), 1.2))
    # setup pipeline
    fwdScale.source = kbFwd
    fwdFilt.source = fwdScale
    fwdInt.source1 = fwdFilt
    fwdInt.source2 = fwdScale
    fwdFilt2.source = fwdInt
    self.filtFwd = fwdFilt2
    # create pipeline nodes
    self.rawRight = kbRight   = KeyboardInputFilter(QtCore.Qt.Key_Right, QtCore.Qt.Key_Left)
    rightFilt = SecondOrderFilter(10.)
    # setup pipeline    
    rightFilt.source = kbRight    
    self.filtRight = rightFilt
    # We don't use the gears mode with keyboards, so just make 
    # this class return the sensible value of 1., i.e. full speed.
    self.filtGears = ConstantState(1.)
    self.filtCamera = self.rawCamera = ConstantState(0.)
    # gui stuff
    self.eventHandlers += [kbFwd, kbRight]

  def setupJoystickControl(self):
    filename = os.path.join(os.environ['HOME'], '.local', 'rccar', 'joystick.json')
    config = LoadJoystickConfig(filename)
    if config is None:
      return False

    if not all((k in config) for k in ['steerAxis', 'fwdAxis', 'faster', 'slower', 'camAxis']):
      return False

    self.rawRight = stickRight = config['steerAxis']
    self.rawFwd   = stickFwd = config['fwdAxis']
    self.rawFaster = stickFaster = config['faster']
    self.rawSlower = stickSlower = config['slower']
    self.rawCamera = stickCamera = config['camAxis']
    stickFwd.multiplier = -1.
    stickSlower.multiplier = -1.

    gears = chain(BinaryFunctionMapFilter(lambda a,b: a + b), stickFaster, stickSlower)
    gears = chain(DiscretValueSwitch([0.05, 0.1, 0.2, 0.5, 1.], 0.05, 0.5, 0.1), gears)
    fwd = chain(BinaryFunctionMapFilter(lambda a,b: a * b), 
                gears,
                chain(FunctionMapFilter(lambda x: sgn(x)*pow(abs(x), 2.0)),
                      stickFwd))
    fwd = chain(SecondOrderFilter(30.), fwd)
    self.filtFwd = fwd
    self.filtGears   = gears
    self.filtCamera = self.rawCamera # Is filtered on the uC.
    
    right = chain(SecondOrderFilter(30.),
                  stickRight)
    self.filtRight = right
    return True

  def keyPressEvent(self, e):
    for h in self.eventHandlers:
      if h.willThisHandle(e):
        self.update()
        h.keyPressEvent(e)
        return True
    return False
    
  def keyReleaseEvent(self, e):
    for h in self.eventHandlers:
      if h.willThisHandle(e):
        self.update()
        h.keyReleaseEvent(e)
        return True
    return False
  
  def update(self):
    tNew = time.time()
    if self.lastTime is not None:
      t = self.lastTime
      dt = tNew - t
      self.filtFwd.updateS(dt)
      self.filtRight.updateS(dt)
      self.filtCamera.updateS(dt)
    self.lastTime = tNew

  @property
  def gears(self):
    return self.filtGears.state

  @property
  def output(self):
    return self.filtFwd.state, self.filtRight.state, self.filtCamera.state

  @property
  def rawOutput(self):
    return self.rawFwd.state, self.rawRight.state, self.rawCamera.state


if __name__ == '__main__':
  pygame.init()
  pygame.joystick.init()
  control = SteeringControl()
  while True:
    pygame.event.pump()
    control.update()
    time.sleep(0.033)
    print ('output: %f, %f' % control.output)