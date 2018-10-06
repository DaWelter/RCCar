# -*- coding: utf-8 -*-

import math
import collections

from rccarcommon.motor_pb2 import MotorScopeMsg, MotorControlMsg, MotorReportContainerMsg
from rccarcommon.rccar_pb2 import CarStatusMsg, PingMsg, CarControlContainerMsg, CarReportContainerMsg, CarStatusMsg, CarSteerMsg
import protocobs
from protocobs import encode


def CarReportDecoder(cb):
  return protocobs.ProtobufDecoder(CarReportContainerMsg, cb)

def CarCommandDecoder(cb):
  return protocobs.ProtobufDecoder(CarControlContainerMsg, cb)

def MotorStatusDecoder(cb):
  return protocobs.ProtobufDecoder(MotorReportContainerMsg, cb)