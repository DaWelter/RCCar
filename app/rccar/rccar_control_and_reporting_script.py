#!/usr/bin/env python2
import sys
import serial
import socket
import select
import threading
import time
import Queue as queue
import subprocess
from collections import namedtuple
import contextlib

from rccarcommon import misc
from rccarcommon import telemetry

# GPIO library is unusable. It creates large random fluctuations in the pulse width.
#import RPi.GPIO as GPIO
import pigpio as _pigpio
pigpio = _pigpio.pi()



MOTOR_DRIVER_SERIAL_DEV='/dev/ttyAMA0'
MOTOR_DRIVER_SERIAL_SPEED=115200

REMOTE_CONTROL_RX_PORT=5700
REMOTE_CONTROL_TX_PORT=5600

SYSTEM_STATE_UPDATE_INTERVAL=1.

ITEM_ID_SYSTEM_STATE=1
ITEM_ID_MOTOR_PACKET=2
ITEM_ID_IMU=3

PIN_WHEEL_STEER=17
PIN_CAM_YAW=18
PIN_CAM_PITCH=19
COMMAND_SELECT_STEER='1'


class ServoControl(object):
  '''
    Combines affine linear value mapping with generation of
    pwm signal to control servoes.

    Note: input values are not clipped.
  '''

  def __init__(self, pin, input_value_range, pulse_width_range_ms):
    self.pin = pin
    self.f_pwm = 50. # Hz
    self._init_coeff(input_value_range, pulse_width_range_ms)

  def _init_coeff(self, input_value_range, pulse_width_range_ms):
    # Tpulse = Tpwm*Pduty = Pduty/Fpwm
    # -> Pduty = Tpulse*Fpwm
    # Value range of Tpulse is usually from Tpmin=1 ms, to Tpmax = 2 ms
    # Tpulse_fractional = Tpf = (input - input_min)/(input_max-input_min)
    # Tpulse = Tpf * (Tpmax - Tpmin) + Tpmin
    #        = (input - input_min)/(input_max-input_min)*(Tpmax - Tpmin) + Tpmin
    #        = input*(Tpmax - Tpmin)/(input_max-input_min) - input_min*(Tpmax - Tpmin)/(input_max-input_min) + Tpmin
    if 0: # Calculations for GPIO library
      in_l, in_h = map(float,input_value_range)
      out_l, out_h = map(float, pulse_width_range_ms)
      out_l *= 1.e-3
      out_h *= 1.e-3
      scale = (out_h-out_l) / (in_h-in_l)
      self.affine_offset = (-in_l*scale + out_l)*self.f_pwm*100.
      self.affine_scale  = scale*self.f_pwm*100.
    else: # Calculations for pigpio.set_servo_pulsewidth
      in_l, in_h = map(float, input_value_range)
      out_l, out_h = map(float, pulse_width_range_ms)
      out_l *= 1.e3
      out_h *= 1.e3
      scale = (out_h-out_l) / (in_h-in_l)
      self.affine_offset = (-in_l*scale + out_l)
      self.affine_scale  = scale

  def start_gpio(self):
    print("Powering servo at GPIO {}".format(self.pin))

  def stop_gpio(self):
    print("Stopping servo at GPIO {}".format(self.pin))
    pigpio.set_servo_pulsewidth(self.pin, 0)

  def _pulse_width_for(self, input):
    return self.affine_scale*input + self.affine_offset

  def __enter__(self):
    self.start_gpio()
    return self

  def __exit__(self, *args):
    self.stop_gpio()

  def move_to(self, input):
    d = self._pulse_width_for(input)
    assert (900. <= d <= 2100.)
    #print ("Duty cycle for {} = {}".format(self.pin, d))
    pigpio.set_servo_pulsewidth(self.pin, d)


def open_link_to_motor_driver():
  ser = serial.Serial(MOTOR_DRIVER_SERIAL_DEV, MOTOR_DRIVER_SERIAL_SPEED)
  ser.timeout = 0.001 # seconds
  return ser


def open_remote_control_sockets():
  # Messages come in here ...
  sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock_rx.bind(('', REMOTE_CONTROL_RX_PORT))
  sock_rx.settimeout(0.1)
  # Messages go out this way ...
  sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock_tx.connect(('127.0.0.1', REMOTE_CONTROL_TX_PORT))
  return sock_rx, sock_tx


def run_imu_process():
  imu_process = subprocess.Popen(['minimu9-ahrs', '-b', '/dev/i2c-1', '--output=euler'],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             bufsize=0)
  imu_fd = imu_process.stdout
  misc.enable_non_blocking_mode(imu_fd)
  return imu_process, imu_fd


class SystemStatePoll(object):
  def __init__(self):
    pass

  def poll_loop(self, callback):
    while 1:
      time.sleep(SYSTEM_STATE_UPDATE_INTERVAL)
      callback(self._parse_load_avg())

  @staticmethod
  def _parse_load_avg():
    try:
      with open('/proc/loadavg', 'r') as f:
        s = f.read().strip()
      # Load average displays a short time average first. Then come additional numbers
      s = s.split(' ')[0]
      return float(s)
    except:
      return -1


def parse_imu_output(data):
  # Extract euler angles as float
  # TODO: named tuple would be nicer
  data = data.split()
  data = data[:3]
  data = list(map(float, data))
  assert len(data) == 3
  return data


class SendToStation(object):
  ''' Telemetry goes to the command station '''

  def __init__(self, inlet_queue, send_function):
    self.inlet_queue = inlet_queue
    self.send_function = send_function
    self.car_status_msg = telemetry.CarStatusMsg(
      speed = 0,
      phase_current = 0,
      voltage = 0,
      pwm_magnitude = 0,
      temperature = 0,
      euler_h = 0,
      euler_p = 0,
      euler_b = 0,
      system_load = -1
    )

  def update_loop(self):
    while 1:
      item = self.inlet_queue.get()
      self.update(item)

  def update(self, item):
    identifier, data = item
    if identifier == ITEM_ID_SYSTEM_STATE:
      # Store this one. Send together with motor data.
      self.car_status_msg.system_load = data
    if identifier == ITEM_ID_IMU:
      h, p, b = data
      self.car_status_msg.euler_h = h
      self.car_status_msg.euler_p = p
      self.car_status_msg.euler_b = b
    elif identifier == ITEM_ID_MOTOR_PACKET:
      if data.HasField('status'):
        data = data.status
        m = self.car_status_msg
        m.speed = data.speed
        m.phase_current = data.phase_current
        m.voltage = data.voltage
        m.pwm_magnitude = data.pwm_magnitude
        m.temperature = data.temperature
        self.send_system_and_motor_state(m)
      elif data.HasField('scope_phase_current'):
        data = data.scope_phase_current
        self.send_scope_readings(data)


  def send_scope_readings(self, scope):
    # Oscilloscope style measurement of phase current.
    msg = telemetry.CarReportContainerMsg(
      scope_phase_current = scope)
    away_with_it = telemetry.encode(msg)
    #print("SENDING SCOPE: {}".format(away_with_it))
    self.send_function(away_with_it)

  def send_system_and_motor_state(self, msg):
    msg = telemetry.CarReportContainerMsg(
      status = msg)
    #print ("SENDING: {}".format(msg))
    self.send_function(telemetry.encode(msg))


class ProcessCommandThread(threading.Thread):
  def __init__(self, motor_link, servos, sock_rx):
    threading.Thread.__init__(self)
    self.motor_link = motor_link
    self.servos = servos
    self.sock_rx = sock_rx
    self.command_decoder = telemetry.CarCommandDecoder(self.on_command_received)
    self.want_stop = threading.Event()

  def stop(self):
    print ("Requested stop for {}".format(type(self).__name__))
    self.want_stop.set()

  def run(self):
    while not self.want_stop.is_set():
      self.read_and_process_commands()
    print ("Thread Stopped")

  def read_and_process_commands(self):
    # Select is not working. But does not matter ...
    #print('wait for input')
    #fd, _, _ = select.select([self.sock_rx], [], [], 1000.)
    #print ('loop')
    #if fd == self.sock_rx:
    #  print ('select ...')
    # Read size set to 1024. But UDP always reads the whole message. And recvfrom just blocks until
    # anything is available. Not knowing what happens if message size is larger than argument.
    try:
      data, addr = self.sock_rx.recvfrom(1024)
    except socket.timeout as e:
      return
    else:
      #print('Read from RX socket: "{:s}"'.format(data))
      self.command_decoder.putc_and_maybe_decode(data)

  def on_command_received(self, msg):
    #print ("Command packet received: {}".format(msg))
    if msg.HasField('steer'):
      self.respond_to_steer_command(msg.steer)

  def respond_to_steer_command(self, msg):
    # First send to motor controller
    motor_cmd = telemetry.MotorControlMsg(
      speed = msg.speed)
    motor_packet_data = telemetry.encode(motor_cmd)
    self.motor_link.write(motor_packet_data)
    # Then control servos
    right = misc.clamp(msg.right, -1.,1.)
    cam_yaw = misc.clamp(msg.cam_right, -1., 1.)
    self.servos.steer.move_to(-right)
    actual_cam_yaw = cam_yaw * 0.8 + right * 0.2
    self.servos.cam_yaw.move_to(-actual_cam_yaw)



def main():
  motor_link = open_link_to_motor_driver()
  sock_rx, sock_tx = open_remote_control_sockets()
  imu_process, imu_fd = run_imu_process()

  Servos = namedtuple('Servos', 'steer cam_yaw cam_pitch')
  with \
       ServoControl(PIN_WHEEL_STEER, (-1., 1.), (1., 2.)) as steer_servo,\
       ServoControl(PIN_CAM_YAW, (-1., 1.), (1., 2.)) as cam_yaw_servo,\
       ServoControl(PIN_CAM_PITCH, (-1., 1.), (1., 2.)) as cam_pitch_servo:
    servos = Servos(steer_servo, cam_yaw_servo, cam_pitch_servo)

    process_command_thread = ProcessCommandThread(motor_link, servos, sock_rx)

    tx_queue = queue.Queue(10)
    system_state_poll = SystemStatePoll()
    system_state_poll_thread = threading.Thread(
      target=system_state_poll.poll_loop,
      args=(lambda x: tx_queue.put((ITEM_ID_SYSTEM_STATE, x)),))
    system_state_poll_thread.daemon = True
    send_to_tx = SendToStation(inlet_queue=tx_queue, send_function=sock_tx.send)
    send_to_tx_thread = threading.Thread(
      target=send_to_tx.update_loop)
    send_to_tx_thread.daemon = True

    def on_new_motor_msg(msg):
      tx_queue.put((ITEM_ID_MOTOR_PACKET, msg))
    motor_msg_decoder = telemetry.MotorStatusDecoder(on_new_motor_msg)

    try:
      process_command_thread.start()
      system_state_poll_thread.start()
      send_to_tx_thread.start()

      while 1:
        avail_read, avail_write, avail_error = select.select([motor_link, sock_tx, imu_fd, imu_process.stderr], [], [], 1000.)
        for read_fd in avail_read:
          if read_fd == motor_link:
            # Try to read a little bit more than is currently available, because in the mean time there is probably more.
            data = motor_link.read(motor_link.inWaiting() + 1024)
            #print ('Read from Motor Link: "{:s}"'.format(data))
            motor_msg_decoder.putc_and_maybe_decode(data)
          elif read_fd == imu_fd:
            imu_reading = parse_imu_output(imu_fd.read())
            tx_queue.put((ITEM_ID_IMU, imu_reading))
          elif read_fd == sock_tx:
            print ('Error: Should not receive from tx socket!')
            sys.exit(1)
          elif read_fd == imu_process.stderr:
            print ('IMU ERROR: ' + imu_process.stderr.read())

    finally:
      process_command_thread.stop()
      process_command_thread.join()


if __name__ == '__main__':
  main()
