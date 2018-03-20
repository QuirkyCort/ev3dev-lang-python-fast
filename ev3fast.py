#!/usr/bin/env python3

import os
import struct
import select

class Sensor:
  _DIRECTORY_BASE = '/sys/class/lego-sensor/sensor'

  _PRE_OPENS = [
    ('bin_data', os.O_RDONLY),
    ('bin_data_format', os.O_RDONLY),
    ('command', os.O_WRONLY),
    ('decimals', os.O_RDONLY),
    ('mode', os.O_RDWR),
    ('num_values', os.O_RDONLY),
    ('units', os.O_RDONLY)
  ]

  _PRE_READS = [
    ('address', str),
    ('driver_name', str),
    ('modes', str)
  ]

  _BYTES_FMT = {
    'u8':     (1, '<B'),
    's8':     (1, '<b'),
    'u16':    (2, '<H'),
    's16':    (2, '<h'),
    's16_be': (2, '>h'),
    's32':    (4, '<i'),
    'float':  (4, '<f')
  }

  _DRIVER_NAME = None

  def __init__(self, address=None):
    self._currentMode = None
    self._fd = {}
    self._bin_data_format_mode = None
    self._decimals_mode = None
    self._num_values_mode = None
    self._units_mode = None

    for i in range(0,10):
      directory = self._DIRECTORY_BASE + str(i) + '/'
      if address:
        try:
          with open(directory + 'address', 'r') as f:
            if f.read().rstrip() == address:
              self._directory = directory
              break
        except FileNotFoundError:
          pass
      else:
        try:
          with open(directory + 'driver_name', 'r') as f:
            if f.read().rstrip() == self._DRIVER_NAME:
              self._directory = directory
              break
        except FileNotFoundError:
          pass
    self._pre_open()
    self._pre_read()
    with open(self._directory + 'mode', 'r') as f:
      self._currentMode = f.read()

  def _pre_open(self):
    for pre_open in self._PRE_OPENS:
      fd = os.open(self._directory + pre_open[0], pre_open[1])
      if (len(pre_open) > 2):
        self._fd[pre_open[2]] = fd
      else:
        self._fd[pre_open[0]] = fd

  def _pre_read(self):
    for pre_read in self._PRE_READS:
      try:
        with open(self._directory + pre_read[0], 'r') as f:
          setattr(self, pre_read[0], pre_read[1](f.read().rstrip()))
      except:
        pass
  
  def bin_data(self, fmt=None):
    value_size = self._BYTES_FMT[self.bin_data_format][0]
    num_bytes = self.num_values * value_size
    data = os.read(self._fd['bin_data'], num_bytes)
    if fmt:
      return struct.unpack(fmt, data)
    else:
      return bytearray(data)

  @property
  def bin_data_format(self):
    if self._bin_data_format_mode != self._currentMode:
      os.lseek(self._fd['bin_data_format'], 0, os.SEEK_SET)
      self._bin_data_format = os.read(self._fd['bin_data_format'], 100).decode('utf-8').rstrip()
      self._bin_data_format_mode = self._currentMode
    return self._bin_data_format

  @property
  def command(self):
    raise Exception("command is a write-only property!")

  @command.setter
  def command(self, value):
    os.write(self._fd['command'], str(value).encode('ascii'))
    return 0

  @property
  def commands(self):
    with open(self._directory + 'commands', 'r') as f:
      return f.read().rstrip()

  @property
  def decimals(self):
    if self._decimals_mode != self._currentMode:
      os.lseek(self._fd['decimals'], 0, os.SEEK_SET)
      self._decimals = int(os.read(self._fd['decimals'], 10).decode('utf-8').rstrip())
      self._decimals_mode = self._currentMode
    return self._decimals

  @property
  def mode(self):
    return self._currentMode.rstrip()

  @mode.setter
  def mode(self, mode):
    if (self._currentMode != mode):
      os.write(self._fd['mode'], str(mode).encode('ascii'))
      self._currentMode = mode
    return 0

  @property
  def num_values(self):
    if self._num_values_mode != self._currentMode:
      os.lseek(self._fd['num_values'], 0, os.SEEK_SET)
      self._num_values = int(os.read(self._fd['num_values'], 10).decode('utf-8').rstrip())
      self._num_values_mode = self._currentMode
    return self._num_values

  @property
  def units(self):
    if self._units_mode != self._currentMode:
      os.lseek(self._fd['num_values'], 0, os.SEEK_SET)
      self._units = str(os.read(self._fd['units'], 10).decode('utf-8').rstrip())
      self._units_mode = self._currentMode
    return self._units

  def value(self, n=0):
    fmt = self._BYTES_FMT[self.bin_data_format]
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    startByte = n * fmt[0]
    endByte = startByte + fmt[0]
    data = os.read(self._fd['bin_data'], endByte)
    return struct.unpack(fmt[1], data[startByte:endByte])[0]


class TouchSensor(Sensor):
  _DRIVER_NAME = 'lego-ev3-touch'

  MODE_TOUCH = 'TOUCH'
  
  def __init__(self, address=None):
    self._PRE_OPENS.append(('value0', os.O_RDONLY))
    super().__init__(address)

  @property
  def is_pressed(self):
    os.lseek(self._fd['value0'], 0, os.SEEK_SET)
    if (os.read(self._fd['value0'], 1) == '1'):
      return True
    else:
      return False


class ColorSensor(Sensor):
  _DRIVER_NAME = 'lego-ev3-color'

  COLOR_NOCOLOR = 0
  COLOR_BLACK = 1
  COLOR_BLUE = 2
  COLOR_GREEN = 3
  COLOR_YELLOW = 4
  COLOR_RED = 5
  COLOR_WHITE = 6
  COLOR_BROWN = 7

  MODE_COL_AMBIENT = 'COL-AMBIENT'
  MODE_COL_COLOR = 'COL-COLOR'
  MODE_COL_REFLECT = 'COL-REFLECT'
  MODE_REF_RAW = 'REF-RAW'
  MODE_RGB_RAW = 'RGB-RAW'
    
  @property
  def raw(self):
    if (self._currentMode != self.MODE_RGB_RAW):
      self.mode = self.MODE_RGB_RAW
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 6)
    return struct.unpack('<hhh', data)

  @property
  def ambient_light_intensity(self):
    if (self._currentMode != self.MODE_COL_AMBIENT):
      self.mode = self.MODE_COL_AMBIENT
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 1)
    return struct.unpack('<b', data)[0]

  @property
  def red(self):
    if (self._currentMode != self.MODE_RGB_RAW):
      self.mode = self.MODE_RGB_RAW
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 2)
    return struct.unpack('<h', data)[0]

  @property
  def green(self):
    if (self._currentMode != self.MODE_RGB_RAW):
      self.mode = self.MODE_RGB_RAW
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 4)
    return struct.unpack('<h', data[2:4])[0]

  @property
  def blue(self):
    if (self._currentMode != self.MODE_RGB_RAW):
      self.mode = self.MODE_RGB_RAW
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 6)
    return struct.unpack('<h', data[4:6])[0]

  @property
  def reflected_light_intensity(self):
    if (self._currentMode != self.MODE_COL_REFLECT):
      self.mode = self.MODE_COL_REFLECT
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 1)
    return struct.unpack('<b', data)[0]

  @property
  def color(self):
    if (self._currentMode != self.MODE_COL_COLOR):
      self.mode = self.MODE_COL_COLOR
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 1)
    return struct.unpack('<b', data)[0]


class UltrasonicSensor(Sensor):
  _DRIVER_NAME = 'lego-ev3-us'

  MODE_US_DIST_CM = 'US-DIST-CM'
  MODE_US_DIST_IN = 'US-DIST-IN'
  MODE_US_LISTEN = 'US-LISTEN'
  MODE_US_SI_CM = 'US-SI-CM'
  MODE_US_SI_IN = 'US-SI-IN'
    
  @property
  def distance_centimeters(self):
    if (self._currentMode != self.MODE_US_DIST_CM):
      self.mode = self.MODE_US_DIST_CM
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 2)
    return struct.unpack('<h', data)[0] / 10

  @property
  def distance_inches(self):
    if (self._currentMode != self.MODE_US_DIST_IN):
      self.mode = self.MODE_US_DIST_IN
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 2)
    return struct.unpack('<h', data)[0] / 10

  @property
  def other_sensor_present(self):
    if (self._currentMode != self.MODE_US_LISTEN):
      self.mode = self.MODE_US_LISTEN
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 1)
    return struct.unpack('<b', data)[0]


class GyroSensor(Sensor):
  _DRIVER_NAME = 'lego-ev3-gyro'

  MODE_GYRO_ANG = 'GYRO-ANG'
  MODE_GYRO_CAL = 'GYRO-CAL'
  MODE_GYRO_FAS = 'GYRO-FAS'
  MODE_GYRO_G_A = 'GYRO-G&A'
  MODE_GYRO_RATE = 'GYRO-RATE'
    
  @property
  def angle(self):
    if (self._currentMode != self.MODE_GYRO_ANG):
      self.mode = self.MODE_GYRO_ANG
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 2)
    return struct.unpack('<h', data)[0]

  @property
  def rate(self):
    if (self._currentMode != self.MODE_GYRO_RATE):
      self.mode = self.MODE_GYRO_RATE
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 2)
    return struct.unpack('<h', data)[0]

  @property
  def rate_and_angle(self):
    if (self._currentMode != self.MODE_GYRO_G_A):
      self.mode = self.MODE_GYRO_G_A
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 4)
    return struct.unpack('<hh', data)


class Motor:
  COMMAND_RESET = 'reset'
  COMMAND_RUN_DIRECT = 'run-direct'
  COMMAND_RUN_FOREVER = 'run-forever'
  COMMAND_RUN_TIMED = 'run-timed'
  COMMAND_RUN_TO_ABS_POS = 'run-to-abs-pos'
  COMMAND_RUN_TO_REL_POS = 'run-to-rel-pos'
  COMMAND_STOP = 'stop'
  ENCODER_POLARITY_INVERSED = 'inversed'
  ENCODER_POLARITY_NORMAL = 'normal'
  POLARITY_INVERSED = 'inversed'
  POLARITY_NORMAL = 'normal'
  STATE_HOLDING = 'holding'
  STATE_OVERLOADED = 'overloaded'
  STATE_RAMPING = 'ramping'
  STATE_RUNNING = 'running'
  STATE_STALLED = 'stalled'
  STOP_ACTION_BRAKE = 'brake'
  STOP_ACTION_COAST = 'coast'
  STOP_ACTION_HOLD = 'hold'

  _DIRECTORY_BASE = '/sys/class/tacho-motor/motor'

  _PRE_OPENS = [
    ('command', os.O_WRONLY),
    ('duty_cycle', os.O_RDONLY),
    ('duty_cycle_sp', os.O_RDWR),
    ('polarity', os.O_RDWR),
    ('position', os.O_RDWR),
    ('hold_pid/Kd', os.O_RDWR, 'position_d'),
    ('hold_pid/Ki', os.O_RDWR, 'position_i'),
    ('hold_pid/Kp', os.O_RDWR, 'position_p'),
    ('position_sp', os.O_RDWR),
    ('ramp_down_sp', os.O_RDWR),
    ('ramp_up_sp', os.O_RDWR),
    ('speed', os.O_RDONLY),
    ('speed_pid/Kd', os.O_RDWR, 'speed_d'),
    ('speed_pid/Ki', os.O_RDWR, 'speed_i'),
    ('speed_pid/Kp', os.O_RDWR, 'speed_p'),
    ('speed_sp', os.O_RDWR),
    ('state', os.O_RDONLY),
    ('stop_action', os.O_RDWR),
    ('time_sp', os.O_RDWR)
  ]

  _PRE_READS = [
    ('address', str),
    ('commands', str),
    ('count_per_rot', int),
    ('count_per_m', int),
    ('full_travel_count', int),
    ('driver_name', str),
    ('max_speed', int),
    ('stop_actions', str)
  ]
    
  _DRIVER_NAME = None

  def __init__(self, address=None):
    self._fd = {}

    for i in range(0,10):
      directory = self._DIRECTORY_BASE + str(i) + '/'
      if address:
        try:
          with open(directory + 'address', 'r') as f:
            if f.read().rstrip() == address:
              self._directory = directory
              break
        except FileNotFoundError:
          pass
      else:
        try:
          with open(directory + 'driver_name', 'r') as f:
            if f.read().rstrip() == self._DRIVER_NAME:
              self._directory = directory
              break
        except FileNotFoundError:
          pass
    self._pre_open()
    self._pre_read()

  def _pre_open(self):
    for pre_open in self._PRE_OPENS:
      fd = os.open(self._directory + pre_open[0], pre_open[1])
      if (len(pre_open) > 2):
        self._fd[pre_open[2]] = fd
      else:
        self._fd[pre_open[0]] = fd

  def _pre_read(self):
    for pre_read in self._PRE_READS:
      try:
        with open(self._directory + pre_read[0], 'r') as f:
          setattr(self, pre_read[0], pre_read[1](f.read()))
      except:
        pass

  @property
  def command(self):
    raise Exception("command is a write-only property!")

  @command.setter
  def command(self, value):
    os.write(self._fd['command'], str(value).encode('ascii'))
    return 0

  @property
  def duty_cycle(self):
    os.lseek(self._fd['duty_cycle'], 0, os.SEEK_SET)
    return int(os.read(self._fd['duty_cycle'], 5))

  @property
  def duty_cycle_sp(self):
    os.lseek(self._fd['duty_cycle_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['duty_cycle_sp'], 5))

  @duty_cycle_sp.setter
  def duty_cycle_sp(self, value):
    os.write(self._fd['duty_cycle_sp'], str(value).encode('ascii'))
    return 0

  @property
  def is_holding(self):
    return self.STATE_HOLDING in self.state

  @property
  def is_overloaded(self):
    return self.STATE_OVERLOADED in self.state

  @property
  def is_ramping(self):
    return self.STATE_RAMPING in self.state

  @property
  def is_running(self):
    return self.STATE_RUNNING in self.state

  @property
  def is_stalled(self):
    return self.STATE_STALLED in self.state

  @property
  def polarity(self):
    os.lseek(self._fd['polarity'], 0, os.SEEK_SET)
    return str(os.read(self._fd['polarity'], 100))

  @polarity.setter
  def polarity(self, value):
    os.write(self._fd['polarity'], str(value).encode('ascii'))
    return 0

  @property
  def position(self):
    os.lseek(self._fd['position'], 0, os.SEEK_SET)
    return int(os.read(self._fd['position'], 100))

  @position.setter
  def position(self, value):
    os.write(self._fd['position'], str(value).encode('ascii'))
    return 0

  @property
  def position_d(self):
    os.lseek(self._fd['position_d'], 0, os.SEEK_SET)
    return int(os.read(self._fd['position_d'], 100))

  @position_d.setter
  def position_d(self, value):
    os.write(self._fd['position_d'], str(value).encode('ascii'))
    return 0

  @property
  def position_i(self):
    os.lseek(self._fd['position_i'], 0, os.SEEK_SET)
    return int(os.read(self._fd['position_i'], 100))

  @position_i.setter
  def position_i(self, value):
    os.write(self._fd['position_i'], str(value).encode('ascii'))
    return 0

  @property
  def position_p(self):
    os.lseek(self._fd['position_p'], 0, os.SEEK_SET)
    return int(os.read(self._fd['position_p'], 100))

  @position_p.setter
  def position_p(self, value):
    os.write(self._fd['position_p'], str(value).encode('ascii'))
    return 0

  @property
  def position_sp(self):
    os.lseek(self._fd['position_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['position_sp'], 100))

  @position_sp.setter
  def position_sp(self, value):
    os.write(self._fd['position_sp'], str(value).encode('ascii'))
    return 0

  @property
  def ramp_down_sp(self):
    os.lseek(self._fd['ramp_down_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['ramp_down_sp'], 100))

  @ramp_down_sp.setter
  def ramp_down_sp(self, value):
    os.write(self._fd['ramp_down_sp'], str(value).encode('ascii'))
    return 0

  @property
  def ramp_up_sp(self):
    os.lseek(self._fd['ramp_up_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['ramp_up_sp'], 100))

  @ramp_up_sp.setter
  def ramp_up_sp(self, value):
    os.write(self._fd['ramp_up_sp'], str(value).encode('ascii'))
    return 0

  @property
  def speed(self):
    os.lseek(self._fd['speed'], 0, os.SEEK_SET)
    return int(os.read(self._fd['speed'], 100))

  @property
  def speed_d(self):
    os.lseek(self._fd['speed_d'], 0, os.SEEK_SET)
    return int(os.read(self._fd['speed_d'], 100))

  @speed_d.setter
  def speed_d(self, value):
    os.write(self._fd['speed_d'], str(value).encode('ascii'))
    return 0

  @property
  def speed_i(self):
    os.lseek(self._fd['speed_i'], 0, os.SEEK_SET)
    return int(os.read(self._fd['speed_i'], 100))

  @speed_i.setter
  def speed_i(self, value):
    os.write(self._fd['speed_i'], str(value).encode('ascii'))
    return 0

  @property
  def speed_p(self):
    os.lseek(self._fd['speed_p'], 0, os.SEEK_SET)
    return int(os.read(self._fd['speed_p'], 100))

  @speed_p.setter
  def speed_p(self, value):
    os.write(self._fd['speed_p'], str(value).encode('ascii'))
    return 0

  @property
  def speed_sp(self):
    os.lseek(self._fd['speed_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['speed_sp'], 100))

  @speed_sp.setter
  def speed_sp(self, value):
    os.write(self._fd['speed_sp'], str(value).encode('ascii'))
    return 0

  @property
  def state(self):
    os.lseek(self._fd['state'], 0, os.SEEK_SET)
    return str(os.read(self._fd['state'], 1024))

  @property
  def stop_action(self):
    os.lseek(self._fd['stop_action'], 0, os.SEEK_SET)
    return str(os.read(self._fd['stop_action'], 100))

  @stop_action.setter
  def stop_action(self, value):
    os.write(self._fd['stop_action'], str(value).encode('ascii'))
    return 0

  @property
  def time_sp(self):
    os.lseek(self._fd['time_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['time_sp'], 100))

  @time_sp.setter
  def time_sp(self, value):
    os.write(self._fd['time_sp'], str(value).encode('ascii'))
    return 0

  def reset(self, **kwargs):
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'reset\n')
    
  def run_direct(self, **kwargs):
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'run-direct\n')
    
  def run_forever(self, **kwargs):
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'run-forever\n')

  def run_timed(self, **kwargs):
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'run-timed\n')

  def run_to_abs_pos(self, **kwargs):
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'run-to-abs-pos\n')

  def run_to_rel_pos(self, **kwargs):
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'run-to-rel-pos\n')

  def stop(self, **kwargs):
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'stop\n')

  def wait(self, cond, timeout=None):
    poll = select.poll()
    poll.register(self._fd['state'], select.POLLIN)

    while True:
      event = poll.poll(timeout)

      if len(event) == 0:
        return False
      
      if cond(self.state):
        return True

  def wait_until(self, s, timeout=None):
    return self.wait(lambda state: s in state, timeout)

  def wait_until_not_moving(self, timeout=None):
    return self.wait(lambda state: self.STATE_RUNNING not in state or self.STATE_STALLED in state, timeout)

  def wait_while(self, s, timeout=None):
    return self.wait(lambda state: s not in state, timeout)


class LargeMotor(Motor):
  _DRIVER_NAME = 'lego-ev3-l-motor'


class MediumMotor(Motor):
  _DRIVER_NAME = 'lego-ev3-m-motor'
