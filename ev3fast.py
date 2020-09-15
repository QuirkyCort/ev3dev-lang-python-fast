#!/usr/bin/env python3

import os
import sys
import struct
import select
import time
from collections import OrderedDict
from math import pi

WAIT_RUNNING_TIMEOUT = 100
USE_OS_OPEN = True

if sys.implementation.name == 'micropython':
  USE_OS_OPEN = False

if USE_OS_OPEN:
  RDONLY = os.O_RDONLY
  WRONLY = os.O_WRONLY
  RDWR = os.O_RDWR
else:
  RDONLY = 'rb'
  WRONLY = 'wb'
  RDWR = 'wb+'

class Sensor:
  _DIRECTORY_BASE = '/sys/class/lego-sensor/sensor'

  _PRE_OPENS = {
    'bin_data': [ RDONLY ],
    'bin_data_format': [ RDONLY ],
    'command': [ WRONLY ],
    'decimals': [ RDONLY ],
    'mode': [ RDWR ],
    'num_values': [ RDONLY ],
    'units': [ RDONLY ]
  }

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
            if address in f.read():
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

    for key in self._PRE_OPENS:
      if len(self._PRE_OPENS[key]) < 2:
        self._PRE_OPENS[key].append(key)

    self._pre_open()
    self._pre_read()
    with open(self._directory + 'mode', 'r') as f:
      self._currentMode = f.read()

  def _pre_open(self):
    for key in self._PRE_OPENS:
      pre_open = self._PRE_OPENS[key]
      mode = pre_open[0]
      filepath = self._directory + pre_open[1]
      if USE_OS_OPEN:
        self._fd[pre_open[0]] = os.open(filepath, mode)
      else:
        self._fd[pre_open[0]] = open(filepath, mode)

  def _pre_read(self):
    for pre_read in self._PRE_READS:
      try:
        with open(self._directory + pre_read[0], 'r') as f:
          setattr(self, pre_read[0], pre_read[1](f.read().rstrip()))
      except:
        pass

  def seek_to_start(self, file_key):
    if USE_OS_OPEN:
      os.lseek(self._fd[file_key], 0, os.SEEK_SET)
    else:
      self._fd[file_key].close()
      mode = self._PRE_OPENS[file_key][0]
      filepath = self._directory + self._PRE_OPENS[file_key][1]
      self._fd[file_key] = open(filepath, mode)


  def bin_data(self, fmt=None):
    value_size = self._BYTES_FMT[self.bin_data_format][0]
    num_bytes = self.num_values * value_size
    if USE_OS_OPEN:
      data = os.read(self._fd['bin_data'], num_bytes)
    else:
      data = self._fd['bin_data'].read(num_bytes)
    if fmt:
      return struct.unpack(fmt, data)
    else:
      return bytearray(data)

  @property
  def bin_data_format(self):
    if self._bin_data_format_mode != self._currentMode:
      self.seek_to_start('bin_data_format')

      if USE_OS_OPEN:
        self._bin_data_format = os.read(self._fd['bin_data_format'], 100).decode('utf-8').rstrip()
      else:
        self._bin_data_format = self._fd['bin_data_format'].read(100).decode('utf-8').rstrip()
      self._bin_data_format_mode = self._currentMode
    return self._bin_data_format

  @property
  def command(self):
    raise Exception("command is a write-only property!")

  @command.setter
  def command(self, value):
    if USE_OS_OPEN:
      os.write(self._fd['command'], str(value).encode('ascii'))
    else:
      self._fd['command'].write(str(value).encode('ascii'))
    return 0

  @property
  def commands(self):
    with open(self._directory + 'commands', 'r') as f:
      return f.read().rstrip()

  @property
  def decimals(self):
    if self._decimals_mode != self._currentMode:
      self.seek_to_start('decimals')
      if USE_OS_OPEN:
        self._decimals = int(os.read(self._fd['decimals'], 10).decode('utf-8').rstrip())
      else:
        self._decimals = int(self._fd['decimals'].read(10).decode('utf-8').rstrip())
      self._decimals_mode = self._currentMode
    return self._decimals

  @property
  def mode(self):
    return self._currentMode.rstrip()

  @mode.setter
  def mode(self, mode):
    if (self._currentMode != mode):
      if USE_OS_OPEN:
        os.write(self._fd['mode'], str(mode).encode('ascii'))
      else:
        self._fd['mode'].write(str(mode).encode('ascii'))
      self._currentMode = mode
    return 0

  @property
  def num_values(self):
    if self._num_values_mode != self._currentMode:
      self.seek_to_start('num_values')
      if USE_OS_OPEN:
        self._num_values = int(os.read(self._fd['num_values'], 10).decode('utf-8').rstrip())
      else:
        self._num_values = int(self._fd['num_values'].read(10).decode('utf-8').rstrip())
      self._num_values_mode = self._currentMode
    return self._num_values

  @property
  def units(self):
    if self._units_mode != self._currentMode:
      self.seek_to_start('units')
      if USE_OS_OPEN:
        self._units = str(os.read(self._fd['units'], 10).decode('utf-8').rstrip())
      else:
        self._units = str(self._fd['units'].read(10).decode('utf-8').rstrip())
      self._units_mode = self._currentMode
    return self._units

  def value(self, n=0):
    fmt = self._BYTES_FMT[self.bin_data_format]
    self.seek_to_start('bin_data')
    startByte = n * fmt[0]
    endByte = startByte + fmt[0]
    if USE_OS_OPEN:
      data = os.read(self._fd['bin_data'], endByte)
    else:
      data = self._fd['bin_data'].read(endByte)
    return struct.unpack(fmt[1], data[startByte:endByte])[0]


class TouchSensor(Sensor):
  _DRIVER_NAME = 'lego-ev3-touch'

  MODE_TOUCH = 'TOUCH'

  def __init__(self, address=None):
    self._PRE_OPENS['value0'] = [ os.O_RDONLY ]
    super().__init__(address)

  @property
  def is_pressed(self):
    self.seek_to_start('value0')
    if USE_OS_OPEN:
      result = os.read(self._fd['value0'], 1)
    else:
      result = self._fd['value0'].read(1)
    if (result == '1'):
      return True
    else:
      return False

  @property
  def is_released(self):
    return not self.is_pressed

  def _wait(self, desired_state, timeout_ms, sleep_ms):
    tic = time.time()

    if sleep_ms:
      sleep_ms = float(sleep_ms/1000)

    while True:
      if self.is_pressed == desired_state:
        return True

      if timeout_ms is not None and time.time() >= tic + timeout_ms/1000:
        return False

      if sleep_ms:
        time.sleep(sleep_ms)

  def wait_for_pressed(self, timeout_ms=None, sleep_ms=10):
    return self._wait(True, timeout_ms, sleep_ms)

  def wait_for_released(self, timeout_ms=None, sleep_ms=10):
    return self._wait(False, timeout_ms, sleep_ms)

  def wait_for_bump(self, timeout_ms=None, sleep_ms=10):
    start_time = time.time()

    if self.wait_for_pressed(timeout_ms, sleep_ms):
      if timeout_ms is not None:
        timeout_ms -= int((time.time()-start_time) * 1000)
      return self.wait_for_pressed(timeout_ms, sleep_ms)
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

  COLORS = (
      'NoColor',
      'Black',
      'Blue',
      'Green',
      'Yellow',
      'Red',
      'White',
      'Brown',
    )

  MODE_COL_REFLECT = 'COL-REFLECT'
  MODE_COL_AMBIENT = 'COL-AMBIENT'
  MODE_COL_COLOR = 'COL-COLOR'
  MODE_REF_RAW = 'REF-RAW'
  MODE_RGB_RAW = 'RGB-RAW'

  def __init__(self, address=None):
    self.red_max = 300
    self.green_max = 300
    self.blue_max = 300
    super().__init__(address)

  @property
  def reflected_light_intensity(self):
    if (self._currentMode != self.MODE_COL_REFLECT):
      self.mode = self.MODE_COL_REFLECT
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 1)
    return struct.unpack('<b', data)[0]

  @property
  def ambient_light_intensity(self):
    if (self._currentMode != self.MODE_COL_AMBIENT):
      self.mode = self.MODE_COL_AMBIENT
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

  @property
  def color_name(self):
    return self.COLORS[self.color]

  @property
  def raw(self):
    if (self._currentMode != self.MODE_RGB_RAW):
      self.mode = self.MODE_RGB_RAW
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 6)
    return struct.unpack('<hhh', data)

  @property
  def calibrate_white(self):
    (self.red_max, self.green_max, self.blue_max) = self.raw

  @property
  def rgb(self):
    (red, green, blue) = self. raw
    return (
      int(red * 255 / self.red_max),
      int(green * 255, self.green_max),
      int(blue * 255 / self.blue_max)
    )

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
  MODE_TILT_ANG = 'TILT-ANGLE'
  MODE_TILT_RATE = 'TILT-RATE'

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
  def angle_and_rate(self):
    if (self._currentMode != self.MODE_GYRO_G_A):
      self.mode = self.MODE_GYRO_G_A
    os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
    data = os.read(self._fd['bin_data'], 4)
    return struct.unpack('<hh', data)


class SpeedValue:
  """
  The base class for the SpeedValue classes.
  Not meant to be used directly.
  Use SpeedNativeUnits, SpeedPercent, SpeedRPS, SpeedRPM, SpeedDPS, SpeedDPM
  """

  def __lt__(self, other):
    return self.to_native_units() < other.to_native_units()

  def __rmul__(self,other):
    return self.__mul__(other)

  def to_native_units(self):
    pass

  def __mul__(self, other):
    pass


class SpeedPercent(SpeedValue):
  """
  Speed as a percentage of the motor's maximum rated speed.
  Returns Tacho Counts via motor.max_speed
  """
  def __init__(self, percent):
    if -100 <= percent <= 100:
      self.percent = percent
    else:
      raise ValueError("Value must be between -100 and 100")

  def __str__(self):
    return str(self.percent) + '%'

  def __mul__(self,other):
    if isinstance(other, (float, int)):
      return SpeedPercent(self.percent * other)
    else:
      raise TypeError("Multiplier must be of int or float type.")

  def to_native_units(self, motor):
    return self.percent / 100 * motor.max_speed


class SpeedNativeUnits(SpeedValue):
  """
  Speed in tacho counts per second
  Returns no. of Tacho Counts
  """
  def __init__(self,native_counts):
    self.native_counts = native_counts

  def __str__(self):
    return str(self.native_counts) + "counts/sec"

  def __mul__(self, other):
    if isinstance(other, (float, int)):
      SpeedNativeUnits(self.native_counts * other)
    else:
      raise TypeError("Multiplier must be of int or float type.")

  def to_native_units(self,motor):
    return self.native_counts


class SpeedRPS(SpeedValue):
  """
  Speed in rotations-per-second
  Returns Tacho Counts via motor.max_rps
  """
  def __init__(self, rotations_per_second):
    self.rotations_per_second = rotations_per_second

  def __str__(self):
    return str(self.rotations_per_second) + " rot/sec"

  def __mul__(self, other):
    if isinstance(other, (float, int)):
      return SpeedRPS(self.rotations_per_second * other)
    else:
      raise TypeError("Multiplier must be of float or int type")

  def to_native_units(self, motor):
    if abs(self.rotations_per_second) <= motor.max_rps:
      return self.rotations_per_second/motor.max_rps * motor.max_speed
    else:
      raise ValueError("RPS value must be <= motor.max_rps")


class SpeedRPM(SpeedValue):
  """
  Speed in rotations-per-minute
  Returns RPM to Tacho Counts via motor.max_rpm
  """

  def __init__(self, rotations_per_minute):
    self.rotations_per_minute = rotations_per_minute

  def __str__(self):
    return str(self.rotations_per_minute) + " rot/min"

  def __mul__(self, other):
    if isinstance(other, (float, int)):
      return SpeedRPM(self.rotations_per_minute * other)
    else:
      raise TypeError("Multiplier must be of type float or int")

  def to_native_units(self, motor):
    if abs(self.rotations_per_minute) <= motor.max_rpm:
      return self.rotations_per_minute/motor.max_rpm * motor.max_speed
    else:
      raise ValueError("RPM Value must be <= motor.max_rpm")


class SpeedDPS(SpeedValue):
  """
  Speed in degrees-per-second.
  Converts to Tacho Counts via motor.max_dps
  """

  def __init__(self, degrees_per_second):
    self.degrees_per_second = degrees_per_second

  def __str__(self):
    return str(self.degrees_per_second) + " deg/sec"

  def __mul__(self, other):
    if isinstance(other, (float, int)):
      return SpeedDPS(self.degrees_per_second * other)
    else:
      raise TypeError("Multiplier must be of type float or int")

  def to_native_units(self, motor):
    if abs(self.degrees_per_second) <= motor.max_dps:
      return self.degrees_per_second/motor.max_dps * motor.max_speed
    else:
      raise ValueError("DPS Value must be <= motor.max_dps")


class SpeedDPM(SpeedValue):
  """
  Speed in degrees-per-min
  Returns Tacho Counts via motor.max_dpm
  """

  def __init__(self, degrees_per_minute):
    self.degrees_per_minute = degrees_per_minute

  def __str__(self):
    return str(self.degrees_per_minute) + " deg/min"

  def __mul__(self, other):
    if isinstance(other, (float, int)):
      return SpeedDPM(self.degrees_per_minute * other)
    else:
      raise TypeError("Multiplier must be of type float or int")

  def to_native_units(self, motor):
    if abs(self.degrees_per_minute) <= motor.max_dpm:
      return self.degrees_per_minute/motor.max_dpm * motor.max_speed
    else:
      raise ValueError("DPM Value must be <= motor.max_dpm")


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

  speed_sp_table = []
  for i in range(0, 1561):
    speed_sp_table.append(str(i).encode())

  def __init__(self, address=None):
    self._fd = {}

    for i in range(0,10):
      directory = self._DIRECTORY_BASE + str(i) + '/'
      if address:
        try:
          with open(directory + 'address', 'r') as f:
            if address in f.read():
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

    self.max_rps = float(self.max_speed/self.count_per_rot)
    self.max_rpm = self.max_rps * 60
    self.max_dps = self.max_rps * 360
    self.max_dpm = self.max_rpm * 360

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
    os.write(self._fd['command'], value.encode('ascii'))
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
    os.write(self._fd['duty_cycle_sp'], str(int(value)).encode('ascii'))
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
    os.write(self._fd['polarity'], value.encode('ascii'))
    return 0

  @property
  def position(self):
    os.lseek(self._fd['position'], 0, os.SEEK_SET)
    return int(os.read(self._fd['position'], 100))

  @position.setter
  def position(self, value):
    os.write(self._fd['position'], str(int(value)).encode('ascii'))
    return 0

  @property
  def position_d(self):
    os.lseek(self._fd['position_d'], 0, os.SEEK_SET)
    return int(os.read(self._fd['position_d'], 100))

  @position_d.setter
  def position_d(self, value):
    os.write(self._fd['position_d'], str(int(value)).encode('ascii'))
    return 0

  @property
  def position_i(self):
    os.lseek(self._fd['position_i'], 0, os.SEEK_SET)
    return int(os.read(self._fd['position_i'], 100))

  @position_i.setter
  def position_i(self, value):
    os.write(self._fd['position_i'], str(int(value)).encode('ascii'))
    return 0

  @property
  def position_p(self):
    os.lseek(self._fd['position_p'], 0, os.SEEK_SET)
    return int(os.read(self._fd['position_p'], 100))

  @position_p.setter
  def position_p(self, value):
    os.write(self._fd['position_p'], str(int(value)).encode('ascii'))
    return 0

  @property
  def position_sp(self):
    os.lseek(self._fd['position_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['position_sp'], 100))

  @position_sp.setter
  def position_sp(self, value):
    os.write(self._fd['position_sp'], str(int(value)).encode('ascii'))
    return 0

  @property
  def ramp_down_sp(self):
    os.lseek(self._fd['ramp_down_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['ramp_down_sp'], 100))

  @ramp_down_sp.setter
  def ramp_down_sp(self, value):
    os.write(self._fd['ramp_down_sp'], str(int(value)).encode('ascii'))
    return 0

  @property
  def ramp_up_sp(self):
    os.lseek(self._fd['ramp_up_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['ramp_up_sp'], 100))

  @ramp_up_sp.setter
  def ramp_up_sp(self, value):
    os.write(self._fd['ramp_up_sp'], str(int(value)).encode('ascii'))
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
    os.write(self._fd['speed_d'], str(int(value)).encode('ascii'))
    return 0

  @property
  def speed_i(self):
    os.lseek(self._fd['speed_i'], 0, os.SEEK_SET)
    return int(os.read(self._fd['speed_i'], 100))

  @speed_i.setter
  def speed_i(self, value):
    os.write(self._fd['speed_i'], str(int(value)).encode('ascii'))
    return 0

  @property
  def speed_p(self):
    os.lseek(self._fd['speed_p'], 0, os.SEEK_SET)
    return int(os.read(self._fd['speed_p'], 100))

  @speed_p.setter
  def speed_p(self, value):
    os.write(self._fd['speed_p'], str(int(value)).encode('ascii'))
    return 0

  @property
  def speed_sp(self):
    os.lseek(self._fd['speed_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['speed_sp'], 100))

  @speed_sp.setter
  def speed_sp(self, value):
    os.write(self._fd['speed_sp'], self.speed_sp_table[value])
    return 0

  @property
  def state(self):
    os.lseek(self._fd['state'], 0, os.SEEK_SET)
    return str(os.read(self._fd['state'], 1024))

  @property
  def stop_action(self):
    """
    Reading returns the current stop action.
    Use stop_actions for the list of possible values.
    """
    os.lseek(self._fd['stop_action'], 0, os.SEEK_SET)
    return str(os.read(self._fd['stop_action'], 100))

  @stop_action.setter
  def stop_action(self, value):
    os.write(self._fd['stop_action'], value.encode('ascii'))
    return 0

  @property
  def time_sp(self):
    os.lseek(self._fd['time_sp'], 0, os.SEEK_SET)
    return int(os.read(self._fd['time_sp'], 100))

  @time_sp.setter
  def time_sp(self, value):
    os.write(self._fd['time_sp'], str(int(value)).encode('ascii'))
    return 0

  def reset(self, **kwargs):
    """
    Resets the motor the default value. It will also stop the motor.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'reset\n')

  def run_direct(self, **kwargs):
    """
    Run the motor at the duty cycle specified by duty_cycle_sp.
    Unlike other run commands, changing duty_cycle_sp
    while running will take effect immediately.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'run-direct\n')

  def run_forever(self, **kwargs):
    """
    Run the motor until another command is sent.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'run-forever\n')

  def run_timed(self, **kwargs):
    """
    Run for the amount of time specified in time_sp.
    Then, stop the motor as specified by stop_action.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'run-timed\n')

  def run_to_abs_pos(self, **kwargs):
    """
    Run to the absolute position as specified by position_sp.
    Then, stop the motor as specified by stop_action.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'run-to-abs-pos\n')

  def run_to_rel_pos(self, **kwargs):
    """
    Run to the relative position as specified by position_sp.
    New position will be current position + position_sp
    When the new position is reached, the motor will stop, as specified
    by stop_action.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'run-to-rel-pos\n')

  def stop(self, **kwargs):
    """
    Stop any of the run commands before they are complete using the
    action specified by stop_action.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    os.write(self._fd['command'], b'stop\n')

  def wait(self, cond, timeout=None):
    """
    Blocks until ``cond(self.state)`` is ``True``.  The condition is
    checked when there is an I/O event related to the ``state`` attribute.
    Exits early when ``timeout`` (in milliseconds) is reached.

    Returns ``True`` if the condition is met, and ``False`` if the timeout
    is reached.

    Valid flags for state attribute: running, ramping, holding,
    overloaded and stalled
    """
    poll = select.poll()
    poll.register(self._fd['state'], select.POLLIN)

    while True:
      event = poll.poll(timeout)

      if len(event) == 0:
        return False

      if cond(self.state):
        return True

  def wait_until(self, s, timeout=None):
    """
    Blocks until ``s`` is in ``self.state``.  The condition is checked when
    there is an I/O event related to the ``state`` attribute.  Exits early
    when ``timeout`` (in milliseconds) is reached.

    Returns ``True`` if the condition is met, and ``False`` if the timeout
    is reached.

    Example::
        m.wait_until('stalled')
    """
    return self.wait(lambda state: s in state, timeout)

  def wait_until_not_moving(self, timeout=None):
    """
    Blocks until one of the following conditions are met:
    - ``running`` is not in ``self.state``
    - ``stalled`` is in ``self.state``
    - ``holding`` is in ``self.state``
    The condition is checked when there is an I/O event related to
    the ``state`` attribute.  Exits early when ``timeout`` (in
    milliseconds) is reached.

    Returns ``True`` if the condition is met, and ``False`` if the timeout
    is reached.

    Example::

        m.wait_until_not_moving()
    """
    return self.wait(lambda state: self.STATE_RUNNING not in state or self.STATE_STALLED in state, timeout)

  def wait_while(self, s, timeout=None):
    """
    Blocks until ``s`` is not in ``self.state``.  The condition is checked
    when there is an I/O event related to the ``state`` attribute.  Exits
    early when ``timeout`` (in milliseconds) is reached.

    Returns ``True`` if the condition is met, and ``False`` if the timeout
    is reached.

    Example::

        m.wait_while('running')
    """
    return self.wait(lambda state: s not in state, timeout)

  def _set_rel_position_degrees_and_speed_sp(self, degrees, speed):
    degrees = degrees if speed >= 0 else -degrees
    speed = abs(speed)

    position_delta = int(round((degrees * self.count_per_rot)/360))
    speed_sp = int(round(speed))

    self.position_sp = position_delta
    self.speed_sp = speed_sp

  def on_for_rotations(self, speed, rotations, brake=True, block=True):
    """
    Rotate the motor at ``speed`` for ``rotations``

    ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
    object, enabling use of other units.
    """
    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed = SpeedPercent(speed)
        speed_sp = speed.to_native_units(self)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_sp = int(round(speed.to_native_units(self)))

    self._set_rel_position_degrees_and_speed_sp(rotations*360, speed_sp)

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.run_to_rel_pos()

    if block:
      self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
      self.wait_until_not_moving()

  def on_for_degrees(self, speed, degrees, brake=True, block=True):
    """
    Rotate the motor at ``speed`` for ``degrees``

    ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
    object, enabling use of other units.
    """
    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed = SpeedPercent(speed)
        speed_sp = speed.to_native_units(self)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_sp = int(round(speed.to_native_units(self)))

    self._set_rel_position_degrees_and_speed_sp(degrees, speed_sp)

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.run_to_rel_pos()

    if block:
      self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
      self.wait_until_not_moving()

  def on_to_position(self, speed, position, brake=True, block=True):
    """
    Rotate the motor at ``speed`` to ``position``

    ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
    object, enabling use of other units.
    """
    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed = SpeedPercent(speed)
        speed_sp = speed.to_native_units(self)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_sp = int(round(speed.to_native_units(self)))

    self.speed_sp = int(round(speed_sp))
    self.position_sp = position

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.run_to_abs_pos()

    if block:
      self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
      self.wait_until_not_moving()

  def on_for_seconds(self, speed, seconds, brake=True, block=True):
    """
    Rotate the motor at ``speed`` for ``seconds``

    ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
    object, enabling use of other units.
    """
    if seconds < 0:
      raise ValueError("Seconds is negative.")

    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed = SpeedPercent(speed)
        speed_sp = speed.to_native_units(self)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_sp = int(round(speed.to_native_units(self)))

    self.speed_sp = int(round(speed_sp))
    self.time_sp = int(seconds * 1000)

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.run_timed()

    if block:
      self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
      self.wait_until_not_moving()

  def on(self, speed, brake=True, block=False):
    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed = SpeedPercent(speed)
        speed_sp = speed.to_native_units(self)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_sp = int(round(speed.to_native_units(self)))

    self.speed_sp = int(round(speed_sp))

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.run_forever()

    if block:
      self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
      self.wait_until_not_moving()

  def off(self, brake=True):

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.stop()


class LargeMotor(Motor):
  _DRIVER_NAME = 'lego-ev3-l-motor'


class MediumMotor(Motor):
  _DRIVER_NAME = 'lego-ev3-m-motor'


class MotorSet:
  def __init__(self, motor_specs, desc=None):
    self.motors = OrderedDict()
    for motor_port in sorted(motor_specs.keys()):
      motor_class = motor_specs[motor_port]
      self.motors[motor_port] = motor_class(motor_port)
      self.motors[motor_port].reset()
    self.desc = desc
    self.WAIT_RUNNING_TIMEOUT = 100

  def __str__(self):
    if self.desc:
      return self.desc
    else:
      return self.__class__.__name__

  def reset(self, **kwargs):
    for motor in self.motors.values():
      for k in kwargs:
        setattr(self, k, kwargs[k])
      os.write(motor._fd['command'], b'reset\n')

  def run_forever(self, **kwargs):
    for motor in self.motors.values():
      for k in kwargs:
        setattr(motor, k, kwargs[k])
      os.write(motor._fd['command'], b'run-forever\n')

  def run_to_abs_pos(self, **kwargs):
    for motor in self.motors.values():
      for k in kwargs:
        setattr(motor, k, kwargs[k])
      os.write(motor._fd['command'], b'run-to-abs-pos\n')

  def run_to_rel_pos(self, **kwargs):
    for motor in self.motors.values():
      for k in kwargs:
        setattr(motor, k, kwargs[k])
      os.write(motor._fd['command'], b'run-to-rel-pos\n')

  def run_timed(self, **kwargs):
    for motor in self.motors.values():
      for k in kwargs:
        setattr(motor, k, kwargs[k])
      os.write(motor._fd['command'], b'run-timed\n')

  def run_direct(self, **kwargs):
    for motor in self.motors.values():
      for k in kwargs:
        setattr(motor, k, kwargs[k])
      os.write(motor._fd['command'], b'run-direct\n')

  def off(self, motors=None, brake=True):
    motors = motors if motors is not None else self.motors.values()

    for motor in motors:
      motor.stop_action = motor.STOP_ACTION_HOLD if brake else motor.STOP_ACTION_COAST

    for motor in motors:
      motor.stop()

  def stop(self, motors=None, brake=True):
    self.off(motors, brake)

  @property
  def is_running(self, motors=None, state=Motor.STATE_RUNNING):
    motors = motors if motors is not None else self.motors.values()
    for motor in motors:
      if state not in motor.state:
        return False
    return True

  @property
  def is_ramping(self, motors=None, state=Motor.STATE_RAMPING):
    motors = motors if motors is not None else self.motors.values()
    for motor in motors:
      if state not in motor.state:
        return False
    return True

  @property
  def is_holding(self, motors=None, state=Motor.STATE_HOLDING):
    motors = motors if motors is not None else self.motors.values()
    for motor in motors:
      if state not in motor.state:
        return False
    return True

  @property
  def is_overloaded(self, motors=None, state=Motor.STATE_OVERLOADED):
    motors = motors if motors is not None else self.motors.values()
    for motor in motors:
      if state not in motor.state:
        return False
    return True

  @property
  def is_stalled(self, motors=None, state=Motor.STATE_STALLED):
    motors = motors if motors is not None else self.motors.values()
    for motor in motors:
      if state not in motor.state:
        return False
    return True

  def wait(self, cond, timeout=None, motors=None):
    motors = motors if motors is not None else self.motors.values()

    for motor in motors:
      motor.wait(cond, timeout)

  def wait_until_not_moving(self, timeout=None, motors=None):
    motors = motors if motors is not None else self.motors.values()

    for motor in motors:
      motor.wait_until_not_moving(timeout)

  def wait_until(self, s, timeout=None, motors=None):
    motors = motors if motors is not None else self.motors.values()

    for motor in motors:
      motor.wait_until(s, timeout)

  def wait_while(self, s, timeout=None, motors=None):
    motors = motors if motors is not None else self.motors.values()

    for motor in motors:
      motor.wait_while(s, timeout)

  def _block(self):
    self.wait_until('running', timeout=self.WAIT_RUNNING_TIMEOUT)
    self.wait_until_not_moving()


class MoveTank(MotorSet):

  def __init__(self, left_motor_port, right_motor_port, desc=None, motor_class=LargeMotor):
    motor_specs = {
      left_motor_port : motor_class,
      right_motor_port : motor_class,
      }
    MotorSet.__init__(self, motor_specs, desc)
    self.left_motor = self.motors[left_motor_port]
    self.right_motor = self.motors[right_motor_port]
    self.max_speed = self.left_motor.max_speed

  def on_for_degrees(self, left_speed, right_speed, degrees, brake=True, block=True):
    """
    Rotate the motors at 'left_speed' and 'right_speed' for 'degrees'.
    Speeds can be percentages or any SpeedValue implementation.

    If the left speed is not equal to the right speed (i.e., the robot will
    turn), the motor on the outside of the turn will rotate for the full
    ``degrees`` while the motor on the inside will have its requested
    distance calculated according to the expected turn.
    """
    if not isinstance(left_speed, SpeedValue):
      if -100 <= left_speed <= 100:
        left_speed_obj = SpeedPercent(left_speed)
        left_speed_var = int(round(left_speed_obj.to_native_units(self.left_motor)))
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      left_speed_var = int(round(left_speed.to_native_units(self.left_motor)))

    if not isinstance(right_speed, SpeedValue):
      if -100 <= right_speed <= 100:
        right_speed_obj = SpeedPercent(right_speed)
        right_speed_var = int(round(right_speed_obj.to_native_units(self.right_motor)))
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      right_speed_var = int(round(right_speed.to_native_units(self.right_motor)))

    if degrees == 0 or (left_speed_var == 0 and right_speed_var == 0):
      left_degrees = 0
      right_degrees = 0
    elif abs(left_speed_var) > abs(right_speed_var):
      left_degrees = degrees
      right_degrees = abs(right_speed_var / left_speed_var) * degrees
    else:
      left_degrees = abs(left_speed_var / right_speed_var) * degrees
      right_degrees = degrees

    left_degrees_in = round((left_degrees * self.left_motor.count_per_rot)/360)
    right_degrees_in = round((right_degrees * self.right_motor.count_per_rot)/360)

    self.left_motor.position_sp = left_degrees_in if left_speed_var >= 0 else -left_degrees_in
    self.left_motor.speed_sp = abs(left_speed_var)
    self.right_motor.position_sp = right_degrees_in if right_speed_var >= 0 else -right_degrees_in
    self.right_motor.speed_sp = abs(right_speed_var)

    self.left_motor.stop_action = self.left_motor.STOP_ACTION_HOLD if brake else self.left_motor.STOP_ACTION_COAST
    self.right_motor.stop_action = self.right_motor.STOP_ACTION_HOLD if brake else self.right_motor.STOP_ACTION_COAST

    self.left_motor.run_to_rel_pos()
    self.right_motor.run_to_rel_pos()

    if block:
      self._block()

  def on_for_rotations(self, left_speed, right_speed, rotations, brake=True, block=True):
    """
    Rotate the motors at 'left_speed' and 'right_speed' for 'rotations'.
    Speeds can be percentages or any SpeedValue implementation.

    If the left speed is not equal to the right speed (i.e., the robot will
    turn), the motor on the outside of the turn will rotate for the full
    ``rotations`` while the motor on the inside will have its requested
    distance calculated according to the expected turn.
    """
    MoveTank.on_for_degrees(self, left_speed, right_speed, rotations * 360, brake, block)

  def on_for_seconds(self, left_speed, right_speed, seconds, brake=True, block=True):
    """
    Rotate the motors at 'left_speed & right_speed' for 'seconds'.
    Speeds can be percentages or any SpeedValue implementation.
    """
    if seconds < 0:
      raise ValueError("Seconds is negative.")

    if not isinstance(left_speed, SpeedValue):
      if -100 <= left_speed <= 100:
        left_speed_obj = SpeedPercent(left_speed)
        left_speed_var = int(round(left_speed_obj.to_native_units(self.left_motor)))
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      left_speed_var = int(round(left_speed.to_native_units(self.left_motor)))


    if not isinstance(right_speed, SpeedValue):
      if -100 <= right_speed <= 100:
        right_speed_obj = SpeedPercent(right_speed)
        right_speed_var = int(round(right_speed_obj.to_native_units(self.right_motor)))
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      right_speed_var = int(round(right_speed.to_native_units(self.right_motor)))

    self.left_motor.speed_sp = left_speed_var
    self.left_motor.time_sp = seconds * 1000
    self.right_motor.speed_sp = right_speed_var
    self.right_motor.time_sp = seconds * 1000

    self.left_motor.stop_action = self.left_motor.STOP_ACTION_HOLD if brake else self.left_motor.STOP_ACTION_COAST
    self.right_motor.stop_action = self.right_motor.STOP_ACTION_HOLD if brake else self.right_motor.STOP_ACTION_COAST

    self.left_motor.run_timed()
    self.right_motor.run_timed()

    if block:
      self._block()

  def on(self, left_speed, right_speed):
    """
    Start rotating the motors according to ``left_speed`` and ``right_speed`` forever.
    Speeds can be percentages or any SpeedValue implementation.

    """
    if not isinstance(left_speed, SpeedValue):
      if -100 <= left_speed <= 100:
        left_speed_obj = SpeedPercent(left_speed)
        left_speed_var = int(round(left_speed_obj.to_native_units(self.left_motor)))
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      left_speed_var = int(round(left_speed.to_native_units(self.left_motor)))

    if not isinstance(right_speed, SpeedValue):
      if -100 <= right_speed <= 100:
        right_speed_obj = SpeedPercent(right_speed)
        right_speed_var = int(round(right_speed_obj.to_native_units(self.right_motor)))
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      right_speed_var = int(round(right_speed.to_native_units(self.right_motor)))

    self.left_motor.speed_sp = left_speed_var
    self.right_motor.speed_sp = right_speed_var
    self.left_motor.run_forever()
    self.right_motor.run_forever()


class MoveSteering(MoveTank):

  def get_speed_steering(self, steering, speed):
    if steering > 100 or steering < -100:
      raise ValueError("Invalid Steering Value. Between -100 and 100 (inclusive).")

    # Assumes left motor's speed stats are the same as the right motor's
    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed_obj = SpeedPercent(speed)
        speed_var = speed_obj.to_native_units(self.left_motor)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_var = speed.to_native_units(self.left_motor)

    left_speed = speed_var
    right_speed = speed_var
    speed_factor = (50 - abs(float(steering))) / 50

    if steering >= 0:
      right_speed *= speed_factor
    else:
      left_speed *= speed_factor

    return(left_speed, right_speed)

  def on_for_rotations(self, steering, speed, rotations, brake=True, block=True):
    """
    Rotate the motors according to the provided ``steering``.

    The distance each motor will travel follows the rules of :meth:`MoveTank.on_for_rotations`.
    """
    (left_speed, right_speed) = self.get_speed_steering(steering, speed)
    MoveTank.on_for_rotations(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed), rotations, brake, block)

  def on_for_degrees(self, steering, speed, degrees, brake=True, block=True):
    """
    Rotate the motors according to the provided ``steering``.

    The distance each motor will travel follows the rules of :meth:`MoveTank.on_for_degrees`.
    """
    (left_speed, right_speed) = self.get_speed_steering(steering, speed)
    MoveTank.on_for_degrees(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed), degrees, brake, block)

  def on_for_seconds(self, steering, speed, seconds, brake=True, block=True):
    """
    Rotate the motors according to the provided ``steering`` for ``seconds``.
    """
    (left_speed, right_speed) = self.get_speed_steering(steering, speed)
    MoveTank.on_for_seconds(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed), seconds, brake, block)

  def on(self, steering, speed):
    """
    Start rotating the motors according to the provided ``steering`` and
    ``speed`` forever.
    """
    (left_speed, right_speed) = self.get_speed_steering(steering, speed)
    MoveTank.on(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed))