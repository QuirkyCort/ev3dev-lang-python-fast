Python language bindings for ev3dev
===================================

A high performance drop-in replacement for ev3dev-lang-python_.  It does not
support all features (eg.  onboard buttons and screen), but it can be loaded
concurrently with the ev3dev-lang-python.  Use ev3fast for your performance
critical loops, and ev3dev-lang-python for everything else.  As an added
bonus, your Python program will load significantly faster if you use only
ev3dev-lang-python-fast, and do not import ev3dev-lang-python.

If you are not familiar with ev3dev-lang-python, you should start there first.

Getting Started
---------------

Read the documents for ev3dev-lang-python. Whatever applies there, applies
here.

Download ev3fast.py from this repository, and put it in the same directory
as your python program. You can alternatively put it in one of the
directories along your python import path, but this is not recommended.

Performance Comparison
----------------------

**Sensor Benchmark (Loops per second)**

===================== ====== ======= ============
Sensor Type           ev3dev ev3fast Improvements
===================== ====== ======= ============
Touch                 379    1886    5x
Color (Ambient)       179    2343    13x
Color (Color)         157    2375    15x
Color (Raw RGB)       118    1859    16x
Color (Reflected)     178    2021    11x
Ultrasonic (Distance) 48     1882    40x
Ultrasonic (Listen)   60     2094    35x
Gyro (Angle)          45     2314    51x
Gyro (Rate)           231    2174    9x
Gyro (Rate & Angle)   152    2130    14x
===================== ====== ======= ============

**Line Follower Benchmark**

============================= ================
Program Type                  Loops per second
============================= ================ 
EV3-G                         427
Python on ev3dev              32
Python on ev3dev with ev3fast 205
============================= ================

The benchmark program is provided here if you want to try it out yourself.

.. code-block:: python

  from time import perf_counter
  from ev3dev.ev3 import *
  #from ev3fast import *

  lMotor = LargeMotor('outA')
  rMotor = LargeMotor('outD')
  lSensor = ColorSensor('in1')
  rSensor = ColorSensor('in4')

  LOOPS = 1000

  startTime = perf_counter()
  for a in range(0,LOOPS):
    valueL = lSensor.raw
    valueR = rSensor.raw
    totalL = (valueL[0] + valueL[1] + valueL[2])
    totalR = (valueR[0] + valueR[1] + valueR[2])
    error = totalL - totalR
    lMotor.speed_sp = 200 + error
    rMotor.speed_sp = 200 - error
    lMotor.run_forever()
    rMotor.run_forever()
  endTime = perf_counter()

  lMotor.stop()
  rMotor.stop()

  print(str(LOOPS / (endTime - startTime)))

To benchmark the ev3fast version, uncomment the line..

.. code-block:: python

  #from ev3fast import *

Supported Classes
-----------------

| class **Sensor** (address=None)
| class **TouchSensor** (address=None)
| class **ColorSensor** (address=None)
| class **UltrasonicSensor** (address=None)
| class **GyroSensor** (address=None)
| class **Motor** (address=None)
| class **LargeMotor** (address=None)
| class **MediumMotor** (address=None)

These are replacements for ev3dev-lang-python, and all properties and
methods documented in the `ev3dev-lang-python Read the Docs page`_ are
supported.  If address isn’t specified, the first device matching the class
will be loaded.

Usage Examples
--------------

Add the following to the top of your file:

.. code-block:: python

  import ev3fast

If you're loading ev3dev-lang-python concurrently, you can either import
them under their own name...

.. code-block:: python

  import ev3dev.ev3 as ev3
  import ev3fast

  slow = ev3.LargeMotor('outA')
  fast = ev3fast.LargeMotor('outB')

...or override ev3dev-lang-python...

.. code-block:: python

  import ev3dev.ev3 as ev3
  import ev3fast as ev3

  fast = ev3.LargeMotor('outA')

User Resources
--------------

Library Documentation
    **Class documentation for this library can be found on the**
    `ev3dev-lang-python Read the Docs page`_ **.** If it's a supported class
    in ev3dev-lang-python-fast, it should (...probably) work identically to
    ev3dev-lang-python.


.. _ev3dev: http://ev3dev.org
.. _ev3dev-lang-python: https://github.com/ev3dev/ev3dev-lang-python
.. _ev3dev-lang-python Read the Docs page: http://python-ev3dev.readthedocs.org/en/stable/
