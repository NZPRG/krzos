#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-26
# modified: 2024-10-26
#

from pmw3901 import BG_CS_FRONT_BCM, PAA5100

from core.component import Component
from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class NearOpticalFlowSensor(Component):
    '''
    A wrapper for the PAA5100JE sensor, that uses a low-resolution camera and
    some clever algorithms to detect motion of surfaces. This sensor has a
    range of 15-35mm, a frame rate of 242 FPS, FoV of 42°, and a maximum speed
    of 1.14 metres per second (with sensor 25mm away from surface).

    In configuration there are X and Y trim values as multipliers on the values
    read by the sensor.

    Connections:

      * 3-5V to any 5V or 3V pin
      * CS to BCM 7
      * SCK to BCM 11
      * MOSI to BCM 10
      * MISO to BCM 9
      * INT to BCM 19
      * GND to any ground pin

    Values are kept internally as floats, returned as ints.

    :param config:          the application configuration
    :param level            the log level
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger('ofs', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising paa5100je…')
        _cfg = config['mros'].get('hardware').get('paa5100je')
        _rotation    = _cfg.get('rotation') # options: 0, 90, 180, 270; rotation of sensor in degrees
        _slot        = BG_CS_FRONT_BCM # front SPI slot
        # the class for the specified breakout (PWM3901 or PAA5100)
        SensorClass  = PAA5100
        self._nofs   = SensorClass(spi_port=0, spi_cs=_slot)
        self._nofs.set_rotation(_rotation)
        self._x_trim = _cfg.get('x_trim') # percentage X trim as a multiplier
        self._y_trim = _cfg.get('y_trim') # percentage Y trim as a multiplier
        self._x      = 0.0
        self._y      = 0.0
        self._tx     = 0.0
        self._ty     = 0.0
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _poll(self):
        self._x, self._y = self._nofs.get_motion()
        self._x *= self._x_trim
        self._y *= self._y_trim
        self._tx += self._x
        self._ty += self._y

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def relative(self):
        '''
        Return the relative positional change since the previous poll.
        '''
        self._poll()
        return self._x, self._y

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def absolute(self):
        '''
        Return the absolute (cumulative) positional change since the
        previous poll.
        '''
        self._poll()
        return int(self._tx), int(self._ty)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def y_variance(self):
        return 0 if self._tx == 0 else int( self._ty / self._tx * 100.0 )

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        self._tx = 0.0
        self._ty = 0.0

#EOF
