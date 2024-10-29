#!/usr/bin/env python3 # -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-18
# modified: 2021-06-29
#

import itertools
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component
from core.orientation import Orientation
from hardware.decoder import Decoder
from hardware.slew_limiter import SlewLimiter

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Motor(Component):
    '''
    Controls a motor that uses a Hall Effect encoder to determine the robot's
    velocity and distance traveled.

    The motors provide 69.3 steps per axle rotation. The Mecanum wheels have
    a 48mm nominal diameter:

       https://www.robotshop.com/products/48mm-steel-mecanum-wheel-set-2x-left-2x-right

    With the given wheel size, there are 6.63 rotations per meter, or
    461 steps per meter.

    :param picon_zero:  reference to the PiconZero motor controller
    :param orientation: motor orientation
    :param level:       log level
    '''
    def __init__(self, picon_zero, orientation, level=Level.INFO):
        if picon_zero is None:
            raise ValueError('null picon zero argument.')
        self._picon_zero = picon_zero
        self._orientation = orientation
        if self._orientation.side is Orientation.PORT:
            self._channel = 1
        elif self._orientation.side is Orientation.STBD:
            self._channel = 0
        else:
            raise ValueError('expected PORT or STBD side.')
        self._log = Logger('motor:{}'.format(orientation.label), level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._log.info('initialising {} motor with {} at address 0x{:02X} as motor controller…'.format(
                orientation.name, type(self._picon_zero).__name__, self._picon_zero.I2cAddress))
        # encoder/decoder ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._counter = itertools.count()
        self._steps_per_rotation = 693  # wheels
        self._steps      = 0 # step counter
        self._last_steps = 0 # last step count for direction detection
        self._step_timestamps = [] # used for calculating RPM
        self._rpm        = 0.0
        self._max_rpm    = 0.0
        self._rotations  = 0.0
        self._timestamp_limit = 10 # the maximum number of timestamps to keep
        if orientation is Orientation.PFWD:
            _encoder_a   = 16 # C1
            _encoder_b   = 26 # C2
        elif orientation is Orientation.SFWD:
            _encoder_a   = 21 # C1
            _encoder_b   = 20 # C2
        elif orientation is Orientation.PAFT:
            _encoder_a   =  5 # C1
            _encoder_b   = 12 # C2
        elif orientation is Orientation.SAFT:
            _encoder_a   = 13 # C1
            _encoder_b   =  6 # C2
        else:
            raise Exception('unexpected motor orientation.')
        self._decoder = Decoder(orientation, _encoder_a, _encoder_b, self._callback_step_count, self._log.level)
        # slew limiter ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._default_slew_rate = 90
        self._braking_scale     = 70
        self._slew_limiter = SlewLimiter(self, rate=self._default_slew_rate, level=self._log.level)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def orientation(self):
        '''
        Returns the orientation of this motor.
        '''
        return self._orientation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property   
    def rpm(self):
        return self._rpm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property   
    def max_rpm(self):
        return self._max_rpm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property   
    def decoder(self):
        return self._decoder
        
    @decoder.setter
    def decoder(self, decoder):
        self._decoder = decoder 

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        '''
        Resets the step counters and other variables related to historical
        movement, as well as resetting the slew limiter to its default rate.
        '''
        self._slew_limiter.reset()
        # as well as other detritus
        self._steps      = 0 # step counter
        self._last_steps = 0 # last step count for direction detection
        self._step_timestamps.clear()
        self._rpm        = 0.0
        self._max_rpm    = 0.0
        self._rotations  = 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps(self):
        return self._steps

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps_per_rotation(self):
        return self._steps_per_rotation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _callback_step_count(self, pulse):
        '''
        This callback is used to capture encoder steps, as well as
        calculate motor RPM.

        Note that this is the opposite (i.e.,adding/subtracting steps by
        orientation) than on the KR01 or MR01.
        ''' 
        if self._orientation.side is Orientation.PORT:
            self._steps = self._steps - pulse
        elif self._orientation.side is Orientation.STBD:
            self._steps = self._steps + pulse
        if self._orientation.side is Orientation.PORT:
#           self._log.info(Fore.RED   + Style.DIM + '{} motor: {:d} steps.'.format(self._orientation, self._steps))
            pass
        else:
#           self._log.info(Fore.GREEN + Style.DIM + '{} motor: {:d} steps.'.format(self._orientation, self._steps))
            pass
        self._calculate_rpm()
        _speed = self._slew_limiter()
        if _speed:
            self._picon_zero.set_motor(self._channel, _speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _calculate_rpm(self):
        '''
        Update step count and timestamps. The step count is incremented or
        decremented based on encoder input.
        '''
        # append current timestamp for each step
        self._step_timestamps.append(dt.now())
        # check if steps are counting up or down based on the last steps count
        _fore = Fore.GREEN
        if self._steps > self._last_steps:
#           self._steps += 1 # increment step count for forward
            pass
        else:
            _fore = Fore.MAGENTA
#           self._steps -= 1 # decrement step count for reverse
            pass
        self._last_steps = self._steps # Update the last step count

        if len(self._step_timestamps) > 1:
            # calculate the time intervals between consecutive steps
            intervals = [
                (t2 - t1).total_seconds() for t1, t2 in itertools.pairwise(self._step_timestamps)
            ]
            # calculate average frequency if there are multiple intervals
            avg_interval = sum(intervals) / len(intervals)
            freq_hz = 1 / avg_interval
            self._rpm = (freq_hz / self._steps_per_rotation) * 60
            self._max_rpm = max(self._max_rpm, self._rpm)
            self._rotations = self._steps / self._steps_per_rotation
            if next(self._counter) % 10 == 0:
                self._log.info(_fore + '{: >5d} steps; {: >4.2f} rotations; freq: {: >4d}Hz; speed: {: >3d} rpm; max: {: >3d} rpm; target speed: {:d}'.format(
                        self._steps, self._rotations, int(freq_hz), int(self._rpm), int(self._max_rpm), int(self._slew_limiter.target_speed)))
            # keep only the latest timestamps based on the limit
            if len(self._step_timestamps) > self._timestamp_limit:
                self._step_timestamps = self._step_timestamps[-self._timestamp_limit:]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_speed(self, speed):
        '''
        Set the speed of this motor to the provided value.
        '''
        if self._slew_limiter.current_speed != speed:
            self._slew_limiter.target_speed = speed
        for _ in range(20):
            self._slew_limiter.update()
            _current_speed = self._slew_limiter()
            self._picon_zero.set_motor(self._channel, _current_speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def brake(self):
        '''
        Stops the motor quickly.

        This changes the slew rate, so following brake you must reset the
        slew rate to its default.
        '''
        self._slew_limiter.scale = self._braking_scale
        self.set_speed(0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stops the motor immediately.
        '''
        self._slew_limiter.stop()
        self._picon_zero.set_motor(self._channel, 0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_enabled(self):
        return self.enabled

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self.enabled:
            Component.enable(self)
        self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')
        self.off() # in any case

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        # just do it anyway
        self.stop()
        if self.enabled:
            self.disable()
        self._log.info('closed.')

#EOF
