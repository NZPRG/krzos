#!/usr/bin/env python3 # -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-10-29
# modified: 2024-10-30
#

from math import isclose

from core.logger import Level, Logger
from core.component import Component

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SlewLimiter(Component):
    '''
    The slew limiter for this Motor is much simplified from previous
    versions, as it simply uses the step counter to resolve the
    difference between the current speed and the target speed.
    '''
    def __init__(self, motor, rate=100.0, level=Level.INFO):
        self._log = Logger('slew:{}'.format(motor.orientation.label), level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._motor = motor
        self.rate  = rate
        self._target_speed   = 0.0  # target speed at high resolution (float)
        self._internal_speed = 0.0  # Internal speed at high resolution (float)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def rate(self):
        return self._rate

    @rate.setter
    def rate(self, rate):
        if rate <= 0:
            raise ValueError("Slew rate must be a positive value.")
        self._rate = float(rate)  # ensure rate is stored as float

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        self._slew_limiter.rate = self._default_slew_rate

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def target_speed(self):
        '''
        Get the target speed in the output range [-100, 100].
        '''
        return self._target_speed / 1000.0

    @target_speed.setter
    def target_speed(self, target):
        '''
        Set the target speed in the output range and scale it internally.
        '''
        if not (-100 <= target <= 100):
            raise ValueError("Target speed must be between -100 and 100.")
        self._target_speed = float(target * 1000)  # Scale to internal resolution

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def current_speed(self):
        '''
        Get the current speed in the output range [-100, 100].
        '''
        return max(-100, min(100, self._internal_speed / 1000.0))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def resolve(self):
        '''
        Adjust internal speed by a scaled fraction towards the target speed.
        '''
        # Calculate the difference at internal resolution (float)
        difference = self._target_speed - self._internal_speed

        # Adjust the internal speed incrementally towards the target speed
        if abs(difference) > 0:
            # Determine the step size based on the rate and the direction
            step = min(abs(difference), self._rate) * (1 if difference > 0 else -1)
            self._internal_speed += step

        # Snap to target using math.isclose
        if isclose(self._target_speed, self._internal_speed, abs_tol=1e-3):
            self._internal_speed = self._target_speed

        # Return the constrained output speed as an integer
        return int(self.current_speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def update(self):
        '''
        Manually update the internal speed without needing the motor to be active.
        '''
        self.resolve()  # Call the resolve method to adjust internal speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Set the target and internal speed to zero.
        '''
        self._target_speed   = 0.0
        self._internal_speed = 0.0 

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __call__(self):
        """Make the class callable to resolve speed each time it's called."""
        return self.resolve()

#EOF
