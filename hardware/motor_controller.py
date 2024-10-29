#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2024-06-02
#

import sys, traceback
import time
import statistics
import itertools
from math import isclose
from threading import Thread
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.component import Component
from core.direction import Direction
from core.orientation import Orientation
from core.rate import Rate
from core.logger import Logger, Level
from hardware.picon_zero import PiconZero
from hardware.motor import Motor

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorController(Component):
    '''
    The controller for 4 motors:

        pfwd: Port-Forward        sfwd: Starboard-Forward
        paft: Port-Aft            saft: Starboard-Aft

    :param config:            the YAML based application configuration
    :param suppressed         if True the controller is suppressed
    :param enabled            if True the controller is enabled upon instantiation
    :param level:             the logging Level
    '''
    def __init__(self, config, suppressed=False, enabled=True, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._log = Logger("motor-ctrl", level)
        Component.__init__(self, self._log, suppressed, enabled)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['mros'].get('motor_controller')
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._loop_freq_hz   = _cfg.get('loop_freq_hz') # main loop frequency
        self._loop_delay_sec = 1 / self._loop_freq_hz
        self._rate           = Rate(self._loop_freq_hz, Level.ERROR)
        self._log.info('loop frequency:\t{}Hz ({:4.2f}s)'.format(self._loop_freq_hz, self._loop_delay_sec))
        self._verbose        = False
        # motor controller ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._fwd_pz = PiconZero(orientation=Orientation.FWD)
        self._fwd_pz.initialise()
        self._aft_pz = PiconZero(orientation=Orientation.AFT)
        self._aft_pz.initialise()
        self._pfwd_motor     = Motor(self._fwd_pz, Orientation.PFWD, level=Level.INFO)
        self._sfwd_motor     = Motor(self._fwd_pz, Orientation.SFWD, level=Level.INFO)
        self._paft_motor     = Motor(self._aft_pz, Orientation.PAFT, level=Level.INFO)
        self._saft_motor     = Motor(self._aft_pz, Orientation.SAFT, level=Level.INFO)
        self._all_motors     = self._get_motors()
        # finish up…
        self._log.info('ready with {} motors.'.format(len(self._all_motors)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motors(self):
        '''
        Returns a list containing all instantiated motors.
        This includes only instantiated motors.
        '''
        return self._all_motors

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_motors(self):
        '''
        Returns a list of all extant motors.
        '''
        _list = []
        if self._pfwd_motor:
            _list.append(self._pfwd_motor)
        if self._sfwd_motor:
            _list.append(self._sfwd_motor)
        if self._paft_motor:
            _list.append(self._paft_motor)
        if self._saft_motor:
            _list.append(self._saft_motor)
        return _list

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motor(self, orientation):
        '''
        Returns the motor corresponding to the orientation.
        '''
        if orientation is Orientation.PFWD:
            return self._pfwd_motor
        elif orientation is Orientation.SFWD:
            return self._sfwd_motor
        elif orientation is Orientation.PAFT:
            return self._paft_motor
        elif orientation is Orientation.SAFT:
            return self._saft_motor
        else:
            raise Exception('unsupported orientation.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enables the motors. This issues a warning if already enabled, but
        no harm is done in calling it repeatedly.
        '''
        if self.enabled:
            self._log.warning('already enabled.')
        else:
            Component.enable(self)
            if self._external_clock:
                self._external_clock.add_callback(self.external_callback_method)
                for _motor in self._all_motors:
                    _motor.enable()
            self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        self._log.info('stop.')
        self._fwd_pz.stop()
        self._aft_pz.stop()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def brake(self):
        _brake_scale = 80
        for _motor in self._all_motors:
            _motor.brake()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        for _motor in self._all_motors:
            _motor.reset()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_direction(self, direction, speed=0):
        if direction is Direction.STOPPED:             # (  0, 'stopped',           'stop', ' ')
            self._log.info('set direction: STOPPED.')
            self.stop()

        elif direction is Direction.AHEAD:             # (  1, 'ahead',             'ahed', 'w')
            self._log.info('set direction: AHEAD.')
            self._pfwd_motor.set_speed(speed)
            self._sfwd_motor.set_speed(speed)
            self._paft_motor.set_speed(speed)
            self._saft_motor.set_speed(speed)

        elif direction is Direction.ASTERN:            # (  2, 'astern',            'astn', 'z')
            self._log.info('set direction: ASTERN.')
            self._pfwd_motor.set_speed(-1 * speed)
            self._sfwd_motor.set_speed(-1 * speed)
            self._paft_motor.set_speed(-1 * speed)
            self._saft_motor.set_speed(-1 * speed)

        elif direction is Direction.ROTATE_CW:         # (  3, 'rotate-cw',         'rtcw', 's')
            self._log.info('set direction: ROTATE CLOCKWISE.')
            self._pfwd_motor.set_speed(speed)
            self._sfwd_motor.set_speed(-1 * speed)
            self._paft_motor.set_speed(speed)
            self._saft_motor.set_speed(-1 * speed)

        elif direction is Direction.ROTATE_CCW:        # (  4, 'rotate-ccw',        'rtcc', 'a')
            self._log.info('set direction: ROTATE COUNTER-CLOCKWISE.')
            self._pfwd_motor.set_speed(-1 * speed)
            self._sfwd_motor.set_speed(speed)
            self._paft_motor.set_speed(-1 * speed)
            self._saft_motor.set_speed(speed)

        elif direction is Direction.CRAB_PORT:         # (  5, 'crab-port',         'crap', 'c')
            self._log.info('set direction: CRAB PORT.')
            self._pfwd_motor.set_speed(-1 * speed)
            self._sfwd_motor.set_speed(speed)
            self._paft_motor.set_speed(speed)
            self._saft_motor.set_speed(-1 * speed)

        elif direction is Direction.CRAB_STBD:         # (  6, 'crab-stbd',         'cras', 'v')
            self._log.info('set direction: CRAB STBD.')
            self._pfwd_motor.set_speed(speed)
            self._sfwd_motor.set_speed(-1 * speed)
            self._paft_motor.set_speed(-1 * speed)
            self._saft_motor.set_speed(speed)

        elif direction is Direction.DIAGONAL_PORT:     # (  7, 'diagonal-port',     'diap', 'd')
            self._log.info('set direction: DIAGONAL PORT.')
        elif direction is Direction.DIAGONAL_STBD:     # (  8, 'diagonal-stbd',     'dias', 'f')
            self._log.info('set direction: DIAGONAL STBD.')

        elif direction is Direction.PIVOT_FWD_CW:      # (  9, 'pivot-fwd-cw',      'pfcw', 'p')
            self._log.info('set direction: PIVOT FWD CW.')
            self._pfwd_motor.set_speed( 1 * speed)
            self._sfwd_motor.set_speed(-1 * speed)
            self._paft_motor.set_speed(0)
            self._saft_motor.set_speed(0)

        elif direction is Direction.PIVOT_FWD_CCW:     # ( 10, 'pivot-fwd-ccw',     'pfcc', 'o')
            self._log.info('set direction: PIVOT FWD CCW.')
            self._pfwd_motor.set_speed(-1 * speed)
            self._sfwd_motor.set_speed( 1 * speed)
            self._paft_motor.set_speed(0)
            self._saft_motor.set_speed(0)

        elif direction is Direction.PIVOT_AFT_CW:      # ( 11, 'pivot-aft-cw',      'pacw', 'l')
            self._log.info('set direction: PIVOT AFT.')
            self._pfwd_motor.set_speed(0)
            self._sfwd_motor.set_speed(0)
            self._paft_motor.set_speed(-1 * speed)
            self._saft_motor.set_speed(-1 * speed)

        elif direction is Direction.PIVOT_PORT_CW:     # ( 13, 'pivot-port-cw',     'ppcw', 'i')
            self._log.info('set direction: PIVOT PORT.')
            self._pfwd_motor.set_speed(-1 * speed)
            self._sfwd_motor.set_speed(-1 * speed)
            self._paft_motor.set_speed(-1 * speed)
            self._saft_motor.set_speed(-1 * speed)

        elif direction is Direction.PIVOT_STBD_CW:      #( 15, 'pivot-stbd-cw',     'pscw', 'j')
            self._log.info('set direction: PIVOT STBD.')
            self._pfwd_motor.set_speed(-1 * speed)
            self._sfwd_motor.set_speed(-1 * speed)
            self._paft_motor.set_speed(-1 * speed)
            self._saft_motor.set_speed(-1 * speed)

        else:                                          # ( 99, 'unknown',           'unkn', '?') # n/a or indeterminate
            self._log.info('set direction: UNKNOWN.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_motor_speed(self, orientation, speed):
        '''
        If provided PORT or STBD, sets the speed of all motors associated with 
        the port or starboard orientation, otherwise sets the speed of the 
        specified motor.
        '''
        if not self.enabled:
            self._log.error('motor controller not enabled.')
            speed = 0.0
        if isinstance(speed, float):
            speed = int(speed)
        elif not isinstance(speed, int):
            raise ValueError('expected target speed as int, not {}'.format(type(speed)))
        if orientation is Orientation.PORT:
            if self._verbose:
                self._log.info(Fore.RED + 'set {} motor speed: {:5.2f}'.format(orientation.name, speed))
            self._pfwd_motor.set_speed(speed)
            self._paft_motor.set_speed(speed)
        elif orientation is Orientation.STBD:
            if self._verbose:
                self._log.info(Fore.GREEN + 'set {} motor speed: {:5.2f}'.format(orientation.name, speed))
            self._sfwd_motor.set_speed(speed)
            self._saft_motor.set_speed(speed)
        elif orientation is Orientation.PFWD:
            if self._verbose:
                self._log.info('set {} motor speed: {:5.2f}'.format(orientation.name, speed))
            self._pfwd_motor.set_speed(speed)
        elif orientation is Orientation.SFWD:
            if self._verbose:
                self._log.info('set {} motor speed: {:5.2f}'.format(orientation.name, speed))
            self._sfwd_motor.set_speed(speed)
        elif orientation is Orientation.PAFT:
            if self._verbose:
                self._log.info('set {} motor speed: {:5.2f}'.format(orientation.name, speed))
            self._paft_motor.set_speed(speed)
        elif orientation is Orientation.SAFT:
            if self._verbose:
                self._log.info('set {} motor speed: {:5.2f}'.format(orientation.name, speed))
            self._saft_motor.set_speed(speed)
        else:
            self._log.warning('expected a motor orientation, not {}'.format(orientation))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the motors.
        '''
        if self.enabled:
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.debug('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Closes the motor controller.
        '''
        if not self.closed:
            Component.close(self) # calls disable
            self._pfwd_motor.stop()
            self._sfwd_motor.stop()
            self._paft_motor.stop()
            self._saft_motor.stop()
            self._fwd_pz.close()
            self._aft_pz.close()
            self._log.info('motor controller closed.')
        else:
            self._log.warning('motor controller already closed.')

#EOF
