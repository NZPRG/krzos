#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.

# Picon Zero Motor Test
# Moves: Forward, Reverse, turn Right, turn Left, Stop - then repeat
# Press Ctrl-C to stop
#
# To check wiring is correct ensure the order of movement as above is correct

import sys, traceback
import time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.direction import Direction
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from hardware.motor_controller import MotorController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('motor-test', Level.INFO)

FUNCTIONAL_TEST = False
CRAB_TEST       = False
ROTATE_TEST     = False
PIVOT_TEST      = True

_motor_controller = None
try:

    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()
    _motor_controller = MotorController(_config)
    _delay_sec = 1.5

    if FUNCTIONAL_TEST:

        _orientations = [ Orientation.PORT, Orientation.STBD, Orientation.PFWD, Orientation.SFWD, Orientation.PAFT, Orientation.SAFT ]

        for _orientation in _orientations:
            if _orientation.side is Orientation.PORT:
                print(Fore.RED   + 'set motor speed for {} motor(s).'.format(_orientation) + Style.RESET_ALL)
            else:
                print(Fore.GREEN + 'set motor speed for {} motor(s).'.format(_orientation) + Style.RESET_ALL)
            _motor_controller.set_motor_speed(_orientation, 30)
            time.sleep(_delay_sec)
            _motor_controller.set_motor_speed(_orientation, 0)
            time.sleep(0.33)
    
        print('set direction to AHEAD.')
        _motor_controller.set_direction(Direction.AHEAD, 20)
        time.sleep(_delay_sec)
        _motor_controller.set_direction(Direction.STOPPED)
        time.sleep(0.33)
    
        print('set direction to ASTERN.')
        _motor_controller.set_direction(Direction.ASTERN, 20)
        time.sleep(_delay_sec)
        _motor_controller.set_direction(Direction.STOPPED)
        time.sleep(0.33)

    if CRAB_TEST:
        _directions = [ Direction.CRAB_PORT, Direction.CRAB_STBD ]
        for _direction in _directions:
            print('set direction to {}.'.format(_direction))
            _motor_controller.set_direction(_direction, 90)
            time.sleep(3)
            _motor_controller.set_direction(Direction.STOPPED)
            time.sleep(0.33)

    if ROTATE_TEST:
        _directions = [ Direction.ROTATE_CW, Direction.ROTATE_CCW ]
        for _direction in _directions:
            print('set direction to {}.'.format(_direction))
            _motor_controller.set_direction(_direction, 90)
            time.sleep(3)
            _motor_controller.set_direction(Direction.STOPPED)
            time.sleep(0.33)

    if PIVOT_TEST:
        _directions = [ Direction.PIVOT_FWD_CW, Direction.PIVOT_FWD_CCW ]
        for _direction in _directions:
            print('set direction to {}.'.format(_direction))
            _motor_controller.set_direction(_direction, 30)
            time.sleep(3)
            _motor_controller.set_direction(Direction.STOPPED)
            time.sleep(0.33)

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
     _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _motor_controller:
        _motor_controller.close()

#EOF
