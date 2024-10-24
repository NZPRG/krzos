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
import tty
import termios # reading single character by forcing stdin to raw mode
from hardware.picon_zero import PiconZero
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.direction import Direction
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from hardware.motor_controller import MotorController

_log = Logger('motor-test', Level.INFO)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '0x03':
        raise KeyboardInterrupt
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)  # 16=Up, 17=Down, 18=Right, 19=Left arrows

# End of single character reading
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def help():
    print(Fore.GREEN)
    print("""
        Tests the motors by using the arrow keys to control")
    
          Use space to stop.
          Use 'w' for forward
          Use 'z' for reverse
          Use 's' for spin-clockwise
          Use 'a' for spin-counter-clockwise
          Use 'c' for crab-port
          Use 'v' for crab-starboard
          Use 'd' for diagonal-port
          Use 'f' for diagonal-starboard
          Use 'o' for pivot-rotate-cw
          Use 'p' for pivot-rotate-ccw
          Use 'k' for pivot-strafe-cw
          Use 'l' for pivot-strafe-ccw
          Use ',' or '<' to slow down
          Use '.' or '>' to speed up
          Use '?' for help.
          Use 'q' to quit.
    
        Speed changes take effect when the next arrow key is pressed.
    
        Type Ctrl-C to end.
    """)
    print(Style.RESET_ALL)


_fwd_pz = None
_aft_pz = None

STOPPED = 0
FORWARD = 1
REVERSE = 2
SPIN    = 3
CRAB    = 4
_direction = FORWARD
_speed  = 20 # initial speed


try:

    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()

    _motor_controller = MotorController(_config)

    # main loop
    while True:
        keyp = readkey()
        _ord = ord(keyp)
        _dir = Direction.get_direction_for_key(keyp)
        if keyp == '?':
            help()
        elif keyp == 'q':
            print('quit.')
            break
        else:
            _motor_controller.set_direction(_dir)

        continue
        if keyp == 'w' or _ord == 16:
            _direction = FORWARD
            _fwd_pz.forward(_speed)
            _aft_pz.forward(_speed)
            _log.info('forward: {}'.format(_speed))

        elif keyp == 'z' or _ord == 17:
            _direction = REVERSE
            _fwd_pz.reverse(_speed)
            _aft_pz.reverse(_speed)
            _log.info('reverse: {}'.format(_speed))

        elif keyp == 'c':
#           Use 'c' for crab-left
            _direction = CRAB
            _fwd_pz.spinLeft(_speed)
            _aft_pz.spinRight(_speed)

        elif keyp == 'v':
#       Use 'v' for crab-right
            _direction = CRAB
            _fwd_pz.spinRight(_speed)
            _aft_pz.spinLeft(_speed)

        elif keyp == 's' or _ord == 18:
            _direction = SPIN
            _fwd_pz.spinRight(_speed)
            _aft_pz.spinRight(_speed)
            _log.info('spin right: {}'.format(_speed))

        elif keyp == 'a' or _ord == 19:
            _direction = SPIN
            _fwd_pz.spinLeft(_speed)
            _aft_pz.spinLeft(_speed)
            _log.info('spin left: {}'.format(_speed))

        elif keyp == '.' or keyp == '>':
            _speed = min(100, _speed+10)
            if _direction == FORWARD:
                _log.info('increase speed: {} (fwd)'.format(_speed))
                _fwd_pz.forward(_speed)
                _aft_pz.forward(_speed)
            elif _direction == REVERSE:
                _log.info('increase speed: {} (rev)'.format(_speed))
                _fwd_pz.reverse(_speed)
                _aft_pz.reverse(_speed)
            else:
                _log.info('increase speed: {}'.format(_speed))

        elif keyp == ',' or keyp == '<':
            _speed = max(0, _speed-10)
            if _direction == FORWARD:
                _log.info('decrease speed: {} (fwd)'.format(_speed))
                _fwd_pz.forward(_speed)
                _aft_pz.forward(_speed)
            elif _direction == REVERSE:
                _log.info('decrease speed: {} (rev)'.format(_speed))
                _fwd_pz.reverse(_speed)
                _aft_pz.reverse(_speed)
            else:
                _log.info('decrease speed: {}'.format(_speed))

        elif keyp == ' ':
            _direction = STOPPED
            _fwd_pz.stop()
            _aft_pz.stop()
            _log.info('Stop')

        elif keyp == 'q':
            break

        elif _ord == 3:
            break

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
     _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _fwd_pz:
        _fwd_pz.close()
    if _aft_pz:
        _aft_pz.close()
    
#EOF
