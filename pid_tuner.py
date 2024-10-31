#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# A test/tuner of the pid controller.

import sys, traceback, time, itertools
from math import isclose
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.orientation import Orientation
from core.rate import Rate
from hardware.color import Color
#from hardware.slew_limiter import SlewLimiter
from hardware.irq_clock import IrqClock
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.motor_controller import MotorController
#from hardware.motor_configurer import MotorConfigurer
from hardware.motor import Motor
from hardware.rotary_encoder import RotaryEncoder, Selector
from hardware.digital_pot import DigitalPotentiometer
from hardware.button import Button

IN_MIN  = 0.0    # minimum analog value from IO Expander
IN_MAX  = 3.3    # maximum analog value from IO Expander

_motor_controller = None

def button_pushed():
    global _motor_controller
    if _motor_controller:
        _motor_controller.reset()
        print(Fore.CYAN + '-- motor controller reset.' + Style.RESET_ALL)
    else:
        print(Fore.RED + '-- no motor controller available.' + Style.RESET_ALL)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    global _motor_controller

    _level = Level.INFO
    _log = Logger('main', _level)

    _rate = Rate(20) # Hz

    _speed_pot        = None

    try:

        _config = ConfigLoader(Level.INFO).configure()

        _pin = 23
        _btn = Button(_config, pin=_pin, impl='gpiozero', level=Level.INFO)

#       _value = _btn.pushed()
        _btn.add_callback(button_pushed, bouncetime_ms=300)

        _log.info('creating IRQ clock…')
        _irq_clock = IrqClock(_config, level=Level.INFO)

        _counter = itertools.count()

        _speed_pot = DigitalPotentiometer(_config, 0x0C, level=_level)
        _speed_pot.set_input_range(IN_MIN, IN_MAX)
        _speed_pot.set_output_range(-100, 100)

        _motor_controller = MotorController(_config)

        # motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _orientation = Orientation.PFWD

        while True:

            _count = next(_counter)

            _tol = 3
            _target_speed = _speed_pot.get_scaled_value(absolute_tolerance=_tol)
            if isclose(_target_speed, 0.0, abs_tol=_tol):
#               _log.info('target speed: stopped.')
                _motor_controller.set_motor_speed(_orientation, 0)
                pass
            else:
#               _log.info('target speed: {:4.2f}'.format(_target_speed))
                _motor_controller.set_motor_speed(_orientation, _target_speed)
                pass

#           _log.info(_fore + '[{:d}] selected: {} pid: {:6.3f}|{:6.3f}|{:6.3f};\tset: {:>7.4f}; \tvel: {:>6.3f}; spd: {:>6.3f}'.format(\
#                   _count, _var, kp, ki, kd, _motor.get_current_power(), _motor.velocity, _target_speed))

            _rate.wait()

    except KeyboardInterrupt:
        _log.info(Fore.CYAN + Style.BRIGHT + 'B. motor test complete.')
    except Exception as e:
        _log.info(Fore.RED + Style.BRIGHT + 'error in PID controller: {}'.format(e))
        traceback.print_exc(file=sys.stdout)
    finally:
        _log.info('finally.')
        if _speed_pot:
            _speed_pot.set_black()
            _speed_pot.close()
        if _motor_controller:
            _motor_controller.stop()
            _motor_controller.close()

if __name__== "__main__":
    main()

#EOF
