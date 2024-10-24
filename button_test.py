#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2024-05-23
#
# A test script for the Button class.
#

import pytest
import sys, traceback
import time
from colorama import init, Fore, Style
init()

try:
    import RPi.GPIO as GPIO
except Exception:
    print('This script requires the RPi.GPIO module.\nInstall with: sudo pip3 install RPi.GPIO')
    sys.exit(1)

from core.logger import Level
from core.config_loader import ConfigLoader
from hardware.button import Button

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
@pytest.mark.unit
def test_pushbutton():
    try:

        print('begin')

        _pin = 25
        _config = ConfigLoader(Level.INFO).configure()
        _btn = Button(_config, pin=_pin, level=Level.INFO)

        _old_value = None
        while True:
            _value = _btn.pushed()
            if _value != _old_value:
                if _value:
                    print(Fore.GREEN + 'button: {}'.format(_value))
                else:
                    print(Fore.RED   + 'button: {}'.format(_value))
                _old_value = _value
            time.sleep(0.1)

    except KeyboardInterrupt:
        print('Ctrl-C caught, exiting...')
        sys.exit(0)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    try:
        test_pushbutton()
    except Exception as e:
        print(Fore.RED + 'error in motor test: {}'.format(e) + Style.RESET_ALL)
        traceback.print_exc(file=sys.stdout)
    finally:
        pass
        
if __name__== "__main__":
    main()
    
#EOF
