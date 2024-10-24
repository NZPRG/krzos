#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-10-24
#

import sys, time, traceback
from math import pi as π
from colorama import init, Fore, Style
init()

from core.cardinal import Cardinal
from core.convert import Convert
from core.logger import Logger, Level
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from hardware.icm20948 import Icm20948

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_icm20948 = None
_log = Logger('test', Level.INFO)
_cardinal = Cardinal.NORTH
_threshold = 4
HALF_PI = π / 2.0

try:
    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()
    _icm20948 = Icm20948(_config, None, Level.INFO)
    _icm20948._show_console = True
    if not _icm20948.is_calibrated:
        _icm20948.calibrate()

    # just scan continually...
    _icm20948.scan(enabled=True, callback=None)

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
    _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    pass

#EOF
