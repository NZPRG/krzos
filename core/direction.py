#}!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-07-29
# modified: 2024-05-19
#
# Note that stopped, clockwise and counter-clockwise are descriptive, not prescriptive.
#

from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Direction(Enum):
    STOPPED           = (  0, 'stopped',           'stop', ' ')
    AHEAD             = (  1, 'ahead',             'ahed', 'w')
    ASTERN            = (  2, 'astern',            'astn', 'z')
    ROTATE_CW         = (  3, 'rotate-cw',         'rtcw', 's')
    ROTATE_CCW        = (  4, 'rotate-ccw',        'rtcc', 'a')
    CRAB_PORT         = (  5, 'crab-port',         'crap', 'c')
    CRAB_STBD         = (  6, 'crab-stbd',         'cras', 'v')
    DIAGONAL_PORT     = (  7, 'diagonal-port',     'diap', 'd')
    DIAGONAL_STBD     = (  8, 'diagonal-stbd',     'dias', 'f')

    PIVOT_FWD_CW      = (  9, 'pivot-fwd-cw',      'pfcw', 'p')
    PIVOT_FWD_CCW     = ( 10, 'pivot-fwd-ccw',     'pfcc', 'o')
    PIVOT_AFT_CW      = ( 11, 'pivot-aft-cw',      'pacw', 'l')
    PIVOT_AFT_CCW     = ( 12, 'pivot-aft-ccw',     'pacc', 'k')
    PIVOT_PORT_CW     = ( 13, 'pivot-port-cw',     'ppcw', 'i')
    PIVOT_PORT_CCW    = ( 14, 'pivot-port-ccw',    'ppcc', 'u')
    PIVOT_STBD_CW     = ( 15, 'pivot-stbd-cw',     'pscw', 'j')
    PIVOT_STBD_CCW    = ( 16, 'pivot-stbd-ccw',    'pscc', 'h')

    UNKNOWN           = ( 99, 'unknown',           'unkn', '?') # n/a or indeterminate

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, label, key):
        self._name  = name
        self._label = label
        self._key   = key

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def label(self):
        return self._label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def key(self):
        return self._key

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_direction_for(port_velocity, stbd_velocity):
        '''
        NOTE: Not yet implemented for enums > 4.
        '''
        if port_velocity and stbd_velocity:
            if port_velocity == 0.0 and stbd_velocity == 0.0:
                return Direction.STOPPED
            elif port_velocity > 0.0 and stbd_velocity > 0.0:
                return Direction.AHEAD
            elif port_velocity < 0.0 and stbd_velocity < 0.0:
                return Direction.ASTERN
            elif port_velocity > 0.0 and stbd_velocity <= 0.0:
                return Direction.CLOCKWISE
            elif port_velocity <= 0.0 and stbd_velocity > 0.0:
                return Direction.COUNTER_CLOCKWISE
            else:
                raise TypeError('unable to discern direction for port: {}; stbd: {}'.format(port_velocity, stbd_velocity))
        else:
            return Direction.UNKNOWN

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_direction_for_key(key):
        '''
        NOTE: Not yet implemented for all enums.
        '''
        for _direction in Direction:
            if _direction.key == key:
                return _direction
        return Direction.UNKNOWN

#EOF
