#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-10-23
# modified: 2024-10-23
#

import time
import random
from threading import Thread

from core.logger import Level, Logger
from core.component import Component
from core.orientation import Orientation
from hardware.rgbmatrix import RgbMatrix
from hardware.color import Color
from hardware.eyeball import Eyeball

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Eyeballs(Component):
    '''
    A display of eyes on a pair of 5x5 RGB LED matrix displays.

    :param level:   the logging Level
    '''
    def __init__(self, level=Level.INFO):
        self._log = Logger('eyeballs', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._rgbmatrix = RgbMatrix(True, True, Level.INFO)
        self._port_rgbmatrix = self._rgbmatrix.get_rgbmatrix(Orientation.PORT)
        self._stbd_rgbmatrix = self._rgbmatrix.get_rgbmatrix(Orientation.STBD)
        self._thread = None
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Component.enable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self._rgbmatrix.clear_all()
        self._rgbmatrix.close()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear(self):
        self._rgbmatrix.clear_all()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_matrix(self, array, matrix, color=Color.ORANGE):
        for y in range(0,5):
            for x in range(0,5):
                if array[x][y] == 1:
                    _color = color
                else:
                    _color = Color.BLACK
                matrix.set_pixel(x, y, _color.red, _color.green, _color.blue)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _show(self):
        self._port_rgbmatrix.show()
        self._stbd_rgbmatrix.show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def normal(self):
        self._log.info('normal…')
        _eyeball = Eyeball.NORMAL
        self.set_matrix(_eyeball.array, self._port_rgbmatrix, _eyeball.color)
        self.set_matrix(_eyeball.array, self._stbd_rgbmatrix, _eyeball.color)
        self._show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def happy(self):
        self._log.info('happy…')
        _eyeball = Eyeball.HAPPY
        self.set_matrix(_eyeball.array, self._port_rgbmatrix, _eyeball.color)
        self.set_matrix(_eyeball.array, self._stbd_rgbmatrix, _eyeball.color)
        self._show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def wink(self):
        self._log.info('wink…')
        self.set_matrix(Eyeball.WINK_PORT.array, self._port_rgbmatrix, Eyeball.WINK_PORT.color)
        self.set_matrix(Eyeball.WINK_STBD.array, self._stbd_rgbmatrix, Eyeball.WINK_PORT.color)
        self._show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def blush(self):
        self._log.info('blush…')
        _eyeball = Eyeball.BLUSH
        self.set_matrix(_eyeball.array, self._port_rgbmatrix, _eyeball.color)
        self.set_matrix(_eyeball.array, self._stbd_rgbmatrix, _eyeball.color)
        self._show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def look_port(self):
        self._log.info('look port…')
        _eyeball = Eyeball.LOOK_PORT
        self.set_matrix(_eyeball.array, self._port_rgbmatrix, _eyeball.color)
        self.set_matrix(_eyeball.array, self._stbd_rgbmatrix, _eyeball.color)
        self._show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def look_stbd(self):
        self._log.info('look starboard…')
        _eyeball = Eyeball.LOOK_STBD
        self.set_matrix(_eyeball.array, self._port_rgbmatrix, _eyeball.color)
        self.set_matrix(_eyeball.array, self._stbd_rgbmatrix, _eyeball.color)
        self._show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def confused(self):
        self._log.info('confused…')
        self.set_matrix(Eyeball.CONFUSED_STBD.array, self._port_rgbmatrix, Eyeball.CONFUSED_STBD.color)
        self.set_matrix(Eyeball.CONFUSED_PORT.array, self._stbd_rgbmatrix, Eyeball.CONFUSED_PORT.color)
        self._show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def drugged(self):
        self._log.info('drugged…')
#       self.set_matrix(Eyeball.CONFUSED_STBD.array, self._port_rgbmatrix, Eyeball.CONFUSED_STBD.color)
#       self.set_matrix(Eyeball.CONFUSED_PORT.array, self._stbd_rgbmatrix, Eyeball.CONFUSED_PORT.color)
#       self._show()
#       return
        _eyeball = Eyeball.HAPPY
        self.set_matrix(_eyeball.array, self._port_rgbmatrix, _eyeball.color)
        self.set_matrix(_eyeball.array, self._stbd_rgbmatrix, _eyeball.color)
        self._rgbmatrix.show()
        time.sleep(2)
        n = 128
        for y in range(0,5):
            for x in range(0,5):
                if _eyeball.array[x][y] == 1:
                    self._port_rgbmatrix.set_pixel(x, y, n, n, n)
                    self._stbd_rgbmatrix.set_pixel(x, y, n, n, n)
                else:
                    self._port_rgbmatrix.set_pixel(x, y, 0, 0, 0)
                    self._stbd_rgbmatrix.set_pixel(x, y, 0, 0, 0)
            self._rgbmatrix.show()
            time.sleep(0.03)
        self._rgbmatrix.clear_all()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def sad(self):
        self._log.info('sad…')
        _eyeball = Eyeball.SAD
        self.set_matrix(_eyeball.array, self._port_rgbmatrix, _eyeball.color)
        self.set_matrix(_eyeball.array, self._stbd_rgbmatrix, _eyeball.color)
        self._show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def blank(self):
        self._log.info('blank…')
        _eyeball = Eyeball.BLANK
        self.set_matrix(_eyeball.array, self._port_rgbmatrix, _eyeball.color)
        self.set_matrix(_eyeball.array, self._stbd_rgbmatrix, _eyeball.color)
        self._show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def wow(self, count=5):
        self._log.info('wow…')
        self._thread = Thread(name='wow', target=self._wow, args=[count], daemon=True)
        self._thread.start()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _wow(self, count=5):
        '''
        Displays randomly-colored bulging eyes.
        '''
        _delay = 0.05
        _colors = [ 
            Color.RED, Color.GREEN, Color.BLUE, Color.CYAN, Color.MAGENTA, Color.YELLOW,
            Color.TURQUOISE, Color.ORANGE, Color.VIOLET, Color.CORAL, Color.YELLOW_GREEN,
            Color.TANGERINE, Color.FUCHSIA ]
        for _ in range(0,count):
            self.set_matrix(Eyeball.WOW1.array, self._port_rgbmatrix, random.choice(_colors))
            self.set_matrix(Eyeball.WOW1.array, self._stbd_rgbmatrix, random.choice(_colors))
            self._show()
            time.sleep(_delay)
            self.set_matrix(Eyeball.WOW2.array, self._port_rgbmatrix, random.choice(_colors))
            self.set_matrix(Eyeball.WOW2.array, self._stbd_rgbmatrix, random.choice(_colors))
            self._show()
            time.sleep(_delay)
            self.set_matrix(Eyeball.WOW3.array, self._port_rgbmatrix, random.choice(_colors))
            self.set_matrix(Eyeball.WOW3.array, self._stbd_rgbmatrix, random.choice(_colors))
            self._show()
            time.sleep(_delay)
            self.set_matrix(Eyeball.WOW2.array, self._port_rgbmatrix, random.choice(_colors))
            self.set_matrix(Eyeball.WOW2.array, self._stbd_rgbmatrix, random.choice(_colors))
            self._show()
            time.sleep(_delay)
            self.set_matrix(Eyeball.WOW1.array, self._port_rgbmatrix, random.choice(_colors))
            self.set_matrix(Eyeball.WOW1.array, self._stbd_rgbmatrix, random.choice(_colors))
            self._show()
            time.sleep(_delay)
        time.sleep(1.0)
        self._rgbmatrix.set_color(Color.BLACK)
        self._rgbmatrix.clear_all()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def dead(self, include_fade=False):
        self._log.info('dead…')
        _eyeball = Eyeball.DEAD
        self.set_matrix(_eyeball.array, self._port_rgbmatrix, _eyeball.color)
        self.set_matrix(_eyeball.array, self._stbd_rgbmatrix, _eyeball.color)
        self._rgbmatrix.show()
        time.sleep(2)
        if include_fade:
            for n in range(96, 0, -2):
                for y in range(0,5):
                    for x in range(0,5):
                        if _eyeball.array[x][y] == 1:
                            self._port_rgbmatrix.set_pixel(x, y, n, n, n)
                            self._stbd_rgbmatrix.set_pixel(x, y, n, n, n)
                        else:
                            self._port_rgbmatrix.set_pixel(x, y, 0, 0, 0)
                            self._stbd_rgbmatrix.set_pixel(x, y, 0, 0, 0)
                self._rgbmatrix.show()
                time.sleep(0.03)
        self._rgbmatrix.clear_all()

#EOF
