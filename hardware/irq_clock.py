#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# author:   Murray Altheim
# created:  2021-10-09
# modified: 2024-06-02
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#

import itertools
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.util import Util

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class IrqClock(Component):
    '''
    Sets up a falling-edge interrupt on a GPIO pin, whose toggle (an Interrupt
    Request) is generated by an external source. When the interrupt is
    triggered any registered callbacks are executed.

    This is used predominantly for motor control timing.

    The callbacks are not executed asynchronously so any of them can block and
    throw off the clock timing. Hence callbacks should all return immediately.
    This class is ideally only used by a single subscriber/respondent.

    Make sure to call close() when finished to free up the Pi resources.

    Lazily-imports and configures pigpio when the enabled.

    :param config:     The application configuration.
    :param pin:        the optional input pin, overriding the configuration.
    :param level:      the logging level.
    '''
    def __init__(self, config, pin=None, level=Level.INFO):
        if pin is None:
            self._log = Logger('irq-clock', level)
        else:
#           self._log = Logger('irq-clock-{:d}'.format(pin), level)
            self._log = Logger('irq-clock-slo', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['mros'].get('hardware').get('irq_clock')
        self._initd         = False
        self._counter       = itertools.count()
        self.__callbacks    = []
        self.__lf_callbacks = []
        self._freq_divider  = _cfg.get('freq_divider')
        self._pi            = None
        self._pi_callback   = None
        self._pin = pin if pin else _cfg.get('pin')
        self._log.info('IRQ clock pin:\t{:d}'.format(self._pin))
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'irq-clock'

     # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Component.enable(self)
        if self.enabled:
            if not self._initd:
                try:
                    self._log.info('importing pigpio...')
                    import pigpio
                    self._pi = pigpio.pi()
                    if not self._pi.connected:
                        raise Exception('unable to establish connection to Pi.')
                    self._log.info('establishing callback on pin {:d}.'.format(self._pin))
                    self._pi.set_mode(gpio=self._pin, mode=pigpio.INPUT)
                    _edge = pigpio.EITHER_EDGE
#                   _edge = pigpio.FALLING_EDGE
                    self._pi_callback = self._pi.callback(self._pin, _edge, self._callback_method)
                    self._log.info('configured IRQ clock.')
                except Exception as e:
                    self._log.error('unable to enable IRQ clock: {}'.format(e))
                finally:
                    self._initd = True
        else:
            self._log.warning('unable to enable IRQ clock.')

     # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_callback(self, callback):
        '''
        Adds a callback to those triggered by clock ticks.
        '''
        if not callable(callback):
            raise Exception('callback argument is not a function.')
        if callback:
            if callback in self.__callbacks:
                raise Exception('callback already exists.')
            self._log.info('added callback: {}.{}()'.format(Util.get_class_name_of_method(callback), callback.__name__))
            self.__callbacks.append(callback)
        else:
            raise TypeError('null callback argument')

     # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_low_frequency_callback(self, callback):
        '''
        Adds a callback to those triggered by clock ticks, called at a
        lower frequency than the regular callbacks.
        '''
        if not callable(callback):
            raise Exception('callback argument is not a function.')
        if callback:
            if callback in self.__lf_callbacks:
                raise Exception('callback already exists.')
            self.__lf_callbacks.append(callback)
        else:
            raise TypeError('null callback argument')

     # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def remove_callback(self, callback):
        '''
        Removes a callback from the internal list.
        '''
        if callback:
            self.__callbacks.remove(callback)
        else:
            raise TypeError('null callback argument')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _callback_method(self, gpio, level, tick):
        if self.enabled:
            for callback in self.__callbacks:
                callback()
            if next(self._counter) % self._freq_divider == 0:
                for lf_callback in self.__lf_callbacks:
                    lf_callback()
#       else:
#           self._log.warning('IRQ clock disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        try:
            self._log.info('IRQ clock closing...')
            if self._pi_callback:
                self._pi_callback.cancel()
            self._log.info('IRQ clock closed.')
        except Exception as e:
            self._log.error('error closing pigpio: {}'.format(e))
        finally:
            if self._pi:
                self._pi.stop()
            self._log.info('pigpio connection closed.')
        Component.close(self)
        self._log.info('closed.')

#EOF
