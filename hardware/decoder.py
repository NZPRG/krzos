#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-18
# modified: 2024-06-02
#
# This uses either pigpiod or RPi.GPIO to implement a callback on a GPIO pin.

# For installing pigpio, see:  http://abyz.me.uk/rpi/pigpio/download.html
#
# To enable the pigpio daemon on startup:
# 
#   sudo systemctl enable pigpiod
# 
# or to disable the pigpio daemon on startup:
# 
#   sudo systemctl disable pigpiod
# 
# or start it immediately:
# 
#   sudo systemctl start pigpiod 
# 
# To obtain the status of the pigpiod deamon: 
# 
#   sudo systemctl status pigpiod 
# 

import psutil
import sys, traceback
from colorama import init, Fore, Style
init()

# see if pigpiod is running
def is_pigpiod_running():
    for process in psutil.process_iter(['name']):
        if process.info['name'] == 'pigpiod':
            return True
    return False

from core.logger import Logger

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Decoder(object):
    '''
    Class to decode mechanical rotary encoder pulses, implemented
    using either RPI.GPIO or pigpio's callback method.

    Decodes the rotary encoder A and B pulses, e.g.:

                     +---------+         +---------+      0
                     |         |         |         |
           A         |         |         |         |
                     |         |         |         |
           +---------+         +---------+         +----- 1

               +---------+         +---------+            0
               |         |         |         |
           B   |         |         |         |
               |         |         |         |
           ----+         +---------+         +---------+  1

    '''

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __init__(self, orientation, gpio_a, gpio_b, callback, level):
        '''
        Instantiate the class with the pi and gpios connected to
        rotary encoder contacts A and B. The common contact should
        be connected to ground. The callback is called when the
        rotary encoder is turned. It takes one parameter which is
        +1 for clockwise and -1 for counterclockwise.

        If using the RPi.GPIO library you should call close() when
        finished to free up the GPIO pins.

        EXAMPLE

        from lib.decoder import Decoder

        def my_callback(self, step):
           self._steps += step

        pin_a = 17
        pin_b = 18
        decoder = Decoder(pi, pin_a, pin_b, my_callback)
        ...
        decoder.cancel()

        :param orientation:  the motor orientation
        :param gpio_a:        pin number for A
        :param gpio_b:        pin number for B
        :param callback:     the callback method
        :param level:        the log Level
        '''
        self._log = Logger('enc:{}'.format(orientation.label), level)
        self._gpio_a    = gpio_a
        self._gpio_b    = gpio_b
        self._log.debug('pin A: {:d}; pin B: {:d}'.format(self._gpio_a,self._gpio_b))
        self._callback  = callback
        self._level_a   = 0
        self._level_b   = 0
        self._last_gpio = None
        self._increment = 1

        # TODO use config
        self._encoder_a    = None
        self._encoder_b    = None
        self._use_pigpiod  = False
        self._use_gpiozero = True 
        self._use_rpigpio  = False
        try:
            if self._use_pigpiod and is_pigpiod_running():
                self._log.info('using ' + Fore.WHITE + 'pigpiod' + Fore.CYAN + ' for motor encoders…')

                import pigpio

                _pi = pigpio.pi()
                if _pi is None:
                    raise Exception('unable to instantiate pigpio.pi().')
                elif _pi._notify is None:
                    raise Exception('can\'t connect to pigpio daemon; did you start it?')
                _pi._notify.name = 'pi.callback'
                self._log.debug('pigpio version {}'.format(_pi.get_pigpio_version()))
                _pi.set_mode(self._gpio_a, pigpio.INPUT)
                _pi.set_mode(self._gpio_b, pigpio.INPUT)
                _pi.set_pull_up_down(self._gpio_a, pigpio.PUD_UP)
                _pi.set_pull_up_down(self._gpio_b, pigpio.PUD_UP)
                # _edge = pigpio.RISING_EDGE or pigpio.FALLING_EDGE
                _edge = pigpio.EITHER_EDGE
                self.callback_a = _pi.callback(self._gpio_a, _edge, self._pulse_a)
                self.callback_b = _pi.callback(self._gpio_b, _edge, self._pulse_b)
                self._log.info('configured {} motor encoder with channel A on pin {}, channel B on pin {}, using pigpiod.'.format(orientation.name, self._gpio_a, self._gpio_b))

            elif self._use_gpiozero:
                self._log.info('using ' + Fore.WHITE + 'gpiozero' + Fore.CYAN + ' for motor encoders…')
   
                from gpiozero import DigitalInputDevice
#               from gpiozero import Button

                # setup DigitalInputDevices for Hall Effect sensor channels
                self._encoder_a = DigitalInputDevice(self._gpio_a, pull_up=True, bounce_time=None)
                self._encoder_b = DigitalInputDevice(self._gpio_b, pull_up=True, bounce_time=None)
                # attach edge detection callbacks directly to GPIO pins
                self._encoder_a.when_activated   = self._active_a
                self._encoder_b.when_activated   = self._active_b
                self._encoder_a.when_deactivated = self._active_a
                self._encoder_b.when_deactivated = self._active_b


            elif self._use_rpigpio:
                self._log.info('using ' + Fore.WHITE + 'RPi.GPIO' + Fore.CYAN + ' for motor encoders…')
    
                import RPi.GPIO as GPIO
    
                # cleanup any previous configurations
                GPIO.cleanup()

                GPIO.setmode(GPIO.BCM) # set the GPIO mode to use GPIO rather than pin numbers
                # set up the GPIO pins as input
                GPIO.setup(self._gpio_a, GPIO.IN)
                GPIO.setup(self._gpio_b, GPIO.IN)
#               GPIO.setup(self._gpio_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#               GPIO.setup(self._gpio_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                # set up event detection for both rising and falling edges
                GPIO.add_event_detect(self._gpio_a, GPIO.BOTH, callback=self._pulse_a)
                GPIO.add_event_detect(self._gpio_b, GPIO.BOTH, callback=self._pulse_b)
    
                self._log.info('configured {} motor encoder with channel A on pin {}, channel B on pin {}, using RPi.GPIO'.format(orientation.name, self._gpio_a, self._gpio_b))

        except ImportError as ie:
            self._log.error("This script requires the pigpio module.\nInstall with: pip3 install --user pigpio")
            raise ModuleNotFoundError('pigpio not installed.')
        except Exception as e:
            self._log.error('{} thrown configuring decoder: {}\n{}'.format(type(e), e, traceback.format_exc()))
            raise Exception('unable to configure decoder.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_reversed(self):
        '''
        If called, sets this encoder for reversed operation.
        '''
        self._increment = -1

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _pulse_a(self, gpio, level, tick):
        self._level_a = level
        if level == 1 and self._level_b == 1:
#           self._log.info('pulse A; level: {}; level_b: {}'.format(level, self._level_b))
            self._callback(self._increment)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _pulse_b(self, gpio, level, tick):
        self._level_b = level;
        if level == 1 and self._level_a == 1:
#           self._log.info('pulse B; level: {}; level_a: {}'.format(level, self._level_a))
            self._callback(-1 * self._increment)

    # gpiozero ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _active_a(self, device):
        self._level_a = self._encoder_a.value
#       if self._level_a == 1 and self._level_b == 1:
        if self._level_a == self._level_b:
            self._log.info(Fore.MAGENTA + '[+] A({}); level: {}; level_b: {}'.format(device.pin, self._level_a, self._level_b))
            self._callback(self._increment) # COUNT UP
        else:
            self._log.info(Fore.MAGENTA + Style.DIM + '[+] A({}); level: {}; level_b: {}'.format(device.pin, self._level_a, self._level_b))
            pass

    def _active_b(self, device):
        self._level_b = self._encoder_b.value
#       if self._level_b == 1 and self._level_a == 1:
        if self._level_b == self._level_a:
            self._log.info(Fore.GREEN + '[-] B({}); level: {}; level_a: {}'.format(device.pin, self._level_b, self._level_a))
            self._callback(-1 * self._increment) # COUNT DOWN
        else:
            self._log.info(Fore.GREEN + Style.DIM + '[-] B({}); level: {}; level_a: {}'.format(device.pin, self._level_b, self._level_a))
            pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def cancel(self):
        '''
        Cancel the rotary encoder decoder.
        '''
        if self._use_pigpiod:
            self.callback_a.cancel()
            self.callback_b.cancel()
        elif self._use_gpiozero:
            self._encoder_a.close()
            self._encoder_b.close()
        else:
            # Remove event detection and callbacks
            GPIO.remove_event_detect(self._gpio_a)
            GPIO.remove_event_detect(self._gpio_b)

    def close(self):
        GPIO.cleanup()  # Clean up when the program is terminated

#EOF
