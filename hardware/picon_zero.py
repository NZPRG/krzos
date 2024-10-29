#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-10-18
# modified: 2024-10-19
#

import time
from smbus2 import SMBus
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.orientation import Orientation

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class PiconZero(object):
    BUS_NUMBER = 1     # indicates /dev/i2c-1; For revision 1 Raspberry Pi, change to bus = smbus.SMBus(0)
    # general variables ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    RETRIES = 10   # max number of retries for I2C calls
    # command definitions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    MOTORA    = 0
    OUTCFG0   = 2
    OUTPUT0   = 8
    INCFG0    = 14
    SETBRIGHT = 18
    UPDATENOW = 19
    RESET     = 20
    '''
    Based on original Python library for 4tronix Picon Zero.

    Note that all I2C accesses are wrapped in try clauses with repeats.

    Address is 0x22 or 0x23.

    :param: orientation  FWD or AFT
    :param: level       the logging level
    '''
    def __init__(self, orientation=None, level=Level.INFO):
        super().__init__()
        self._log = Logger("picon-zero", level)
        self._bus = None
        if orientation is Orientation.FWD:
            self._i2c_address = 0x22
        elif orientation is Orientation.AFT:
            self._i2c_address = 0x23
        else:
            raise ValueError('expected FWD or AFT orientation.')
        try:
            self._log.info('initialising smbus…')
            self._bus = SMBus(PiconZero.BUS_NUMBER)
            self._log.info(Fore.WHITE + 'picon-zero ready on 0x{:02X}.'.format(self._i2c_address))
        except ImportError:
            self._log.warning('import error, unable to initialise: this script requires smbus2.')
        except Exception as e:
            self._log.error('{} encountered while initialising I²C bus: {}\n{}'.format(type(e), e, traceback.format_exc()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def I2cAddress(self):
        return self._i2c_address

    # version/revision info ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def getRevision(self):
        '''
        Get Version and Revision info.
        '''
        for _ in range(PiconZero.RETRIES):
            try:
                rval = self._bus.read_word_data(self._i2c_address, 0)
                return [rval/256, rval%256]
            except Exception as e:
                self._log.error("Error in getRevision(): {}; retrying…".format(e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def initialise(self, debug=False):
        '''
        Initialise the Board (same as close).
        '''
        for _ in range(PiconZero.RETRIES):
            try:
                self._bus.write_byte_data(self._i2c_address, PiconZero.RESET, 0)
                break
            except Exception as e:
                self._log.error("Error in initialise(): {}; retrying…".format(e))
        time.sleep(0.01) # 1ms delay to allow time to complete

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Close the Board (same as initialise).
        This was called cleanup in the original code.
        '''
        for _ in range(PiconZero.RETRIES):
            try:
                self._bus.write_byte_data(self._i2c_address, PiconZero.RESET, 0)
                break
            except Exception as e:
                self._log.error("Error in close(): {}; retrying…".format(e))
        time.sleep(0.001) # 1ms delay to allow time to complete

    # motor control ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_motor(self, motor, value):
        '''
        Motor must be in range 0..1
        Value must be in range -128 - +127
        Values of -127, -128, +127 are treated as always ON,, no PWM
        '''
#       self._log.info(Fore.GREEN + 'set motor {} to {}.'.format(motor, value))
        if (motor>=0 and motor<=1 and value>=-128 and value<128):
            for _ in range(PiconZero.RETRIES):
                try:
                    self._bus.write_byte_data(self._i2c_address, motor, value)
                    break
                except Exception as e:
                    self._log.error("Error in set_motor(): {}; retrying…".format(e))

    def forward(self, speed):
        self._log.info('🍏 forward.')
        self.set_motor(0, speed)
        self.set_motor(1, speed)

    def reverse(self, speed):
        self._log.info('🍏 reverse..')
        self.set_motor(0, -speed)
        self.set_motor(1, -speed)

    def spinLeft(self, speed):
        self._log.info('🍏 spin left.')
        self.set_motor(0, -speed)
        self.set_motor(1, speed)

    def spinRight(self, speed):
        self._log.info('🍏 spin right.')
        self.set_motor(0, speed)
        self.set_motor(1, -speed)

    def stop(self):
        self._log.info('🍎 stop.')
        self.set_motor(0, 0)
        self.set_motor(1, 0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def readInput(self, channel):
        '''
        Read data for selected input channel (analog or digital).
        Channel is in range 0 to 3.
        '''
        if (channel>=0 and channel <=3):
            for _ in range(PiconZero.RETRIES):
                try:
                    return self._bus.read_word_data(self._i2c_address, channel + 1)
                except Exception as e:
                    self._log.error("Error in readChannel(): {}; retrying…".format(e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def setOutputConfig(self, output, value):
        '''
        Set configuration of selected output.

        0: On/Off, 1: PWM, 2: Servo, 3: WS2812B
        '''
        if (output>=0 and output<=5 and value>=0 and value<=3):
            for _ in range(PiconZero.RETRIES):
                try:
                    self._bus.write_byte_data(self._i2c_address, PiconZero.OUTCFG0 + output, value)
                    break
                except Exception as e:
                    self._log.error("Error in setOutputConfig(): {}; retrying…".format(e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def setInputConfig(self, channel, value, pullup = False):
        '''
        Set configuration of selected input channel.

        0: Digital, 1: Analog
        '''
        if (channel>=0 and channel <=3 and value>=0 and value<=3):
            if (value==0 and pullup==True):
                value = 128
            for _ in range(PiconZero.RETRIES):
                try:
                    self._bus.write_byte_data(self._i2c_address, PiconZero.INCFG0 + channel, value)
                    break
                except Exception as e:
                    self._log.error("Error in setInputConfig(): {}; retrying…".format(e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def setOutput(self, channel, value):
        '''
        Set output data for selected output channel.

          Mode   Name      Type      Values
          0      On/Off    Byte      0 is OFF, 1 is ON
          1      PWM       Byte      0 to 100 percentage of ON time
          2      Servo     Byte      -100 to + 100 Position in degrees
          3      WS2812B   4 Bytes   0:Pixel ID, 1:Red, 2:Green, 3:Blue
        '''
        if (channel>=0 and channel<=5):
            for _ in range(PiconZero.RETRIES):
                try:
                    self._bus.write_byte_data(self._i2c_address, PiconZero.OUTPUT0 + channel, value)
                    break
                except Exception as e:
                    self._log.error("Error in setOutput(): {}; retrying…".format(e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def setPixel(self, Pixel, Red, Green, Blue, Update=True):
        '''
        Set the colour of an individual pixel (always output 5).
        '''
        pixelData = [Pixel, Red, Green, Blue]
        for _ in range(PiconZero.RETRIES):
            try:
                self._bus.write_i2c_block_data(self._i2c_address, Update, pixelData)
                break
            except Exception as e:
                self._log.error("Error in setPixel(): {}; retrying…".format(e))

    def setAllPixels(self, Red, Green, Blue, Update=True):
        '''
        Set the colour of all pixels (always output 5).
        '''
        pixelData = [100, Red, Green, Blue]
        for _ in range(PiconZero.RETRIES):
            try:
                self._bus.write_i2c_block_data(self._i2c_address, Update, pixelData)
                break
            except:
                self._log.error("Error in setAllPixels(), retrying…")

    def updatePixels(self):
        '''
        Update the colour of all pixels (always output 5).
        '''
        for _ in range(PiconZero.RETRIES):
            try:
                self._bus.write_byte_data(self._i2c_address, PiconZero.UPDATENOW, 0)
                break
            except Exception as e:
                self._log.error("Error in updatePixels(): {}; retrying…".format(e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def setBrightness(self, brightness):
        '''
        Set the overall brightness of pixel array.
        '''
        for _ in range(PiconZero.RETRIES):
            try:
                self._bus.write_byte_data(self._i2c_address, PiconZero.SETBRIGHT, brightness)
                break
            except Exception as e:
                self._log.error("Error in setBrightness(): {}; retrying…".format(e))

#EOF
