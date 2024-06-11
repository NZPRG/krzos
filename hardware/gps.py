#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-06-11
# modified: 2024-06-11
#

from colorama import init, Fore, Style
init()

from pa1010d import PA1010D

from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GPS(object):
    def __init__(self, level):
        '''
        Wraps a PA1010D GPS sensor in a class. A typical output is:

            Timestamp: 05:09:29+00:00
            Latitude:  -41.27849
            Longitude: 174.77646
            Altitude:  8.2
            Sats:      9
            Quality:   1
            Speed:     0.25
            Fix Type:  3
            PDOP:      1.38
            VDOP:      0.90
            HDOP:      1.05

        Values for Position Dilution-of-Precision (PDOP), Horizontal DOP (HDOP) and Vertical DOP (VDOP):

            DOP Value     Rating     Description
            1             Ideal      Highest possible confidence level to be used for applications
                                     demanding the highest possible precision at all times.
            1-2           Excellent  At this confidence level, positional measurements are
                                     considered accurate enough to meet all but the most sensitive
                                     applications.
            2-5           Good       Represents a level that marks the minimum appropriate for
                                     making accurate decisions. Positional measurements could be used
                                     to make reliable in-route navigation suggestions to the user.
            5-10          Moderate   Positional measurements could be used for calculations, but the
                                     fix quality could still be improved. A more open view of the sky
                                     is recommended.
            10-20         Fair       Represents a low confidence level. Positional measurements should
                                     be discarded or used only to indicate a very rough estimate of the
                                     current location.
            >20           Poor       At this level, measurements are inaccurate by as much as 300 meters
                                     with a 6-meter accurate device (50 DOP × 6 meters) and should be
                                     discarded.
        '''
        self._log  = Logger('gps', level)
        self._gps  = PA1010D()
        self._data      = None
        self._timestamp = None
        self._latitude  = None
        self._longitude = None
        self._lat_dir   = None
        self._lon_dir   = None
        self._altitude  = None
        self._geo_sep   = None
        self._num_sats  = 0
        self._gps_qual  = None
        self._speed     = 0.0
        self._pdop      = 0
        self._hdop      = 0
        self._vdop      = 0
        self._verbose   = False
        self._log.info('ready.')

    def set_verbose(self, verbose):
        self._verbose = verbose

    @property
    def timestamp(self):
        return self._timestamp

    @property
    def latitude(self):
        return self._latitude

    @property
    def longitude(self):
        return self._longitude

    @property
    def lat_dir(self):
        return self._lat_dir

    @property
    def lon_dir(self):
        return self._lon_dir

    @property
    def altitude(self):
        return self._altitude

    @property
    def geo_sep(self):
        return self._geo_sep

    @property
    def number_of_satellites(self):
        return self._num_sats

    @property
    def gps_quality(self):
        return self._gps_qual

    @property
    def speed(self):
        return self._speed

    @property
    def pdop(self):
        return self._pdop

    @property
    def hdop(self):
        return self._hdop

    @property
    def vdop(self):
        return self._vdop


    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def poll(self):
        '''
        Polls the GPS sensor, populating the various values.
        If the poll fails this will zero or nullify all values.
        '''
        _style = Style.DIM
        result = self._gps.update()
        _data = self._gps.data
        if result and (_data is not None):
            self._timestamp = _data.get('timestamp')
            self._latitude  = float(_data.get('latitude'))
            self._longitude = float(_data.get('longitude'))
            self._lat_dir   = _data.get('lat_dir')
            self._lon_dir   = _data.get('lon_dir')

            _altitude  = _data.get('altitude')
            self._altitude  = 0.0 if _altitude is None else float(_altitude)
            self._geo_sep   = _data.get('geo_sep')

            _num_sats = _data.get('num_sats')
            self._num_sats  = 0 if _num_sats is None else int(_num_sats)
            self._gps_qual  = float(_data.get('gps_qual'))
            self._speed     = _data.get('speed_over_ground')
            self._pdop      = _data.get('pdop')
            self._hdop      = _data.get('hdop')
            self._vdop      = _data.get('vdop')
            if self._num_sats is not None and self._num_sats > 1:
                _style = Style.NORMAL
            if self._verbose:
                self._log.info(_style + '''
                                  Timestamp:  {timestamp}
                                  Latitude:   {latitude}
                                  Longitude:  {longitude}
                                  Lat Dir:    {lat_dir}
                                  Long Dir:   {lon_dir}
                                  Altitude:   {altitude}
                                  Geo Sep:    {geo_sep}
                                  Sats:       {num_sats}
                                  Quality:    {gps_qual}
                                  Speed:      {speed_over_ground}
                                  Fix Type:   {mode_fix_type}
                                  PDOP:       {pdop}
                                  VDOP:       {vdop}
                                  HDOP:       {hdop}'''.format(**_data))
            return True
        else:
            self._timestamp = 'na'
            self._latitude  = 0.0
            self._longitude = 0.0
            self._lat_dir   = 'na'
            self._lon_dir   = 'na'
            self._altitude  = 0.0
            self._geo_sep   = 0.0
            self._num_sats  = 0
            self._gps_qual  = 0.0
        return False

#EOF
