# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Parsing functions for NMEA sentence strings."""

import re
import datetime
import calendar
import math
import logging


logger = logging.getLogger('rosout')


field_delimiter_regex = re.compile(r'[,*]')


def safe_float(field):
    """Convert  field to a float.

    Args:
        field: The field (usually a str) to convert to float.

    Returns:
        The float value represented by field or NaN if float conversion throws a ValueError.
    """
    try:
        return float(field)
    except ValueError:
        return float('NaN')


def safe_int(field):
    """Convert  field to an int.

    Args:
        field: The field (usually a str) to convert to int.

    Returns:
        The int value represented by field or 0 if int conversion throws a ValueError.
    """
    try:
        return int(field)
    except ValueError:
        return 0


def convert_latitude(field):
    """Convert a latitude string to floating point decimal degrees.

    Args:
        field (str): Latitude string, expected to be formatted as DDMM.MMM, where
            DD is the latitude degrees, and MM.MMM are the minutes latitude.

    Returns:
        Floating point latitude in decimal degrees.
    """
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0


def convert_longitude(field):
    """Convert a longitude string to floating point decimal degrees.

    Args:
        field (str): Longitude string, expected to be formatted as DDDMM.MMM, where
            DDD is the longitude degrees, and MM.MMM are the minutes longitude.

    Returns:
        Floating point latitude in decimal degrees.
    """
    return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0


def convert_time(nmea_utc):
    """Extract time info from a NMEA UTC time string and use it to generate a UNIX epoch time.

    Time information (hours, minutes, seconds) is extracted from the given string and augmented
    with the date, which is taken from the current system time on the host computer (i.e. UTC now).
    The date ambiguity is resolved by adding a day to the current date if the host time is more than
    12 hours behind the NMEA time and subtracting a day from the current date if the host time is
    more than 12 hours ahead of the NMEA time.

    Args:
        nmea_utc (str): NMEA UTC time string to convert. The expected format is HHMMSS[.SS] where
            HH is the number of hours [0,24), MM is the number of minutes [0,60),
            and SS[.SS] is the number of seconds [0,60) of the time in UTC.

    Returns:
        tuple(int, int): 2-tuple of (unix seconds, nanoseconds) if the sentence contains valid time.
        tuple(float, float): 2-tuple of (NaN, NaN) if the sentence does not contain valid time.
    """
    # If one of the time fields is empty, return NaN seconds
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
        return (float('NaN'), float('NaN'))

    # Get current time in UTC for date information
    utc_time = datetime.datetime.utcnow()
    hours = int(nmea_utc[0:2])
    minutes = int(nmea_utc[2:4])
    seconds = int(nmea_utc[4:6])
    nanosecs = 0
    # If the seconds includes a decimal portion, convert it to nanoseconds
    if len(nmea_utc) > 7:
        nanosecs = int(nmea_utc[7:]) * pow(10, 9 - len(nmea_utc[7:]))

    # Resolve the ambiguity of day
    day_offset = int((utc_time.hour - hours)/12.0)
    utc_time += datetime.timedelta(day_offset)
    utc_time.replace(hour=hours, minute=minutes, second=seconds)

    unix_secs = calendar.timegm(utc_time.timetuple())
    return (unix_secs, nanosecs)


def convert_time_rmc(date_str, time_str):
    """Convert a NMEA RMC date string and time string to UNIX epoch time.

    Args:
        date_str (str): NMEA UTC date string to convert, formatted as DDMMYY.
        nmea_utc (str): NMEA UTC time string to convert. The expected format is HHMMSS.SS where
            HH is the number of hours [0,24), MM is the number of minutes [0,60),
            and SS.SS is the number of seconds [0,60) of the time in UTC.

    Returns:
        tuple(int, int): 2-tuple of (unix seconds, nanoseconds) if the sentence contains valid time.
        tuple(float, float): 2-tuple of (NaN, NaN) if the sentence does not contain valid time.
    """
    # If one of the time fields is empty, return NaN seconds
    if not time_str[0:2] or not time_str[2:4] or not time_str[4:6]:
        return (float('NaN'), float('NaN'))

    pc_year = datetime.date.today().year

    # Resolve the ambiguity of century
    """
    example 1: utc_year = 99, pc_year = 2100
    years = 2100 + int((2100 % 100 - 99) / 50.0) = 2099
    example 2: utc_year = 00, pc_year = 2099
    years = 2099 + int((2099 % 100 - 00) / 50.0) = 2100
    """
    utc_year = int(date_str[4:6])
    years = pc_year + int((pc_year % 100 - utc_year) / 50.0)

    months = int(date_str[2:4])
    days = int(date_str[0:2])

    hours = int(time_str[0:2])
    minutes = int(time_str[2:4])
    seconds = int(time_str[4:6])
    nanosecs = int(time_str[7:]) * pow(10, 9 - len(time_str[7:]))

    unix_secs = calendar.timegm((years, months, days, hours, minutes, seconds))
    return (unix_secs, nanosecs)


def convert_status_flag(status_flag):
    """Convert a NMEA RMB/RMC status flag to bool.

    Args:
        status_flag (str): NMEA status flag, which should be "A" or "V"

    Returns:
        True if the status_flag is "A" for Active.
    """
    if status_flag == "A":
        return True
    elif status_flag == "V":
        return False
    else:
        return False


def convert_knots_to_mps(knots):
    """Convert a speed in knots to meters per second.

    Args:
        knots (float, int, or str): Speed in knots.

    Returns:
        The value of safe_float(knots) converted from knots to meters/second.
    """
    return safe_float(knots) * 0.514444444444


def convert_deg_to_rads(degs):
    """Convert an angle in degrees to radians.

    This wrapper is needed because math.radians doesn't accept non-numeric inputs.

    Args:
        degs (float, int, or str): Angle in degrees

    Returns:
        The value of safe_float(degs) converted from degrees to radians.
    """
    return math.radians(safe_float(degs))


parse_maps = {
    "GGA": [
        ("fix_type", int, 6),
        ("latitude", convert_latitude, 2),
        ("latitude_direction", str, 3),
        ("longitude", convert_longitude, 4),
        ("longitude_direction", str, 5),
        ("altitude", safe_float, 9),
        ("mean_sea_level", safe_float, 11),
        ("hdop", safe_float, 8),
        ("num_satellites", safe_int, 7),
        ("utc_time", convert_time, 1),
    ],
    "RMC": [
        ("fix_valid", convert_status_flag, 2),
        ("latitude", convert_latitude, 3),
        ("latitude_direction", str, 4),
        ("longitude", convert_longitude, 5),
        ("longitude_direction", str, 6),
        ("speed", convert_knots_to_mps, 7),
        ("true_course", convert_deg_to_rads, 8),
    ],
    "GST": [
        ("utc_time", convert_time, 1),
        ("ranges_std_dev", safe_float, 2),
        ("semi_major_ellipse_std_dev", safe_float, 3),
        ("semi_minor_ellipse_std_dev", safe_float, 4),
        ("semi_major_orientation", safe_float, 5),
        ("lat_std_dev", safe_float, 6),
        ("lon_std_dev", safe_float, 7),
        ("alt_std_dev", safe_float, 8),
    ],
    "HDT": [
        ("heading", safe_float, 1),
    ],
    "VTG": [
        ("true_course", safe_float, 1),
        ("speed", convert_knots_to_mps, 5)
    ]
}
"""A dictionary that maps from sentence identifier string (e.g. "GGA") to a list of tuples.
Each tuple is a three-tuple of (str: field name, callable: conversion function, int: field index).
The parser splits the sentence into comma-delimited fields. The string value of each field is passed
to the appropriate conversion function based on the field index."""


def parse_nmea_sentence(nmea_sentence):
    """Parse a NMEA sentence string into a dictionary.

    Args:
        nmea_sentence (str): A single NMEA sentence of one of the types in parse_maps.

    Returns:
        A dict mapping string field names to values for each field in the NMEA sentence or
        False if the sentence could not be parsed.
    """
    # Check for a valid nmea sentence
    nmea_sentence = nmea_sentence.strip()  # Cut possible carriage return or new line of NMEA Sentence
    if not re.match(
            r'(^\$GP|^\$GN|^\$GL|^\$IN).*\*[0-9A-Fa-f]{2}$', nmea_sentence):
        logger.debug(
            "Regex didn't match, sentence not valid NMEA? Sentence was: %s" %
            repr(nmea_sentence))
        return False
    fields = [field for field in field_delimiter_regex.split(nmea_sentence)]

    # Ignore the $ and talker ID portions (e.g. GP)
    sentence_type = fields[0][3:]

    if sentence_type not in parse_maps:
        logger.debug("Sentence type %s not in parse map, ignoring."
                     % repr(sentence_type))
        return False

    parse_map = parse_maps[sentence_type]

    parsed_sentence = {}
    for entry in parse_map:
        parsed_sentence[entry[0]] = entry[1](fields[entry[2]])

    if sentence_type == "RMC":
        parsed_sentence["utc_time"] = convert_time_rmc(fields[9], fields[1])

    return {sentence_type: parsed_sentence}
