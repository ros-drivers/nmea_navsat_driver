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

import re
import time
import calendar
import math

def convert_latitude(field):
    return float(field[0:2])+float(field[2:])/60.0

def convert_longitude(field):
    return float(field[0:3])+float(field[3:])/60.0

def convert_time(nmea_utc):
    #Get current time in UTC for date information
    utc_struct = time.gmtime() #immutable, so cannot modify this one
    utc_list = list(utc_struct)
    hours = int(nmea_utc[0:2])
    minutes = int(nmea_utc[2:4])
    seconds = int(nmea_utc[4:6])
    utc_list[3] = hours
    utc_list[4] = minutes
    utc_list[5] = seconds
    unix_time = calendar.timegm(tuple(utc_list))
    return unix_time

def convert_status_flag(status_flag):
    if status_flag == "A":
        return True
    elif status_flag == "V":
        return False
    else:
        return False

def convert_knots_to_mps(knots):
    return float(knots)*0.514444444444

# Need this wrapper because math.radians doesn't auto convert inputs
def convert_deg_to_rads(degs):
    return math.radians(float(degs))

"""Format for this is a sentence identifier (e.g. "GGA") as the key, with a
tuple of tuples where each tuple is a field name, conversion function and index
into the split sentence"""
parse_maps = {
        "GGA": [
            ("fix_type", int, 6),
            ("latitude", convert_latitude, 2),
            ("latitude_direction", str, 3),
            ("longitude", convert_longitude, 4),
            ("longitude_direction", str, 5),
            ("altitude", float, 9),
            ("mean_sea_level", float, 11),
            ("hdop", float, 8),
            ("num_satellites", int, 7),
            ("utc_time", convert_time, 1),
            ],
        "RMC": [
            ("utc_time", convert_time, 1),
            ("fix_valid", convert_status_flag, 2),
            ("latitude", convert_latitude, 3),
            ("latitude_direction", str, 4),
            ("longitude", convert_longitude, 5),
            ("longitude_direction", str, 6),
            ("speed", convert_knots_to_mps, 7),
            ("true_course", convert_deg_to_rads, 8),
            ]
        }

def parse_nmea_sentence(nmea_sentence):
    # Check for a valid nmea sentence
    if not re.match('^\$GP.*\*[0-9A-Fa-f]{2}$', nmea_sentence):
        #print "Regex didn't match"
        return False
    fields = [field.strip(',') for field in nmea_sentence.split(',')]

    # Ignore the $ and talker ID portions (e.g. GP)
    sentence_type = fields[0][3:]

    if not sentence_type in parse_maps:
        #print sentence_type
        #print "Sentence type not in parse map"
        return False

    parse_map = parse_maps[sentence_type]

    parsed_sentence = {}
    for entry in parse_map:
        parsed_sentence[entry[0]] = entry[1](fields[entry[2]])

    return {sentence_type: parsed_sentence}
