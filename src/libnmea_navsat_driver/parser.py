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
import rclpy

logger = rclpy.logging.get_logger('nmea_navsat_driver')


def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float('NaN')

def safe_float_except_star(field):
    try:
        return float(field.split('*')[0])
    except ValueError:
        return float('NaN')


def safe_int(field):
    try:
        return int(field)
    except ValueError:
        return 0


def convert_latitude(field):
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0


def convert_longitude(field):
    degree = safe_float(field[-len(field):-11])
    minute = safe_float(field[-11:-1])
    if(degree < 0):
        minute = -minute
    return degree + minute / 60.0


def convert_time(nmea_utc):
    # Get current time in UTC for date information
    utc_struct = time.gmtime()  # immutable, so cannot modify this one
    utc_list = list(utc_struct)
    # If one of the time fields is empty, return NaN seconds
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
        return float('NaN')
    else:
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
    return safe_float(knots) * 0.514444444444


# Need this wrapper because math.radians doesn't auto convert inputs
def convert_deg_to_rads(degs):
    return math.radians(safe_float(degs))


"""Format for this dictionary is a sentence identifier (e.g. "GGA") as the key, with a
list of tuples where each tuple is a field name, conversion function and index
into the split sentence"""
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
        ("utc_time", convert_time, 1),
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
        ("true_course", convert_deg_to_rads, 1),
        ("speed", convert_knots_to_mps, 5)
    ],
    "CHC": [
        ("gps_week", int, 1),          # 自 1980-1-6 至当前的星期数(GPS 时间)
        ("gps_second", safe_float, 2), # 自本周日 0:00:00 至当前的秒数(GPS 时间)
        ("heading", safe_float, 3),    # 偏航角（0 至 359.99),单位度
        ("pitch", safe_float, 4),      # 俯仰角（-90 至 90),单位度
        ("roll", safe_float, 5),       # 横滚角（-180 至 180),单位度
        ("angular_velocity_x", safe_float, 6),    # 角速度 X 轴
        ("angular_velocity_y", safe_float, 7),    # 角速度 Y 轴
        ("angular_velocity_z", safe_float, 8),    # 角速度 Z 轴
        ("linear_acceleration_x", safe_float, 9), # 加速度 X 轴
        ("linear_acceleration_y", safe_float, 10),# 加速度 Y 轴
        ("linear_acceleration_z", safe_float, 11),# 加速度 Z 轴
        
        ("latitude", safe_float, 12),             # 纬度(-90°至90°),单位度
        ("longitude", safe_float, 13),            # 经度(-180°至180°),单位度
        ("altitude", safe_float, 14),             # 高度，单位（米）
        ("linear_velocity_east", safe_float, 15), # 东向速度，m/s 
        ("linear_velocity_north", safe_float, 16),# 北向速度，m/s
        ("linear_velocity_z", safe_float, 17),    # 天向速度 m/s
        ("linear_velocity_vehihle", safe_float, 18),    #  车辆速度 m/s
        
        ("main_antenna_1_satellite_count", int, 19),    # 主天线 1 卫星数
        ("auxiliary_antenna_2_satellite_count", int, 20),    # 副天线 2 卫星数
        
        ("fix_valid", int, 21), # 系统状态 0-9
        ("age", safe_float, 22), # 差分延时
    ], 

    "TMSENMSG": [
        ("timestamp", int, 2),                      # 时间戳
        ("temp", safe_float, 3),                    # 温度（单位：°C）
        ("angular_velocity_x", safe_float, 5),      # 角速度 X 轴   （硬件 y -> 逻辑 x）
        ("angular_velocity_y", safe_float, 4),      # 角速度 Y 轴   （硬件 x -> 逻辑 y）
        ("angular_velocity_z", safe_float, 6),      # 角速度 Z 轴
        ("linear_acceleration_x", safe_float, 8),   # 线性加速度 X 轴  （硬件 y -> 逻辑 x）
        ("linear_acceleration_y", safe_float, 7),   # 线性加速度 Y 轴  （硬件 x -> 逻辑 y）
        ("linear_acceleration_z", safe_float_except_star, 9),   # 线性加速度 Z 轴
    ],
}


def parse_nmea_sentence(nmea_sentence):
    # Check for a valid nmea sentence

    if not re.match(r'(^\$GP|^\$GN|^\$GL|^\$IN|^\$PQ).*\*[0-9A-Fa-f]{2}$', nmea_sentence):
        logger.debug("Regex didn't match, sentence not valid NMEA? Sentence was: %s"
                     % repr(nmea_sentence))
        return False
    fields = [field.strip(',') for field in nmea_sentence.split(',')]

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

    return {sentence_type: parsed_sentence}
