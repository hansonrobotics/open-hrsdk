#
# Copyright (C) 2017-2024 Hanson Robotics
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
import datetime as dt
import math
import os
import random
import subprocess

from babel.dates import format_date, format_time

from haipy.parameter_server_proxy import ParameterServerProxy

counter = 0


def init_counter(init: int = 0, **kwargs):
    """Initialize the counter"""
    global counter
    counter = init
    return counter


def get_counter(**kwargs):
    return counter


def inc_counter(step: int = 1, **kwargs):
    """Increase the counter"""
    global counter
    counter += step
    return counter


def dec_counter(step: int = 1, **kwargs):
    """Decrease the counter"""
    global counter
    counter -= step
    return counter


def time(offset=0, format="short", locale="en_US", **kwargs):
    now = dt.datetime.now()
    now += dt.timedelta(seconds=offset)
    return format_time(now, format=format, locale=locale)


def date(offset, format="short", locale="en_US", **kwargs):
    now = dt.datetime.now()
    now += dt.timedelta(seconds=offset)
    return format_date(now, format=format, locale=locale)


def run(command, **kwargs):
    return subprocess.check_output(command, shell=True, stderr=subprocess.STDOUT)


def env(variable, **kwargs):
    return os.environ.get(variable)


def sqrt(num, **kwargs):
    return math.sqrt(num)


def random_choice(items, **kwargs):
    return random.choice(items)


def ExpiryValue(value, time, **kwargs):
    from haipy.utils import ExpiryValue

    return ExpiryValue(value, time)
