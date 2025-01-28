#
# Copyright (C) 2017-2025 Hanson Robotics
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
import logging
import random
import re

import six
from babel.dates import (
    _get_datetime,
    format_date,
    format_datetime,
    format_time,
    format_timedelta,
    parse_date,
)

logger = logging.getLogger(__name__)
PUNCTUATORS = re.compile(r"""([.?!]+)""")


def plural(text, **kwargs):
    return "%ss" % text


def rstrip(value, text, **kwargs):
    return str(value).rstrip(str(text))


def lstrip(value, text, **kwargs):
    return str(value).lstrip(str(text))


def repr(text, **kwargs):
    return text.replace("_", " ")


def title_case(text, **kwargs):
    if text and text[0].isupper():
        return text
    else:
        return text.title()


def article(text, **kwargs):
    if text.startswith("a ") or text.startswith("an "):
        return text
    if text and text[0].isalpha():
        vowels = ["a", "e", "i", "o", "u"]
        if any([text.startswith(v) for v in vowels]):
            return "an " + text
        else:
            return "a " + text
    return text


def date(text, format="medium", locale="en_US", **kwargs):
    """format date
    format: full", "long", "medium", or "short"
    """
    if isinstance(text, dt.datetime):
        return format_date(text, format=format, locale=locale)
    elif text and isinstance(text, six.string_types):
        return format_date(parse_date(text), format=format, locale=locale)
    raise ValueError("Can't format date %s" % text)


def time(text, format="medium", locale="en_US", **kwargs):
    """format time
    format: full", "long", "medium", or "short"
    """
    if isinstance(text, dt.datetime):
        return format_time(text, format=format, locale=locale)
    else:
        return format_time(parse_date(text), format=format, locale=locale)


def datetime(text, format="medium", locale="en_US", **kwargs):
    """format datetime
    format: full", "long", "medium", or "short"
    """
    if isinstance(text, dt.datetime):
        return format_datetime(text, format=format, locale=locale)
    else:
        return format_datetime(parse_date(text), format=format, locale=locale)


def timestamp(text, **kwargs):
    """convert timestamp to datetime"""
    if text and isinstance(text, str):
        float_pattern = re.compile(r"^[+-]?([0-9]*[.])?[0-9]+$")
        if float_pattern.match(text):
            text = float(text)
    return _get_datetime(text)


def timedelta(datetime, locale="en_US", **kwargs):
    """format time delta"""
    now = dt.datetime.now()
    delta = (now - datetime).seconds
    return format_timedelta(delta)


def binary(integer, **kwargs):
    return " ".join(list("{0:08b}".format(integer)))


def shorten(text, max_len, **kwargs):
    """soft truncate the text"""

    def str_cleanup(text):
        if text:
            text = text.strip()
            text = " ".join(text.split())
            if text and text[0] == ".":
                text = text[1:]
        return text

    if not text or len(text.split()) < max_len:
        # No need to cut off
        return text
    sens = PUNCTUATORS.split(text)
    ret = ""
    idx = 0
    while idx < len(sens):
        chunk = ""
        if sens[idx]:
            if idx + 1 < len(sens):
                punctuator = sens[idx + 1]
                chunk = sens[idx] + punctuator
            else:
                chunk = sens[idx]
            next_text = ret + chunk
            if len(next_text.split()) > max_len:
                if len(ret.split()) > 3:  # if truncated text is long enough, stop
                    break
            ret = next_text
            idx += 1
        idx += 1

    res = "".join(sens[idx:])

    # If rest part is too short, then don't cut
    if len(res.split()) < 4:
        ret = text
        res = ""

    ret = str_cleanup(ret)
    res = str_cleanup(res)
    if text != ret:
        logger.warn(
            "Truncate text: %s, length: %s, max length: %s",
            ret,
            len(ret.split()),
            max_len,
        )
    else:
        logger.info("Doesn't truncate text")
    return ret


def random_pick(arr, **kwargs):
    return random.choice(arr)


def tokenize(text, **kwargs):
    if text:
        return text.split()
    else:
        return ""
