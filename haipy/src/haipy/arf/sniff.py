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
import csv
import datetime
import logging
import re

from haipy.utils import parse_yaml_str, to_number

logger = logging.getLogger(__name__)
BOOLEAN_MAPPING = {
    "t": True,
    "true": True,
    "yes": True,
    "f": False,
    "false": False,
    "no": False,
}

TIMEZONE_PATTERN = re.compile(
    r"""^UTC([+-])(\d{1,2})(:(\d{1,2}))?$""", flags=re.IGNORECASE
)


def sniff(csvfile, convert=True):
    """Detects the beginning row of the header and parse template settings"""
    header = {}
    skiprows = 0
    with open(csvfile, newline="") as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            row = [r.strip() for r in row]
            cleanrow = [r for r in row if r]  # non-empty row
            if cleanrow:
                if skiprows > 0:
                    break
                key = row[0]
                value = row[1] if len(row) > 1 else ""
                header[key] = value
            else:
                skiprows = i + 1
    if skiprows == 0:
        skiprows = len(header) + 1

    if convert:
        tzinfo = datetime.timezone.utc
        if "Timezone" in header and header["Timezone"].strip():
            timezone = header["Timezone"].strip()
            matchobj = TIMEZONE_PATTERN.match(timezone)
            if matchobj:
                sign, hours, _, minutes = matchobj.groups()
                if hours is None:
                    hours = 0
                if minutes is None:
                    minutes = 0
                hours = int(hours)
                minutes = int(minutes)
                if sign == "-":
                    if hours:
                        hours = -int(hours)
                    if minutes:
                        minutes = -int(minutes)
                timedelta = datetime.timedelta(hours=hours, minutes=minutes)
                tzinfo = datetime.timezone(timedelta, name="Customized")
            else:
                logger.error("Illegal timezone format %r", timezone)
        for key, value in header.items():
            if key in ["Start", "End"]:
                value = datetime.datetime.fromisoformat(value).replace(tzinfo=tzinfo)
                if key == "End" and value.time() == datetime.time(0, 0):
                    value += datetime.timedelta(hours=24)
            if key == "Variables":
                value = parse_yaml_str(value)
            if isinstance(value, str):
                if "\n" in value:
                    value = [v.strip() for v in re.split("\n", value) if v.strip()]
            if isinstance(value, str):
                if re.match(r"""^(\d+(?:\.\d+)?)$""", value):
                    value = to_number(value)
            if isinstance(value, str):
                value = BOOLEAN_MAPPING.get(value.lower(), value)
            if value != header[key]:
                header[key] = value

    return header, skiprows
