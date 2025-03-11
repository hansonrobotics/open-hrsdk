##
## Copyright (C) 2017-2025 Hanson Robotics
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.
##

import yaml
import json
import os

current_dir = os.path.dirname(os.path.realpath(__file__))

def parse(filename):
    with open(os.path.join(current_dir, filename + '.yaml'), 'r') as stream:
        try:
            return json.dumps(yaml.safe_load(stream))
        except yaml.YAMLError as exc:
            return False

def load(filename):
    with open(os.path.join(current_dir, filename + '.yaml'), 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            return False
