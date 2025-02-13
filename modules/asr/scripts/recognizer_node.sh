#!/usr/bin/env bash
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

BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

if [[ ! -z $VIRTUAL_ENV ]]; then
    PYTHON=python
else
    if [[ -f /opt/hansonrobotics/py2env/bin/python ]]; then
        PYTHON=/opt/hansonrobotics/py2env/bin/python
    else
        echo "ERROR: No python environment was found!" >&2
        exit 1
    fi
fi

$PYTHON $BASEDIR/recognizer_node.py $@
