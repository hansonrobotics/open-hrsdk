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

set -e

if ! hash mysqldump; then
    sudo apt install mysql-client
fi

DATABASE=conversations
dump_file=${1}
if [[ -f $dump_file ]]; then
    echo "MySql Password"
    mysql --protocol tcp -u root -P 30306 -p $DATABASE < $dump_file
    echo "imported"
else
    echo "Usage: $0 <dump_file>"
fi
