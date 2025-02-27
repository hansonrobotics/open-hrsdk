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

upload() {
    # upload dump file to s3
    local dump_file=$1
    BUCKET=s3://datastore.hansonrobotics.com
    SYNC="aws s3 cp --region=ap-east-1 --no-follow-symlinks"
    $SYNC ${dump_file} "$BUCKET/mysql dumps/"
}

if ! hash mysqldump; then
    sudo apt install mysql-client
fi

datestr=$(date -Iseconds)
dump_file_prefix="$(whoami)_$(hostname)_${datestr}"
DATABASE=conversations
MYSQL="mysqldump --protocol tcp -u root $DATABASE --complete-insert --insert-ignore"

export MYSQL_PWD=$(>&2 read -s -p "Input MySQL database password: "; echo "$REPLY")
echo

dump_file="${dump_file_prefix}_${DATABASE}.dump"
$MYSQL --no-create-info > ${dump_file}
echo "${dump_file}"
upload ${dump_file}
