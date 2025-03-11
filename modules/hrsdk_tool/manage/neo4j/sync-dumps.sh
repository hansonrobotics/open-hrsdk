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

# start to upload the speech audio to s3
#

set -e

BUCKET=s3://datastore.hansonrobotics.com
SYNC="aws s3 sync --region=ap-east-1 --no-follow-symlinks --include *.dump"

data_dir=$(docker volume inspect hrsdk_graphdb | jq -r ".[0].Mountpoint")

if sudo [ -d $data_dir/dumps ]; then
    sudo -E env "PATH=$PATH" $SYNC ${data_dir}/dumps "$BUCKET/neo4j dumps"
fi
