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
#
# Sync the local files with S3
# Needs to configure the AWS credentials to get the permission to
# write to S3
#
# aws configure --profile hrsdk_admin
#
# Usage:
# ./download.sh <folder_name>
#

set -e
BUCKET=s3://dl.hrsdk.hansonrobotics.com
SYNC="aws s3 sync --region=ap-east-1 --delete --profile hrsdk_admin"

folder_name=${1}
dest=/tmp/$1
if [[ ! -z $folder_name ]]; then
    echo "Folder \"$folder_name\", destination \"$dest\""
    $SYNC ${BUCKET}/${folder_name} ${dest}
    if [[ $? == 0 ]]; then
        echo "Downloaded $folder_name to $dest"
        find $dest -type d -empty -delete  # remove empty folders due the the bug of s3 sync https://github.com/aws/aws-cli/issues/2685
    else
        echo "Download error"
    fi
else
    echo "./download.sh <folder_name>"
    exit 1
fi
