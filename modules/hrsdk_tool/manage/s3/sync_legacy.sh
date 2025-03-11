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

# Sync the local files with S3
# Needs to configure the AWS credentials to get the permission to
# write to S3
#
# aws configure --profile hrsdk_admin
#
# Usage:
# ./sync.sh <root_dir>
# default root_dir is $HOME/hansonrobotics
#

BUCKET=s3://dl.hrsdk.hansonrobotics.com/hrsdk
SYNC="aws s3 sync --region=ap-east-1 --no-follow-symlinks --delete --profile hrsdk_admin "

HR_WORKSPACE=${1:-$HOME/hansonrobotics}

# configs
$SYNC $HR_WORKSPACE/configs $BUCKET/configs --exclude ".*"

# performances
for robot in sophia20 sophia17; do
    $SYNC $HR_WORKSPACE/performances_content/$robot $BUCKET/performances/$robot --exclude ".*"
done

# blender
$SYNC $HR_WORKSPACE/HEAD/src/sophia_blender_lfs/ $BUCKET/blend_files/ --exclude "*" --include "Sophia17.blend" --include "Sophia20.blend" --include "Virtual_sophia.blend"

# roodle
$SYNC $HR_WORKSPACE/tools/roodle/ $BUCKET/roodle/ --exclude "*" --include "Robot/*" --include "config/*" --include "strokes/*"

# tts voice
$SYNC $HR_WORKSPACE/HEAD/src/ttsserver/voices/cereproc/voices/ $BUCKET/tts/voices/ --exclude ".*"
