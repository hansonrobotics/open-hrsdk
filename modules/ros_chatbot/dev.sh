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

source ~/workspace/hrsdk_configs/scripts/_env/10-defaults.sh 
export HR_OPENAI_PROXY=https://openai.hr-tools.io
export HR_CHARACTER="sophia"
export ROBOT_NAME="sophia54"
export CMS_DIR="/tmp/cms"
export HR_CHATBOT_WORLD_DIR=/home/hr/workspace/hrsdk_configs/characters/sophia/worlds/lab/
export HR_CHATBOT_DATA_DIR=/home/hr/workspace/hrsdk_configs/characters/sophia/data/
export SOULTALK_HOT_UPLOAD_DIR=/tmp
export HR_MODES_FILE=/home/hr/workspace/hrsdk_configs/configs/common/all/modes.yaml
roslaunch launch/sdk.launch