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

####################### Credentials #######################

# AWS user to download the containers and configs
export AWS_ACCESS_KEY_ID= #<provided by Hanson Robotics>
export AWS_SECRET_ACCESS_KEY= #<provided by Hanson Robotics>
export AWS_REGION_NAME= #<provided by Hanson Robotics>
export REGISTRY_PREFIX=default # optional for custom docker registry
export AWS_DEFAULT_REGION=$AWS_REGION_NAME

####################### SDK Settings ######################

# SDK version
export HRSDK_VERSION= #<provided by Hanson Robotics>

# the config name for fetching the configs
export CONFIG_NAME= #<provided by Hanson Robotics>

# the environment of the HRSDK deployment
export ENVIRONMENT=default

####################### Robot Settings ####################

# The SDK perception requires at least a working chest camera in order to
# stream the video. The device path could be /dev/video0
export CHEST_CAMERA_DEVICE= # <path of the device>
export EYE_CAMERA_DEVICE= # <path of the device>

export USING_LOCAL_STORAGE=  # 1: enable
export LOCAL_STORAGE_PATH=  # <path of the local storage directory>
