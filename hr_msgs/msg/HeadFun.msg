##
## Copyright (C) 2017-2024 Hanson Robotics
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
geometry_msgs/Quaternion headRotation
geometry_msgs/Vector3    neckDirection

float64 mouth          # 0: closed mouth -> 1: open mouth
float64 eyeClosedLeft  # 0: open eye -> 1: closed eye
float64 eyeClosedRight # 0: open eye -> 1: closed eye
float64 eyeHDirLeft    # horizontal direction (-1 to 1)
float64 eyeHDirRight   # horizontal direction (-1 to 1)
