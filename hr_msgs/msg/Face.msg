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
std_msgs/Header header
string id
geometry_msgs/Pose left_gaze
geometry_msgs/Pose right_gaze
geometry_msgs/Vector3 gaze_angle
geometry_msgs/Point location
geometry_msgs/Pose head_pose
geometry_msgs/Point[] landmarks
hr_msgs/FacialActionUnit[] action_units
sensor_msgs/RegionOfInterest bounding_box
