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
Header header
Object object
std_msgs/Int32 initial_points
std_msgs/Int32 active_points #Moving avaerage to destroy as it wouldn't make sense just to remove.
std_msgs/Bool quality_results # Makes sense
std_msgs/String tracker_name #This is needed.
std_msgs/String recognized_name #This is assigned when it's known to have a name
std_msgs/Bool recognized # This is updated after tracker_name is updated from the recognizer function.
std_msgs/Int32 before_demotion
std_msgs/Bool validated #To check for merge fucntions.
std_msgs/String google_best_guess