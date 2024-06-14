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
float64 target_x # target position on canvas in meters
float64 target_y # target position on canvas in meters
float64 speed # speed in m/s, hack: negative speed will be interpreted as a stop motion in seconds
float64 penHeight # distance of the pen from the canvas in meters
float64 scale_done_dist # this should be by default 1., larger values (like 2.) make the drawing less precise but "faster" in execution
