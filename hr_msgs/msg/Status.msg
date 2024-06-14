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
bool has_current_target    # indicates if the robot arm is currently moved

float64 current_target_x   # x/y position of the current goal (only valid if has_current_target is true)
float64 current_target_y

bool has_current_pen_on_canvas   # indicates if the current_pen_on_canvas values are valid
float64 current_pen_on_canvas_x  # current x/y position of the pen on the canvas
float64 current_pen_on_canvas_y

float64 current_pen_x            # current x/y/z position of pen in 3d space
float64 current_pen_y
float64 current_pen_z

int32 queue                # amount of strokes that are still in the queue
