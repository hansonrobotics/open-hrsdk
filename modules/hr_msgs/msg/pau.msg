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
geometry_msgs/Quaternion m_headRotation
geometry_msgs/Vector3    m_headTranslation
geometry_msgs/Quaternion m_neckRotation

float32 m_eyeGazeLeftPitch
float32 m_eyeGazeLeftYaw
float32 m_eyeGazeRightPitch
float32 m_eyeGazeRightYaw

#An array of blendshape coefficients.
#They describe an expression of a virtual face.
float32[] m_coeffs
# Then setting shapekey values, names must be passed
string[] m_shapekeys

#An array of arm/hand joint angles.
float32[] m_angles
string[] m_joints
