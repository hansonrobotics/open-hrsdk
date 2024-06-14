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
float64 timestamp    # motor state is at this time
string name          # motor name
float64 position     # current position (in r/s)
float64 speed        # current speed (in r/s)
float64 load         # current load - ratio of applied torque over maximum torque
float64 voltage      # current voltage (V)
int32 temperature    # current temperature (degrees Celsius)
bool torque_enabled  # indicates enabled torque on motor
float64 torque_limit # current limit applied on motor (0-1)
uint8 errorCode      # last error codes

# ErrorCodes:
# for general information see http://support.robotis.com/en/product/actuator/dynamixel/communication/dxl_packet.htm
#
# a value of 0xff(255) means, the motor is in simulation mode
# a value of 0x80(128) means, the motor is offline (no power?)
