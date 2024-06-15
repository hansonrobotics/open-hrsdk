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
Header header

uint8 encoding

uint8 SINT_8_PCM=0
uint8 UINT_8_PCM=1
uint8 SINT_16_PCM=2
uint8 SINT_24_PCM=3
uint8 SINT_32_PCM=4
uint8 FLOAT_32=5
uint8 FLOAT_64=6

uint8  is_bigendian
uint8  channels
uint32 sample_rate

# interleaved channels
uint8[]  data
