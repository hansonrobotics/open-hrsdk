#!/usr/bin/env python
#
# Copyright (C) 2017-2024 Hanson Robotics
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
import os
import pyaudio
import contextlib
import rospy

from std_msgs.msg import UInt8MultiArray, MultiArrayDimension

INPUT_DEVICE_INDEX = os.environ.get('INPUT_DEVICE_INDEX')
if INPUT_DEVICE_INDEX:
    INPUT_DEVICE_INDEX = int(INPUT_DEVICE_INDEX)

@contextlib.contextmanager
def record_audio(channels, rate, chunk):
    """Opens a recording stream in a context manager."""
    audio_interface = pyaudio.PyAudio()
    audio_stream = audio_interface.open(
        format=pyaudio.paInt16, channels=channels, rate=rate,
        input=True, frames_per_buffer=chunk,
        input_device_index=INPUT_DEVICE_INDEX,
    )

    yield audio_stream

    audio_stream.stop_stream()
    audio_stream.close()
    audio_interface.terminate()

if __name__ == '__main__':
    rospy.init_node('audio_stream')
    audio_stream_topic = rospy.get_param('audio_stream_topic', 'speech_audio')
    audio_publisher = rospy.Publisher(audio_stream_topic, UInt8MultiArray, queue_size=1)
    rate = rospy.get_param('audio_rate', 16000)
    channels = rospy.get_param('audio_channels', 1)
    chunk = int(rate / 10)  # 100ms
    with record_audio(channels, rate, chunk) as audio_stream:
        while not rospy.is_shutdown():
            data = audio_stream.read(chunk)
            if not data:
                raise StopIteration()
            msg = UInt8MultiArray()
            msg.data = data
            msg.layout.dim = [
                MultiArrayDimension('wave', chunk, channels*2),
                MultiArrayDimension('channel', channels, channels)
            ]
            audio_publisher.publish(msg)
