#!/usr/bin/python
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

import logging
import threading

import rospy
from std_msgs.msg import String
from dynamic_reconfigure.server import Server as DynamicReconfigServer
from asr.cfg import AudioAnalyzerConfig

from hr_msgs.msg import OnsetMessage
from speech_recognizer.audio import AudioStreamer
from speech_recognizer.onset import OnsetDetector

logger = logging.getLogger("hr.asr.audio_analyzer")

class AudioAnalyzer(object):
    """Detects onset and other features in audio (esp. music)"""

    def __init__(self):
        self.audio_rate = rospy.get_param('audio_rate', 48000)
        chunk = 1600
        format = 'float32'
        self.streamer = AudioStreamer(1, self.audio_rate, chunk, format)
        self.onset_detector = OnsetDetector(self.audio_rate, chunk)
        self.robot_speaking = False

        onset_topic = rospy.get_param('onset_topic', 'audio_onset')
        robot_speech_event_topic = rospy.get_param('robot_speech_event_topic', 'speech_events')
        self.onset_publisher = rospy.Publisher(onset_topic, OnsetMessage, queue_size=1)
        rospy.Subscriber(robot_speech_event_topic, String, self.robot_speech_event_cb, queue_size=1)

        DynamicReconfigServer(AudioAnalyzerConfig, self.dynamic_reconfig_cb)

        # start audio streaming
        audio_streaming_job = threading.Thread(target=self.start_audio_stream)
        audio_streaming_job.daemon = True
        audio_streaming_job.start()

        # onset detection
        onset_job = threading.Thread(target=self.onset_detector.handle_stream, args=(self._onset_callback,))
        onset_job.daemon = True
        onset_job.start()

    def dynamic_reconfig_cb(self, config, level):
        self.config = config
        return config

    def start_audio_stream(self):
        """ Enqueue the audio chunk data from the audio steam """
        with self.streamer.stream() as stream:
            while True:
                data = stream.read(self.streamer.chunk)

                if not data:
                    raise StopIteration()
                else:
                    if self.config.enable and not self.robot_speaking:
                        self.onset_detector.add_audio_buffer(data)

    def robot_speech_event_cb(self, msg):
        """ Used in continuous listening mode """
        if msg.data:
            if msg.data.startswith('start'):
                self.robot_speaking = True
                # Reset speech recognition then TTS starts (restarting after TTS finish might be too late).
            if msg.data.startswith('stop'):
                self.robot_speaking = False
            # wait for silence message if available
            if msg.data.startswith('silence'):
                self.robot_speaking = False

    def _onset_callback(self, onset, pitch, notes, tempo):
        msg = OnsetMessage()
        msg.onset = onset
        msg.pitch = pitch
        msg.notes = notes
        msg.tempo = tempo
        self.onset_publisher.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node('audio_analyzer')
        AudioAnalyzer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
