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

# ROS Node that subscribes speech online and offline speech recognition and if online speech recognition is unresponsive will forward the offline speech recognition contents
import time
import rospy
from hr_msgs.msg import ChatMessage
from std_msgs.msg import String

class AsrMonitor(object):
    def __init__(self, delay=2.0):
        self.last_heard = time.time()
        offline_topic = rospy.get_param('~offline_topic', '/offline_speech')
        self.asr_online_sub = rospy.Subscriber('/hr/perception/hear/interim_speech', ChatMessage, self.asr_online_cb)
        self.asr_offline_sub = rospy.Subscriber(offline_topic, String, self.asr_offline_cb)
        self.asr_online_pub = rospy.Publisher('/hr/perception/hear/sentence', ChatMessage, queue_size=1)
        self.delay = delay

    def asr_online_cb(self, msg):
        self.last_heard = time.time()
        
    def asr_offline_cb(self, str):
        if str.data in ['huh']:
            return
        # Likely google speech not gonna respond on time
        if time.time() - self.last_heard > self.delay:
            msg = ChatMessage()
            msg.utterance = str.data
            msg.confidence = 50
            msg.lang = "en-US"  # TODO: rospy.get('/hr/perception/whisper_asr/language')
            msg.source = "fallback"
            self.asr_online_pub.publish(msg)


def main():
    rospy.init_node('fallback_asr')
    asr_monitor = AsrMonitor()
    rospy.spin()


if __name__ == '__main__':
    main()
