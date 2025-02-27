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

import rospy
from hr_msgs.msg import ChatMessage, TTS


class Echo(object):
    """A STT to TTS echo.

    It speaks out what it hears by the connection the STT and the TTS.
    """

    def __init__(self):
        rospy.Subscriber("/hr/perception/hear/sentence", ChatMessage, self.callback)
        self.pub = rospy.Publisher("/hr/control/speech/say", TTS)

    def callback(self, msg):
        """The callback function simply redirects what it hears to the TTS"""
        self.pub.publish(msg.utterance, msg.lang)


if __name__ == "__main__":
    rospy.init_node("hello")
    Echo()
    rospy.spin()
