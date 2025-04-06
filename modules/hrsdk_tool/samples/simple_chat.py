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
from hr_msgs.srv import ChatSession, ChatService, ChatServiceRequest


class SimpleChat(object):
    """A simple STT to TTS interaction example.

    It invokes the chat service on the built-in bot.
    """

    def __init__(self):
        self.sid = None
        rospy.wait_for_service(
            "/hr/interaction/session"
        )  # wait for the interaction contaner to be up and ready

        rospy.Subscriber(
            "/hr/perception/hear/sentence", ChatMessage, self.chat_callback
        )
        self.say_pub = rospy.Publisher("/hr/control/speech/say", TTS, queue_size=10)
        self.session = rospy.ServiceProxy("/hr/interaction/session", ChatSession)
        self.chat = rospy.ServiceProxy("/hr/interaction/chat", ChatService)

    def chat_callback(self, msg):
        """The callback function triggered by the message from STT"""
        if not self.sid:
            # It always needs to get a chat session from the chat service
            # first. And it uses the same session ID for the following
            # conversation.
            response = self.session.call()
            self.sid = response.session
        request = ChatServiceRequest()
        request.text = msg.utterance
        request.session = self.sid
        request.lang = msg.lang
        try:
            response = self.chat.call(request)
            self.say_pub.publish(response.response, msg.lang)
        except Exception as ex:
            rospy.logerr(ex)


if __name__ == "__main__":
    rospy.init_node("simple_chat")
    SimpleChat()
    rospy.spin()
