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

import re

import requests

import rospy
from hr_msgs.msg import ChatMessage, TTS


class DDGChat(object):
    """An example showing how to integrate an online chatbot.

    In this example we will show you how to connect it to DuckDuckGo service.
    """

    def __init__(self):
        # the regular expresson matches the sentence begins with any of the
        # words in the list
        self.keywords_interested = re.compile(r'(?i)^(%s)\b.*$' % '|'.join("what is,what's,what are,what're,who is,who's,who are,who're,where is,where's,where are,where're".split(',')))

        # the regular expresson matches the sentence with occurance of any of
        # of the words in the list anywhere.
        self.keywords_to_ignore = re.compile(r'(?i)(%s)\b.*$' % '|'.join('I,i,me,my,mine,we,us,our,ours,you,your,yours,he,him,his,she,her,hers,it,its,the,they,them,their,theirs,time,date,weather,day,this,that,those,these'.split(',')))

        rospy.Subscriber(
            "/hr/perception/hear/sentence", ChatMessage, self.chat_callback
        )
        self.say_pub = rospy.Publisher(
                "/hr/control/speech/say", TTS, queue_size=10
        )

    def ask_ddg(self, question):
        try:
            response = requests.get(
                    'http://api.duckduckgo.com',
                    params={'q': question, 'format': 'json'},
                    timeout=1)
        except:
            return "Sorry I couldn't answer that question"
        json = response.json()
        return json['Abstract'] or json['Answer']

    def check_question(self, question):
        """Checks if the question is what it is interested"""
        return self.keywords_interested.match(question) and \
                not self.keywords_to_ignore.match(question)

    def chat_callback(self, msg):
        """The callback function triggered by the message from STT"""
        question = msg.utterance.strip()
        if self.check_question(question):
            answer = self.ask_ddg(question)
            self.say_pub.publish(answer, msg.lang)

if __name__ == "__main__":
    rospy.init_node("ddg_chat")
    DDGChat()
    rospy.spin()
