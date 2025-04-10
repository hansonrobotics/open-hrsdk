#!/usr/bin/env python

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

import logging
import rospy
import random
from hr_msgs.srv import AgentRegister, AgentUnregister, AgentChat, StringArray

logger = logging.getLogger("hr.ros_chatbot.chatbot_agent_example")


class ChatAgentExample(object):
    def __init__(self) -> None:
        super().__init__()
        rospy.Service("~chat", AgentChat, self._callback)

    def _callback(self, req):
        response = AgentChat._response_class()
        response.state = random.choice([0, 1])
        response.answer = req.text
        logger.warning(response)
        return response

    def register(self):
        register_service = rospy.ServiceProxy("/hr/interaction/register", AgentRegister)
        request = AgentRegister._request_class()
        request.node = rospy.get_name()
        request.level = 100
        request.ttl = 20
        response = register_service(request)
        logger.warning(response)


if __name__ == "__main__":
    rospy.init_node("chatbot_agent_example")
    agent = ChatAgentExample()
    agent.register()
    rospy.spin()
