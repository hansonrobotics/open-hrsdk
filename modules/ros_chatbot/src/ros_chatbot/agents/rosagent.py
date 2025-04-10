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
import threading
import uuid
from queue import Empty

import rospy
from hr_msgs.msg import ChatResponse
from hr_msgs.srv import AgentChat, AgentFeedback

from ros_chatbot.agents.model import (
    AgentResponse,
    AgentStreamResponse,
    ConfigurableAgent,
)

logger = logging.getLogger(__name__)


class ROSGenericAgent(ConfigurableAgent):
    """
    ROSGenericAgent is a class that represents a generic ROS-based agent.
    """

    type = "ROSGenericAgent"

    def __init__(
        self,
        id: str,
        node: str,
        lang: str,
        runtime_config_description: int,
    ):
        super().__init__(id, lang, runtime_config_description)
        self.chat_service_name = f"{node}/chat"
        self.feedback_service_name = f"{node}/feedback"
        self.response_topic_name = f"{node}/responses"
        self.chat_proxy = rospy.ServiceProxy(self.chat_service_name, AgentChat)
        self.feedback_proxy = rospy.ServiceProxy(
            self.feedback_service_name, AgentFeedback
        )
        self.responses = {}
        self.response_events = {}

        rospy.Subscriber(
            self.response_topic_name, ChatResponse, self._response_callback
        )

    def ping(self):
        """Check the availability of ROS services."""
        for service in [self.chat_service_name, self.feedback_service_name]:
            try:
                rospy.wait_for_service(service, 0.5)
            except rospy.ROSException as ex:
                logger.error("Service %r is not available: %s", service, ex)
                return False
        return True

    def feedback(self, request_id, chosen, hybrid):
        req = AgentFeedback._request_class()
        req.request_id = request_id
        req.chosen = chosen
        req.hybrid = hybrid
        try:
            return self.feedback_proxy(req)
        except Exception as ex:
            logger.error("Error during feedback: %s", ex)

    def chat(self, agent_sid, request):
        if self.languages and request.lang not in self.languages:
            logger.warning("Language %s is not supported", request.lang)
            return
        if not self.ping():
            return

        req = AgentChat._request_class()
        req.text = request.question
        req.lang = request.lang
        req.session = agent_sid or ""
        req.request_id = str(uuid.uuid4())

        agent_response = AgentStreamResponse()
        agent_response.agent_sid = agent_sid
        agent_response.sid = request.sid
        agent_response.request_id = request.request_id
        agent_response.response_id = req.request_id
        agent_response.agent_id = self.id
        agent_response.lang = request.lang
        agent_response.question = request.question
        agent_response.preference = self.config.get("preference", -1)
        print("Agent preference: ", agent_response.preference)
        if request.question:
            # Create an event for the response and store it
            response_event = threading.Event()
            self.responses[req.request_id] = agent_response
            self.response_events[req.request_id] = response_event
            try:
                self.chat_proxy(req)

            except Exception as ex:
                logger.error("Error during chat request: %s", ex)
                return

            # Wait for the event to be set when the first response is received
            response_event.wait(timeout=10.0)

            if not response_event.is_set():
                logger.error("No response received for request_id: %s", req.request_id)
                del self.responses[req.request_id]
                del self.response_events[req.request_id]
                return

            if agent_response.answer:
                agent_response.attachment[
                    "match_excluded_expressions"
                ] = self.check_excluded_expressions(agent_response.answer)
                agent_response.answer = self.post_processing(agent_response.answer)
                self.score(agent_response)
            else:
                # No answer makes it clear that response in not good for ranker.
                agent_response.attachment["score"] = -1
                agent_response.attachment['blocked'] = True
                           
            print("agent answer ", agent_response.answer)
        agent_response.end()
        return agent_response

    def _response_callback(self, msg):
        """Callback for handling responses from the response topic."""
        if msg.request_id in self.responses:
            agent_response = self.responses[msg.request_id]
            if msg.text:
                # Response events are there for first sentence.
                if msg.request_id not in self.response_events:
                    agent_response.stream_data.put(msg.text)
                else:
                    agent_response.answer = (
                        agent_response.answer + " " + msg.text
                    ).strip()
                if msg.request_id in self.response_events:
                    self.response_events[msg.request_id].set()
                    del self.response_events[msg.request_id]
            if msg.request_id in self.response_events:
                self.response_events[msg.request_id].set()
                del self.response_events[msg.request_id]
            if msg.label == "|end|":
                agent_response.stream_finished.set()

    def score(self, response):
        response.attachment["score"] = 100
        if response.attachment["match_excluded_expressions"]:
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
