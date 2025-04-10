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
import random
import re
import uuid

from .model import Agent, AgentResponse

logger = logging.getLogger("hr.ros_chatbot.agents.dummy")

EN_QUESTION = re.compile(r"(how|what|when|where|why|who)")
ZH_QUESTION = re.compile(r"(怎么|什么|如何|怎样|几个|几种|是不是|有没有|多少|哪|谁)")


class DummyAgent(Agent):
    type = "DummyAgent"

    def __init__(self, id, lang):
        super(DummyAgent, self).__init__(id, lang)

    def get_dialog_act(self, request):
        # TODO: classify the question and choose the dummy answers
        # perhaps using Dialog Act Server
        if request.lang == "en-US":
            if EN_QUESTION.search(request.question):
                return "question"
        if request.lang == "cmn-Hans-CN":
            if ZH_QUESTION.search(request.question):
                return "question"
        return "acknowledge"

    def chat(self, agent_sid, request):
        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question

        dialog_act = self.get_dialog_act(request)
        answers = self.config["dummy_answers"][dialog_act]

        if request.lang in answers:
            response.answer = random.choice(answers[request.lang])
            self.score(response)

        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 10

        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.answer
        ):
            response.attachment["score"] = -1
            logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
