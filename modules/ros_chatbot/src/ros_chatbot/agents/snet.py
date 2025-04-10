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

import json
import logging
import random
import uuid
from time import time

from websocket import create_connection

from ..utils import shorten
from .model import Agent, AgentResponse

logger = logging.getLogger(__name__)


class SNetAgent(Agent):
    type = "SNetAgent"

    def __init__(self, id, lang, host="localhost", port=8181, timeout=2):
        super(SNetAgent, self).__init__(id, lang)
        self.timeout = timeout

        self.event_name = "sophia_dialog"
        self.uri = "ws://{}:{}{}".format(
            host, port, "/services/" + self.event_name + "/"
        )
        self.request_id = 5000  # const

    def ask(self, question):
        context = [["user1", question, time()]]
        data = {
            "context": context,
            "request_id": self.request_id,
            "event": self.event_name,
        }

        ws = create_connection(self.uri, timeout=self.timeout)
        try:
            ws.send(json.dumps(data))
            response = json.loads(ws.recv())
            if response["event"] == "success":
                return response["answer"]
        finally:
            ws.close()

    def chat(self, agent_sid, request):
        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question

        if request.question:
            answer = self.ask(request.question)
            if answer:
                response.attachment[
                    "match_excluded_expressions"
                ] = self.check_excluded_expressions(answer)
                response.attachment[
                    "match_excluded_question"
                ] = self.check_excluded_question(request.question)
                if "response_limit" in self.config:
                    answer, res = shorten(answer, self.config["response_limit"])
                if answer:
                    response.answer = answer
                    self.score(response)
        else:
            response.trace = "Can't answer"
        return response

    def score(self, response):
        response.attachment["score"] = 70
        if response.attachment.get("match_excluded_expressions"):
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        if response.attachment.get("match_excluded_question"):
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        input_len = len(response.question)
        if input_len > 100:
            response.attachment["score"] -= 10  # penalty on long input
        response.attachment["score"] += random.randint(
            -10, 10
        )  # give it some randomness

        if (
            "minimum_score" in self.config
            and response.attachment["score"] < self.config["minimum_score"]
        ):
            logger.info(
                "Score didn't pass lower threshold: %s", response.attachment["score"]
            )
            response.attachment["score"] = -1
        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.answer
        ):
            response.attachment["score"] = -1
            logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
