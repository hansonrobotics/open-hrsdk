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

import requests

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.youchat")


class YouChatAgent(SessionizedAgent):
    type = "YouChatAgent"

    def __init__(self, id, lang, host="localhost", port=8804, timeout=5):
        super(YouChatAgent, self).__init__(id, lang)
        self.host = host
        self.port = port
        if self.host not in ["localhost", "127.0.0.1"]:
            logger.warning("youchat server: %s:%s", self.host, self.port)
        self.timeout = timeout
        self.support_priming = True

    def new_session(self):
        sid = str(uuid.uuid4())
        self.sid = sid
        return sid

    def reset(self):
        try:
            requests.post(
                "http://{host}:{port}/reset".format(host=self.host, port=self.port),
                timeout=self.timeout,
            )
        except Exception as ex:
            logger.error(ex)

    def ask(self, request):
        response = None
        timeout = request.context.get("timeout") or self.timeout
        try:
            response = requests.post(
                f"http://{self.host}:{self.port}/ask",
                json={"question": request.question},
                timeout=timeout,
            )
        except Exception as ex:
            logger.error("error %s", ex)
            return ""

        if response and response.status_code == 200:
            json = response.json()
            if "error" in json and json["error"]:
                logger.error(json["error"])
            elif "answer" in json:
                return json["answer"]

    def chat(self, agent_sid, request):
        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question
        response.attachment["repeating_words"] = False

        try:
            answer = self.ask(request)
            if answer:
                response.attachment[
                    "match_excluded_expressions"
                ] = self.check_excluded_expressions(answer)
                response.attachment[
                    "match_excluded_question"
                ] = self.check_excluded_question(request.question)
                answer = self.post_processing(answer)
                response.answer = answer
                self.score(response)
        except Exception as ex:
            logger.exception(ex)

        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 70
        if response.attachment.get("match_excluded_expressions"):
            logger.info("Answer %r is not allowed", response.answer)
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        if response.attachment.get("match_excluded_question"):
            logger.info("Question %r is not allowed", response.question)
            response.attachment["score"] = -1
            response.attachment["blocked"] = True

        response.attachment["score"] += random.randint(
            -10, 10
        )  # give it some randomness

        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.answer
        ):
            response.attachment["score"] = -1
            logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
