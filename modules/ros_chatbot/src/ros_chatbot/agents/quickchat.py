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
from functools import partial

import requests

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger(__name__)

variable_pattern = re.compile(r"\(\(([^()]*)\)\)", flags=re.IGNORECASE)  # eg ((User))

baseUrl = "https://dedicatedhbqw2e.quickchat.ai"


class QuickChatAgent(SessionizedAgent):
    type = "QuickChatAgent"

    def __init__(self, id, lang, version, scenario_id, api_key, timeout=2):
        super(QuickChatAgent, self).__init__(id, lang)
        self.timeout = timeout
        self.allow_repeat = True
        self.version = version
        self.scenario_id = scenario_id
        self.api_key = api_key
        self.conv_id = None
        self.support_priming = True

        self.url = baseUrl

    def set_config(self, config, base):
        super(QuickChatAgent, self).set_config(config, base)

    def priming(self, request):
        timeout = request.context.get("timeout") or self.timeout
        context = request.question
        self._priming(context, timeout)

    def _priming(self, context, timeout):
        params = {
            "version": self.version,
            "api_key": self.api_key,
            "scenario_id": self.scenario_id,
            "context": context,
        }
        if self.conv_id is not None:
            params["conv_id"] = self.conv_id
        try:
            requests.post(
                f"{self.url}/api/hanson/context/",
                json=params,
                timeout=timeout,
            )
        except requests.exceptions.ReadTimeout as ex:
            logger.error(ex)
            return ""

    def ask(self, request):
        question = request.question

        timeout = request.context.get("timeout") or self.timeout
        params = {
            "version": self.version,
            "api_key": self.api_key,
            "scenario_id": self.scenario_id,
            "text": question,
        }
        if self.conv_id is not None:
            params["conv_id"] = self.conv_id
        try:
            response = requests.post(
                f"{self.url}/chat",
                json=params,
                timeout=timeout,
            )
        except requests.exceptions.ReadTimeout as ex:
            logger.error(ex)
            return ""
        if response.status_code == 200:
            json = response.json()
            if "reply" in json:
                logger.info("Agent answer %s", json)
                if "conv_id" in json:
                    self.conv_id = json["conv_id"]
                return json["reply"]
            else:
                logger.warning("No answre")
        return ""

    def new_session(self):
        sid = str(uuid.uuid4())
        self.sid = sid
        return sid

    def reset(self, sid=None):
        self.conv_id = None

    def eval_variable(self, answer, session_context):
        def repl(m, user):
            var = m.group(1).strip()
            if var.lower() == "user":
                return user
            else:
                # delete unknown variable
                return ""

        if variable_pattern.search(answer) and session_context is not None:
            user = session_context.get("username")
            if user is None:
                substitutes = self.config.get("substitutes")
                if substitutes and "User" in substitutes:
                    user = random.choice(substitutes["User"])
            if user is None:
                user = ""
            answer = variable_pattern.sub(partial(repl, user=user), answer)
            answer = " ".join(answer.split())
        return answer

    def chat(self, agent_sid, request):
        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question

        try:
            answer = self.ask(request)
            answer = self.eval_variable(answer, request.session_context)
            if answer:
                response.answer = answer
                response.attachment[
                    "match_excluded_expressions"
                ] = self.check_excluded_expressions(answer)
                response.attachment[
                    "match_excluded_question"
                ] = self.check_excluded_question(request.question)
                response.attachment[
                    "risky_named_entity_detected"
                ] = self.check_named_entity(answer)
                self.score(response)
        except Exception as ex:
            logger.exception(ex)

        return response

    def score(self, response):
        response.attachment["score"] = 80
        if self.version == 0:  # GPT-3
            response.attachment["score"] = 90
        if response.attachment.get("match_excluded_expressions"):
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        if response.attachment.get("match_excluded_question"):
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        if response.attachment.get("risky_named_entity_detected"):
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
