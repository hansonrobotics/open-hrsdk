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

from ros_chatbot.utils import check_repeating_words, shorten, token_sub

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.blenderbot")


class BlenderBotAgent(SessionizedAgent):
    type = "BlenderBotAgent"
    token_pattern = re.compile(r"\b( '\s?(m|ve|d|ll|t|s|re))\b")  # such as ' m, ' ve
    hyphen_pattern = re.compile(r"(\w+)\s+-\s+(\w+)")  # such "human - like"
    variable_pattern = re.compile(
        r"\(\s*\(([^()]*)\)\s*\)", flags=re.IGNORECASE
    )  # eg ((User))

    goodbye_pattern = re.compile(r"\b(goodbye|bye|see you)\b")

    def __init__(self, id, lang, host="localhost", port=8105, timeout=2, persona=None):
        super(BlenderBotAgent, self).__init__(id, lang)
        self.host = host
        self.port = port
        if self.host not in ["localhost", "127.0.0.1"]:
            logger.warning("blenderbot server: %s:%s", self.host, self.port)
        self.timeout = timeout
        self.default_persona = persona or []
        self.support_priming = True

        self.persona = []

    def set_config(self, config, base):
        super(BlenderBotAgent, self).set_config(config, base)
        persona = self.config.get("persona", [])
        if persona:
            persona = [p.lower() for p in persona]
            self.persona = persona[:]
            self.set_persona()

    def new_session(self):
        """The blenderbot doesn't maintain the session. Whenever it needs to
        start a new conversation, it will simply reset the current session"""
        sid = str(uuid.uuid4())
        self.reset()
        return sid

    def reset(self, sid=None):
        try:
            requests.get(
                "http://{host}:{port}/reset".format(host=self.host, port=self.port),
                timeout=self.timeout,
            )
            self.persona = self.default_persona[:]
            self.set_persona()
        except Exception as ex:
            logger.error(ex)

    def ping(self):
        try:
            response = requests.get(
                "http://{host}:{port}/status".format(host=self.host, port=self.port),
                timeout=self.timeout,
            )
        except Exception as ex:
            logger.error(ex)
            return False
        if response.status_code == requests.codes.ok:
            json = response.json()
            if json["success"]:
                return True
        else:
            logger.error(
                "BlenderBot server %s:%s is not available", self.host, self.port
            )
        return False

    def ask(self, question):
        try:
            response = requests.post(
                "http://{host}:{port}/chat".format(host=self.host, port=self.port),
                json={"text": question},
                timeout=self.timeout,
            )
        except Exception as ex:
            logger.error("error %s", ex)
            return ""
        if response and response.status_code == 200:
            json = response.json()
            if "text" in json:
                return json["text"]

    def set_persona(self):
        if not self.persona:
            return False
        try:
            persona_desc = "\\n".join(
                ["your persona: {}".format(p) for p in self.persona if p]
            )
            logger.info("Setting persona %r", self.persona)
            response = requests.get(
                "http://{host}:{port}/set_persona".format(
                    host=self.host, port=self.port
                ),
                params={"text": persona_desc},
                timeout=self.timeout,
            )
        except Exception as ex:
            logger.error("error %s", ex)
            return ""
        json = response.json()
        if json["success"]:
            return True
        else:
            logger.error(
                "BlenderBot server %s:%s is not available", self.host, self.port
            )
        return False

    def check_reset(self, answer):
        """Check if it needs a reset according to the answer"""
        if answer and self.goodbye_pattern.search(answer):
            logger.warning("Reset the blenderbot by goodbye")
            self.reset()

    def remove_unsafe_label(self, text):
        """Checks if it contains the unsafe label"""
        if text.endswith("_POTENTIALLY_UNSAFE__"):
            return text[: -len("_POTENTIALLY_UNSAFE__")]
        return text

    def cleanup(self, answer):
        answer = answer.replace("( ( user ) ) s", "users")
        answer = answer.replace("( ( user ) )", "user")
        return answer

    def eval_variable(self, answer):
        def repl(m, user):
            var = m.group(1).strip()
            if var.lower() == "user":
                return user
            else:
                # delete unknown variable
                return ""

        user = ""
        substitutes = self.config.get("substitutes")
        if substitutes and "User" in substitutes:
            user = random.choice(substitutes["User"])

        if self.variable_pattern.search(answer):
            answer = self.variable_pattern.sub(partial(repl, user=user), answer)
            answer = " ".join(answer.split())
        elif re.search(r"\buser\b", answer):
            # replace plain text: user
            answer = re.sub(r"\buser\b", user, answer)
            answer = " ".join(answer.split())

        return answer

    def chat(self, agent_sid, request):
        if agent_sid is None:
            logger.warning("Agent session was not provided")
            return
        if not self.ping():
            logger.error(
                "BlenderBot server %s:%s is not available", self.host, self.port
            )
            return

        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question
        response.attachment["repeating_words"] = False
        response.attachment["unsafe"] = False

        if request.question:
            answer = self.ask(request.question)
            if answer:
                response.attachment["repeating_words"] = check_repeating_words(answer)
                unsafe_answer = self.remove_unsafe_label(answer)
                if answer != unsafe_answer:
                    answer = unsafe_answer
                    response.attachment["unsafe"] = True
                response.attachment[
                    "match_excluded_expressions"
                ] = self.check_excluded_expressions(answer)
                response.attachment[
                    "match_excluded_question"
                ] = self.check_excluded_question(request.question)
                answer = self.cleanup(answer)
                self.check_reset(answer)
                answer = token_sub(self.token_pattern, answer)
                answer = self.hyphen_pattern.sub(r"\1-\2", answer)
                answer = self.eval_variable(answer)

                if "response_limit" in self.config:
                    answer, res = shorten(answer, self.config["response_limit"])
                if answer:
                    response.answer = answer
                    response.attachment[
                        "risky_named_entity_detected"
                    ] = self.check_named_entity(answer)
                    self.score(response)
        else:
            response.trace = "Can't answer"
        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 80
        if response.attachment["repeating_words"]:
            response.attachment["score"] = 10
        if response.attachment["unsafe"]:
            response.attachment["score"] -= 10
        if response.attachment.get("match_excluded_expressions"):
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        if response.attachment.get("match_excluded_question"):
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        if response.attachment.get("risky_named_entity_detected"):
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
