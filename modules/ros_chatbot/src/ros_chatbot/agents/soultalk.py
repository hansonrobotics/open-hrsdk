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
import uuid

import requests
from benedict import benedict

from ros_chatbot.utils import LANGUAGE_CODE_INV_MAPPING, LANGUAGE_CODE_MAPPING

from .model import Agent, AgentResponse, SessionizedAgent


class SoulTalkAgent(SessionizedAgent):
    type = "SoulTalkAgent"

    def __init__(self, id, lang, host, port, runtime_config_description, timeout=2):
        super(SoulTalkAgent, self).__init__(id, lang)
        self.host = host
        self.port = port
        if self.host not in ["localhost", "127.0.0.1"]:
            self.logger.warning("soultalk server: %s:%s", self.host, self.port)
        self.runtime_config_description = runtime_config_description
        self.timeout = timeout
        self.preferred_topics = []
        self.blocked_topics = []
        self.uid = "default"
        self.sid = None

    def set_config(self, config, base):
        super(SoulTalkAgent, self).set_config(config, base)
        if "preferred_topics" in config:
            self.preferred_topics = config["preferred_topics"]

    @Agent.enabled.setter
    def enabled(self, enabled):
        self._config.maps[1]["enabled"] = enabled
        self.config = benedict(self._config)

    def ping(self):
        """Agent is disabled if ping fails"""
        try:
            response = requests.get(
                "http://{host}:{port}/status".format(host=self.host, port=self.port),
                timeout=max(2, self.timeout),
            )
        except Exception as ex:
            self.logger.error(ex)
            return False
        if response.status_code == requests.codes.ok:
            json = response.json()
            if json["err_no"] == 0:
                return True
            else:
                return False
        else:
            self.logger.error(
                "SoulTalk server %s:%s is not available", self.host, self.port
            )
        return False

    def ask(self, request):
        self.sid = request.sid
        timeout = request.context.get("timeout") or self.timeout
        try:
            response = requests.post(
                "http://{host}:{port}/chat".format(host=self.host, port=self.port),
                json={
                    "uid": self.uid,
                    "sid": request.sid,
                    "text": request.question,
                    "lang": LANGUAGE_CODE_MAPPING.get(request.lang, request.lang),
                    "request_id": request.request_id,
                },
                timeout=timeout,
            )
        except Exception as ex:
            self.logger.error(ex)
            return ""
        if response.status_code == requests.codes.ok:
            json = response.json()
            if "response" in json and json["response"] and "text" in json["response"]:
                return json["response"]

    def run_reflection(self, sid, text, lang):
        try:
            response = requests.post(
                "http://{host}:{port}/reflect".format(host=self.host, port=self.port),
                json={
                    "uid": self.uid,
                    "sid": sid,
                    "text": text,
                    "lang": LANGUAGE_CODE_MAPPING.get(lang, lang),
                },
                timeout=self.timeout,
            )
        except Exception as ex:
            self.logger.error(ex)
            return
        if response.status_code == requests.codes.ok:
            json = response.json()
            if "response" in json and json["response"]:
                return json["response"]

    def new_session(self):
        return str(uuid.uuid4())

    def reset_session(self):
        self.logger.info("Reset session")
        if self.sid is not None:
            try:
                response = requests.post(
                    "http://{host}:{port}/reset".format(host=self.host, port=self.port),
                    json={
                        "uid": self.uid,
                        "sid": self.sid,
                    },
                    timeout=self.timeout,
                )
                if response.status_code == requests.codes.ok:
                    return True
            except Exception as ex:
                self.logger.error(ex)
            self.sid = None
        return False

    def chat(self, agent_sid, request):
        if agent_sid is None:
            self.logger.error("Agent session is missing")
            return
        if not self.ping():
            self.logger.error(
                "SoulTalk server %s:%s is not available", self.host, self.port
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

        if request.question:
            json = self.ask(request)
            if json:
                answer = json.get("text")
                if answer:
                    answer = self.post_processing(answer)
                response.answer = answer
                response.attachment["agent_type"] = self.type
                response.attachment["topic"] = json.get("topic", "")
                response.attachment["actions"] = json.get("actions")
                response.attachment["topic_type"] = json.get("topic_type", "")
                response.attachment["line"] = json.get("line", "")
                response.attachment["intent"] = json.get("intent", "")
                response.attachment["confidence"] = json.get("confidence", 1)
                response.attachment["probability"] = json.get("probability")
                response.attachment["fallback"] = json.get("fallback")
                response.trace = json.get("trace")
                response.attachment["tag"] = json.get("tag")
                response.attachment["output_context"] = json.get("output_context")
                response.attachment["input_context"] = json.get("input_context")
                response.attachment["allow_repeat"] = (
                    response.attachment["topic_type"] in ["ARF", "Skill"]
                    or "repeat" in response.attachment["tag"]
                )
                # if (
                #    response.answer
                #    and response.answer.startswith("|")
                #    and response.answer.endswith("|")
                # ):
                #    response.attachment["non-verbal"] = True

                if not response.answer and response.attachment["actions"]:
                    actions = "+".join(
                        [action["name"] for action in response.attachment["actions"]]
                    )
                    response.answer = f"|{actions}|"

                if response.attachment["output_context"]:
                    for output_context in response.attachment["output_context"]:
                        response.answer = (
                            response.answer + f" |context: {output_context}|"
                        )
                # response could be in different language
                lang = json.get("lang")
                if lang:
                    response.lang = LANGUAGE_CODE_INV_MAPPING.get(lang, lang)
                self.score(response)
            else:
                response.trace = "No answer"
        else:
            response.trace = "Can't answer"
        self.handle_translate(request, response)
        response.end()
        return response

    def set_output_context(self, sid, context, finished):
        if not self.ping():
            self.logger.error(
                "SoulTalk server %s:%s is not available", self.host, self.port
            )
            return
        try:
            self.logger.info("Set output context %s", context)
            for token, lifespan in context.items():
                response = requests.post(
                    "http://{host}:{port}/set_output_context".format(
                        host=self.host, port=self.port
                    ),
                    json={
                        "token": token,
                        "lifespan": lifespan,
                        "uid": self.uid,
                        "sid": sid,
                        "finished": finished,
                    },
                    timeout=self.timeout,
                )
                if response.status_code == requests.codes.ok:
                    json = response.json()
                    if json["err_no"] != 0:
                        self.logger.error("error %s", json["err_msg"])
        except Exception as ex:
            self.logger.error("error %s", ex)

    def score(self, response):
        response.attachment["score"] = 50
        if response.attachment.get("fallback"):
            response.attachment["blocked"] = True
            response.attachment["score"] = -1
        else:
            if (
                self.preferred_topics
                and response.attachment.get("topic") not in self.preferred_topics
            ):
                response.attachment["blocked"] = True
                response.attachment["score"] = -1
            else:
                if response.attachment.get("confidence") < 0.25:
                    if response.attachment["topic_type"] in ["ARF", "Skill"]:
                        response.attachment["score"] = 90
                    else:
                        response.attachment["score"] = 50
                else:
                    response.attachment["score"] = 100
                if "llm" in response.attachment["tag"]:
                    response.attachment["score"] = 100
            input_len = len(response.question)
            if input_len > 100:
                response.attachment["score"] -= 20  # penalty on long input
        probability = response.attachment.get("probability")
        if probability is not None:
            if probability < random.random():
                self.logger.warning("Probability not passing %s", probability)
                response.attachment["score"] = 20
            else:
                response.attachment["score"] = response.attachment["score"] * max(
                    0.8, probability
                )
        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.answer
        ):
            response.attachment["score"] = -1
            self.logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
