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

from ros_chatbot.utils import check_repeating_words, shorten, token_sub

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.gpt2")


class GPT2Agent(SessionizedAgent):
    type = "GPT2Agent"
    token_pattern = re.compile(r"\b( '\s?(m|ve|d|ll|t|s|re))\b")  # such as ' m, ' ve

    def __init__(self, id, lang, media_agent, host="localhost", port=8108, timeout=2):
        super(GPT2Agent, self).__init__(id, lang)
        if media_agent is None:
            raise ValueError("Media agent cannot be None")
        self.media_agent = media_agent
        self.host = host
        self.port = port
        if self.host not in ["localhost", "127.0.0.1"]:
            logger.warning("gpt2 server: %s:%s", self.host, self.port)
        self.timeout = timeout
        self.seed_length = 5
        self.response_limit = 40
        self.header = ""
        self.depth = 20

    def set_config(self, config, base):
        super(GPT2Agent, self).set_config(config, base)
        if "header" in self.config:
            header = self.config["header"]
            self.header = "\n".join(header)
            logger.info("Loaded header %r", self.header)
        if "seed_length" in self.config:
            self.seed_length = int(self.config["seed_length"])
            logger.info("Seed length %s", self.seed_length)
        if "context_depth" in self.config:
            self.depth = int(self.config["context_depth"])
            logger.info("Context depth %s", self.depth)

    def new_session(self):
        self.reset()
        if isinstance(self.media_agent, SessionizedAgent):
            return self.media_agent.new_session()
        else:
            return str(uuid.uuid4())

    def reset(self):
        try:
            requests.get(
                "http://{host}:{port}/reset".format(host=self.host, port=self.port),
                timeout=self.timeout,
            )
            self.set_seed(random.randint(0, 100))
        except Exception as ex:
            logger.error(ex)

    def set_seed(self, seed):
        try:
            requests.get(
                "http://{host}:{port}/set_seed".format(host=self.host, port=self.port),
                params={"seed": seed},
                timeout=self.timeout,
            )
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
        if response.status_code == 200:
            json = response.json()
            if "status" in json and json["status"] == "OK":
                return True
        else:
            logger.error("GPT2 server %s:%s is not available", self.host, self.port)
        return False

    def ask(self, question, seed):
        try:
            requests.get(
                "http://{host}:{port}/add_context".format(
                    host=self.host, port=self.port
                ),
                params={"text": question},
                timeout=self.timeout,
            )
        except Exception as ex:
            logger.error("error %s", ex)
            return ""

        response = None

        try:
            response = requests.get(
                "http://{host}:{port}/generate".format(host=self.host, port=self.port),
                params={"text": seed, "header": self.header, "depth": self.depth},
                timeout=self.timeout,
            )
        except Exception as ex:
            logger.error("error %s", ex)
            return ""

        if response:
            json = response.json()
            if "error" in json and json["error"]:
                logger.error(json["error"])
            elif "answer" in json:
                return json["answer"]

    def chat(self, agent_sid, request):
        if agent_sid is None:
            logger.error("Agent session was not provided")
            return
        if not self.ping():
            logger.error("GPT2 server %s:%s is not available", self.host, self.port)
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

        if request.question:
            agent_response = self.media_agent.chat(agent_sid, request)
            if agent_response and agent_response.valid():
                try:
                    seed = " ".join(agent_response.answer.split()[: self.seed_length])
                    if "|" in seed:  # ignore ||
                        seed = ""
                except Exception as ex:
                    logger.error(ex)
                    seed = ""
                answer = self.ask(request.question, seed)
                response.attachment["repeating_words"] = check_repeating_words(answer)
                response.attachment[
                    "match_excluded_expressions"
                ] = self.check_excluded_expressions(answer)
                answer = token_sub(self.token_pattern, answer)
                answer, res = shorten(answer, self.response_limit)
                if answer:
                    response.answer = answer
                    self.score(response)
                else:
                    response.trace = "No answer"
        else:
            response.trace = "Can't answer"
        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 50
        if response.attachment["match_excluded_expressions"]:
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        elif response.attachment["repeating_words"]:
            response.attachment["score"] = 10
        else:
            input_len = len(response.question)
            if input_len > 100:
                response.attachment["score"] = 60
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
            logger.warning("Question response is not allowed")

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
