# -*- coding: utf-8 -*-

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
import os
import random
import uuid

import requests

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.ai21")


class WebAPI(object):
    def __init__(self):
        self.API_KEY = os.environ.get("AI21_API_KEY")
        if not self.API_KEY:
            raise ValueError("API KEY is required")
        self.chat_server = "https://api.ai21.com/studio/v1/j1-large/complete"

    def ask(self, prompt, topK=1, temperature=1.0, maxTokens=140):
        response = requests.post(
            self.chat_server,
            headers={"Authorization": f"Bearer {self.API_KEY}"},
            json={
                "prompt": prompt,
                "numResults": 1,
                "maxTokens": maxTokens,
                "stopSequences": [".", "\n"],
                "topKReturn": topK,
                "temperature": temperature,
            },
        )
        if response and response.status_code == 200:
            data = response.json()
            return data["completions"][0]["data"]["text"]


class AI21Agent(SessionizedAgent):
    type = "AI21Agent"

    def __init__(self, id, lang, media_agent):
        super(AI21Agent, self).__init__(id, lang)
        self.api = WebAPI()

        if media_agent is None:
            raise ValueError("Media agent cannot be None")
        self.media_agent = media_agent
        self.prompt_length = 5
        self.topK = 1
        self.temperature = 1.0
        self.maxTokens = 140

    def set_config(self, config, base):
        super(AI21Agent, self).set_config(config, base)
        if "topK" in self.config:
            self.topK = self.config["topK"]
        if "temperature" in self.config:
            self.temperature = self.config["temperature"]
        if "maxTokens" in self.config:
            self.maxTokens = self.config["maxTokens"]
        if "prompt_length" in self.config:
            self.prompt_length = self.config["prompt_length"]

    def new_session(self):
        if isinstance(self.media_agent, SessionizedAgent):
            return self.media_agent.new_session()
        else:
            return str(uuid.uuid4())

    def chat(self, agent_sid, request):
        if agent_sid is None:
            logger.error("Agent session was not provided")
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
            agent_response = self.media_agent.chat(agent_sid, request)
            if agent_response and agent_response.valid():
                try:
                    prompt = " ".join(
                        agent_response.answer.split()[: self.prompt_length]
                    )
                except Exception as ex:
                    logger.error(ex)
                    prompt = ""
                answer = self.api.ask(
                    prompt, self.topK, self.temperature, self.maxTokens
                )
                if answer:
                    response.answer = prompt + " " + answer
                    self.score(response)
                else:
                    response.trace = "No answer"
        else:
            response.trace = "Can't answer"

        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 50
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
