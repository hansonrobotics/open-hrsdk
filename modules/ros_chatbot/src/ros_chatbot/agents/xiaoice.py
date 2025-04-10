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

import hashlib
import json
import logging
import os
import uuid
from time import time

import requests

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.xiaoice")


class WebAPI(object):
    def __init__(self):
        self.subscription_key = os.environ.get("XIAOICE_API_SUBSCRIPTION_KEY")
        self.app_key = os.environ.get("XIAOICE_API_APP_KEY")
        self.url = os.environ.get("XIAOICE_API_BASEURL")
        if not self.subscription_key:
            raise ValueError("xiaoice subscription key was not provided")
        if not self.app_key:
            raise ValueError("xiaoice app key was not provided")

    def get_headers(self, timestamp, body):
        signature = hashlib.sha512(
            (body + self.app_key + str(timestamp)).encode("utf-8")
        )
        headers = {
            "Content-Type": "application/json",
            "subscription-key": self.subscription_key,
            "timestamp": timestamp,
            "signature": signature.hexdigest(),
        }
        return headers

    def ask(self, question):
        timestamp = str(int(time()))
        body = json.dumps(
            {
                "content": {
                    "text": question,
                    "ContentType": "text",
                    "Metadata": {"Character": "xiaoc"},
                },
                "senderId": str(uuid.uuid4()),
                "timestamp": timestamp,
                "msgId": str(uuid.uuid4()),
            },
            ensure_ascii=False,
        )
        headers = self.get_headers(timestamp, body)

        res = requests.post(self.url, data=body.encode("utf-8"), headers=headers)
        if res.status_code == 200:
            answer = []
            for result in res.json():
                text = result["content"].get("text")
                if text:
                    answer.append(text)
            answer = "\n".join(answer)
            return answer


class XiaoIceAgent(SessionizedAgent):
    type = "XiaoIceAgent"

    def __init__(self, id, lang):
        super(XiaoIceAgent, self).__init__(id, lang)
        self.api = WebAPI()

    def new_session(self):
        sid = str(uuid.uuid4())
        return sid

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
            answer = self.api.ask(request.question)
        except Exception as ex:
            logger.exception(ex)
            return
        response.answer = answer
        self.score(response)
        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = self.weight * 100
        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and ("?" in response.answer or "？" in response.answer)
        ):
            response.attachment["score"] = -1
            logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
