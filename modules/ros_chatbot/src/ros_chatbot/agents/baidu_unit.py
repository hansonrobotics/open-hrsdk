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

import json
import logging
import os
import re
import uuid

import requests
import six

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.baidu_unit")


class WebUnit(object):
    def __init__(self):
        app_key_id = "BAIDU_UNIT_APP_KEY"
        app_key_secret = "BAIDU_UNIT_APP_SECRET"
        self.app_key_id = os.environ.get(app_key_id)
        self.app_key_secret = os.environ.get(app_key_secret)
        if not self.app_key_id:
            raise ValueError("baidu app key was not provided")
        if not self.app_key_secret:
            raise ValueError("baidu app secret was not provided")

        self.auth_server = "https://aip.baidubce.com/oauth/2.0/token"
        self.chat_server = "https://aip.baidubce.com/rpc/2.0/unit/service/chat"
        self.bot_server = "https://aip.baidubce.com/rpc/2.0/unit/bot/chat"
        self.access_token = None
        self.sid = ""

    def ask(self, question):
        if isinstance(question, six.binary_type):
            question = question.decode("utf-8")

        access_token = self.get_access_token()
        url = self.chat_server + "?access_token=" + access_token
        post_data = {
            "log_id": "UNITTEST_10000",
            "version": "2.0",
            "service_id": "S30732",
            "session_id": self.sid,
            "skill_ids": ["1031954", "1031624", "1031625", "1031626"],
            "request": {
                "query": question,
                "user_id": "hr-user-20367856",
                "query_info": {"source": "ASR"},
            },
            "dialog_state": {
                "contexts": {
                    "SYS_REMEMBERED_SKILLS": [
                        "1031954",
                        "1031624",
                        "1031625",
                        "1031626",
                        "1031627",
                    ]
                }
            },
        }
        headers = {"content-type": "application/x-www-form-urlencoded"}
        response = requests.post(url, data=json.dumps(post_data), headers=headers)
        if response:
            response = response.json()
            if response["error_code"] == 0:
                self.sid = response["result"]["session_id"] or ""
                logger.info("Session %s", self.sid)
                for result in response["result"]["response_list"]:
                    for action in result["action_list"]:
                        text = action["say"]
                        if isinstance(text, six.text_type):
                            text = text.encode("utf-8")
                        type_str = action["type"]
                        if isinstance(type_str, six.text_type):
                            type_str = type_str.encode("utf-8")
                        logger.info(
                            "Action say: %r confidence: %s type: %s",
                            text,
                            action["confidence"],
                            type_str,
                        )
                        if action["type"] == "satisfy":
                            text = action["say"]
                            return text
                        if action["type"] == "chat" and action["confidence"] > 0.45:
                            text = action["say"]
                            return text
            else:
                logger.error(response["error_msg"])

    def get_access_token(self):
        if self.access_token:
            # May need to check "expires_in" for longer session (e.g. over 1 month)
            return self.access_token
        params = {
            "grant_type": "client_credentials",
            "client_id": self.app_key_id,
            "client_secret": self.app_key_secret,
        }
        response = requests.get(self.auth_server, params=params)
        if response:
            self.access_token = response.json()["access_token"]
            return self.access_token

    def reset(self):
        self.sid = ""


class BaiduUnitAgent(SessionizedAgent):
    type = "BaiduUnitAgent"
    name_patch = re.compile("(小度)")

    def __init__(self, id, lang):
        super(BaiduUnitAgent, self).__init__(id, lang)
        self.api = WebUnit()

    def new_session(self):
        sid = str(uuid.uuid4())
        self.api.reset()
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
            answer = self.name_patch.sub("我", answer)
            response.answer = answer
            self.score(response)
        except Exception as ex:
            logger.error(ex)
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
