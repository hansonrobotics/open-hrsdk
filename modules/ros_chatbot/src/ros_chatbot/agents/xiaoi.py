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
import logging
import os
import re
import uuid
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import six

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.xiaoi")


class WebXiaoi(object):
    def __init__(self):
        app_key_id = "XIAOI_APP_KEY_ID"
        app_key_secret = "XIAOI_APP_KEY_SECRET"
        self.app_key_id = os.environ.get(app_key_id)
        self.app_key_secret = os.environ.get(app_key_secret)
        if not self.app_key_id:
            raise ValueError("xiaoi app key was not provided")
        if not self.app_key_secret:
            raise ValueError("xiaoi app secret was not provided")

    def get_headers(self):
        realm = "xiaoi.com"
        method = "POST"
        uri = "/ask.do"
        nonce = "0" * 40
        sha1 = hashlib.sha1(
            ":".join([self.app_key_id, realm, self.app_key_secret]).encode("utf-8")
        ).hexdigest()
        sha2 = hashlib.sha1(":".join([method, uri]).encode("utf-8")).hexdigest()
        sign = hashlib.sha1(":".join([sha1, nonce, sha2]).encode("utf-8")).hexdigest()

        headers = {
            "X-Auth": 'app_key="{}",nonce="{}",signature="{}"'.format(
                self.app_key_id, nonce, sign
            )
        }
        return headers

    def ask(self, userId, question):
        if isinstance(question, six.text_type):
            question = question.encode("utf-8")
        # url = 'http://nlp.xiaoi.com/ask.do'
        url = "http://robot.open.xiaoi.com/ask.do"
        values = {
            "userId": userId,
            "question": question,
            "type": 0,
            "platform": "custom",
        }
        data = urlencode(values).encode("utf-8")
        headers = self.get_headers()
        req = Request(url, data, headers)
        response = urlopen(req)
        answer = response.read()
        if isinstance(answer, six.binary_type):
            answer = answer.decode("utf-8")
        http_url = (
            "http[s]?://(?:[a-zA-Z]|[0-9]|[$-_@.&+]|"  # noqa
            "[!*\(\),]|(?:%[0-9a-fA-F][0-9a-fA-F]))+"  # noqa
        )
        answer = re.sub(http_url, "", answer)
        return answer.strip()


class XiaoIAgent(SessionizedAgent):
    type = "XiaoIAgent"
    name_patch = re.compile("(小i|xiaoi|xiao i|小 i)", flags=re.IGNORECASE)

    def __init__(self, id, lang):
        super(XiaoIAgent, self).__init__(id, lang)
        self.api = WebXiaoi()

    def patch_name(self, text):
        return self.name_patch.sub("索菲亚", text)

    def validate_answer(self, text):
        if "默认回复" in text:
            return False
        if "该功能正在开发中" in text:
            return False
        if "主人" in text:
            return False
        if "请点击语音键" in text:
            return False
        if "重复回复" in text:
            return False
        if "点击此链接" in text:
            return False
        if "illegalWordReply" in text:
            return False
        return True

    def new_session(self):
        sid = str(uuid.uuid4())
        return sid

    def remove_cmd_tag(self, text):
        return re.sub(r"\[CMD\].*\[/CMD\]", "", text).strip()

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
            answer = self.api.ask(agent_sid, request.question)
        except Exception as ex:
            logger.exception(ex)
            return
        if not self.validate_answer(answer):
            return
        answer = self.remove_cmd_tag(answer)
        answer = self.patch_name(answer)
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
