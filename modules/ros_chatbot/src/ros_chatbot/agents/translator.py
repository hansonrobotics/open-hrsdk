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
import random
import uuid
from copy import copy

import requests

import ros_chatbot.shared as shared

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.translator")


class CantoneseTranslator(object):
    """https://fanyi-api.baidu.com/api/trans/product/apidoc"""

    def __init__(self):
        self.url = "https://fanyi-api.baidu.com/api/trans/vip/translate"
        self.appid = os.environ.get("BAIDU_TRANSLATE_APPID")
        self.secretKey = os.environ.get("BAIDU_TRANSLATE_SECRETKEY")

    def translate(self, text, target_language):
        if self.appid is None:
            logger.error("BAIDU_TRANSLATE_APPID is missing")
            return
        if self.secretKey is None:
            logger.error("BAIDU_TRANSLATE_SECRETKEY is missing")
            return
        salt = random.randint(32768, 65536)
        sign = self.appid + text + str(salt) + self.secretKey
        sign = hashlib.md5(sign.encode()).hexdigest()

        if target_language == "zh-TW":
            target_language = "yue"
        if target_language == "zh-CN":
            target_language = "yue"

        params = {
            "appid": self.appid,
            "q": text,
            "from": "auto",
            "to": target_language,
            "salt": str(salt),
            "sign": sign,
        }
        response = requests.get(self.url, params)
        ret = {}
        if response.status_code == 200:
            result = response.json()
            if "error_msg" in result:
                logger.error(result)
                return
            elif "trans_result" in result:
                ret["translatedText"] = result["trans_result"][0]["dst"]
                return ret


class TranslatorAgent(SessionizedAgent):
    """
    For supported languages see https://cloud.google.com/translate/docs/languages
    """

    type = "TranslatorAgent"
    KNOWN_LANGUAGE_CODE = {
        "cmn-Hans-CN": "zh-CN",
        "en-US": "en",
        "yue-Hant-HK": "yue",
    }

    def __init__(self, id, language_codes, media_language, media_agent):
        """
        parameters
        ----------
            media_agent: the agent that does the chat
            media_language: the language of the media agent
            lang: the language of the translator
        """
        super(TranslatorAgent, self).__init__(id, list(language_codes.keys()))
        self.language_codes = language_codes

        from google.cloud import translate

        self.client = translate.Client()
        self.cantonest_client = CantoneseTranslator()

        if media_agent is None:
            raise ValueError("Media agent cannot be None")
        self.media_agent = media_agent
        self.media_language = media_language

    def new_session(self):
        if isinstance(self.media_agent, SessionizedAgent):
            return self.media_agent.new_session()
        else:
            return str(uuid.uuid4())

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
            result = self.translate(request.question, request.lang, self.media_language)
            if result and result["translated"]:
                question = result["text"]
                media_request = copy(request)
                media_request.lang = self.media_language
                media_request.question = question
                logger.info("Media agent request %s", media_request)
                agent_response = self.media_agent.chat(agent_sid, media_request)
                if agent_response and agent_response.valid():
                    if "actions" in response.attachment:
                        response.attachment["actions"] = response.attachment["actions"]
                    if hasattr(self.media_agent, "score"):
                        self.media_agent.score(agent_response)
                        response.attachment["score"] = agent_response.attachment[
                            "score"
                        ]
                    if (
                        "topic" not in response.attachment
                        or response.attachment["topic"] != "language"
                    ):
                        # do not translate response from language skill
                        result = self.translate(
                            agent_response.answer, self.media_language, request.lang
                        )
                        if result and result["translated"]:
                            response.answer = result["text"]
                            response.attachment[
                                "media response"
                            ] = agent_response.answer
                else:
                    logger.warning("No media agent resopnse")
        except Exception as ex:
            logger.exception(ex)
            return
        response.end()
        return response

    def text2cachekey(self, text, language):
        return "%s@%s" % (text.lower().strip(), language)

    def translate(self, text, source_language, target_language):
        key = self.text2cachekey(text, target_language)
        if key in shared.cache:
            logger.info("Using cache")
            return shared.cache[key]
        ret = {}

        if target_language in self.language_codes:
            target_language = self.language_codes[target_language]
        elif target_language in self.KNOWN_LANGUAGE_CODE:
            target_language = self.KNOWN_LANGUAGE_CODE[target_language]
        if source_language == target_language:
            ret["text"] = text
            ret["translated"] = True
            return ret

        logger.warning(
            "Translating %r from %r to %r", text, source_language, target_language
        )
        if source_language == "yue-Hant-HK" or target_language == "yue":
            result = self.cantonest_client.translate(text, target_language)
        else:
            result = self.client.translate(text, target_language=target_language)

        if result:
            ret["text"] = result["translatedText"]
            ret["translated"] = True
            logger.warning("Translate result %s", result["translatedText"])

            shared.cache[key] = ret
            return ret
