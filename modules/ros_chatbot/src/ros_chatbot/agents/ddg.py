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
import re
import uuid

import requests

from ros_chatbot.utils import shorten

from .model import Agent, AgentResponse

logger = logging.getLogger("hr.ros_chatbot.agents.ddg")
ILLEGAL_CHARACTER = re.compile(r"[?~@#^&*()`/<>{}\[\]=+|\\·•]", flags=re.IGNORECASE)


class DDGAgent(Agent):
    type = "DDGAgent"

    def __init__(self, id, lang, timeout=2):
        super(DDGAgent, self).__init__(id, lang)
        self.stop_words_pattern = None
        HR_CHATBOT_STOP_WORDS_FIlE = os.environ.get("HR_CHATBOT_STOP_WORDS_FIlE")
        if HR_CHATBOT_STOP_WORDS_FIlE:
            with open(HR_CHATBOT_STOP_WORDS_FIlE) as f:
                words = f.read().splitlines()
                words = [
                    word for word in words if word.strip() and not word.startswith("#")
                ]
                self.stop_words_pattern = re.compile(
                    r"\b(%s)\b" % "|".join(words), flags=re.IGNORECASE
                )

        self.timeout = timeout

        # the regular expresson matches the sentence begins with any of the
        # words in the list
        self.keywords_interested = re.compile(
            r"(?i)^(%s)\b.*$"
            % "|".join(
                (
                    "what is,what's,what are,what're,who is,who's,who are,"
                    "who're,where is,where's,where are,where're"
                ).split(",")
            )
        )

        # the regular expresson matches the sentence with occurance of any of
        # of the words in the list anywhere.
        self.keywords_to_ignore = re.compile(
            r"(?i).*\b(%s)\b.*$"
            % "|".join(
                (
                    "I,i,me,my,mine,we,us,our,ours,you,your,yours,he,him,"
                    "his,she,her,hers,it,its,the,they,them,their,theirs,time,"
                    "date,weather,day,this,that,those,these,about"
                ).split(",")
            )
        )

    def check_question(self, question):
        """Checks if the question is what it is interested"""
        question = question.lower()
        return self.keywords_interested.match(
            question
        ) and not self.keywords_to_ignore.match(question)

    def ask(self, request):
        question = request.question
        orig_question = request.question

        if question.lower().startswith("so "):
            question = question[3:]  # remove so
        # to let ddg find the definition
        question = question.replace("what are ", "what is ")
        question = question.replace("What are ", "What is ")

        if self.stop_words_pattern:
            question = self.stop_words_pattern.sub("", question)
            question = " ".join(question.split())
            if orig_question != question:
                logger.warning("Simplified Question: %s", question)
        timeout = request.context.get("timeout") or self.timeout
        try:
            response = requests.get(
                "http://api.duckduckgo.com",
                params={"q": question, "format": "json"},
                timeout=timeout,
            )
        except requests.exceptions.ReadTimeout as ex:
            logger.error(ex)
            return ""
        json = response.json()
        if json["AnswerType"] not in ["calc"]:
            return json["Abstract"] or json["Answer"]
        else:
            return ""

    def chat(self, agent_sid, request):
        if not self.check_question(request.question):
            return

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
            if "response_limit" in self.config:
                answer, res = shorten(answer, self.config["response_limit"])
            if answer:
                response.answer = answer
                self.score(response)
        except Exception as ex:
            logger.exception(ex)

        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 90
        if ILLEGAL_CHARACTER.search(response.answer):
            response.attachment["score"] = response.attachment["score"] - 40
        else:
            response.attachment["score"] = response.attachment["score"]
        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.answer
        ):
            response.attachment["score"] = -1
            logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
