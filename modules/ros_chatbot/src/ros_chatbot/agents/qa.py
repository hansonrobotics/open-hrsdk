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
import re
import uuid

import requests

from ros_chatbot.utils import search_answer_post_processing

from .model import Agent, AgentResponse

logger = logging.getLogger(__name__)

ILLEGAL_CHARACTER = re.compile(
    r"[?~@#^&*()`/<>{}\[\]=+|\\·•ʾ]", flags=re.IGNORECASE + re.UNICODE
)


class QAAgent(Agent):
    type = "QAAgent"

    def __init__(self, id, lang, host="localhost", port=8802, timeout=2):
        super(QAAgent, self).__init__(id, lang)
        self.timeout = timeout
        self.allow_repeat = True
        self.url = "http://{host}:{port}/ask".format(host=host, port=port)
        self.keywords_interested = None

    def set_config(self, config, base):
        super(QAAgent, self).set_config(config, base)
        if "keywords_interested" in self.config:
            # the regular expresson matches the sentence begins with any of the
            # words in the list
            self.keywords_interested = re.compile(
                r"%s" % self.config["keywords_interested"], re.IGNORECASE
            )

    def check_question(self, question):
        """Checks if the question is what it is interested"""
        question = question.lower()
        if len(question.split()) <= 2:
            return False
        if not self.keywords_interested:
            return False
        match = self.keywords_interested.search(question)
        if match:
            text = question[match.start() :]
            excluded = self.check_excluded_expressions(text)
            return not excluded
        return match

    def ask(self, request):
        page = self.config.get("page", 3)
        mininum_score = self.config.get("mininum_score", 0.02)
        question = request.question
        if question.lower().startswith("so "):
            question = question[3:]  # remove so
        ret = {"answer": "", "confidence": 0}

        timeout = request.context.get("timeout") or self.timeout
        try:
            response = requests.post(
                self.url,
                json={"question": question, "page": page},
                timeout=timeout,
            )
        except requests.exceptions.ReadTimeout as ex:
            logger.error(ex)
            return ret
        if response.status_code == 200:
            json = response.json()
            if "answers" in json:
                logger.info("QA agent answer %s", json)
                answers = json["answers"]
                for answer in answers:
                    if answer["score"] > 0.8:
                        ret["confidence"] = 90
                        answer_text = answer["answer"]
                        if (
                            "answer_sentence" in answer
                            and len(answer["answer"])
                            / max(1, len(answer["answer_sentence"]))
                            < 0.1
                        ):
                            answer_text = "{}. {}".format(
                                answer["answer"], answer["answer_sentence"]
                            )
                        ret["answer"] = search_answer_post_processing(answer_text)
                        return ret
                    if answer["score"] > mininum_score and answer["answer_sentence"]:
                        ret["confidence"] = 55
                        answer_text = answer["answer_sentence"]
                        if (
                            "answer_sentence" in answer
                            and len(answer["answer"])
                            / max(1, len(answer["answer_sentence"]))
                            < 0.1
                        ):
                            answer_text = "{}. {}".format(
                                answer["answer"], answer["answer_sentence"]
                            )
                        ret["answer"] = search_answer_post_processing(answer_text)
                        return ret
        logger.info("QA agent has no answer")
        return ret

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
            if answer["answer"]:
                response.answer = answer["answer"]
                response.attachment["confidence"] = answer["confidence"]
                self.score(response)
        except Exception as ex:
            logger.exception(ex)

        response.end()
        return response

    def score(self, response):
        if ILLEGAL_CHARACTER.search(response.answer):
            response.attachment["score"] = response.attachment["confidence"] - 40
        else:
            response.attachment["score"] = response.attachment["confidence"]

        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.answer
        ):
            response.attachment["score"] = -1
            logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
