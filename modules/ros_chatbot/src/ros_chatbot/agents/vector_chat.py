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
import random
import uuid

from haipy.vectordb import ChatVectorDB

from .model import Agent, AgentResponse

logger = logging.getLogger(__name__)


class VectorChatAgent(Agent):
    type = "VectorChatAgent"

    def __init__(
        self,
        id,
        lang,
        namespace,
    ):
        super(VectorChatAgent, self).__init__(id, lang)
        self.index = ChatVectorDB(namespace)
        self.timeout = 2
        self.similarity_threashold = 0.68
        self.topk = 5
        self.last_conversation_id = None
        self.last_response = None

    def chat(self, agent_sid, request):
        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question

        result = self.index.get_answer(request.question, self.topk)
        logger.info("Get result %s", result)
        if (
            result
            and result["score"] > self.similarity_threashold
            and result["answers"]
        ):
            answers = result["answers"]
            if self.last_response and self.last_response.question != request.question:
                # find answers that are in the same conversation
                # assume the answers in the same conversation are better
                answers_in_conversation = [
                    answer
                    for answer in result["answers"]
                    if self.last_response.attachment["conversation_id"]
                    == answer["conversation_id"]
                ]
                if answers_in_conversation:
                    answers = answers_in_conversation
                    response.attachment["context_match"] = True
            answer = random.choice(answers)
            response.answer = answer["answer"]
            if len(response.answer.split(" ")) > 60:
                response.attachment["confidence"] = 0
            response.answer = answer["answer"]
            response.attachment["conversation_id"] = answer["conversation_id"]
            response.attachment["confidence"] = result["score"]
            response.attachment["label"] = answer["label"]
            response.attachment["resolver"] = answer["resolver"]
            self.last_response = response
            self.score(response)
        else:
            response.trace = "Can't answer"

        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 50
        if response.attachment.get("context_match"):
            # boost confidence by 20% if the context matches
            response.attachment["confidence"] = min(
                1, response.attachment["confidence"] * 1.2
            )
            logger.info("Increase the confidence")
        if response.attachment["confidence"] > 0.7:
            response.attachment["score"] = 60
        if response.attachment["confidence"] > 0.8:
            response.attachment["score"] = 70
        if response.attachment["confidence"] > 0.9:
            response.attachment["score"] = 80
        if response.attachment["confidence"] > 1.0:
            response.attachment["score"] = 85
        if response.attachment["resolver"] == "human":
            response.attachment["score"] = 90
        if response.attachment.get("label") in ["ChatGPT", "GPT3"]:
            # boost score by 10% for GPT3 answers
            response.attachment["score"] = min(85, response.attachment["score"] * 1.1)
            logger.info("Boost the score for GPT3/ChatGPT answers")
        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.answer
        ):
            response.attachment["score"] = -1
            logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
