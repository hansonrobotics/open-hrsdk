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
import threading
import time
import uuid

import grpc

from . import conversation_pb2, conversation_pb2_grpc
from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger(__name__)

CWD = os.path.abspath(os.path.dirname(__file__))
URL = "ec2-3-75-243-249.eu-central-1.compute.amazonaws.com:8080"


class LegendSession(object):
    def __init__(self, url, character, player):
        self.url = url
        self.character = character
        self.player = player
        self.chatServiceClient = None
        self.player_id = None
        self.character_id = None
        self.conversation_id = None
        self.message_queue = []
        self.new_message = threading.Event()

    def start_session(self):
        with grpc.insecure_channel(self.url) as channel:
            self.chatServiceClient = conversation_pb2_grpc.ChatServiceStub(channel)

            # find character id
            listCharacterResponse = self.chatServiceClient.ListCharacters(
                conversation_pb2.ListCharactersRequest()
            )
            if not listCharacterResponse.characters:
                raise RuntimeError("No characters")
            for character in listCharacterResponse.characters:
                if character.name == self.character:
                    self.character_id = character.id
                    break
            if self.character_id is None:
                raise RuntimeError(f"Can't find character {self.character}")

            # find player id
            listPlayerResponse = self.chatServiceClient.ListPlayers(
                conversation_pb2.ListPlayersRequest()
            )
            if not listPlayerResponse.players:
                raise RuntimeError("No players")

            for player in listPlayerResponse.players:
                if player.name == self.player:
                    self.player_id = player.id
                    break
            if self.player_id is None:
                self.player_id = listPlayerResponse.players[0].id

            # create conversation
            createConversationResponse = self.chatServiceClient.CreateConversation(
                conversation_pb2.CreateConversationRequest(
                    character_ids=[self.character_id], player_ids=[self.player_id]
                )
            )
            self.conversation_id = createConversationResponse.id

            # start streaming
            message_stream = self.chatServiceClient.StreamConversationMessages(
                conversation_pb2.JoinConversationRequest(
                    conversation_id=self.conversation_id
                )
            )
            logger.warning("Started conversation message stream")
            for message in message_stream:
                logger.info("Got message %s", message.content)
                if message.type == "Character":
                    self.message_queue.append(message)
                    self.new_message.set()

    def send_message(self, content, timeout):
        self.new_message.clear()
        cursor = len(self.message_queue)
        message_request = conversation_pb2.SendMessageRequest(
            player_id=self.player_id,
            message_content=content,
            conversation_id=self.conversation_id,
        )
        self.chatServiceClient.SendMessage(message_request)
        logger.info("Sent message %s", message_request.message_content)
        signaled = self.new_message.wait(timeout)
        if signaled:
            return self.message_queue[cursor:]
        else:
            logger.warning("No response")

    def commit_message(self, accept: bool, message: str):
        """
        accept: True if the response is accepted or False otherwise
        message: The message to send back
        """
        return  # not implemented
        commitType = (
            conversation_pb2.CommitMessageRequest.COMMIT_TYPE_ACCEPTED
            if accept
            else conversation_pb2.CommitMessageRequest.COMMIT_TYPE_REJECTED
        )
        message = conversation_pb2.Message(
            player_id=self.player_id,
            content=message,
            timestamp=int(round(time.time())),
            conversation_id=self.conversation_id,
            type="Player",
        )
        self.chatServiceClient.CommitMessage(
            conversation_pb2.CommitMessageRequest(type=commitType, message=message)
        )


class LegendChatAgent(SessionizedAgent):
    type = "LegendChatAgent"

    def __init__(self, id, lang, url, character, player, timeout=2):
        super(LegendChatAgent, self).__init__(id, lang)
        self.timeout = timeout
        self.legend_session = LegendSession(url=url, character=character, player=player)
        self.last_response = None
        self.emotional_states = {}

        job = threading.Thread(target=self.legend_session.start_session, daemon=True)
        job.start()

    def ask(self, request):
        timeout = request.context.get("timeout") or self.timeout
        messages = self.legend_session.send_message(request.question, timeout)
        if messages:
            return {
                "answer": messages[-1].content,
                "confidence": 1,
                "classified_emotion": messages[-1].emotion,
            }

    def new_session(self):
        sid = str(uuid.uuid4())
        self.sid = sid
        return sid

    def feedback(self, request_id, chosen, hybrid):
        if chosen and self.last_response:
            try:
                message = self.last_response["answer"]
                self.legend_session.commit_message(chosen, message)
                self.emotional_states["classified_emotion"] = self.last_response[
                    "classified_emotion"
                ]
                logger.info("Emotional states %s", self.emotional_states)
                return self.emotional_states
            except Exception as ex:
                logger.error("Commit error %s", ex)

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
            result = self.ask(request)
            if result:
                logger.info("Get response %s", result)
                self.last_response = result
                answer = result["answer"]
                response.answer = answer
                response.attachment[
                    "match_excluded_expressions"
                ] = self.check_excluded_expressions(answer)
                response.attachment[
                    "match_excluded_question"
                ] = self.check_excluded_question(request.question)
                response.attachment["confidence"] = result["confidence"]
                response.attachment["classified_emotion"] = result["classified_emotion"]
                self.score(response)
        except Exception as ex:
            logger.exception(ex)

        return response

    def score(self, response):
        response.attachment["score"] = 80
        if response.attachment.get("match_excluded_expressions"):
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        if response.attachment.get("match_excluded_question"):
            response.attachment["score"] = -1
            response.attachment["blocked"] = True

        response.attachment["score"] += random.randint(
            -10, 10
        )  # give it some randomness

        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.answer
        ):
            response.attachment["score"] = -1
            logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
