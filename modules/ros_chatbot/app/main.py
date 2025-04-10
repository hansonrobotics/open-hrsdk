#!/usr/bin/env python

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
import sys
from typing import Dict, List

import coloredlogs
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from ros_chatbot.chat_server import ChatServer
from ros_chatbot.utils import get_current_time_str

server = ChatServer()
logger = logging.getLogger("hr.ros_chatbot.app.main")


class SessionRequest(BaseModel):
    uid: str  # User id
    reset: bool = False  # Reset the session


class SessionResponse(BaseModel):
    sid: str
    err_code: int = 0
    err_msg: str = ""


class ListResponse(BaseModel):
    agents: List[str]


class ChatRequest(BaseModel):
    sid: str
    question: str
    request_id: str = ""
    time: str = get_current_time_str()
    audio: str = ""  # path of the audio if the request is from Speech-to-Text
    tag: str = ""  # tag for the conversation
    lang: str = "en-US"
    context: Dict = {}
    mode: str = "ranking"
    scene: str = ""
    user_id: str = ""  # graph user id

    def __hash__(self) -> int:
        return hash(
            (
                self.sid,
                self.question,
                self.request_id,
                self.time,
                self.audio,
                self.tag,
                self.lang,
                str(self.context.items()),
                self.mode,
                self.scene,
                self.user_id,
            )
        )


class AgentResponse(BaseModel):
    sid: str
    request_id: str
    agent_id: str
    response_id: str = ""
    agent_sid: str = ""
    start_dt: str = get_current_time_str()
    end_dt: str = get_current_time_str()
    answer: str = ""
    trace: str = ""
    priority: int = -1
    attachment: Dict = {}


class ChatResponse(BaseModel):
    responses: List[AgentResponse] = []
    err_code: int = 0
    err_msg: str = ""


class StatusResponse(BaseModel):
    err_code: int = 0
    err_msg: str = ""


if "coloredlogs" in sys.modules and os.isatty(2):
    formatter_str = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"
    coloredlogs.install(logging.INFO, fmt=formatter_str)

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"])


@app.post(
    "/session", summary="Get or retrieve a session", response_model=SessionResponse
)
def session(request: SessionRequest):
    response = {}
    try:
        sid = server.get_client_session(request.uid, request.reset)
        response["sid"] = sid
    except Exception as ex:
        logger.error(ex)
        response["sid"] = ""
        response["err_code"] = 1
        response["err_msg"] = str(ex)
    return response


@app.get(
    "/agents", summary="List of installed chat agents", response_model=ListResponse
)
def list_all_installed_agents():
    agents = []
    agents.extend(
        [
            "%s:%s:%s" % (agent.type, agent.id, agent.enabled)
            for agent in server.agents.values()
        ]
    )
    return {"agents": agents}


@app.post("/chat", summary="Chat", response_model=ChatResponse)
def chat(request: ChatRequest):
    response = {}

    try:
        agent_responses = []
        for responses in server.chat_with_ranking(request):
            if responses:
                agent_responses.extend(responses)
            logger.warning("responses %r", responses)
        response["responses"] = agent_responses
    except Exception as ex:
        response["err_code"] = 1
        response["err_msg"] = str(ex)
        logger.error(ex)

    logger.error("response %r", response)
    return response


@app.get("/", summary="Check chat server status", response_model=StatusResponse)
def status():
    return {}
