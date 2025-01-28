#
# Copyright (C) 2017-2025 Hanson Robotics
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
from sqlalchemy import JSON, Column, DateTime, Integer, String, Unicode
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.ext.mutable import MutableDict

Base = declarative_base()


class ChatRequest(Base):
    __tablename__ = "chat_requests"

    id = Column(Integer(), primary_key=True, autoincrement=True)
    app_id = Column(String(64), nullable=False)
    request_id = Column(String(64), nullable=False)
    created_at = Column(DateTime(), nullable=False)
    user_id = Column(String(64), nullable=False, default="default")
    conversation_id = Column(String(64), nullable=False, default="default")
    text = Column(Unicode(2048), nullable=False, default="")
    lang = Column(String(16), nullable=False, default="")
    attachment = Column(MutableDict.as_mutable(JSON()))

    def __init__(
        self,
        app_id,
        request_id,
        created_at,
        user_id,
        conversation_id,
        text,
        lang,
        **kwargs,
    ):
        self.app_id = app_id
        self.request_id = request_id
        self.created_at = created_at
        self.user_id = user_id
        self.conversation_id = conversation_id
        self.text = text[:2048]
        self.lang = lang
        self.attachment = {}
        self.attachment.update(kwargs)

    def __repr__(self):
        return f"<ChatRequest: {self.text} {self.lang}>"


class ChatResponse(Base):
    __tablename__ = "chat_responses"

    id = Column(Integer(), primary_key=True, autoincrement=True)
    request_id = Column(String(64), nullable=False)
    response_id = Column(String(64), nullable=False)
    created_at = Column(DateTime(), nullable=False)
    conversation_id = Column(String(64), nullable=False, default="default")
    agent_id = Column(String(64), nullable=False)
    text = Column(String(2048), nullable=False, default="")
    lang = Column(String(16), nullable=False, default="")
    trace = Column(String(2048), nullable=False, default="")
    published_at = Column(DateTime(), nullable=True)
    attachment = Column(MutableDict.as_mutable(JSON()))

    def __init__(
        self,
        request_id,
        response_id,
        conversation_id,
        created_at,
        agent_id,
        text,
        lang,
        trace,
        **kwargs,
    ):
        self.request_id = request_id
        self.response_id = response_id
        self.conversation_id = conversation_id
        self.created_at = created_at
        self.agent_id = agent_id
        self.text = text[:2048]
        self.lang = lang
        self.trace = trace[:2048]
        self.attachment = {}
        self.attachment.update(kwargs)

    def __repr__(self):
        return f"<ChatResponse: {self.text} {self.lang}>"


class ConvInsight(Base):
    """Stores user story, conversation summary, visual memory etc"""

    __tablename__ = "conv_insights"

    id = Column(Integer(), primary_key=True, autoincrement=True)
    created_at = Column(DateTime(), nullable=False)
    conversation_id = Column(String(64), nullable=False, default="default")
    type = Column(String(64), nullable=False)
    insight = Column(MutableDict.as_mutable(JSON()))

    def __repr__(self):
        return f"<ConvInsight: {self.type} {self.insight}>"