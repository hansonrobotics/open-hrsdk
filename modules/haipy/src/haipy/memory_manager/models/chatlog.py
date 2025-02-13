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
import re
import uuid
from datetime import datetime
from enum import Enum, EnumMeta
from typing import Dict, NewType, Optional, Union

from bunnet import BackLink, Document, Link, PydanticObjectId
from pydantic import Field, field_validator

Datetime = NewType("Datetime", datetime)
UNICODE_PUNCTUATION_MARKS = re.compile(r"""[.?!。？！]""", flags=re.UNICODE)


def remove_puncuation_marks(text):
    if text:
        text = re.sub(UNICODE_PUNCTUATION_MARKS, "", text)
        text = text.strip()
    return text


def create_uuid4():
    return uuid.uuid4().hex


class ChatModeEnum(str, Enum):
    Ranking = "ranking"


class MyEnumMeta(EnumMeta):
    def __contains__(cls, item):
        return item in cls.__members__.values()


class AudioFormat(str, Enum, metaclass=MyEnumMeta):
    wav = "wav"
    mp3 = "mp3"


class Product(str, Enum, metaclass=MyEnumMeta):
    undefined = "undefined"


class ChatRequestBase(Document):
    user_id: str  # user id (required)
    conversation_id: Optional[str] = None
    text: str
    audio: str = ""  # audio identifier if the request is from Speech-to-Text
    tts: bool = False  # generate audio for the answer
    audio_format: AudioFormat = AudioFormat.mp3
    animation: bool = False  # generate animation
    product: Product = Product.undefined
    lang: str = "en-US"
    context: Dict = {}
    shortcut: bool = True
    app_id: str = ""

    @field_validator("lang")
    def set_lang(cls, v):
        return v or "en-US"

    @field_validator("text")
    def check_empty(cls, v):
        if v == "":
            raise ValueError('"text" can\'t be empty')
        return v

    def __str__(self):
        return "<%s user_id: %r, conv_id: %r, text: %r>" % (
            self.__class__.__name__,
            self.user_id,
            self.conversation_id,
            self.text,
        )

    __repr__ = __str__


class ChatRequest(ChatRequestBase):
    mode: ChatModeEnum = ChatModeEnum.Ranking
    request_id: str = Field(default_factory=create_uuid4)
    created_at: Datetime = Field(default_factory=datetime.utcnow)
    client_ip: str = ""
    responses: BackLink["ChatResponse"] = Field(original_field="request")

    @field_validator("request_id")
    def set_uuid(cls, v):
        return v or uuid.uuid4().hex

    @field_validator("created_at")
    def set_datetime(cls, v):
        return v or datetime.utcnow()

    def __hash__(self) -> int:
        return hash(self.request_id)

    class Settings:
        name = "chat_requests"


class ChatResponseBase(Document):
    """An indivisual chat agent response"""

    response_id: str = Field(default_factory=create_uuid4)
    text: str
    lang: str = ""
    agent_id: str
    conversation_id: str
    confidence: float = 0.0  # TODO this is in attachment
    attachment: Dict = {}

    @field_validator("response_id")
    def set_uuid(cls, v):
        return v or uuid.uuid4().hex

    def __str__(self):
        return "<%s answer: %r, agent: %r, confidence: %s>" % (
            self.__class__.__name__,
            self.text,
            self.agent_id,
            self.confidence,
        )

    __repr__ = __str__


class ChatResponse(ChatResponseBase):
    """chat response in database"""

    request: Optional[Link["ChatRequest"]] = None
    created_at: Datetime = Field(default_factory=datetime.utcnow)
    agent_session_id: str = ""
    published_at: Optional[Datetime] = None
    trace: str = ""

    @field_validator("created_at")
    def set_datetime(cls, v):
        return v or datetime.utcnow()

    @field_validator("trace")
    def character_limit(cls, v):
        return v[:2000]

    def valid(self):
        text = remove_puncuation_marks(self.text)
        return (
            bool(text)
            or self.attachment.get("state") == 1
            or self.attachment.get("actions")
        )

    class Settings:
        name = "chat_responses"


class Scene(Document):
    name: str
    conversation_id: str
    metadata: Dict = {}
    person_id: Union[None, PydanticObjectId] = None
    created_at: Datetime = Field(default_factory=datetime.utcnow)

    @field_validator("created_at")
    def set_datetime(cls, v):
        return v or datetime.utcnow()

    class Settings:
        name = "scenes"
