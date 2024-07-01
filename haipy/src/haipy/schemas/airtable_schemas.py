#
# Copyright (C) 2017-2024 Hanson Robotics
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
import logging
from datetime import datetime
from enum import Enum, unique
from typing import Dict, List, Union

import yaml
from pydantic import BaseModel, validator

from haipy.utils import parse_yaml_str

logger = logging.getLogger(__name__)


@unique
class Stage(str, Enum):
    CurrentStage = "CurrentStage"
    PostStage = "PostStage"
    PreStage = "PreStage"


@unique
class ScriptType(str, Enum):
    GoogleSheet = "Google Sheet"
    BehaviorTree = "Behavior Tree"
    Soultalk = "SoulTalk"
    Timeline = "Timeline"
    TimelinePackage = "Timeline Package"
    ARFTemplate = "ARF Template"


@unique
class ARFType(str, Enum):
    ARF = "ARF"
    Chatbot = "Chatbot"


@unique
class ScriptStatus(str, Enum):
    Todo = "Todo"
    InProgress = "InProgress"
    Review = "Review"
    Done = "Done"
    Test = "Test"


@unique
class EventStatus(str, Enum):
    Cancelled = "Cancelled"
    Pending = "Pending"
    Confirmed = "Confirmed"
    Permanent = "Permanent"
    Completed = "Completed"
    Test = "Test"


@unique
class SceneStatus(str, Enum):
    Todo = "Todo"
    InProgress = "InProgress"
    Review = "Review"
    Done = "Done"
    Test = "Test"


@unique
class LanguageEnum(str, Enum):
    English = "English"
    Mandarin = "Mandarin"
    Cantonese = "Cantonese"
    Korean = "Korean"


LANGUAGE_CODES = {
    "English": "en",
    "Mandarin": "zh",
    "Chinese": "zh",
    "Cantonese": "hk",
    "Korean": "ko",
}


def norm_field(string: str) -> str:
    if string == "RobotNames":
        return "Name (from Robots)"
    if string == "CharacterNames":
        return "Name (from Characters)"
    return " ".join(string.split("_"))


class AttachmentField(BaseModel):
    id: str
    url: str
    filename: str
    size: int
    type: str


class Sheet(BaseModel):
    id: str
    name: str
    scene: str = None  # scene name
    header: Dict = {}
    skiprows: int = 0
    start: datetime = None
    end: datetime = None
    event_status: EventStatus = None
    parameters: Dict = {}  # sheet parameters
    variables: Dict = {}  # context to export
    arf_events: List[str] = []  # list of arf events such as event.arf.gambit


class ScriptRecord(BaseModel):
    class Script(BaseModel):
        Name: str
        Start: datetime = None
        End: datetime = None
        Status: Union[ScriptStatus, None, str] = None
        Events: List[str] = []
        Type: Union[ScriptType, None, str] = None
        Last_Modified: datetime
        Robots: List[Union[str, None]] = []
        Scenes: List[Union[str, None]] = []
        Characters: List[Union[str, None]] = []
        Doc_Link: str = None
        Language: Union[LanguageEnum, None, str] = None
        Published_Link: str = None
        Attachments: List[AttachmentField] = []
        Context: Dict = {}  # populated by context in spreadsheet header
        Location: str = None
        Parameters: Union[Dict, None] = None  # airtable Parameters field
        Sheets: Dict[str, Sheet] = {}

        @validator("Parameters", pre=True, always=True)
        def parse_yaml(cls, v):
            if v is not None:
                return parse_yaml_str(v)

        class Config:
            alias_generator = norm_field

    id: str
    fields: Script
    createdTime: datetime


class EventRecord(BaseModel):
    class Event(BaseModel):
        Event: str
        Status: Union[EventStatus, None, str] = None
        Start: datetime = None
        End: datetime = None
        Robots: List[Union[str, None]] = []
        RobotNames: List[Union[str, None]] = []
        Scripts: List[Union[str, None]] = []
        Location: str = None
        Parameters: Union[Dict, None] = None
        Last_Modified: datetime

        @validator("Parameters", pre=True, always=True)
        def parse_yaml(cls, v):
            if v is not None:
                try:
                    params = yaml.safe_load(v)
                    if isinstance(params, dict):
                        return params
                    else:
                        return {"root": params}
                except Exception as ex:
                    logger.error(ex)
                    return

        class Config:
            alias_generator = norm_field

    id: str
    fields: Event
    createdTime: datetime


class SceneRecord(BaseModel):
    class Scene(BaseModel):
        Name: str
        Status: Union[SceneStatus, None, str] = None
        Characters: List[str] = []
        CharacterNames: List[str] = []
        Type: Union[ARFType, None, str] = None
        Scene: str = None
        Scripts: List[Union[str, None]] = []

        class Config:
            alias_generator = norm_field

    id: str
    fields: Scene
    createdTime: datetime


class ScriptTable(BaseModel):
    records: List[ScriptRecord]


class EventTable(BaseModel):
    records: List[EventRecord]


class SceneTable(BaseModel):
    records: List[SceneRecord]
