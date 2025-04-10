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

from datetime import datetime
from typing import Dict, List, Optional, Set

import haipy.memory_manager as mm
from haipy.atlas_vectordb import AtlasVectorDB
from haipy.memory_manager.memory_model import MemoryModel
from pydantic import BaseModel, Field, RootModel, model_validator


class ChatStreamItem(BaseModel):
    Time: datetime
    ConversationID: str
    Type: str
    Name: str
    Text: str
    Source: str


class ChatStreamSet(RootModel):
    root: List[ChatStreamItem]


class Scene(BaseModel):
    name: str
    character: str = "Sophia"
    type: str = "arf"
    default: bool = False
    conditions: List[str] = []
    variables: Dict = {}
    knowledge_base: str = ""
    asr_context: str = ""
    tts_mapping: str = ""

    def __repr__(self):
        return f"Scene(name={self.name}, type={self.type})"


class SceneEvent(BaseModel):
    scene: str
    arf_event: str
    triggered: bool = False
    skipped: bool = False


class SceneContext(BaseModel):
    scenes: Optional[Dict[str, Scene]] = None
    document_manager: Optional[mm.DocumentManager] = None
    scene_document: Optional[mm.SceneDocument] = None
    vector_store: Optional[AtlasVectorDB] = None
    memory_models: Set[MemoryModel] = Field(default_factory=set)

    def update_scene_document(self, blocking: bool = False):
        if self.document_manager and self.scene_document:
            if blocking:
                self.scene_document.save()
            else:
                self.document_manager.add_document(self.scene_document)

    @model_validator(mode="before")
    @classmethod
    def validate_scenes(cls, values):
        if "scenes" in values:
            scenes = values["scenes"]
            if scenes is not None and not isinstance(scenes, dict):
                raise ValueError("scenes must be a dictionary or None")
        return values

    class Config:
        arbitrary_types_allowed = True

    def __init__(self, **data):
        super().__init__(**data)
        if "scenes" in data:
            self.__dict__["scenes"] = data["scenes"]


if __name__ == "__main__":
    # from ros_chatbot.db import get_chat_stream(sid)
    # stream_set = ChatStreamSet(__root__=get_chat_stream(sid))
    event = SceneEvent(**{"scene": "aa", "arf_event": "bb"})
    print(event.model_dump_json())

    scenes = {"aa": Scene(name="aa")}
    scene_context = SceneContext(scenes=scenes)
    print(id(scene_context.scenes) == id(scenes))  # This should now print True
