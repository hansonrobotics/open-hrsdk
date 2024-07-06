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
import json
from datetime import datetime
from typing import Dict, List, NewType, Optional, Union

from pydantic import BaseModel, Field, validator

Datetime = NewType("Datetime", datetime)


class ConnectRequest(BaseModel):

    uid: str
    sid: str
    created_at: Datetime = Field(default_factory=datetime.utcnow)

    @validator("created_at", pre=True, always=True)
    def set_datetime(cls, v):
        return v or datetime.utcnow()


class ActionResult(BaseModel):
    success: bool
    event: str


class Display(BaseModel):
    camera_x: Optional[int] = None
    camera_y: Optional[int] = None
    camera_z: Optional[int] = None
    x: int = 0
    y: int = 0


class Node(BaseModel):
    id: str
    name: str
    title: str
    description: str = ""
    properties: Dict = {}
    display: Display
    children: Optional[List[str]] = []
    child: Optional[str] = ""


class Project(BaseModel):
    name: str
    data: Dict
    path: str


class Tree(BaseModel):
    version: str
    scope: str
    id: str
    title: str
    description: str = ""
    root: Union[str, None]
    properties: Dict = {}
    nodes: Dict[str, Node]
    display: Display

    def to_json(self, fname):
        """Saves the tree to a json file"""

        def cleandict(d):
            """Removes all the None fields"""
            if not isinstance(d, dict):
                return d
            return dict((k, cleandict(v)) for k, v in d.items() if v is not None)

        data = cleandict(self.dict())
        with open(fname, "w") as f:
            json.dump(data, f, indent=2)


class Trigger(BaseModel):
    type: str
    trigger: str


class Gambit(Trigger):
    free_chat_turns: int = 0


class BTreeParam(BaseModel):
    id: str
    name: str
    triggers: List[Trigger]


class ARFBitParam(BaseModel):
    id: str
    name: str
    params: Dict


if __name__ == "__main__":
    file = "default.tree.json"
    with open(file) as f:
        data = json.load(f)
    tree = Tree(**data)
    tree.to_json("/tmp/tree.json")
