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
from datetime import datetime
from typing import List, NewType, Optional

from beanie import Document
from pydantic import BaseModel, Field
from pydantic.functional_validators import AfterValidator
from typing_extensions import Annotated


def sort_timeline(timelines):
    return sorted(timelines, key=lambda t: t.order)


Node = NewType("Node", dict)


class Timeline(BaseModel):
    order: int
    nodes: List[Node]


class Performance(Document):
    name: str
    tag: List[str] = []
    created_at: datetime = Field(default_factory=datetime.utcnow)
    timelines: Annotated[List[Timeline], AfterValidator(sort_timeline)]

    def add_timeline(self, nodes: List[Node]):
        timeline = Timeline(order=len(self.timelines), nodes=nodes)
        self.timelines.append(timeline)

    class Settings:
        name = "performances"

    def __repr__(self):
        return f"<Performance {self.name}>"

    __str__ = __repr__
