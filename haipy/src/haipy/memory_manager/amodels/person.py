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
from datetime import datetime
from typing import List

from beanie import Document
from pydantic import BaseModel, Field
from pymongo import ASCENDING, IndexModel

from .base import BaseDocument


class Attribute(BaseModel):
    created_at: datetime = Field(default_factory=datetime.utcnow)
    type: str
    value: str


class Person(BaseDocument):
    first_name: str
    last_name: str
    attributes: List[Attribute] = []

    def add_attribute(self, type: str, value: str):
        attribute = Attribute(type=type, value=value)
        self.attributes.append(attribute)

    class Settings:
        name = "person"
        indexes = [
            IndexModel(
                [("first_name", ASCENDING), ("last_name", ASCENDING)],
                name="unique_first_last_name_index",
                unique=True,
            ),
        ]

    def __repr__(self):
        return f"<Person name: {self.first_name} {self.last_name}>"

    __str__ = __repr__
