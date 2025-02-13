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
from typing import List, Union

from pydantic import BaseModel, Field
from pymongo import ASCENDING, IndexModel
from typing_extensions import Self

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

    def add_attributes(self, **kwargs):
        for key, value in kwargs.items():
            attribute = Attribute(type=key, value=value)
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

    @staticmethod
    def load_people_object(name: str) -> Union[Self, None]:
        if name:
            if " " in name:
                first_name, last_name = name.split(maxsplit=1)
            else:
                first_name = name
                last_name = ""
            person_object = Person.find_person_object(first_name, last_name)
            if person_object:
                return person_object

    @staticmethod
    def find_person_object(first_name: str, last_name: str) -> Union[None, Self]:
        person = None
        if last_name:
            person = Person.find_one(
                Person.first_name == first_name, Person.last_name == last_name
            ).run()
        else:
            person = Person.find_one(Person.first_name == first_name).run()
        return person

    @staticmethod
    def format_person_object(person: Self) -> str:
        # convert person object to string
        info_text_blocks = []
        info_text_blocks.append(f"name: {person.first_name} {person.last_name}")
        for attribute in person.attributes:
            info_text_blocks.append(f"{attribute.type}: {attribute.value}")
        return "\n".join(info_text_blocks)

    def __repr__(self):
        return f"<Person name: {self.first_name} {self.last_name}>"

    __str__ = __repr__
