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

from abc import ABCMeta, abstractmethod


class EventGenerator(object, metaclass=ABCMeta):
    @abstractmethod
    def generate(self, text, lang):
        """Detects intent for the text and language"""
        pass

    @staticmethod
    def create_event(type, payload=None):
        # TODO: validate event payload
        event = {
            "type": type,
            "payload": payload,
        }
        return event


class BasicEventGenerator(EventGenerator):
    def __init__(self, type):
        self._type = type

    def generate(self, payload):
        return self.create_event(self._type, payload)
