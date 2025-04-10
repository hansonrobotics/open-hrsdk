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
from queue import Queue
from abc import ABCMeta, abstractmethod

logger = logging.getLogger(__name__)


class AgentController(object):
    type = "base"

    def __init__(self, id, store, state):
        """
        id: the unique id of the controller
        store: the action store
        state: the robot state
        """
        self.id = id
        self.store = store
        self.state = state
        self.events = Queue(maxsize=10)
        self.config = {}
        self.subscribe_events = []

    @property
    def enabled(self):
        return self.config.get("enabled", False)

    @enabled.setter
    def enabled(self, enabled: bool):
        self.config["enabled"] = enabled

    def set_config(self, config: dict):
        self.config = config
        if "enabled" in self.config:
            self.enabled = self.config["enabled"]

    def observe(self, event: dict):
        self.events.put(event)  # {type, payload}

    def create_action(self, type: str, payload=None):
        # TODO: validate action payload
        action = {"type": type}
        action["payload"] = payload or {}
        return action


class AsyncAgentController(AgentController, metaclass=ABCMeta):
    """Synchronized agent controller"""

    type = "AsyncAgentController"

    @abstractmethod
    def is_idle(self):
        """indicates whether the controller is idle"""
        pass


class SyncAgentController(AgentController, metaclass=ABCMeta):
    """Synchronized agent controller"""

    type = "SyncAgentController"

    @abstractmethod
    def act(self):
        pass

    def done(self):
        self.event = None
