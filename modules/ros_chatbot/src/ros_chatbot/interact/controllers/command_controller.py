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

from ros_chatbot.interact import action_types
from ros_chatbot.interact import event_types as types

from .base import SyncAgentController

logger = logging.getLogger(__name__)


class CommandController(SyncAgentController):

    type = "CommandController"

    def __init__(self, id, store, state):
        super(CommandController, self).__init__(id, store, state)
        self.subscribe_events = [types.USER_COMMAND, types.UTTERANCE]

    def act(self):
        actions = []
        while not self.events.empty():
            event = self.events.get()
            payload = event["payload"]
            type = event["type"]
            if type == types.USER_COMMAND:
                command = payload["text"]
                if command == ":reset":
                    actions.append(self.create_action(action_types.RESET))
                else:
                    logger.warning("Unknown command %s", command)
        return actions
