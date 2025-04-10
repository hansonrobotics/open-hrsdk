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

from ros_chatbot.interact import event_types as types
from ros_chatbot.interact import action_types

from .base import AgentController

logger = logging.getLogger(__name__)


class InterruptionResumeController(AgentController):

    type = "InterruptionResumeController"

    def __init__(self, id, store, state):
        super(InterruptionResumeController, self).__init__(id, store, state)
        self.subscribe_events = [
            types.ROBOT_INTERRUPTED,
        ]

    def observe(self, event):
        type = event["type"]
        payload = event["payload"]
        if type == types.ROBOT_INTERRUPTED:
            if payload["text"]:
                payload = {
                    "text": payload["text"],
                    "lang": payload["lang"],
                    "controller": self.id,
                }
                action = self.create_action(action_types.INTERRUPTION_RESUME, payload)
                self.store.dispatch(action)
