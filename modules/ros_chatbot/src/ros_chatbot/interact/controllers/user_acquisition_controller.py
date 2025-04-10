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
import threading

from ros_chatbot.interact import event_types as types
from ros_chatbot.interact import action_types

from .base import AsyncAgentController

logger = logging.getLogger(__name__)


class UserAcquisitionController(AsyncAgentController):

    type = "UserAcquisitionController"

    def __init__(self, id, store, state):
        super(UserAcquisitionController, self).__init__(id, store, state)
        self.subscribe_events = [
            types.FACE_EVENT,
            types.USER_PROFILE,
        ]

        self.current_event = None
        self.running = threading.Event()
        job = threading.Thread(target=self.run, name="Thread-%s" % self.id)
        job.daemon = True
        job.start()

    def is_idle(self):
        return self.events.empty() and self.current_event is None

    @AsyncAgentController.enabled.setter
    def enabled(self, enabled):
        self.config["enabled"] = enabled
        if enabled:
            self.running.set()
        else:
            self.running.clear()

    def run(self):
        while True:
            if self.running.is_set():
                self.current_event = self.events.get()
                type = self.current_event["type"]
                if type == types.FACE_EVENT:
                    payload = self.current_event["payload"]
                    payload["controller"] = self.id
                    action = self.create_action(action_types.HANDLE_FACE_EVENT, payload)
                    self.store.dispatch(action)
                if type == types.USER_PROFILE:
                    payload = {
                        "controller": self.id,
                        "profile": self.current_event["payload"],
                    }
                    action = self.create_action(
                        action_types.UPDATE_USER_PROFILE, payload
                    )
                self.current_event = None
            else:
                self.running.wait()
