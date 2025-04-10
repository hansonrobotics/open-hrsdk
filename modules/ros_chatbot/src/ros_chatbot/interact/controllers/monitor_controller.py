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

import os
import re
import logging
import requests
from collections import deque
import threading

from ros_chatbot.interact import event_types as types
from ros_chatbot.interact import action_types

from .base import AsyncAgentController

logger = logging.getLogger(__name__)


class MonitorController(AsyncAgentController):

    type = "MonitorController"

    def __init__(self, id, store, state):
        super(MonitorController, self).__init__(id, store, state)
        self.subscribe_events = [types.ALL_EVENTS]

        self.current_event = None
        self.running = threading.Event()
        job = threading.Thread(target=self.run, name="Thread-%s" % self.id)
        job.daemon = True
        job.start()

    @AsyncAgentController.enabled.setter
    def enabled(self, enabled):
        self.config["enabled"] = enabled
        if enabled:
            self.running.set()
        else:
            self.running.clear()

    def is_idle(self):
        return self.events.empty() and self.current_event is None

    def run(self):
        while True:
            if self.running.is_set():
                self.current_event = self.events.get()
                action = self.create_action(action_types.MONITOR, self.current_event)
                self.store.dispatch(action)
                self.current_event = None
            else:
                self.running.wait()
