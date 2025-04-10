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

import time
import logging
import threading
from typing import Callable

from . import event_types as events
from ros_chatbot.interact.base import BasicEventGenerator, EventGenerator

logger = logging.getLogger(__name__)


class FaceEventGenerator(EventGenerator):
    class Face(object):
        def __init__(self, id) -> None:
            self.id = id
            self.last_seen = time.time()

        @property
        def is_known(self):
            return not self.id.startswith("U")

        def __hash__(self):
            return hash(self.id)

        def __eq__(self, other):
            return self.id == other.id

    def __init__(self, face_lost_timeout=60):
        self.faces = set([])
        self.face_lost_timeout = face_lost_timeout

    def reset(self):
        self.faces = set([])

    def generate(self, face_ids):
        faces = set(
            [FaceEventGenerator.Face(face_id) for face_id in face_ids if face_id]
        )

        lost_faces = []
        now = time.time()
        for face in self.faces - faces:
            if now - face.last_seen > self.face_lost_timeout:
                lost_faces.append(face)
                self.faces.remove(face)

        new_faces = faces - self.faces

        # update faces using newer faces
        self.faces = faces.union(self.faces)

        if new_faces or lost_faces:
            payload = {
                "new_faces": [face.id for face in new_faces],
                "lost_faces": [face.id for face in lost_faces],
            }
            event = self.create_event(events.FACE_EVENT, payload)
            return event


class PoseEventGenerator(EventGenerator):
    def __init__(self):
        self.last = None

    def generate(self, payload):
        # TODO: fix duplicated poses issue
        # number_of_poses = len(payload) if payload else 0
        number_of_poses = 1 if payload else 0
        event = None
        if self.last != number_of_poses:
            if number_of_poses != 0:
                event = self.create_event(events.USER_POSE_DETECTED, payload)
            else:
                if self.last:
                    event = self.create_event(events.USER_POSE_LOST)
            self.last = number_of_poses
            return event


# the state key to event generator mapping
EVENT_GENERATORS = {
    "utterance": BasicEventGenerator(events.UTTERANCE),
    "robot_speaking": BasicEventGenerator(events.ROBOT_SPEAKING),
    "user_speaking": BasicEventGenerator(events.USER_SPEAKING),
    "command": BasicEventGenerator(events.USER_COMMAND),
    "keywords": BasicEventGenerator(events.KEYWORDS),
    "robot_interrupted": BasicEventGenerator(events.ROBOT_INTERRUPTED),
    "user_sentiment_trigger": BasicEventGenerator(events.USER_SENTIMENT_TRIGGER),
    "user_profile": BasicEventGenerator(events.USER_PROFILE),
    "poses": PoseEventGenerator(),
    "face_ids": FaceEventGenerator(10),
}


class StateEvent(object):
    """The class that converts states to events"""

    def __init__(self, state, on_event: Callable[[dict], None]):
        self._state = state
        self._callback = on_event
        self._state_pos = 0  # the position of state the event generated from
        self._generators = EVENT_GENERATORS  # the event generators

        job = threading.Thread(
            name="state_update_checker", target=self.state_update_checker
        )
        job.daemon = True
        job.start()

    def state_update_checker(self):
        while True:
            self._state.wait_for_update()
            self._on_state_change()

    def register_event_generator(self, state, generator: EventGenerator):
        self._generators[state] = generator

    def _on_state_change(self):
        states = self._state.getState()
        state_keys = self._state.getStateChange()
        for state_key in state_keys[self._state_pos :]:
            self._state_pos += 1  # update the position
            generator = self._generators.get(state_key)
            if generator:
                msg_data = states.get(state_key)
                event = generator.generate(msg_data)
                if event:
                    try:
                        self._callback(event)
                    except Exception as ex:
                        logger.exception(ex)

    def reset(self):
        logger.warning("Reset event generator")
        self._state.reset()
        self._state_pos = 0
        for generator in self._generators.values():
            if hasattr(generator, "reset"):
                generator.reset()
