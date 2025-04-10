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

import re
import logging

from haipy.utils import to_list
from haipy.nlp.intent_classifier import IntentDetector
from ros_chatbot.interact import event_types as types
from ros_chatbot.interact import action_types

from .base import SyncAgentController

logger = logging.getLogger(__name__)


class ResponsivityController(SyncAgentController):
    """
    Track the likelihood of response given various conditions.

    The likelihood of response could be changed by
        - preset keyphrases
        - perception
        - control API

    """

    type = "ResponsivityController"

    def __init__(self, id, store, state):
        super(ResponsivityController, self).__init__(id, store, state)
        self.subscribe_events = [
            types.UTTERANCE,
        ]
        self._intent_detector = IntentDetector("soultalk")

    def is_full_stop(self, text, lang):
        if lang == "en-US":
            if "full_stop.keywords" in self.config:
                keywords = to_list(self.config["full_stop.keywords"])
                for keyword in keywords:
                    if keyword.lower() in text.lower():
                        return True
            if "full_stop.intents" in self.config:
                try:
                    result = self._intent_detector.detect_intent(text, lang)
                except Exception as ex:
                    logger.error(ex)
                    return False
                if result:
                    intent = result["intent"]["name"]
                    if intent in to_list(self.config["full_stop.intents"]):
                        return True
        return False

    def is_wakenup(self, text, lang):
        if lang == "en-US":
            if "wakeup.keywords" in self.config:
                keywords = to_list(self.config["wakeup.keywords"])
                for keyword in keywords:
                    if keyword.lower() in text.lower():
                        return True
            if "wakeup.regular_expressions" in self.config:
                expressions = to_list(self.config["wakeup.regular_expressions"])
                for expression in expressions:
                    if re.match(expression, text, re.IGNORECASE):
                        return True
        return False

    def act(self):
        actions = []
        while not self.events.empty():
            event = self.events.get()
            payload = event["payload"]
            if event["type"] == types.UTTERANCE:
                text = payload["text"]
                lang = payload["lang"]
                if self.is_full_stop(text, lang):
                    logger.warning("Full stop detected. Text %r", text)
                    payload = {"controller": self.id, "responsivity": 0}
                    actions.append(
                        self.create_action(action_types.RESPONSIVITY, payload)
                    )
                elif self.is_wakenup(text, lang):
                    logger.warning("Wake up detected")
                    payload = {"controller": self.id, "responsivity": 1}
                    actions.append(
                        self.create_action(action_types.RESPONSIVITY, payload)
                    )
        return actions
