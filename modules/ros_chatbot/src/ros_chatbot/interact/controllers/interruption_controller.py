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
import random
import threading

from haipy.utils import to_list
from haipy.nlp.intent_classifier import IntentDetector
from ros_chatbot.interact import event_types as types
from ros_chatbot.interact import action_types as action_types

from .base import AsyncAgentController

logger = logging.getLogger(__name__)


class InterruptionController(AsyncAgentController):

    type = "InterruptionController"

    def __init__(self, id, store, state):
        super(InterruptionController, self).__init__(id, store, state)
        self.subscribe_events = [
            types.UTTERANCE,
            types.KEYWORDS,
        ]
        self._intent_detector = IntentDetector("soultalk")

        self.current_event = None
        self.running = threading.Event()
        job = threading.Thread(target=self.run, name="Thread-%s" % self.id)
        job.daemon = True
        job.start()

    def observe(self, event):
        if not self.state.is_robot_speaking():
            logger.info("Nothing to interrupt")
        elif self.state.is_interruption_mode():
            logger.info("Interruption resuming mode")
        else:
            self.events.put(event)

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
                type = self.current_event["type"]
                payload = self.current_event["payload"]
                try:
                    if type == types.KEYWORDS:
                        text = payload["text"]
                        lang = payload["lang"]
                        if self.is_short_pause_interruption(text, lang):
                            logger.info("Short pause interruption detected")
                            action = self._make_action("soft_interruption", lang, True)
                            self.store.dispatch(action)
                    if type == types.UTTERANCE:
                        text = payload["text"]
                        lang = payload["lang"]
                        # if self.is_full_stop_interrupt(text, lang):
                        #    logger.warning("Full stop interruption detected")
                        #    action = self._make_action('full_stop_interruption', lang, False)
                        #    self.store.dispatch(action)
                        if self.is_short_pause_interruption(text, lang):
                            logger.info("Short pause interruption detected")
                            action = self._make_action(
                                "short_pause_interruption", lang, False
                            )
                            self.store.dispatch(action)
                        elif self.is_long_input_interruption(text, lang):
                            logger.info("Long input interruption detected")
                            action = self._make_action(
                                "long_input_interruption", lang, False
                            )
                            self.store.dispatch(action)
                except Exception as ex:
                    logger.exception(ex)
                self.current_event = None
            else:
                self.running.wait()

    def _make_action(self, action_type, lang, resume):
        utterance = ""
        key = "%s.utterances.%s" % (action_type, lang)
        if key in self.config:
            utterances = to_list(self.config[key])
            utterance = random.choice(utterances)
        else:
            logger.info("No interruption utterances")
        payload = {
            "text": utterance,
            "lang": lang,
            "type": action_type,
            "resume": resume,
            "controller": self.id,
        }
        action = self.create_action(action_types.INTERRUPTION, payload)
        return action

    def is_full_stop_interrupt(self, text, lang):
        if lang == "en-US":
            if "full_stop_interruption.keywords" in self.config:
                keywords = to_list(self.config["full_stop_interruption.keywords"])
                for keyword in keywords:
                    if keyword.lower() in text.lower():
                        return True
            if "full_stop_interruption.intents" in self.config:
                try:
                    result = self._intent_detector.detect_intent(text, lang)
                except Exception as ex:
                    logger.error(ex)
                    return False
                if result:
                    logger.info("Intent %r", result)
                    intent = result["intent"]["name"]
                    if intent in to_list(self.config["full_stop_interruption.intents"]):
                        return True
        return False

    def is_soft_interruption(self, text, lang):
        if lang == "en-US":
            if "soft_interruption.max_input_words" in self.config:
                max_input_words = self.config["soft_interruption.max_input_words"]
                input_length = text.split(" ")
                if len(input_length) > max_input_words:
                    logger.info("Long input (%s) not soft interrupt", len(input_length))
                    return False
            if "soft_interruption.keywords" in self.config:
                keywords = to_list(self.config["soft_interruption.keywords"])
                for keyword in keywords:
                    if keyword.lower() in text.lower():
                        return True
        return False

    def is_short_pause_interruption(self, text, lang):
        if lang == "en-US":
            if "short_pause_interruption.max_input_words" in self.config:
                max_input_words = self.config[
                    "short_pause_interruption.max_input_words"
                ]
                input_length = text.split(" ")
                if len(input_length) > max_input_words:
                    logger.info(
                        "Long input (len=%s) is not short pause interrupt",
                        len(input_length),
                    )
                    return False
            if "short_pause_interruption.keywords" in self.config:
                keywords = to_list(self.config["short_pause_interruption.keywords"])
                for keyword in keywords:
                    if keyword.lower() in text.lower():
                        logger.info("Interruption words %s", keyword)
                        return True
        return False

    def is_long_input_interruption(self, text, lang):
        if lang == "en-US":
            if "long_input_interruption.min_input_words" in self.config:
                min_input_words = self.config["long_input_interruption.min_input_words"]
                input_length = text.split(" ")
                if len(input_length) >= min_input_words:
                    logger.info("Long input (%s) hard interrupt", len(input_length))
                    return True
        return False
