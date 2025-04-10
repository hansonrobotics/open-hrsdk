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

from haipy.nlp.intent_classifier import IntentDetector
from ros_chatbot.interact import event_types as types
from ros_chatbot.interact import action_types

from .base import SyncAgentController

logger = logging.getLogger(__name__)


class LanguageSwitchController(SyncAgentController):

    type = "LanguageSwitchController"

    def __init__(self, id, store, state):
        super(LanguageSwitchController, self).__init__(id, store, state)
        self.subscribe_events = [types.UTTERANCE]
        self.intent_detector = IntentDetector("dialogflow")

    def set_config(self, config):
        super(LanguageSwitchController, self).set_config(config)
        self.target_language_code = self.config["target_language_code"]
        self.language_switch_response = self.config["language_switch_response"]

    def reset(self):
        if self.intent_detector:
            self.intent_detector.reset()

    def act(self):
        actions = []
        while not self.events.empty():
            event = self.events.get()
            payload = event["payload"]
            type = event["type"]
            if type == types.UTTERANCE:
                if self.intent_detector:
                    intent = ""
                    confidence = 0
                    try:
                        result = self.intent_detector.detect_intent(
                            payload["text"], payload["lang"]
                        )
                        intent = result["intent"]["name"]
                        confidence = result["intent"]["confidence"]
                        logger.info("Intent %s confidence %s", intent, confidence)
                    except Exception as ex:
                        logger.error(ex)
                        return
                    if confidence > 0.3 and intent == "language.switch":
                        target_lang = ""
                        for entity in result["entities"]:
                            if entity["entity"] == "languge":
                                target_lang = entity["value"]
                                logger.info("Detected target language %r", target_lang)
                                break
                        # switch language
                        if target_lang and target_lang in self.target_language_code:
                            target_lang = self.target_language_code[target_lang]
                            response = self.language_switch_response.get(target_lang)
                            if response and isinstance(response, list):
                                response = random.choice(response)
                                payload = {
                                    "text": response,
                                    "lang": target_lang,
                                    "controller": self.id,
                                }
                                actions.append(
                                    self.create_action(
                                        action_types.SWITCH_LANGUAGE, payload
                                    )
                                )
        return actions
