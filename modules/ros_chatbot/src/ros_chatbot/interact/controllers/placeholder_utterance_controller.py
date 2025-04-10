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
import os
import random
import re
import threading
import time

from ros_chatbot.interact import action_types
from ros_chatbot.interact import event_types as types

from .base import AgentController

logger = logging.getLogger(__name__)


class PlaceholderUtteranceController(AgentController):

    type = "PlaceholderUtteranceController"

    def __init__(self, id, store, state):
        super(PlaceholderUtteranceController, self).__init__(id, store, state)
        self.subscribe_events = [types.UTTERANCE, types.ROBOT_SPEAKING]

        self.running = threading.Event()
        self.current_language = None

        self.utterance_cand = []

        self.wait_for_response = threading.Event()
        self.time_since_new_speech = None
        self.speak_to_robot_prob = 0
        self.speak_to_robot_detected = False
        self.prob_escalate_step = 0.25

        # self.robot_awaken_phrase_pattern = None
        # character = os.environ.get('HR_CHARACTER')
        # if character:
        #    self.robot_awaken_phrase_pattern = re.compile(
        #            r"(hi|hey|hello) {}".format(character), re.IGNORECASE)
        # else:
        #    logger.warning("No character name is found")
        #    self.robot_awaken_phrase_pattern = re.compile(
        #            r"(hi|hey|hello) {}".format("sophia"), re.IGNORECASE)

        job = threading.Thread(target=self.run)
        job.daemon = True
        job.start()

    def set_config(self, config):
        super(PlaceholderUtteranceController, self).set_config(config)
        if "prob_escalate_step" in self.config:
            self.prob_escalate_step = self.config["prob_escalate_step"]

    def run(self):
        while True:
            if self.running.is_set():

                utterance_cand = None
                if not self.utterance_cand:
                    if "utterances" in self.config:
                        utterance_cand = self.config["utterances"].get(
                            self.current_language
                        )
                else:
                    utterance_cand = self.utterance_cand

                if (
                    self.time_since_new_speech is not None
                    and self.speak_to_robot_detected
                ):
                    now = time.time()
                    silence = now - self.time_since_new_speech
                    logger.info("Silence %s", silence)
                    silence_factor = int(silence * 2)  # number of 1/2 second
                    if self.wait_for_response.is_set():
                        # fill placeholder
                        prob = min(1, silence_factor * self.prob_escalate_step)
                        if not utterance_cand:
                            logger.warning("No placeholder utterances")
                            self.time_since_new_speech = None
                        logger.info("Probability %s", prob)
                        if utterance_cand and random.random() < prob:
                            utterance = random.choice(utterance_cand)
                            payload = {
                                "text": utterance,
                                "lang": self.current_language,
                                "controller": self.id,
                            }
                            action = self.create_action(
                                action_types.PLACEHOLDER_UTTERANCE, payload
                            )
                            self.store.dispatch(action)
                            self.time_since_new_speech = None
                time.sleep(0.5)
            else:
                self.running.wait()

    def reset(self):
        self.time_since_new_speech = None
        self.wait_for_response.clear()

    @AgentController.enabled.setter
    def enabled(self, enabled: bool):
        self.config["enabled"] = enabled
        if enabled:
            self.running.set()
        else:
            self.running.clear()
        self.reset()

    def observe(self, event):
        if event["type"] == types.ROBOT_SPEAKING:
            if event["payload"]:
                logger.info("Reset")
                self.reset()  # reset timer when robot starts to speak
                return

        if event["type"] == types.UTTERANCE:
            payload = event["payload"]

            # TODO: other means to detect the user speech event
            self.speak_to_robot_detected = True

            self.current_language = payload["lang"] or "en-US"
            if self.speak_to_robot_detected:
                self.time_since_new_speech = time.time()
                self.wait_for_response.set()

            # if self.robot_awaken_phrase_pattern and \
            #        self.robot_awaken_phrase_pattern.search(msg.utterance):
            #    if self.placeholder_config:
            #        self.speak_to_robot_prob = self.placeholder_config['speaking_initial_prob']
            #    logger.info("Awaken robot")
            # elif self.placeholder_config:
            #    # update the probability
            #    #self.speak_to_robot_prob -= self.placeholder_config['speaking_prob_decay_step'] # delay probability 10%
            #    #self.speak_to_robot_prob = max(0, self.speak_to_robot_prob)

            #    #logger.info("Speak to robot prob %s", self.speak_to_robot_prob)
            #    #self.speak_to_robot_detected = random.random() < self.speak_to_robot_prob
            #    #if self.speak_to_robot_detected:
            #    #    logger.info("Speak to robot detected")

    def set_placeholder_utterances(self, utterances):
        # TODO: set default utterances from config
        self.utterance_cand = utterances

    def set_prob_escalate_step(self, step):
        self.prob_escalate_step = step
