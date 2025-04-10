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

from haipy.nlp.sentiment_classifier import TransformerSentimentClassifier
from ros_chatbot.interact import event_types as types

from .base import SyncAgentController

logger = logging.getLogger(__name__)


class EmotionController(SyncAgentController):

    type = "EmotionController"

    def __init__(self, id, store, state):
        super(EmotionController, self).__init__(id, store, state)
        host = os.environ.get("NLP_SERVER_HOST", "localhost")
        port = os.environ.get("NLP_SERVER_PORT", 8401)
        self.state = state
        self.classifier = TransformerSentimentClassifier(host, port)
        self.subscribe_events = [
            types.UTTERANCE,
        ]
        self.sentiment_length = 5
        self.decays = [0.6, 0.7, 0.8, 0.9, 1]

    def act(self):
        actions = []
        while not self.events.empty():
            event = self.events.get()
            if event["type"] == types.UTTERANCE:
                payload = event["payload"]
                sentiment = self.classifier.detect_sentiment(
                    payload["text"], payload["lang"]
                )
                if sentiment:
                    # update sentiments
                    state = self.state.getState()
                    if "user_sentiments" in state:
                        sentiments = state["user_sentiments"]
                    else:
                        sentiments = deque(maxlen=self.sentiment_length)
                        sentiments.extend([0] * self.sentiment_length)
                    sentiments.append(sentiment)
                    self.state.update(user_sentiments=sentiments)

                    # find significant sentiment
                    state = self.state.getState()
                    sentiments = state["user_sentiments"]
                    decayed_sentiments = [
                        abs(i) * j for i, j in zip(sentiments, self.decays)
                    ]  # decay
                    abs_sentiments = [abs(i) for i in decayed_sentiments]
                    index = abs_sentiments.index(max(abs_sentiments))
                    significant_sentiment = sentiments[index]

                    # fire event
                    if significant_sentiment > 0.6 or significant_sentiment < -0.4:
                        self.state.update(user_sentiment_trigger=significant_sentiment)
                        logger.info("user sentiment %s", significant_sentiment)
        return actions
