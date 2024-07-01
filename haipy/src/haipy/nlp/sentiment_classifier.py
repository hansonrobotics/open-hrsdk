#
# Copyright (C) 2017-2024 Hanson Robotics
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
import logging
import os

import requests

from haipy.base import SentimentClassifier

logger = logging.getLogger(__name__)


class TransformerSentimentClassifier(SentimentClassifier):
    def __init__(self, host, port, timeout=2):
        self.host = host
        self.port = int(port)
        self.timeout = timeout

    def ping(self):
        try:
            response = requests.get(
                "http://{}:{}".format(self.host, self.port), timeout=self.timeout
            )
        except Exception as ex:
            logger.error(ex)
            return False
        if response.status_code == 200:
            return True
        else:
            logger.error(
                "Sentiment classification server %s:%s is not available",
                self.host,
                self.port,
            )
        return False

    def detect_sentiment(self, text, lang):
        if self.ping():
            try:
                response = requests.get(
                    "http://{}:{}/nlp".format(self.host, self.port),
                    params={"text": text, "lang": lang, "sentiment": "true"},
                    timeout=self.timeout,
                )
            except Exception as ex:
                logger.error(ex)
                return
            if response and response.status_code == 200:
                response = response.json()
                if "sentiment" in response and response["sentiment"]:
                    sentiment = response["sentiment"]
                    logger.info("Sentiment %s for the text: %r", sentiment, text)
                    return sentiment
        else:
            logger.error(
                "Sentiment classification server %s:%s is not available",
                self.host,
                self.port,
            )


if __name__ == "__main__":
    host = "localhost"
    classifier = TransformerSentimentClassifier(host, 8401)
    print(classifier.detect_sentiment("I love you", "en-US"))
    print(classifier.detect_sentiment("I hate you", "en-US"))
