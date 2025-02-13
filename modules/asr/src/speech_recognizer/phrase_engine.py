#
# Copyright (C) 2017-2025 Hanson Robotics
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
from queue import deque

logger = logging.getLogger("hr.asr.phrase_engine")


class PhraseEngine(object):
    def __init__(self, **kwargs):
        self.speech_phrases = deque(maxlen=100)
        self.response_phrases = deque(maxlen=100)
        self.topic_phrases = deque(maxlen=100)
        self._builtin_init_phrases = []
        self._init_phrases = []
        init_phrases = kwargs.get("init_phrases")
        if init_phrases and isinstance(init_phrases, list):
            self._builtin_init_phrases = init_phrases
            logger.info("initial phrases: %s", self._builtin_init_phrases)

    @property
    def init_phrases(self):
        return self._builtin_init_phrases[:] + self._init_phrases[:]

    @init_phrases.setter
    def init_phrases(self, phrases):
        self._init_phrases = phrases
        logger.info("initial phrases %s", self.init_phrases)

    def get_phrases(self):
        phrases = self.init_phrases
        phrases.extend(self.speech_phrases)
        phrases.extend(self.response_phrases)
        phrases.extend(self.topic_phrases)
        phrases = list(set(phrases))
        return phrases

    def add_speech_phrase(self, phrase):
        self.speech_phrases.append(phrase)

    def add_response_phrase(self, phrase):
        self.response_phrases.append(phrase)

    def add_topic_phrase(self, phrase):
        self.topic_phrases.append(phrase)

    def clear_speech_phrases(self):
        self.speech_phrases.clear()

    def clear_response_phrases(self):
        self.response_phrases.clear()

    def clear_topic_phrases(self):
        self.topic_phrases.clear()
