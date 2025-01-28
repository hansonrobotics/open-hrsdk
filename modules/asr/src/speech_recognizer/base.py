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
import abc


class SpeechRecognizer(object, metaclass=abc.ABCMeta):
    """
    Abstract base class for a speech recognizer.
    """

    def __init__(self):
        # Properties
        self._language = "en-US"
        self._audio_rate = 16000
        self._save_speech = True  # Save audio file to local file system
        self._punctuation = False
        self._phrases = []
        self._alternative_language_codes = ["en-US"]
        self.interim_prefix = ""
        self.last_speech_event = None

        # Callbacks
        self._speech_cb = None
        self._word_cb = None
        self._event_cb = None
        self._service_status_cb = None
        self._interim_cb = None

    def add_speech_cb(self, cb):
        self._speech_cb = cb

    def add_word_cb(self, cb):
        self._word_cb = cb

    def add_event_cb(self, cb):
        self._event_cb = cb

    def add_interim_cb(self, cb):
        self._interim_cb = cb

    def add_service_status_cb(self, cb):
        self._service_status_cb = cb

    def speech_cb(self, text, confidence, audio_path, detected_language):
        if self._speech_cb is not None:
            self._speech_cb(text, confidence, audio_path, detected_language)

    def interim_cb(self, text, stability, language):
        if self._interim_cb is not None:
            self._interim_cb(
                f"{self.interim_prefix} {text}".strip(), stability, language
            )

    def word_cb(self, text, confidence, language):
        if self._word_cb is not None:
            self._word_cb(text, confidence, language)

    def event_cb(self, event):
        self.last_speech_event = event
        if self._event_cb is not None:
            self._event_cb(event)

    @abc.abstractmethod
    def enable(self):
        return

    @abc.abstractmethod
    def disable(self):
        return

    @abc.abstractmethod
    def put_audio(self, data):
        pass

    @property
    def language(self):
        return self._language

    @language.setter
    def language(self, value):
        self._language = value

    @property
    def alternative_language_codes(self):
        return self._alternative_language_codes

    @alternative_language_codes.setter
    def alternative_language_codes(self, value):
        self._alternative_language_codes = value

    @property
    def punctuation(self):
        return self._punctuation

    @punctuation.setter
    def punctuation(self, value):
        self._punctuation = value

    @property
    def audio_rate(self):
        return self._audio_rate

    @audio_rate.setter
    def audio_rate(self, value):
        self._audio_rate = value

    @property
    def save_speech(self):
        return self._save_speech

    @save_speech.setter
    def save_speech(self, value):
        self._save_speech = value

    @property
    def phrases(self):
        return self._phrases

    @phrases.setter
    def phrases(self, value):
        self._phrases = value
