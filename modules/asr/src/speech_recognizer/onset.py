#!/usr/bin/python
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

import collections
import logging
import queue as queue

import aubio
import numpy as np

logger = logging.getLogger("hr.speech_recognizer.onset")


class OnsetDetector(object):
    def __init__(self, sample_rate, chunk):
        self.buffer = queue.Queue()
        self.buf_size = 4096
        self.hop_size = chunk
        logger.warning(
            "Sample rate %s, chunk %s, hop %s", sample_rate, chunk, self.hop_size
        )
        self.onset = aubio.onset("default", self.buf_size, self.hop_size, sample_rate)
        self.pitch = aubio.pitch("default", self.buf_size, self.hop_size, sample_rate)
        self.notes = aubio.notes("default", self.buf_size, self.hop_size, sample_rate)
        self.tempo = aubio.tempo("default", self.buf_size, self.hop_size, sample_rate)

    def add_audio_buffer(self, data):
        """Adds audio frames from PCM audio data to buffer."""
        self.buffer.put(data)

    def handle_stream(self, callback):
        if not isinstance(callback, collections.Callable):
            raise ValueError("callback must be callable")
        while True:
            audiobuffer = self.buffer.get()
            # signal size should equal to hop size
            signal = np.frombuffer(audiobuffer, dtype=np.float32)
            onset = self.onset(signal)
            if onset:
                pitch = self.pitch(signal)
                notes = self.notes(signal)
                tempo = self.tempo(signal)
                logger.info(
                    "onset %s, pitch %s, notes %s tempo %s", onset, pitch, notes, tempo
                )
                callback(onset, pitch, notes, tempo)
