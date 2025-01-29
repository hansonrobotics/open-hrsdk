#!/usr/bin/python
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

import collections
import logging
import queue as queue
from collections import deque

import webrtcvad

logger = logging.getLogger("hr.speech_recognizer.vad")


class VAD(object):
    def __init__(self, sample_rate=16000):
        self.vad = webrtcvad.Vad(3)
        self.sample_rate = sample_rate
        self.speech_start = None
        self.speech_end = None
        self.frame_duration_ms = 30

        self.triggered = False
        self.buffer = queue.Queue()
        self.ring_buffer = deque(maxlen=5)
        self.voiced_bytes = b""
        self.leftover_bytes = b""
        self.timestamp = 0.0  # timestamp in stream

    class Frame(object):
        """Represents a "frame" of audio data."""

        def __init__(self, bytes, timestamp, duration):
            self.bytes = bytes
            self.timestamp = timestamp
            self.duration = duration

        def __repr__(self):
            return "<Frame timestamp: %s, duration: %s" % (
                self.timestamp,
                self.duration,
            )

    def add_audio_buffer(self, data):
        """Adds audio frames from PCM audio data to buffer."""
        audio = self.leftover_bytes + data
        offset = 0
        step = int(self.sample_rate * (self.frame_duration_ms / 1000) * 2)
        duration = (step / self.sample_rate) / 2
        while offset + step < len(audio):
            frame = self.Frame(audio[offset : offset + step], self.timestamp, duration)
            self.timestamp += duration * 1000
            self.buffer.put(frame)
            offset += step
        self.leftover_bytes = audio[offset:]

    def handle_stream(self, callback):
        if not isinstance(callback, collections.Callable):
            raise ValueError("callback must be callable")
        while True:
            frame = self.buffer.get()
            is_speech = self.vad.is_speech(frame.bytes, self.sample_rate)
            if not self.triggered:
                self.ring_buffer.append((frame, is_speech))
                num_voiced = len([f for f, speech in self.ring_buffer if speech])
                # If we're NOTTRIGGERED and more than 90% of the frames in
                # the ring buffer are voiced frames, then enter the
                # TRIGGERED state.
                if num_voiced > 0.8 * self.ring_buffer.maxlen:
                    self.triggered = True
                    first_frame = self.ring_buffer[0][0]
                    start_timestamp = first_frame.timestamp
                    self.speech_start = start_timestamp
                    # We want to yield all the audio we see from now until
                    # we are NOTTRIGGERED, but we have to start with the
                    # audio that's already in the ring buffer.
                    for f, _ in self.ring_buffer:
                        self.voiced_bytes += f.bytes
                    self.ring_buffer.clear()
            else:
                # We're in the TRIGGERED state, so collect the audio data
                # and add it to the ring buffer.
                self.voiced_bytes += frame.bytes
                self.ring_buffer.append((frame, is_speech))
                num_unvoiced = len([f for f, speech in self.ring_buffer if not speech])
                # If more than 90% of the frames in the ring buffer are
                # unvoiced, then enter NOTTRIGGERED and yield whatever
                # audio we've collected.
                if num_unvoiced > 0.8 * self.ring_buffer.maxlen:
                    self.triggered = False
                    end_timestamp = frame.timestamp + frame.duration
                    self.speech_end = end_timestamp
                    self.ring_buffer.clear()

            if self.speech_start and self.speech_end:
                callback(self.speech_start, self.speech_end, self.voiced_bytes)
                self.speech_start = None
                self.speech_end = None
                self.voiced_bytes = b""
