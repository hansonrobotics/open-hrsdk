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

import contextlib
import logging
import os
import uuid
import wave
from queue import Empty, Queue

import pyaudio

logger = logging.getLogger("hr.asr.audio")


def float2pcm(data, dtype="int16"):
    """Convert floating point signal with a range from -1 to 1 to PCM.
    Any signal values outside the interval [-1.0, 1.0) are clipped.
    No dithering is used.
    Note that there are different possibilities for scaling floating
    point numbers to PCM numbers, this function implements just one of
    them.  For an overview of alternatives see
    http://blog.bjornroche.com/2009/12/int-float-int-its-jungle-out-there.html
    """
    import numpy as np

    sig = np.frombuffer(data, dtype=np.float32)

    if sig.dtype.kind != "f":
        raise TypeError("'sig' must be a float array")
    dtype = np.dtype(dtype)
    if dtype.kind not in "iu":
        raise TypeError("'dtype' must be an integer type")

    i = np.iinfo(dtype)
    abs_max = 2 ** (i.bits - 1)
    offset = i.min + abs_max
    sig = (sig * abs_max + offset).clip(i.min, i.max).astype(dtype)
    return sig.tobytes()


class AudioStreamer(object):
    INPUT_DEVICE_INDEX = os.environ.get("INPUT_DEVICE_INDEX")
    if INPUT_DEVICE_INDEX:
        INPUT_DEVICE_INDEX = int(INPUT_DEVICE_INDEX)

    def __init__(self, channels, rate, chunk, format="int16"):
        self.channels = channels
        self.rate = rate
        self.chunk = chunk
        if format == "int16":
            self.format = pyaudio.paInt16
        elif format == "float32":
            self.format = pyaudio.paFloat32
        else:
            raise ValueError("Unknown sample format")

    @contextlib.contextmanager
    def stream(self):
        """Opens a recording stream in a context manager."""
        audio_interface = pyaudio.PyAudio()
        audio_stream = audio_interface.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            frames_per_buffer=self.chunk,
            input=True,
            input_device_index=self.INPUT_DEVICE_INDEX,
        )

        yield audio_stream

        audio_stream.stop_stream()
        audio_stream.close()
        audio_interface.terminate()


class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""

    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1,
            rate=self._rate,
            input=True,
            frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except Empty:
                    break

            yield b"".join(data)


class Audio(object):
    def __init__(self, home_dir, lang="en-US"):
        self.save_dir = os.path.expanduser(os.path.join(home_dir, lang))
        if not os.path.isdir(self.save_dir):
            os.makedirs(self.save_dir)

    def save(self, data, rate, fname=None, sample_width=2, nchannels=1) -> str:
        """Save the audio data to audio file locally."""
        if fname is None:
            fname = "{}.wav".format(str(uuid.uuid1()))
        audio_file = os.path.join(self.save_dir, fname)
        f = None
        try:
            f = wave.open(audio_file, "wb")
            f.setframerate(rate)
            f.setsampwidth(sample_width)
            f.setnchannels(nchannels)
            f.writeframes(data)
            logger.info("Saved audio to {}".format(audio_file))
            return audio_file
        finally:
            if f:
                f.close()


if __name__ == "__main__":
    sample_rate = 48000
    chunk_size = 1600
    nchannels = 1
    steamer = AudioStreamer(nchannels, sample_rate, chunk_size)
    raw = []
    with steamer.stream() as stream:
        while True:
            try:
                data = stream.read(chunk_size)
            except Exception:
                break
            if not data:
                raise StopIteration()
            else:
                print("read %s" % len(data))
            raw.extend(data)
    audio = Audio(".")
    audio.save(bytes(raw), sample_rate, fname="test.wav", nchannels=nchannels)
