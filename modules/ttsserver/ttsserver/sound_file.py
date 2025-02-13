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
import threading
import logging
import subprocess
import os

logger = logging.getLogger("hr.ttsserver.sound_file")


class SoundFile(object):

    def __init__(self):
        self.is_playing = False
        self.lock = threading.RLock()
        self._interrupt = threading.Event()

    def play(self, wavfile):
        self._interrupt.clear()
        try:
            with self.lock:
                with open(os.devnull, "w") as devnull:
                    proc = subprocess.Popen(
                        ["aplay", wavfile], stdout=devnull, stderr=devnull
                    )
                    job = threading.Timer(0, proc.wait)
                    job.daemon = True
                    job.start()
                    self.is_playing = True
                    while proc.poll() is None:
                        if self._interrupt.is_set():
                            try:
                                proc.terminate()
                            except OSError:
                                pass
        finally:
            self.is_playing = False

    def interrupt(self):
        self._interrupt.set()
        logger.warn("Sound is interrupted")


if __name__ == "__main__":
    import time

    sound = SoundFile()
    fname = "sample.wav"
    threading.Timer(0, sound.play, (fname,)).start()
    threading.Timer(0.5, sound.interrupt).start()
    threading.Timer(1, sound.play, (fname,)).start()
    while True:
        print("Playing", sound.is_playing)
        time.sleep(0.1)
