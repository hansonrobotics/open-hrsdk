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
import os
import sys
import wave

sys.path.insert(0, "/opt/hansonrobotics/lib/python2.7/site-packages/")

from pocketsphinx.pocketsphinx import Decoder

MODELDIR = "/opt/hansonrobotics/share/pocketsphinx/model"

config = Decoder.default_config()
config.set_string("-hmm", os.path.join(MODELDIR, "en-us/en-us"))
config.set_string("-allphone", os.path.join(MODELDIR, "en-us/en-us-phone.lm.dmp"))
config.set_float("-lw", 2.0)
config.set_float("-beam", 1e-10)
config.set_float("-pbeam", 1e-10)


def audio2phoneme(audio_file):
    wave_read = wave.open(audio_file, "rb")
    length = wave_read.getnframes() / wave_read.getframerate()
    wave_read.close()

    # Decode streaming data.
    decoder = Decoder(config)

    buf = bytearray(1024)
    with open(audio_file, "rb") as f:
        decoder.start_utt()
        while f.readinto(buf):
            decoder.process_raw(buf, False, False)
        decoder.end_utt()

    nframes = decoder.n_frames()

    phonemes = []
    offset = None
    for seg in decoder.seg():
        if offset is None:
            offset = seg.start_frame
        start_frame = seg.start_frame - offset
        end_frame = seg.end_frame - offset
        phonemes.append(
            (seg.word, start_frame / nframes * length, end_frame / nframes * length)
        )

    return phonemes


if __name__ == "__main__":
    audio_file = os.path.join(
        os.path.expanduser("~/.hr/tts/numb"), "acapelabox_813168.wav"
    )
    # audio_file = os.path.join(os.path.expanduser('~/.hr/tts/numb'), 'hello.wav')
    phonemes = audio2phoneme(audio_file)
    print("Phonemes: ", phonemes)
