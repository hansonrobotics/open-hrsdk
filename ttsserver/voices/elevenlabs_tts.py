# -*- coding: utf-8 -*-
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

import concurrent.futures
import logging
import os
import shutil
import subprocess
import tempfile
from collections import defaultdict
from typing import Optional

from elevenlabs import VoiceSettings
from elevenlabs.client import ElevenLabs

from ttsserver.action_parser import ActionParser
from voices.azure_tts import AzureTTS

logger = logging.getLogger("hr.ttsserver.voices.elevenlabs")


class ElevenLabsVoice(AzureTTS):
    """ElevenLabs TTS voice implementation"""

    VENDOR = "elevenlabs"
    ELEVENLABS_SPEECH_KEY = os.environ.get("ELEVENLABS_SPEECH_KEY")

    def __init__(self):
        """Initialize ElevenLabs voice."""
        azure_voice = "en-US-AmandaMultilingualNeural"
        super().__init__(azure_voice, ssml=True)
        self._default_tts_params["voice"] = ""
        self._default_tts_params[
            "model"
        ] = "eleven_turbo_v2_5"  # "eleven_multilingual_v2"
        self._default_tts_params["stability"] = 0.5
        self._default_tts_params["similarity"] = 0.5
        self.parser = ActionParser(self.VENDOR)
        self.client = None

    def synthesize(self, text: str) -> bytes:
        if self.client is None:
            self.client = ElevenLabs(api_key=self.ELEVENLABS_SPEECH_KEY)
        try:
            logger.info("ElevenLabs synthesizing %r", text)
            voice_settings = VoiceSettings(
                stability=float(self.tts_params["stability"]),
                similarity_boost=float(self.tts_params["similarity"]),
            )
            audio = self.client.generate(
                text=text,
                voice=self.tts_params["voice"],
                model=self.tts_params["model"],
                voice_settings=voice_settings,
            )
            return b"".join(chunk for chunk in audio)
        except Exception as e:
            raise Exception(f"ElevenLabs API call failed: {str(e)}")

    def do_tts(self, tts_data):
        try:
            text = self.parser.parse(tts_data.text)
        except Exception as ex:
            text = tts_data.text
            logger.exception(ex)
        tts_data.text = text

        loaded = self.load_from_cache(tts_data)
        if not loaded:
            azure_future = None
            elevenlabs_future = None

            # Start Azure TTS to get timing info and ElevenLabs synthesis in parallel
            # Get timing info from Azure, and synthesize audio with ElevenLabs
            with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
                azure_future = executor.submit(super().do_tts, tts_data)
                elevenlabs_future = executor.submit(self.synthesize, text)
                azure_future.result()
                audio_content = elevenlabs_future.result()

            if audio_content is None:
                raise RuntimeError("ElevenLabs synthesize audio failed")

            # Write the ElevenLabs audio content
            with tempfile.NamedTemporaryFile(suffix=".mp3") as fp:
                output = fp.name
                with open(output, "wb") as file:
                    file.write(audio_content)

                if tts_data.format == "wav":
                    subprocess.check_call(
                        f"mpg123 -w {tts_data.wavout} {output} >/dev/null 2>&1",
                        shell=True,
                    )
                elif tts_data.format == "mp3":
                    shutil.copy(output, tts_data.wavout)
                else:
                    raise ValueError(f"Unsupported format: {tts_data.format}")

            # Adjust the phonemes timing to match the ElevenLabs audio length
            phoneme_duration = max([phoneme["end"] for phoneme in tts_data.phonemes])
            audio_duration = tts_data.get_duration()
            logger.info(
                "Adjusting phonemes timing from %s to %s",
                phoneme_duration,
                audio_duration,
            )
            self._adjust_phonemes_timing(
                tts_data.phonemes,
                audio_duration / phoneme_duration,
                -0.1,  # send visemes 0.1 second earlier so it matches the audio
            )

            self.save_to_cache(tts_data)


def load_voices():
    return {ElevenLabsVoice.VENDOR: ElevenLabsVoice()}


voices = load_voices()

if __name__ == "__main__":
    api = voices[ElevenLabsVoice.VENDOR]["SOPHIA SPEAKS SKETCH"]
    api.set_output_dir(".")

    # ttsdata = api.tts(u'Привет! Меня зовут Татьяна. Я прочитаю любой текст который вы введете здесь.')
    # ttsdata = api.tts(u'<prosody rate="50%">Привет! Меня зовут Татьяна. Я прочитаю любой текст который вы введете здесь.</prosody>', wavout='a.wav')
    # ttsdata = api.tts(u'<speak><prosody rate="1.00" pitch="+20%" volume="+20dB">Здравствуйте</prosody></speak>', wavout='a.wav')
    # ttsdata = api.tts('<speak><prosody rate="1.00" pitch="+20%" volume="+20dB">hillo</prosody></speak>', wavout='a.wav')
    ttsdata = api.tts(
        '<prosody rate="1.00" pitch="+20%" volume="+20dB">прочитаю</prosody>',
        wavout="a.wav",
    )
