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
import json
import logging
import os
import re
from contextlib import closing

from boto3 import Session
from botocore.exceptions import BotoCoreError, ClientError

from .cacheable import FileCacheable
from .ttsbase import is_ssml, strip_xmltag

logger = logging.getLogger("hr.ttsserver.phoneme_proxy")


class PhonemeProxy(object):
    def get_phonemes(self, *args, **kwargs):
        return NotImplemented


class PollyPhonemeProxy(PhonemeProxy):

    AWS_ACCESS_KEY_ID = os.environ.get("AWS_ACCESS_KEY_ID")
    AWS_SECRET_ACCESS_KEY = os.environ.get("AWS_SECRET_ACCESS_KEY")
    AWS_REGION_NAME = os.environ.get("AWS_REGION_NAME")

    def __init__(self, voice, cache_dir, phoneme_speed_factor=1.2):
        self.cacheable = FileCacheable(cache_dir, suffix=".timing")
        self.session = Session(
            self.AWS_ACCESS_KEY_ID,
            self.AWS_SECRET_ACCESS_KEY,
            region_name=self.AWS_REGION_NAME,
        )
        self.polly = self.session.client("polly")
        self.voice = voice
        self.phoneme_speed_factor = phoneme_speed_factor

    def _polly_markers(self, tts_data):
        """Gets the viseme markers from Polly"""
        text = tts_data.text
        logger.info("Getting phonemes for text %s", text)

        if not is_ssml(text):
            text = "<speak>{}</speak>".format(text)

        try:
            response = self.polly.synthesize_speech(
                Text=text,
                TextType="ssml",
                OutputFormat="json",
                VoiceId=self.voice,
                SpeechMarkTypes=["viseme", "ssml", "word", "sentence"],
            )
            if response is not None and "AudioStream" in response:
                with closing(response["AudioStream"]) as stream:
                    timing = stream.read()
                    return timing
            else:
                logger.error("Could not get audio")
        except (BotoCoreError, ClientError) as error:
            logger.error(error)

    def _get_timing(self, tts_data):
        try:
            timing = self.cacheable.load(tts_data.id)
            return timing
        except IOError as ex:
            logger.error(ex)
            timing = self._polly_markers(tts_data)
            if timing:
                self.cacheable.save(tts_data.id, timing)
            return timing

    def get_phonemes(self, tts_data):
        timing = self._get_timing(tts_data)
        if not timing:
            logger.error("Can't get phonemes")
            return []
        nodes = []
        for node in timing.split("\n"):
            if node:
                nodes.append(json.loads(node))
        phonemes = []
        if timing:
            for i, viseme in enumerate(nodes[:-1]):
                if viseme["type"] == "viseme":
                    phoneme = {"type": "phoneme"}
                    phoneme["start"] = viseme["time"] / 1000.0
                    phoneme["end"] = nodes[i + 1]["time"] / 1000.0
                    phoneme["name"] = viseme["value"]
                    phonemes.append(phoneme)

        # align timing
        phoneme_duration = nodes[-1]["time"] / 1000.0
        audio_duration = tts_data.get_duration()
        ratio = audio_duration / phoneme_duration
        ratio = ratio / self.phoneme_speed_factor
        # print(phoneme_duration)
        # print(audio_duration)
        for p in phonemes:
            p["start"] = p["start"] * ratio
            p["end"] = p["end"] * ratio

        return phonemes


class PinyinPhonemeProxy(PhonemeProxy):
    def nonchinese2pinyin(self, text):
        """replace non-Chinese characters to pinyins"""
        NON_CHN_MAP = {
            "0": "ling",
            "1": "yi",
            "2": "er",
            "3": "san",
            "4": "si",
            "5": "wu",
            "6": "liu",
            "7": "qi",
            "8": "ba",
            "9": "jiu",
        }
        pattern = re.compile("|".join(list(NON_CHN_MAP.keys())))
        new_text = ""
        last_point = 0
        it = re.finditer("[0-9]+", text)
        for i in it:
            new_text += text[last_point : i.span()[0]]
            new_text += pattern.sub(
                lambda x: NON_CHN_MAP[x.group()] + " ", i.group()
            ).strip()
            last_point = i.span()[1]
        new_text += text[last_point:]
        return new_text

    def get_phonemes(self, text, duration):
        import pinyin

        phonemes = []
        regexp = re.compile(
            r"""^(?P<initial>b|p|m|f|d|t|n|l|g|k|h|j|q|x|zh|ch|sh|r|z|c|s|y|w*)(?P<final>\w+)$"""
        )
        if is_ssml(text):
            text = strip_xmltag(text)
        pys = pinyin.get(text, delimiter=" ")
        pys = self.nonchinese2pinyin(pys)
        pys = pys.strip().split(" ")
        logger.info("Get pinyin {}".format(pys))
        if not pys:
            return []
        unit_time = float(duration) / len(pys)
        start_time = 0
        for py in pys:
            match = regexp.match(py)
            if match:
                mid_time = start_time + unit_time / 2
                # Use 2 phonemes for a Chinese character
                initial = match.group("initial")
                final = match.group("final")
                if initial:
                    phonemes.append(
                        {
                            "type": "phoneme",
                            "name": initial.lower(),
                            "start": start_time,
                            "end": mid_time,
                        }
                    )
                if final:
                    phonemes.append(
                        {
                            "type": "phoneme",
                            "name": final.lower(),
                            "start": mid_time,
                            "end": start_time + unit_time,
                        }
                    )
                start_time += unit_time
        logger.debug("phonemes {}".format(phonemes))
        return phonemes


if __name__ == "__main__":
    service = PollyPhonemeProxy()
