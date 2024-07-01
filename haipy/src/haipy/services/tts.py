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
import base64
import logging
import os
import re
from itertools import zip_longest

import aiohttp
import requests
import yaml

logger = logging.getLogger(__name__)

PE_MOTOR_COMMAND_CHUNK = re.compile(
    r"((?:<[^<>]+>){1,})"
)  # consecutive commands such as <MO=HT,0.8,0.5><MO=EL,1.0,0.5><MO=CH,0,0.5>


class TTSResponse(object):
    def __init__(self):
        self.response = None
        self.params = {}

    def get_duration(self):
        if self.response:
            return self.response.get("duration", 0)
        return 0

    def write(self, wavfile):
        if self.response:
            data = self.response["data"]
            data = base64.b64decode(data)
            try:
                with open(wavfile, "wb") as f:
                    f.write(data)
                logger.debug("Write to file {}".format(wavfile))
                return True
            except Exception as ex:
                logger.error(ex)
                f = None
            finally:
                if f:
                    f.close()
        else:
            logger.error("No data to write")
        return False

    def __repr__(self):
        return "<TTSResponse params {}, duration {}>".format(
            self.params, self.get_duration()
        )


class PEMotionGenerator(object):
    def __init__(self, pe_config_file):
        with open(pe_config_file) as f:
            self.visemes_config = yaml.safe_load(f)
            self.lip_motor_command_mapping = self.visemes_config["lip_motor_mappings"]

    def split_text_command(self, text):
        """Split the plain text and motor commands and returns pairs of text and commands

        >>> split_text_command(
            "hello <MO=HT,1,0.5><PM>what did you eat<MO=EL,1.0,0.1><MO=CH,0,0.5>")
        [
            ("hello", "<MO=HT,1,0.5><PM>"),
            ("what did you eat", "<MO=EL,1.0,0.1><MO=CH,0,0.5>")
        ]
        """
        chunks = PE_MOTOR_COMMAND_CHUNK.split(text)
        return list(zip_longest(chunks[::2], chunks[1::2], fillvalue=""))

    def generate_motion_commands(self, text, phonemes, vendor):
        """
        text: text
        phonemes: phonemes list
        vendor: voice vendor name
        """
        motion_commands_sequences = []
        for text, animation_commands in self.split_text_command(text):
            viseme_sequences = self.generate_viseme_sequence(phonemes, vendor)
            for viseme_sequence in viseme_sequences:
                motion_commands = "".join(viseme_sequence)
                if animation_commands:
                    # append the animation commands to the
                    # first batch of lip sync commands
                    motion_commands = motion_commands + "<PM>" + animation_commands
                    animation_commands = ""
                motion_commands_sequences.append(motion_commands)
        return motion_commands_sequences

    def generate_viseme_sequence(self, phonemes, vendor):
        mapping = self.visemes_config["mappings"][vendor]
        reverse_mapping = {
            phoneme: v for v, phonemes in mapping.items() for phoneme in phonemes
        }

        # map phonmes to visemes
        visemes = []
        for phoneme in phonemes:
            viseme = {}
            viseme["start"] = phoneme["start"]
            viseme["end"] = phoneme["end"]
            viseme["type"] = "viseme"
            if phoneme["name"] not in reverse_mapping:
                logger.warning("Ignore unrecognized phoneme %s", phoneme["name"])
                continue
            viseme["name"] = reverse_mapping[phoneme["name"]]
            viseme["duration"] = phoneme["end"] - phoneme["start"]
            visemes.append(viseme)

        logger.debug("Original visemes %s", visemes)

        # merge consecutive visemes
        # merge visemes with same name and short visemes
        for i, viseme in enumerate(visemes):
            if viseme.get("merge"):
                continue
            j = i
            while j + 1 < len(visemes):
                next = visemes[j + 1]
                if next["name"] == viseme["name"] or next["duration"] < 0.08:
                    viseme["end"] = next["end"]
                    next["merge"] = True
                    j = j + 1
                else:
                    break
        visemes = [v for v in visemes if not v.get("merge")]
        logger.debug("Simplifed visemes %s", visemes)

        speed = 0.05
        viseme_sequence = []
        for viseme in visemes:
            duration = viseme["duration"] / 1  # longer pauses
            if "%.2f" % duration == "0.00":
                continue
            viseme_motor_command = self.lip_motor_command_mapping[
                viseme["name"]
            ].format(speed=speed)
            viseme_sequence.append(
                "{viseme}<PA={duration:.2f}>{reset}".format(
                    viseme=viseme_motor_command, duration=duration, reset=""
                )
            )
        if viseme["name"] != "Sil":
            viseme_sequence.append(
                "{reset}".format(
                    reset=self.lip_motor_command_mapping["Sil"].format(speed=speed)
                )
            )

        step = 20
        viseme_sequences = []
        steps = list(range(len(viseme_sequence)))[::step]
        for i in steps:
            j = i + step
            if j > len(viseme_sequence):
                j = len(viseme_sequence)
                viseme_sequences.append(viseme_sequence[i:j])
                break
            viseme_sequences.append(viseme_sequence[i:j])
        return viseme_sequences


class TTSClient(object):
    def __init__(self, format="mp3"):
        self.tts_endpoint = os.environ.get("TTS_ENDPOINT", "http://localhost:10001")
        self.tts3_endpoint = os.environ.get("TTS3_ENDPOINT", "http://localhost:10002")
        self.format = format

    def get_url(self, vendor):
        return (
            f"{self.tts3_endpoint}/{vendor}"
            if vendor != "cereproc"
            else f"{self.tts_endpoint}/{vendor}"
        )

    async def async_tts(self, text, vendor, voice):
        params = {
            "text": text,
            "voice": voice,
            "format": self.format,
        }
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        result = TTSResponse()
        url = self.get_url(vendor)
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(url, headers=headers, params=params) as resp:
                    response = await resp.json()
                    result.response = response.get("response")
                    result.params = params
        except Exception as ex:
            logger.error("TTS error %s", ex)

        return result

    def tts(self, text, vendor, voice):
        params = {
            "text": text,
            "voice": voice,
            "format": self.format,
        }
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        result = TTSResponse()
        url = self.get_url(vendor)
        try:
            r = requests.get(url, headers=headers, params=params)
            response = r.json().get("response")
            result.response = response
            result.params = params
        except Exception as ex:
            logger.error("TTS error %s", ex)

        return result


class VisemeMapper(object):
    def __init__(self, viseme_config, mapping_name, params_name):
        mapping, params = viseme_config["mappings"], viseme_config["params"]
        self._default_viseme_mapping = mapping[mapping_name]
        self._default_viseme_params = params[params_name]
        self._mapping = {}
        self.reset()

    def reset(self):
        """Reset config"""
        self.viseme_mapping = self._default_viseme_mapping
        self.viseme_params = self._default_viseme_params

    @property
    def viseme_mapping(self):
        return self._viseme_mapping

    @viseme_mapping.setter
    def viseme_mapping(self, mapping):
        if isinstance(mapping, dict):
            self._viseme_mapping = mapping
            self._update_mapping()

    def _update_mapping(self):
        self._mapping = {}
        for v, s in self._viseme_mapping.items():
            for p in s:
                self._mapping[p] = v

    @property
    def viseme_params(self):
        return self._viseme_params

    @viseme_params.setter
    def viseme_params(self, params):
        if isinstance(params, dict):
            self._viseme_params = params

    def get_visemes(self, phonemes):
        visemes = []
        for ph in phonemes:
            v = self.get_viseme(ph)
            if v is not None:
                visemes.append(v)
        logger.debug("Get visemes {}".format(visemes))
        self.expand_m_visems(visemes)
        return visemes

    def expand_m_visems(self, visemes):
        """
        Let M last longer to close the mouth
        """
        ids = [
            i for i, v in enumerate(visemes) if v["name"] == "M" or v["name"] == "F-V"
        ]
        logger.debug("Before visemes {}".format(visemes))
        logger.debug("ids of M {}".format(ids))
        for id in ids:
            if id < (len(visemes) - 1):
                t = visemes[id + 1]["duration"] / 2
                visemes[id]["duration"] += t
                visemes[id + 1]["start"] += t
                visemes[id + 1]["duration"] -= t
        logger.debug("After visemes {}".format(visemes))

    def get_viseme(self, ph):
        if not self._mapping:
            logger.error("Mapping is not set")
            return
        try:
            v = {}
            v["type"] = "viseme"
            v["name"] = self._mapping[ph["name"]]
            v["start"] = ph["start"]
            v["end"] = ph["end"]
            v["duration"] = ph["end"] - ph["start"]
        except KeyError as ex:
            logger.error(
                "Can't get viseme for %s from %s: error %s",
                ph["name"],
                self._mapping,
                ex,
            )
            return None
        return v

    def filter_visemes(self, visemes, threshold):
        return [viseme for viseme in visemes if viseme["duration"] > threshold]
