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
import json
import logging
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from contextlib import closing

from boto3 import Session
from botocore.exceptions import BotoCoreError, ClientError
from bs4 import BeautifulSoup

from ttsserver.ttsbase import has_html_tag
from ttsserver.action_parser import ActionParser
from ttsserver.ttsbase import TTSBase, strip_xmltag, wrap_speak_tag

logger = logging.getLogger("hr.ttsserver.voices.polly")


class PollyTTS(TTSBase):
    AWS_ACCESS_KEY_ID = os.environ.get("AWS_ACCESS_KEY_ID")
    AWS_SECRET_ACCESS_KEY = os.environ.get("AWS_SECRET_ACCESS_KEY")
    POLLY_NEURAL_VOICE_ENABLE = os.environ.get("POLLY_NEURAL_VOICE_ENABLE", "1")
    AWS_REGION_NAME = "us-west-2"
    # http://docs.aws.amazon.com/polly/latest/dg/voicelist.html
    ARABIC = ["Zeina"]
    CHINESE_MANDARIN = ["Zhiyu"]
    CHINESE_CANTONESE = ["Hiujin"]
    DANISH = ["Mads", "Naja"]
    DUTCH = ["Lotte", "Ruben"]
    ENGLISH_AUSTRALIAN = ["Nicole", "Olivia", "Russell"]
    ENGLISH_BRITISH = ["Amy", "Brian", "Emma"]
    ENGLISH_INDIAN = ["Aditi", "Raveena"]
    English_New_Zealand = ["Aria"]
    English_South_African = ["Ayanda"]
    ENGLISH_US = [
        "Ivy",
        "Joanna",
        "Joey",
        "Justin",
        "Kevin",
        "Kendra",
        "Kimberly",
        "Matthew",
        "Salli",
    ]
    ENGLISH_WELSH = ["Geraint"]
    FRENCH_FRENCH = ["Celine", "Lea", "Mathieu"]
    FRENCH_CANADIAN = ["Chantal", "Gabrielle"]
    GERMAN = ["Hans", "Marlene", "Vicki"]
    HINDI = ["Aditi"]
    ICELANDIC = ["Dora", "Karl"]
    ITALIAN = ["Carla", "Bianca", "Giorgio"]
    JAPANESE = ["Mizuki", "Takumi"]
    KOREAN = ["Seoyeon"]
    NORWEGIAN = ["Liv"]
    POLISH = ["Jacek", "Jan", "Ewa", "Maja"]
    PORTUGUESE_BRAZILIAN = ["Camila", "Ricardo", "Vitoria"]
    PORTUGUESE_EUROPEAN = ["Cristiano", "Ines"]
    ROMANIAN = ["Carmen"]
    RUSSIAN = ["Maxim", "Tatyana"]
    SPANISH_EUROPEAN = ["Lucia", "Conchita", "Enrique"]
    SPANISH_MEXICAN = ["Mia"]
    SPANISH_US = ["Lupe", "Penelope", "Miguel"]
    SWEDISH = ["Astrid"]
    TURKISH = ["Filiz"]
    WELSH = ["Gwyneth"]
    VOICES = (
        ARABIC
        + CHINESE_MANDARIN
        + CHINESE_CANTONESE
        + DANISH
        + DUTCH
        + ENGLISH_AUSTRALIAN
        + ENGLISH_BRITISH
        + ENGLISH_INDIAN
        + English_New_Zealand
        + English_South_African
        + ENGLISH_US
        + ENGLISH_WELSH
        + FRENCH_FRENCH
        + FRENCH_CANADIAN
        + GERMAN
        + HINDI
        + ICELANDIC
        + ITALIAN
        + JAPANESE
        + KOREAN
        + NORWEGIAN
        + POLISH
        + PORTUGUESE_BRAZILIAN
        + PORTUGUESE_EUROPEAN
        + ROMANIAN
        + RUSSIAN
        + SPANISH_EUROPEAN
        + SPANISH_MEXICAN
        + SPANISH_US
        + SWEDISH
        + TURKISH
        + WELSH
    )
    NEURAL_VOICES = (
        CHINESE_MANDARIN
        + CHINESE_CANTONESE
        + ["Olivia"]
        + ENGLISH_BRITISH
        + English_New_Zealand
        + English_South_African
        + ENGLISH_US
        + ["Lea"]
        + ["Gabrielle"]
        + ["Vicki"]
        + ["Bianca"]
        + ["Takumi"]
        + KOREAN
        + ["Camila"]
        + ["Lucia"]
        + ["Lupe"]
    )

    def __init__(self, voice, ssml=False):
        """
        Polly voice list
        http://docs.aws.amazon.com/polly/latest/dg/voicelist.html
        """
        super(PollyTTS, self).__init__()
        self.voice = voice
        self.ssml = ssml
        self.tts_params = {}
        self.tts_params["voice"] = self.voice
        self.parser = ActionParser("polly")
        self.voice_id = "polly:%s" % self.voice
        self.get_polly()

    def get_polly(self):
        session = Session(
            self.AWS_ACCESS_KEY_ID,
            self.AWS_SECRET_ACCESS_KEY,
            region_name=self.AWS_REGION_NAME,
        )
        self.polly = session.client("polly")

    def get_engine(self):
        engine = "standard"
        if self.POLLY_NEURAL_VOICE_ENABLE == "1":
            if self.voice in self.NEURAL_VOICES:
                engine = "neural"
        return engine

    def synthesize(self, tts_data, engine, marks):
        text = tts_data.text
        if self.ssml:
            text = wrap_speak_tag(text)
        text_type = "ssml" if self.ssml else "text"
        response = None
        speech_args = {
            "Engine": engine,
            "Text": text,
            "TextType": text_type,
            "OutputFormat": "mp3",
            "VoiceId": self.voice,
        }
        if marks:
            speech_args["SpeechMarkTypes"] = ["viseme", "ssml", "word"]
            speech_args["OutputFormat"] = "json"
        else:
            logger.info("Using %r voice engine", engine)
        try:
            tts_data.text = text
            response = self.polly.synthesize_speech(**speech_args)
        except (BotoCoreError, ClientError) as error:
            self.get_polly()
            if engine == "neural":
                logger.warning("Retry with standard voice engine")
                response = self.synthesize(
                    tts_data, "standard", marks
                )  # retry with stardard voice
            else:
                raise RuntimeError(
                    "Synthesize error: %s engine: %s, voice: %s"
                    % (error, engine, self.voice)
                )
        return response

    def do_tts(self, tts_data):
        """See https://docs.aws.amazon.com/polly/latest/dg/API_SynthesizeSpeech.html"""
        loaded = self.load_from_cache(tts_data)
        if not loaded:
            # parse action
            backup = tts_data.text
            try:
                text = self.parser.parse(tts_data.text)
            except Exception as ex:
                text = backup
                logger.exception(ex)
            tts_data.text = text

            logger.info("Synthesizing %r", tts_data.text)
            response = self.synthesize(tts_data, self.get_engine(), False)
            if response is None:
                raise RuntimeError("Synthesize audio failed")

            # Access the audio stream from the response
            if response is not None and "AudioStream" in response:
                # Note: Closing the stream is important as the service throttles on the
                # number of parallel connections. Here we are using contextlib.closing to
                # ensure the close method of the stream object will be called automatically
                # at the end of the with statement's scope.
                with tempfile.NamedTemporaryFile(suffix=".mp3") as fp:
                    output = fp.name
                    with closing(response["AudioStream"]) as stream:
                        with open(output, "wb") as file:
                            file.write(stream.read())
                    if tts_data.format == "wav":
                        subprocess.check_call(
                            "mpg123 -w {} {} >/dev/null 2>&1".format(
                                tts_data.wavout, output
                            ),
                            shell=True,
                        )
                    elif tts_data.format == "mp3":
                        shutil.copy(output, tts_data.wavout)
            else:
                logger.error("Could not get audio")

            logger.info("Getting timing info")
            nodes = self.get_timing_nodes(tts_data)
            tts_data.phonemes = self.get_phonemes(nodes)
            tts_data.markers = self.get_markers(nodes)
            tts_data.words = self.get_words(nodes)
            self.save_to_cache(tts_data)

    def get_timing_nodes(self, tts_data):
        timing_file = "%s.timing" % os.path.splitext(tts_data.wavout)[0]
        text = tts_data.text
        stripped = strip_xmltag(text)
        if not stripped:
            # only markers remains
            # in this case the server can't generate markers
            soup = BeautifulSoup(text, "lxml")
            nodes = []
            for i, mark in enumerate(soup.find_all("mark")):
                node = {
                    "time": i,
                    "type": "ssml",
                    "start": 0,
                    "end": 0,
                    "value": mark.attrs["name"],
                }
                nodes.append(node)
            with open(timing_file, "w") as f:
                for node in nodes:
                    f.write(json.dumps(node))
                    f.write("\n")
        else:
            response = self.synthesize(tts_data, self.get_engine(), True)
            if response is not None and "AudioStream" in response:
                with closing(response["AudioStream"]) as stream:
                    with open(timing_file, "wb") as file:
                        file.write(stream.read())
            else:
                logger.error("Could not get audio")

        with open(timing_file) as f:
            lines = f.read().splitlines()
            raw_nodes = [json.loads(line) for line in lines]
        os.unlink(timing_file)
        return raw_nodes


def load_voices():
    voices = defaultdict(dict)
    for voice in PollyTTS.VOICES:
        if not voice:
            continue
        try:
            api = PollyTTS(voice=voice, ssml=True)
            voices["polly"][voice] = api
        except Exception as ex:
            logger.error(ex)
            break
    logger.info("Added voices: %s" % ", ".join(list(voices["polly"].keys())))
    return voices


voices = load_voices()
