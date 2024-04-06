# -*- coding: utf-8 -*-
import json
import logging
import os
import shutil
import subprocess
import uuid
import xml.etree.ElementTree as ET
import zipfile
from collections import defaultdict

import requests

from ttsserver.action_parser import ActionParser
from ttsserver.ttsbase import TTSBase, wrap_speak_tag

logger = logging.getLogger("hr.ttsserver.voices.acapela")


class AcapelaTTS(TTSBase):

    NEURAL_VOICES = [
        "Jalal22k_NT",
        "Liam22k_NT",
        "Olivia22k_NT",
        "Christinabtob22k_NT",
        "Gabriela22k_NT",
        "Harry22k_NT",
        "Rosie22k_NT",
        "Sophiabtob22k_NT",
        "Merel22k_NT",
        "Tessabtob22k_NT",
        "Thijs22k_NT",
        "Anaisbtob22k_NT",
        "Elise22k_NT",
        "Valentin22k_NT",
        "Ankebtob22k_NT",
        "Jonas22k_NT",
        "Lea22k_NT",
        "Alessio22k_NT",
        "Aurora22k_NT",
        "Barbarabtob22k_NT",
        "Elias22k_NT",
        "Emilie22k_NT",
        "Ida22k_NT",
        "Gosia22k_NT",
        "Lena22k_NT",
        "Anabtob22k_NT",
        "Elenabtob22k_NT",
        "Filip22k_NT",
        "Freja22k_NT",
        "Zeynep22k_NT",
        "Ella22k_NT",
        "EmilioEnglish22k_NT",
        "Josh22k_NT",
        "Lily22k_NT",
        "Scott22k_NT",
        "ValeriaEnglish22k_NT",
        "Emilio22k_NT",
        "Valeria22k_NT",
    ]

    PREMIUM_VOICES = [
        "Jalal22k_HQ",
        "Liam22k_HQ",
        "Olivia22k_HQ",
        "Christinabtob22k_HQ",
        "Gabriela22k_HQ",
        "Harry22k_HQ",
        "RachelTransport22k_HQ",
        "Rosie22k_HQ",
        "Sophiabtob22k_HQ",
        "Merel22k_HQ",
        "Tessabtob22k_HQ",
        "Tessatransport22k_HQ",
        "Thijs22k_HQ",
        "Anaisbtob22k_HQ",
        "Elise22k_HQ",
        "Valentin22k_HQ",
        "Ankebtob22k_HQ",
        "Jonas22k_HQ",
        "Lea22k_HQ",
        "Barbarabtob22k_HQ",
        "Biera22k_CO",
        "Elle22k_CO",
        "Elias22k_HQ",
        "Emilie22k_HQ",
        "Ida22k_HQ",
        "Gosia22k_HQ",
        "Lena22k_HQ",
        "Anabtob22k_HQ",
        "Elenabtob22k_HQ",
        "Filip22k_HQ",
        "Freja22k_HQ",
        "Zeynep22k_HQ",
        "Ella22k_HQ",
        "EmilioEnglish22k_HQ",
        "Josh22k_HQ",
        "Lily22k_HQ",
        "Scott22k_HQ",
        "ValeriaEnglish22k_HQ",
        "Emilio22k_HQ",
        "Valeria22k_HQ",
    ]

    STANDARD_VOICES = [
        "Leila22k_HQ",
        "Mehdi22k_HQ",
        "Nizar22k_HQ",
        "Salma22k_HQ",
        "Nizareng22k_HQ",
        "Lisa22k_HQ",
        "Tyler22k_HQ",
        "Jeroen22k_HQ",
        "JeroenHappy22k_HQ",
        "JeroenSad22k_HQ",
        "Sofie22k_HQ",
        "Zoe22k_HQ",
        "Marcia22k_HQ",
        "Graham22k_HQ",
        "Lucy22k_HQ",
        "Peter22k_HQ",
        "PeterHappy22k_HQ",
        "PeterSad22k_HQ",
        "QueenElizabeth22k_HQ",
        "Rachel22k_HQ",
        "Louise22k_HQ",
        "Laia22k_HQ",
        "Eliska22k_HQ",
        "Mette22k_HQ",
        "Rasmus22k_HQ",
        "Daan22k_HQ",
        "Femke22k_HQ",
        "Jasmijn22k_HQ",
        "Max22k_HQ",
        "Hanna22k_HQ",
        "Hanus22k_HQ",
        "Samuel22k_HQ",
        "Sanna22k_HQ",
        "Alice22k_HQ",
        "Anais22k_HQ",
        "Antoine22k_HQ",
        "AntoineFromAfar22k_HQ",
        "AntoineHappy22k_HQ",
        "AntoineSad22k_HQ",
        "AntoineUpClose22k_HQ",
        "Bruno22k_HQ",
        "Claire22k_HQ",
        "Julie22k_HQ",
        "Manon22k_HQ",
        "Margaux22k_HQ",
        "MargauxHappy22k_HQ",
        "MargauxSad22k_HQ",
        "Andreas22k_HQ",
        "Claudia22k_HQ",
        "ClaudiaSmile22k_HQ",
        "Julia22k_HQ",
        "Klaus22k_HQ",
        "Sarah22k_HQ",
        "Kal22k_HQ",
        "Dimitris22k_HQ",
        "DimitrisHappy22k_HQ",
        "DimitrisSad22k_HQ",
        "Deepa22k_HQ",
        "Chiara22k_HQ",
        "Fabiana22k_HQ",
        "Vittorio22k_HQ",
        "Sakura22k_HQ",
        "Minji22k_HQ",
        "Lulu22k_HQ",
        "Bente22k_HQ",
        "Kari22k_HQ",
        "Olav22k_HQ",
        "Ania22k_HQ",
        "Celia22k_HQ",
        "Isabel22k_HQ",
        "Alyona22k_HQ",
        "Mia22k_HQ",
        "Rhona22k_HQ",
        "Antonio22k_HQ",
        "Ines22k_HQ",
        "Maria22k_HQ",
        "Elin22k_HQ",
        "Emil22k_HQ",
        "Emma22k_HQ",
        "Erik22k_HQ",
        "Ipek22k_HQ",
        "Karen22k_HQ",
        "Kenny22k_HQ",
        "Laura22k_HQ",
        "Micah22k_HQ",
        "Nelly22k_HQ",
        "Rod22k_HQ",
        "Ryan22k_HQ",
        "Saul22k_HQ",
        "Sharon22k_HQ",
        "Tamira22k_HQ",
        "Tracy22k_HQ",
        "Will22k_HQ",
        "WillBadGuy22k_HQ",
        "WillFromAfar22k_HQ",
        "WillHappy22k_HQ",
        "WillLittleCreature22k_HQ",
        "WillOldMan22k_HQ",
        "WillSad22k_HQ",
        "WillUpClose22k_HQ",
        "Rodrigo22k_HQ",
        "Rosa22k_HQ",
    ]

    VOICES = NEURAL_VOICES + PREMIUM_VOICES + STANDARD_VOICES

    def __init__(self, voice, ssml=False):
        super(AcapelaTTS, self).__init__()
        self.voice = voice
        self.ssml = ssml

        # Parameter reference https://www.acapela-cloud.com/docs/
        self.tts_params = {
            "voice": self.voice,
            "mouthpos": "on",
            "wordpos": "on",
            "output": "file",
            "type": "mp3",
        }
        if self.voice == "WillOldMan22k_HQ":
            self.tts_params["speed"] = 125
        if self.voice in ["Ella22k_NT", "Ella22k_HQ"]:
            self.tts_params["speed"] = 90
            self.tts_params["shaping"] = 105

        self.parser = ActionParser("acapela")
        self.url = "https://www.acapela-cloud.com/api"
        self.token = None
        self.voice_id = "acapela:%s" % self.voice

    def get_token(self):
        ACAPELA_ACCOUNT = os.environ.get("ACAPELA_ACCOUNT")
        ACAPELA_PASSWORD = os.environ.get("ACAPELA_PASSWORD")

        if not ACAPELA_ACCOUNT:
            raise RuntimeError('"ACAPELA_ACCOUNT" was not set')
        if not ACAPELA_PASSWORD:
            raise RuntimeError('"ACAPELA_PASSWORD" was not set')

        params = {
            "email": ACAPELA_ACCOUNT,
            "password": ACAPELA_PASSWORD,
        }
        headers = {
            "Content-Type": "application/json",
        }
        response = requests.post(f"{self.url}/login/", headers=headers, json=params)
        if response.status_code == 200:
            respnose = response.json()
            self.token = respnose["token"]

    def online_tts(self, tts_data):
        if self.ssml:
            tts_data.text = wrap_speak_tag(tts_data.text)

        ofile = self.synthesize(tts_data.text)
        if ofile:
            event_file = os.path.join("/tmp/acapela_result", "events.json")
            audio_file = os.path.join("/tmp/acapela_result", "audio.mp3")

            if ofile != audio_file:  # then it is a zip file
                with zipfile.ZipFile(ofile) as myzip:
                    myzip.extractall("/tmp/acapela_result")
                os.unlink(ofile)

                if os.path.isfile(event_file):
                    with open(event_file) as f:
                        data = json.load(f)
                        for event in data["Event"]:
                            if "EventKind" in event:
                                if event["EventKind"] == "Phoneme":
                                    node = {}
                                    node["type"] = "viseme"
                                    if "val" in event["Time"]:
                                        node["time"] = event["Time"]["val"]
                                    else:
                                        node["time"] = float(event["Time"])
                                    node["value"] = event["Viseme"]
                                    self.tts_data.raw_nodes.append(node)
                                elif event["EventKind"] == "Word":
                                    node = {}
                                    node["type"] = "word"
                                    if "val" in event["Time"]:
                                        node["time"] = event["Time"]["val"]
                                    else:
                                        node["time"] = float(event["Time"])
                                    node["value"] = event["Word"]
                                    node["start"] = event["WordOffset"]
                                    node["end"] = (
                                        event["WordOffset"] + event["WordSize"]
                                    )
                                    self.tts_data.raw_nodes.append(node)
                    os.unlink(event_file)
            if tts_data.format == "wav":
                subprocess.check_call(
                    "mpg123 -w {} {} >/dev/null 2>&1".format(
                        tts_data.wavout, audio_file
                    ),
                    shell=True,
                )
            elif tts_data.format == "mp3":
                shutil.copy(audio_file, tts_data.wavout)
            os.unlink(audio_file)

    def synthesize(self, text):
        """Returns a zip file if the voice supports events, otherwise it just returns
        a mp3 file"""
        if self.token is None:
            self.get_token()

        params = {
            "text": text,
        }
        params.update(self.tts_params)
        headers = {"Authorization": f"Token {self.token}"}

        zip_file = "/tmp/acapela_result.zip"
        audio_dir = "/tmp/acapela_result"
        if os.path.isfile(zip_file):
            os.unlink(zip_file)

        response = requests.post(f"{self.url}/command/", headers=headers, json=params)
        if response.status_code == 200:
            if "zip" in response.headers["Content-Type"]:
                with open(zip_file, "wb") as f:
                    f.write(response.content)
                return zip_file
            elif "audio/wav" in response.headers["Content-Type"]:
                audio_file = os.path.join(audio_dir, "audio.wav")
                if not os.path.isdir(audio_dir):
                    os.makedirs(audio_dir)
                with open(audio_file, "wb") as f:
                    f.write(response.content)
                return audio_file
            elif "audio/mpeg" in response.headers["Content-Type"]:
                audio_file = os.path.join(audio_dir, "audio.mp3")
                if not os.path.isdir(audio_dir):
                    os.makedirs(audio_dir)
                with open(audio_file, "wb") as f:
                    f.write(response.content)
                return audio_file
        else:
            if "error" in response.json():
                logger.error(response.json()["error"])

    def do_tts(self, tts_data):
        # parse action
        backup = tts_data.text
        try:
            text = self.parser.parse(tts_data.text)
        except Exception as ex:
            text = backup
            logger.exception(ex)
        tts_data.text = text

        self.tts_data = tts_data
        loaded = self.load_from_cache(tts_data)
        if not loaded:
            self.online_tts(tts_data)
            tts_data.phonemes = self.get_phonemes(self.tts_data.raw_nodes, 1)
            tts_data.markers = self.get_markers(self.tts_data.raw_nodes, 1)
            tts_data.words = self.get_words(self.tts_data.raw_nodes, 1)
            self.save_to_cache(tts_data)
            logger.info("Saved to cache")


def load_voices():
    voices = defaultdict(dict)
    for voice in AcapelaTTS.VOICES:
        if not voice:
            continue
        try:
            api = AcapelaTTS(voice=voice, ssml=False)
            voices["acapela"][voice] = api
        except Exception as ex:
            logger.exception(ex)
            break
    logger.info("Added voices: %s" % ", ".join(list(voices["acapela"].keys())))
    return voices


voices = load_voices()
