#!/usr/bin/env python3
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

import logging
import os
import sys

CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, ".."))

import base64
import json
import shutil
from pathlib import Path

import coloredlogs
import yaml
from flask import Flask, Response, request
from ttsserver.viseme import VisemeMapper
from werkzeug.datastructures import Headers

from ttsserver.ttsbase import TTSBase

if "coloredlogs" in sys.modules and os.isatty(2):
    formatter_str = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"
    coloredlogs.install(logging.INFO, fmt=formatter_str)

app = Flask(__name__)
json_encode = json.JSONEncoder().encode
logger = logging.getLogger("hr.tts.server")

HR_DIR = os.path.expanduser(os.environ.get("HR_DIR", "~/.hr"))
SERVER_LOG_DIR = os.path.join(HR_DIR, "log/ttsserver")
DEFAULT_TTS_OUTPUT_DIR = os.path.join(HR_DIR, "ttsserver")
TTS_CONFIG_FILE = os.environ.get("TTS_CONFIG_FILE", Path(CWD) / "tts_config.yaml")
HR_SINGING_DIR = os.environ.get("HR_SINGING_DIR")
VOICES = {}

if TTS_CONFIG_FILE is None or not os.path.isfile(TTS_CONFIG_FILE):
    raise RuntimeError("TTS config file is not found")
with open(TTS_CONFIG_FILE) as f:
    tts_config = yaml.safe_load(f)

viseme_mapping = {
    "polly:*": "polly:polly",
    "azure:*": "azure:azure",
    "acapela:*": "acapela:cereproc",
    "singing:*": "azure:azure",
    "elevenlabs:*": "azure:azure",
}


def get_viseme_mapper(vendor, voice):
    mapping = viseme_mapping.get("%s:*" % (vendor))
    if mapping:
        mapping_name, param_name = mapping.split(":")
        return VisemeMapper(tts_config, mapping_name, param_name)
    else:
        logger.error("Viseme mapping for %s:%s is not configured", vendor, voice)


def load_voices(voice_paths):
    for voice_path in voice_paths:
        if os.path.isdir(voice_path):
            sys.path.insert(0, voice_path)
            module_names = [f for f in os.listdir(voice_path) if f.endswith(".py")]
            for py_module in module_names:
                try:
                    module = __import__(py_module[:-3])
                    if hasattr(module, "voices"):
                        VOICES.update(module.voices)
                except ImportError as ex:
                    logger.exception(ex)


def get_api(vendor, voice):
    api = None
    try:
        config = VOICES.get(vendor)
        if isinstance(config, dict):
            api = config.get(voice)
        else:
            api = config
    except Exception as ex:
        logger.exception(ex)
    return api


def copy_files_and_directories(src, dest):
    if not os.path.exists(dest):
        os.makedirs(dest)
    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dest, item)
        if os.path.isdir(s):
            shutil.copytree(s, d, dirs_exist_ok=True)
        else:
            shutil.copy2(s, d)


@app.route("/<vendor>")
def _tts(vendor):
    logger.info("Start TTS")
    voice = request.args.get("voice")
    text = request.args.get("text")
    format = request.args.get("format", "wav")  # wav or mp3
    params = request.args.to_dict()
    for p in ["text", "vmap", "vparam"]:
        if p in params:
            params.pop(p)
    response = {}
    response["error"] = ""
    api = get_api(vendor, voice)
    if api:
        try:
            tts_data = api.tts(text, **params)
        except Exception as ex:
            logger.exception("Error in running tts for text %s: %s", text, ex)
            tts_data = None
            response["error"] = str(ex)
        if tts_data is None:
            response["error"] = response["error"] or "No TTS data"
            logger.error("No TTS data {}:{}".format(vendor, voice))
        else:
            response["phonemes"] = tts_data.phonemes
            # viseme mapping
            viseme_mapper = get_viseme_mapper(vendor, voice)
            if viseme_mapper:
                response["visemes"] = viseme_mapper.get_visemes(tts_data.phonemes)
            response["markers"] = tts_data.markers
            response["words"] = tts_data.words
            response["nodes"] = tts_data.get_nodes()
            response["text"] = tts_data.text
            response["id"] = tts_data.id
            response["format"] = format
            # overwrite audio by the embedded marker |audio, filepath|
            audio_nodes = [
                node
                for node in response["nodes"]
                if node["type"] == "marker" and node["name"].startswith("audio")
            ]
            if audio_nodes:
                audio_node_name = audio_nodes[0]["name"]
                if "," in audio_node_name:
                    name, filepath = audio_node_name.split(",", 1)
                    filepath = filepath.strip()
                    filepath = os.path.expanduser(filepath)
                    if os.path.isfile(filepath):
                        try:
                            shutil.copy(filepath, tts_data.wavout)
                            logger.warning("Overwrite audio output with %s", filepath)
                        except Exception as ex:
                            logger.exception(ex)
                            raise ex
                    else:
                        raise Exception("Audio file %s doesn't exist", filepath)
            response["duration"] = tts_data.get_duration()
            response["data"] = ""

            if tts_data.wavout and os.path.exists(tts_data.wavout):
                try:
                    with open(tts_data.wavout, "rb") as f:
                        raw = f.read()
                        response["data"] = base64.b64encode(raw).decode("utf-8")
                    os.unlink(tts_data.wavout)
                except Exception as ex:
                    logger.error("Read wav file %r error %s", tts_data.wavout, ex)
                    f = None
                finally:
                    if f:
                        f.close()

    else:
        response["error"] = "Can't get api {}:{}".format(vendor, voice)
        logger.error("Can't get api {}:{}".format(vendor, voice))

    logger.info("End TTS")
    h = Headers()
    h.add("Access-Control-Allow-Origin", "*")
    return Response(
        json_encode({"response": response}), headers=h, mimetype="application/json"
    )


@app.route("/ping", methods=["GET"])
def _ping():
    h = Headers()
    h.add("Access-Control-Allow-Origin", "*")
    return Response(
        json_encode({"response": {"code": 0, "message": "pong"}}),
        headers=h,
        mimetype="application/json",
    )


def main(args):
    tts_output_dir = os.path.expanduser(option.tts_output_dir)

    load_voices(option.voice_paths)
    if len(VOICES) == 0:
        logger.warning("No any voice is loaded")

    for vendor, voice_config in list(VOICES.items()):
        output_dir = os.path.join(tts_output_dir, vendor)
        if isinstance(voice_config, TTSBase):
            voice_config.set_output_dir(output_dir)
        else:
            for voice in list(voice_config.values()):
                voice.set_output_dir(output_dir)
                if (
                    voice.voice == "singing"
                    and HR_SINGING_DIR
                    and os.path.isdir(HR_SINGING_DIR)
                    and HR_SINGING_DIR != voice.cache_dir
                ):
                    logger.info("Copy singing files")
                    copy_files_and_directories(HR_SINGING_DIR, voice.cache_dir)

    cert_file = os.environ.get("SSL_CERT_FILE")
    privkey_file = os.environ.get("SSL_PRIVATE_KEY_FILE")
    if (
        cert_file
        and os.path.isfile(cert_file)
        and privkey_file
        and os.path.isfile(privkey_file)
    ):
        ssl_context = (cert_file, privkey_file)
    else:
        ssl_context = None
    app.run(
        host=option.host,
        debug=False,
        use_reloader=False,
        port=option.port,
        ssl_context=ssl_context,
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser("HR TTS Server")

    parser.add_argument(
        "--port", dest="port", default=10001, help="Server port", type=int
    )
    parser.add_argument("--host", dest="host", default="localhost", help="Server port")
    parser.add_argument(
        "--tts-output-dir",
        dest="tts_output_dir",
        default=DEFAULT_TTS_OUTPUT_DIR,
        help="TTS wave data save directory",
    )
    parser.add_argument(
        "--voice_paths", required=True, nargs="+", dest="voice_paths", help="Voice path"
    )
    option = parser.parse_args()
    main(option)
