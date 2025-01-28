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
import base64
import json
import logging
import os

import requests
import yaml
from slugify import slugify

from haipy.arf.generators.base import ARFGenerator
from haipy.utils import LANGUAGE_BCP47_CODES, robot_character_mapping

logger = logging.getLogger(__name__)

TIMELINE_GENERATOR_SERVER = os.environ.get("TIMELINE_GENERATOR_SERVER", "localhost")
TIMELINE_GENERATOR_PORT = os.environ.get("TIMELINE_GENERATOR_PORT", "11001")


class TimelineGenerator(ARFGenerator):
    def __init__(self, *args, **kwargs):
        super(TimelineGenerator, self).__init__(*args, **kwargs)
        self.timeline_generator_baseurl = (
            f"http://{TIMELINE_GENERATOR_SERVER}:{TIMELINE_GENERATOR_PORT}"
        )
        if self.meta["base"] == "event":
            self.character = robot_character_mapping(self.meta["name"])
        else:
            self.character = self.meta["name"]

    def generate_timelines(self, tts_dir, timelines_dir):
        # validate columns
        if "Robot" not in self.df.columns:
            raise ValueError('Column "Robot" is missing')
        df = self.df.dropna(subset=["Robot"])
        lines = df.Robot.to_list()
        if "Priming" in df:
            primings = df.Priming.to_list()
        else:
            primings = [""] * len(lines)
        if not lines:
            logger.warning("No lines to generate")
            return
        if not os.path.isdir(timelines_dir):
            os.makedirs(timelines_dir)
        payload = {
            "lines": lines,
            "character": self.character,
            "lang": LANGUAGE_BCP47_CODES.get(self.lang, self.lang),
        }
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        try:
            resp = requests.get(
                f"{self.timeline_generator_baseurl}/generate",
                headers=headers,
                json=payload,
            )
            if resp.status_code != 200:
                logger.error(
                    "Error in timeline generation. Sheet %r, error code %r",
                    self.sheet_name,
                    resp.status_code,
                )
                return
            response = resp.json()
            if response["error"]:
                logger.error(response["error"])
                return
            timelines = response["timelines"]
            for i, (timeline, priming) in enumerate(zip(timelines, primings), start=1):
                ofile = os.path.join(timelines_dir, f"{i}.yaml")
                nodes = timeline["nodes"]
                if priming:
                    for node in nodes:
                        if node["name"] == "speech":
                            node["text"] = node["text"] + f"|priming, {priming}|"
                            break
                with open(ofile, "w") as f:
                    yaml.dump({"nodes": nodes}, f)
                    logger.info("Saved timeline %s", ofile)
                format = timeline["format"]
                vendor = timeline["vendor"]
                voice = timeline["voice"]
                if vendor in ["cereproc", "polly", "azure", "snet", "acapela"]:
                    audio_cache = timeline["audio_cache"]
                    for id, data in audio_cache.items():
                        tts_audio_dir = os.path.join(tts_dir, vendor, "cache")
                        if not os.path.isdir(tts_audio_dir):
                            os.makedirs(tts_audio_dir)
                        audio_file = os.path.join(tts_audio_dir, f"{id}.{format}")
                        with open(audio_file, "wb") as f:
                            f.write(base64.b64decode(data))
                            logger.info("Saved audio %s", audio_file)
                        tts_nodes = timeline["tts_nodes"][id]
                        timing_file = os.path.join(tts_audio_dir, f"{id}.timing")
                        with open(timing_file, "w") as f:
                            for node in tts_nodes:
                                f.write(json.dumps(node))
                                f.write("\n")
                            logger.info("Saved timing file %s", timing_file)
        except Exception as ex:
            logger.exception("Can't generate timeline. Error %s", ex)
            raise ex

    def generate_soultalk(self, output_dir):
        """Creates soultalk topic data"""
        topic = self.init_soultalk_topic(ttl=1, type="ARF")
        rules = self.construct_soultalk_rules()
        topic["rules"] += rules
        if topic["rules"]:
            self.write_soultalk_topic(topic, output_dir)

    def generate(self, dir_params):
        tts_dir = dir_params["tts_dir"]
        timelines_dir = os.path.join(
            dir_params["timelines_dir"],
            slugify(self.script_name),
            slugify(self.sheet_name),
        )
        self.generate_timelines(tts_dir, timelines_dir)
        self.generate_soultalk(dir_params["soultalk_topics_dir"])
