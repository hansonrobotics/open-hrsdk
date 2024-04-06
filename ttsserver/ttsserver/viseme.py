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

logger = logging.getLogger("hr.ttsserver.viseme")


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
        for v, s in list(self._viseme_mapping.items()):
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
