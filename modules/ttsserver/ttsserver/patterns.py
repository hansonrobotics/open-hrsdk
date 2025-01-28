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
import re
import xml.etree.ElementTree as etree

CEREPROC_NAME2GESTUREID = {
    "tut": "g0001_001",
    "tut tut": "g0001_002",
    "cough": "g0001_003",
    "cough2": "g0001_004",
    "cough3": "g0001_005",
    "clear throat": "g0001_006",
    "breath in": "g0001_007",
    "sharp intake of breath": "g0001_008",
    "breath in through teeth": "g0001_009",
    "sigh happy": "g0001_010",
    "sigh sad": "g0001_011",
    "hmm question": "g0001_012",
    "hmm yes": "g0001_013",
    "hmm thinking": "g0001_014",
    "umm": "g0001_015",
    "umm2": "g0001_016",
    "err": "g0001_017",
    "err2": "g0001_018",
    "giggle": "g0001_019",
    "giggle2": "g0001_020",
    "laugh": "g0001_021",
    "laugh2": "g0001_022",
    "laugh3": "g0001_023",
    "laugh4": "g0001_024",
    "ah positive": "g0001_025",
    "ah negative": "g0001_026",
    "yeah question": "g0001_027",
    "yeah positive": "g0001_028",
    "yeah resigned": "g0001_029",
    "sniff": "g0001_030",
    "sniff2": "g0001_031",
    "argh": "g0001_032",
    "argh2": "g0001_033",
    "ugh": "g0001_034",
    "ocht": "g0001_035",
    "yay": "g0001_036",
    "oh positive": "g0001_037",
    "oh negative": "g0001_038",
    "sarcastic noise": "g0001_039",
    "yawn": "g0001_040",
    "yawn2": "g0001_041",
    "snore": "g0001_042",
    "snore phew": "g0001_043",
    "zzz": "g0001_044",
    "raspberry": "g0001_045",
    "raspberry2": "g0001_046",
    "brrr cold": "g0001_047",
    "snort": "g0001_048",
    "ha ha (sarcastic)": "g0001_050",
    "doh": "g0001_051",
    "gasp": "g0001_052",
}


class Pattern(object):
    def __init__(self, pattern):
        self.pattern = pattern
        self.pattern_re = re.compile(self.pattern, re.DOTALL | re.UNICODE)

    def match(self, text):
        return self.pattern_re.match(text)

    def get_nodes(self, match):
        return NotImplemented

    def __repr__(self):
        return self.__class__.__name__


class EmphasisPattern(Pattern):
    def __init__(self, setting):
        super(EmphasisPattern, self).__init__(r"^(.*?)(\*)([^\*]+)\2(.*)$")
        self.setting = setting

    def get_nodes(self, match):
        if self.setting == "cereproc":
            el = etree.Element("prosody")
            el.text = match.group(3)
            el.set("rate", "-20%")
            el.set("pitch", "+5%")
        elif self.setting == "polly" or self.setting == "azure":
            el = etree.Element("emphasis")
            el.text = match.group(3)
        return (el,)


class StrongPattern(Pattern):
    def __init__(self, setting):
        super(StrongPattern, self).__init__(r"^(.*?)(\*{2})([^\*]+)\2(.*)$")
        self.setting = setting

    def get_nodes(self, match):
        if self.setting == "cereproc":
            el = etree.Element("prosody")
            el.text = match.group(3)
            el.set("rate", "-20%")
            el.set("volume", "+6dB")
            el.set("pitch", "+5%")
            el2 = etree.Element("break")
            el2.set("time", "150ms")
            return el2, el
        elif self.setting == "polly" or self.setting == "azure":
            el = etree.Element("emphasis")
            el.text = match.group(3)
            el.set("level", "strong")
            return (el,)


class MarkPattern(Pattern):
    def __init__(self, setting="cereproc"):
        super(MarkPattern, self).__init__(r"^(.*?)(\|)([^\|]+)\2(.*)$")
        self.setting = setting

    def get_nodes(self, match):
        if self.setting in ["snet", "acapela", "elevenlabs"]:
            return [""]
        name = match.group(3)
        name = name.strip()
        if name.startswith("pause"):
            if "," in name:
                name, time = name.split(",", 1)
            else:
                time = "1s"
            time = time.strip()
            if not time.endswith("s"):
                time = time + "s"
            el = etree.Element("break")
            el.set("time", time)
        elif name.startswith("vocal"):
            if self.setting != "cereproc":
                return [""]
            # cereproc only feature
            if "," in name:
                name, gesture = name.split(",", 1)
                name = name.strip()
                gesture = gesture.strip()
                if gesture in CEREPROC_NAME2GESTUREID:
                    text = gesture
                    gid = CEREPROC_NAME2GESTUREID[gesture]
                else:
                    try:
                        gesture = int(gesture)
                        text = "vocal gesture {}".format(gesture)
                        if gesture > 0 and gesture < 53:
                            gid = "g0001_{:03d}".format(gesture)
                        else:
                            raise SyntaxError(
                                "vocal syntax error: Unknown vocal gesture %r" % gesture
                            )
                    except ValueError:
                        raise SyntaxError(
                            "vocal syntax error: Unknown vocal gesture %r" % gesture
                        )
                el = etree.Element("spurt")
                el.set("audio", gid)
                el.text = text
            else:
                raise SyntaxError("vocal syntax error: not enough argument")
        else:
            el = etree.Element("mark")
            el.set("name", name)
        return (el,)
