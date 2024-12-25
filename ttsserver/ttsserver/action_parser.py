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
import re
import logging
import xml.etree.ElementTree as etree
import io
from .patterns import StrongPattern, EmphasisPattern, MarkPattern

logger = logging.getLogger("hr.ttsserver.action_parser")


class ActionParser(object):
    def __init__(self, setting="cereproc"):
        self.setting = setting
        self.patterns = []
        self.build_patterns()
        self.recognized_nodes = {}
        self.counter = 0
        self.sep = "0x1f"

    def reset(self):
        self.counter = 0
        self.recognized_nodes.clear()

    def build_patterns(self):
        self.patterns.append(StrongPattern(self.setting))
        self.patterns.append(EmphasisPattern(self.setting))
        self.patterns.append(MarkPattern(self.setting))

    def add_recognized_nodes(self, node):
        id = "sss{}eee".format(self.counter)
        self.recognized_nodes[id] = node
        self.counter += 1
        return id

    def recover_recognized_nodes(self, text):
        tokens = text.split(self.sep)
        nodes = []
        for token in tokens:
            if token in self.recognized_nodes:
                node = self.recognized_nodes.get(token)
                nodes.append(node)
            else:
                nodes.append(token)
        return nodes

    def parse(self, text):
        if not isinstance(text, str):
            text = text.decode("utf-8")
        text = text.strip()
        self.reset()
        pattern_index = 0
        while pattern_index < len(self.patterns):
            pattern = self.patterns[pattern_index]
            match = pattern.match(text)

            # Search all the matches then try the next pattern
            if not match:
                pattern_index += 1
                continue

            try:
                nodes = pattern.get_nodes(match)
            except Exception as ex:
                logger.error(ex)
                nodes = [""]  # replace the pattern with an empty string
            place_holders = []
            for node in nodes:
                if not self.is_string(node):
                    id = self.add_recognized_nodes(node)
                    place_holders.append(id)
                else:
                    place_holders.append(node)
            text = "{}{}{}{}{}".format(
                match.group(1),
                self.sep,
                self.sep.join(place_holders),
                self.sep,
                match.groups()[-1],
            )

        nodes = self.recover_recognized_nodes(text)
        result = self.to_xml(nodes)
        result = result.strip()
        return result

    def is_string(self, text):
        return isinstance(text, str)

    def to_xml(self, nodes):
        output = []

        for node in nodes:
            if self.is_string(node):
                output.append(node)
            else:
                buf = io.BytesIO()
                tree = etree.ElementTree(node)
                tree.write(buf, encoding="utf-8")
                value = buf.getvalue()
                value = value.decode("utf-8")
                output.append(value)
                buf.close()
        return "".join(output)


if __name__ == "__main__":
    logging.basicConfig()
    parser = ActionParser()
    print(parser.parse("*Hi there* |happy| this is **action mark down**"))
    print(parser.parse("*Hi there* |happy| |pause, 2| this is **action mark down**"))
    print(
        parser.parse(
            "*Hi ß there* |happy| |pause,2| this is **action mark down** |vocal, esx|"
        )
    )
    print(parser.parse("|vocal,2|"))
    print(parser.parse("|vocal,err|"))
    print(parser.parse("ok |vocal,err22|"))
    print(parser.parse("ok |vocal, 170|"))

    polly_parser = ActionParser("polly")
    print(
        polly_parser.parse(
            "*Hi ß there* |happy| |pause,2| this is **action mark down**"
        )
    )
    print(parser.parse("|vocal, 1| sad, |happy| happy. how happy you are?"))
    print(parser.parse("你好?我的朋友你最近去哪里玩了？"))
    print(parser.parse("|t, smile|"))
    print(parser.parse("test . |pause, 0.2| test 2"))
    print(parser.parse("test . |audio, filepath| test 2"))

    polly_parser = ActionParser("azure")
    print(polly_parser.parse("|p| sad"))
