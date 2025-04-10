# -*- coding: utf-8 -*-

##
## Copyright (C) 2013-2025 Hanson Robotics
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.
##

import re
import logging
import xml.etree.ElementTree as etree
import io
from collections import defaultdict

logger = logging.getLogger(__name__)


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


class MarkPattern(Pattern):
    def __init__(self):
        super(MarkPattern, self).__init__(r"^(.*?)(\|)([^\|]+)\2(.*)$")

    def get_nodes(self, match):
        name = match.group(3)
        args = ""
        if "," in name:
            name, args = name.split(",", 1)
            args = args.strip()
        name = name.strip()
        if name == "pause":
            if args:
                time = args
            else:
                time = "1s"
            if not time.endswith("s"):
                time = time + "s"
            el = etree.Element("break")
            el.set("time", time)
        elif name == "c" or name == "context":
            el = etree.Element("context")
            el.set("name", args)
            el.set("finished", False)
        elif name == "f" or name == "finished":
            el = etree.Element("context")
            el.set("name", args)
            el.set("finished", True)
        else:
            el = etree.Element("mark")
            el.set("name", name)
        return (el,)


class ActionResult(object):
    def __init__(self, nodes):
        self.nodes = nodes

    def to_dict(self):
        output = []
        data = defaultdict(list)
        for node in self.nodes:
            if isinstance(node, str):
                output.append(node)
            elif node.tag == "mark":
                mark = node.attrib["name"]
                if mark:
                    data["marks"].append(mark)
            elif node.tag == "context":
                context = node.attrib["name"]
                finished = node.attrib["finished"]
                if context:
                    context = [
                        {"name": c.strip(), "finished": finished}
                        for c in context.split(",")
                    ]
                    data["context"].extend(context)
        text = "".join(output)
        text = text.strip()
        data["text"] = text
        data = dict(data)
        return data

    def to_xml(self):
        output = []

        for node in self.nodes:
            if isinstance(node, str):
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


class ActionParser(object):
    def __init__(self):
        self.patterns = []
        self.build_patterns()
        self.recognized_nodes = {}
        self.counter = 0
        self.sep = "0x1f"

    def reset(self):
        self.counter = 0
        self.recognized_nodes.clear()

    def build_patterns(self):
        self.patterns.append(MarkPattern())

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
                if not isinstance(node, str):
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
        return ActionResult(nodes)


if __name__ == "__main__":
    logging.basicConfig()
    parser = ActionParser()
    #    print(parser.parse("|happy| |c, abc| test context").to_xml())
    #    print(parser.parse("|happy| |c, 测试, abc| 测试").to_dict())
    print(parser.parse("|happy| |c, 测试, abc| |f, finished, abc| 测试").to_dict())
