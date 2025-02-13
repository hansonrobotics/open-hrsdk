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
import logging
import os
import re

import pandas as pd
import yaml
from slugify import slugify

from haipy.arf.generators.base import ARFGenerator, BTreeGenerator
from haipy.schemas.btree import ARFBitParam, BTreeParam, Gambit
from haipy.utils import reduce

logger = logging.getLogger(__name__)


class GeneralChatGenerator(ARFGenerator):
    def validate(self):
        # validate columns
        if "Robot" not in self.df.columns:
            raise ValueError('Column "Robot" is missing')
        if all([column not in self.df.columns for column in ["Trigger", "Question"]]):
            raise ValueError('Either "Trigger" or "Question" column is missing')

    def create_soultalk_topic(self):
        """Creates soultalk topic data"""
        topic = self.init_soultalk_topic()

        if "Name" in self.df.columns and "Trigger" not in self.df.columns:
            self.df["Trigger"] = self.df.Name.apply(lambda x: "event.%s" % x)

        rules = self.construct_soultalk_rules()
        topic["rules"] += rules
        return topic

    def generate_soultalk(self, output_dir):
        topic = self.create_soultalk_topic()
        if topic["rules"]:
            self.write_soultalk_topic(topic, output_dir)

    def generate(self, dir_params):
        """Generates ARF files"""
        self.generate_soultalk(dir_params["soultalk_topics_dir"])


class ReflectionGenerator(GeneralChatGenerator):
    def generate_soultalk(self, output_dir):
        self.tag = ["repeat"]
        topic = self.create_soultalk_topic()
        topic["type"] = "Reflect"
        if topic["rules"]:
            self.write_soultalk_topic(topic, output_dir)


class BasicARFGenerator(GeneralChatGenerator):
    def __init__(self, *args, **kwargs):
        super(BasicARFGenerator, self).__init__(*args, **kwargs)
        self.btree_generator = BTreeGenerator(
            self.lang, self.header, self.df, self.name
        )

    def generate_soultalk(self, output_dir):
        topic = self.create_soultalk_topic()
        topic["type"] = "ARF"
        if self.scene_name:
            topic["settings"]["condition"] = "{{scene == %r}}" % self.scene_name
        if topic["rules"]:
            self.write_soultalk_topic(topic, output_dir)

    def generate(self, dir_params):
        """Generates ARF files"""
        soultalk_topics_dir = dir_params["soultalk_topics_dir"]
        behavior_tree_dir = dir_params["behavior_tree_dir"]
        self.generate_soultalk(soultalk_topics_dir)
        result = self.btree_generator.generate_behavior(
            behavior_tree_dir, self.namespace, self.sheet_name
        )
        return result


class SimpleARFGenerator(GeneralChatGenerator):
    def __init__(self, *args, **kwargs):
        super(SimpleARFGenerator, self).__init__(*args, **kwargs)

    def generate_soultalk(self, output_dir):
        topic = self.create_soultalk_topic()
        topic["type"] = "ARF"
        if self.scene_name:
            topic["settings"]["condition"] = "{{scene == %r}}" % self.scene_name
        if not topic["rules"] or self.header.get("FreeChat"):
            template = self.header.get("DefaultTemplate", "default")
            rule = {
                "input": "*",
                "prompt_template": template,
                "tag": ["llm"],
                "ttl": -1,
            }
            topic["rules"].append(rule)

        if topic["rules"]:
            self.write_soultalk_topic(topic, output_dir)

    def generate(self, dir_params):
        """Generates ARF files"""
        soultalk_topics_dir = dir_params["soultalk_topics_dir"]
        self.generate_soultalk(soultalk_topics_dir)


class GambitBTreeGenerator(GeneralChatGenerator):
    def generate_soultalk(self, output_dir):
        topic = self.create_soultalk_topic()
        # refine the rules: link input/output context
        gambit_count = 0
        context = None
        for rule in topic["rules"]:
            if rule["input"] == "gambit":
                gambit_count += 1
                rule["input"] = "event.gambit%s" % gambit_count
                context = f"{self.name}-gambit{gambit_count}"
                rule["output_context"] = context
            else:
                if context:
                    rule["input_context"] = context
        if topic["rules"]:
            self.write_soultalk_topic(topic, output_dir)

    def generate_gambit_btree_params(self, behavior_tree_dir):
        gambits = []
        gambit_count = 0
        for _, row in self.df.iterrows():
            if not row.Trigger:
                # ignore the rows that have no triggers
                continue
            if row.Trigger == "gambit":
                gambit_count += 1
                data = {"type": "gambit", "trigger": "event.gambit%s" % gambit_count}
                if "Turns" in row and row.Turns:
                    data["free_chat_turns"] = int(float(row.Turns))
                gambit = Gambit(**data)
                gambits.append(gambit)

        btree_name = slugify(self.script_name, lowercase=False)
        btree = BTreeParam(id=self.record.id, triggers=gambits, name=btree_name)
        ofile = os.path.join(behavior_tree_dir, "%s.yaml" % btree_name)
        if not os.path.isdir(behavior_tree_dir):
            os.makedirs(behavior_tree_dir)
        with open(ofile, "w") as f:
            yaml.dump(btree.dict(), f)
        logger.info("Saved behavior tree to %s", ofile)
        return ofile

    def generate(self, dir_params):
        """Generates ARF files"""
        soultalk_topics_dir = dir_params["soultalk_topics_dir"]
        behavior_tree_dir = dir_params["behavior_tree_dir"]
        self.generate_soultalk(soultalk_topics_dir)
        self.generate_gambit_btree_params(behavior_tree_dir)


class ARFBitBTreeGenerator(GeneralChatGenerator):
    def generate_gambit_btree_params(self, behavior_tree_dir):
        btree_name = slugify(self.script_name, lowercase=False)

        params = {}
        for key, value in self.header.items():
            if key not in ["Generator", "Priority"]:
                key = slugify(key, separator="-", lowercase=True)
                params[key] = value
        btree = ARFBitParam(id=self.record.id, params=params, name=btree_name)
        ofile = os.path.join(behavior_tree_dir, "%s.yaml" % btree_name)
        if not os.path.isdir(behavior_tree_dir):
            os.makedirs(behavior_tree_dir)
        with open(ofile, "w") as f:
            yaml.dump(btree.dict(), f)
        logger.info("Saved behavior tree to %s", ofile)
        return ofile

    def generate(self, dir_params):
        """Generates ARF files"""
        soultalk_topics_dir = dir_params["soultalk_topics_dir"]
        behavior_tree_dir = dir_params["behavior_tree_dir"]
        self.generate_soultalk(soultalk_topics_dir)
        self.generate_gambit_btree_params(behavior_tree_dir)
