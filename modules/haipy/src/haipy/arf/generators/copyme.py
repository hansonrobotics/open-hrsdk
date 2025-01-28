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
import copy
import json
import logging
import os

import yaml

from haipy.arf.generators.base import ARFGenerator, BTreeGenerator
from haipy.utils import create_soultalk_package_meta, dump_yaml, mkdir

logger = logging.getLogger(__name__)

HEADER_TEMPLATE = """
name: main
type: ARF
settings:
  ttl: -1
  priority: normal
rules: []
"""

RESPONSE_TEMPLATE = """
- name: Success Response
  choices: []

- name: Failure Response
  choices: []
"""

ACTION_TEMPLATE = """
action:
  name: say
  properties:
    text: hello
    language: English
"""

RULE_TEMPLATE = """
- input:
  - event.start.line1
  name: line1
  queue:
    - output: >
        {% set line1="Hello, I am Janet! What is your name?" %} {{line1}}
  output_context: line1

- input: "*"
  input_context: line1
  queue:
    - output: >
        {% set score = similarity(input, line1) %}
    - selector:
      - condition: "{{ score>0.8 }}"
        goto: Success Response
      - condition: "{{ True }}"
        queue:
        - goto: Failure Response
        - goto: line1
"""


class CopyMe(ARFGenerator):
    def __init__(self, *args, **kwargs):
        super(CopyMe, self).__init__(*args, **kwargs)
        self.btree_generator = BTreeGenerator(
            self.lang, self.header, self.df, self.name
        )

    def generate_soultalk(self, output_dir):
        template_nodes = yaml.safe_load(RULE_TEMPLATE)
        topic = yaml.safe_load(HEADER_TEMPLATE)
        topic["settings"]["priority"] = self.header["Priority"]
        if self.record.fields.Start:
            topic["settings"]["start"] = self.record.fields.Start
        if self.record.fields.End:
            topic["settings"]["end"] = self.record.fields.End
        topic["name"] = self.name
        if self.scene_name:
            topic["settings"]["condition"] = "{{scene == %r}}" % self.scene_name

        # validate the mandatory columns
        for column in ["Trigger", "Robot"]:
            if column not in self.df.columns:
                raise ValueError('Column "%s" was missing' % column)

        # add success and failure responses
        response_template_rules = yaml.safe_load(RESPONSE_TEMPLATE)
        action_template = yaml.safe_load(ACTION_TEMPLATE)
        for success_response in self.header["Success Response"].splitlines():
            action = copy.deepcopy(action_template)
            action["action"]["properties"]["text"] = success_response
            action["action"]["properties"]["language"] = self.header["Language"]
            response_template_rules[0]["choices"].append(action)
        for success_response in self.header["Failure Response"].splitlines():
            action = copy.deepcopy(action_template)
            action["action"]["properties"]["text"] = success_response
            action["action"]["properties"]["language"] = self.header["Language"]
            response_template_rules[1]["choices"].append(action)
        topic["rules"].extend(response_template_rules)

        for i, row in self.df.iterrows():
            if not row.Trigger.startswith("event."):
                continue
            rules = copy.deepcopy(template_nodes)
            name = row.Trigger.split(".")[-1]
            rules[0]["name"] = name
            rules[0]["input"] = row.Trigger
            response = json.dumps(row.Answer)
            rules[0]["queue"][0][
                "output"
            ] = f"{{% set expect_{i+1}={response} %}} {row.Robot}"
            rules[0]["output_context"] = f"{self.name} {name}"
            rules[1]["input_context"] = f"{self.name} {name}"
            rules[1]["queue"][0][
                "output"
            ] = f"{{% set score = similarity(input, expect_{i+1}) %}}"
            rules[1]["queue"][1]["selector"][1]["queue"][1]["goto"] = f"{name}"
            topic["rules"].extend(rules)

        if topic["rules"]:
            outfile = os.path.join(output_dir, f"{self.name}.yaml")
            mkdir(output_dir)
            package_file = os.path.join(output_dir, "../package.yaml")
            if not os.path.isfile(package_file):
                dump_yaml(create_soultalk_package_meta(), package_file)
            dump_yaml(topic, outfile)
            logger.info("Saved soultalk topic file to %s", outfile)

    def generate(self, dir_params):
        """Generates ARF files"""
        soultalk_topics_dir = dir_params["soultalk_topics_dir"]
        behavior_tree_dir = dir_params["behavior_tree_dir"]
        self.generate_soultalk(soultalk_topics_dir)
        self.btree_generator.generate_behavior(behavior_tree_dir, self.scene_name)
