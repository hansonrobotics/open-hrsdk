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
from collections import OrderedDict

import yaml

from haipy.arf.generators.base import ARFGenerator

logger = logging.getLogger(__name__)

HEADER_TEMPLATE = """
name: macro
macros: []
"""


class MacroGenerator(ARFGenerator):
    def generate_macro(self, soultalk_topics_dir):
        """Creates soultalk topic data"""
        topic = yaml.safe_load(HEADER_TEMPLATE)
        topic["name"] = self.name

        # validate the mandatory columns
        for column in ["Trigger", "Macro"]:
            if column not in self.df.columns:
                raise ValueError('Column "%s" was missing' % column)

        for rule_count, row in self.df.iterrows():
            rule = OrderedDict()

            if not row.Macro or not row.Trigger:
                continue

            triggers = row.Trigger.splitlines()
            triggers = [trigger.strip() for trigger in triggers if trigger.strip()]
            # format double curly brackets
            triggers = [
                trigger.replace("{{", "{{{{").replace("}}", "}}}}")
                for trigger in triggers
            ]
            rule = {"name": row.Macro, "expression": triggers}

            topic["macros"].append(rule)
        if topic["macros"]:
            self.write_soultalk_topic(topic, soultalk_topics_dir)

    def generate(self, dir_params):
        soultalk_topics_dir = dir_params["soultalk_topics_dir"]
        self.generate_macro(soultalk_topics_dir)
