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
import re
from collections import defaultdict

from slugify import slugify

from haipy.arf.generators.base import ARFGenerator
from haipy.atlas_vectordb import AtlasVectorDB

logger = logging.getLogger(__name__)


class PromptTemplateGenerator(ARFGenerator):
    def generate_prompt_template(self):
        templates = []
        for idx, row in self.df.iterrows():
            templates.append({"name": row["Name"], "prompt": row["Template"]})
        return templates

    def generate(self, dir_params):
        """Generates ARF files"""
        templates = self.generate_prompt_template()
        return {"prompt_templates": templates, "type": "prompt_template"}


class PromptPresetGenerator(ARFGenerator):
    def generate(self, dir_params):
        """Generates ARF files"""
        presets = {}
        for idx, row in self.df.iterrows():
            preset = defaultdict(dict)
            preset["title"] = slugify(
                row["Event Title"], lowercase=False, separator=" "
            )
            preset["prompts"]["situational_prime"] = row["Situation Prompt"]
            preset["data"]["knowledge_base"] = row["Knowledge Base"].strip()
            AtlasVectorDB.create_vector_store(preset["data"]["knowledge_base"])
            preset["data"]["location"] = row["City"]
            preset["data"]["start"] = row["Start"]
            preset["metadata"]["start"] = row["Start"]
            preset["metadata"]["city"] = row["City"]
            preset["metadata"]["interlocutor"] = row["People"]
            preset["metadata"]["companion"] = row["Companion"]
            presets[slugify(preset["title"], separator="_")] = preset
        return {"prompt_presets": presets, "type": "prompt_presets"}
