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
import hashlib
import logging
import os

from haipy.arf.generators.base import ARFGenerator, BTreeGenerator
from haipy.arf.llm_flow import LLMFlow
from haipy.parameter_server_proxy import GlobalContext
from haipy.schemas.airtable_schemas import ScriptRecord
from haipy.schemas.btree import Node, Tree
from haipy.utils import robot_character_mapping

logger = logging.getLogger(__name__)


class InstructiveARFGenerator(ARFGenerator, BTreeGenerator):
    def __init__(self, record: ScriptRecord, sheet_name, lang, meta, infile):
        ARFGenerator.__init__(self, record, sheet_name, lang, meta, infile)
        BTreeGenerator.__init__(self, self.lang, self.header, self.df, self.name)
        if self.meta["base"] == "event":
            self.character = robot_character_mapping(self.meta["name"])
        else:
            self.character = self.meta["name"]
        self.cache = GlobalContext("global.cache")

    def generate(self, dir_params):
        """Generates ARF files"""
        behavior_tree_dir = dir_params["behavior_tree_dir"]
        self.generate_behavior(behavior_tree_dir, self.scene_name)

    def get_cache_key(self, scene, objective, initial_task, guest):
        data = f"{scene.lower()}#{objective.lower()}#{initial_task.lower()}#{guest.lower()}"
        data = data.encode("utf-8")
        return hashlib.sha1(data).hexdigest()

    def generate_prompts(
        self, scene, root_node, objective, initial_task, guest, task_number, turns, lang
    ):
        cache_key = self.get_cache_key(scene, objective, initial_task, guest)
        tasks = []
        if cache_key in self.cache:
            try:
                backgrounds = self.cache[f"{cache_key}.backgrounds"]
                topics = self.cache[f"{cache_key}.topics"]
                tasks = [
                    {"background": background, "topic": topic}
                    for background, topic in zip(backgrounds, topics)
                ]
                logger.info("Using cached tasks")
            except Exception as ex:
                del self.cache[cache_key]
                tasks = []
                logger.error(ex)

        if not tasks:
            flow = LLMFlow(objective, initial_task)
            tasks = flow.gen_task(task_number)

        backgrounds = []
        topics = []
        for task in tasks:
            background = task["background"]
            topic = task["topic"]
            backgrounds.append(background)
            topics.append(topic)
            node = self.create_auto_gpt_node(
                scene, objective, background, topic, turns, guest, self.character, lang
            )
            self.add_node(node)
            self.root_node.children.append(node.id)

        if cache_key not in self.cache:
            self.cache[f"{cache_key}.backgrounds"] = backgrounds
            self.cache[f"{cache_key}.topics"] = topics

    def generate_behavior(self, output_dir, scene):
        self.root_node = self.create_sequence_node(scene)
        self.add_node(self.root_node)

        compute_scene_node = self.create_compute_node(
            scene, '{{% set scene = "{}" %}}'.format(scene)
        )
        self.add_node(compute_scene_node)
        self.root_node.children.append(compute_scene_node.id)

        if "Beginning" in self.header:
            beginning_node = self.create_say_node(
                scene, self.header["Beginning"], self.lang
            )
            self.add_node(beginning_node)
            self.root_node.children.append(beginning_node.id)

        objective = self.header["Objective"]
        initial_task = self.header["InitialTask"]
        guest = self.header.get("Guest")
        task_number = self.header.get("TaskNumber", 3)
        turns = self.header.get("Turns", 3)

        self.generate_prompts(
            scene,
            self.root_node,
            objective,
            initial_task,
            guest,
            task_number,
            turns,
            self.lang,
        )

        reset_scene_node = self.create_compute_node(scene, '{% set scene = "" %}')
        self.add_node(reset_scene_node)
        self.root_node.children.append(reset_scene_node.id)

        if "SetState" in self.header:
            state = self.header["SetState"]
            set_node = self.create_set_node(scene, {"state": state})
            self.add_node(set_node)
            self.root_node.children.append(set_node.id)
            compute_scene_node = self.create_compute_node(
                scene, '{{% set state = "{}" %}}'.format(state)
            )
            self.add_node(compute_scene_node)
            self.root_node.children.append(compute_scene_node.id)

        self.node_layout(
            x=408, nodes=[self.nodes[node_id] for node_id in self.root_node.children]
        )

        btree_properties = self.header.copy()
        btree_data = {}
        btree_data["version"] = "0.3.0"
        btree_data["scope"] = "tree"
        btree_data["id"] = self.new_id(scene)
        btree_data["title"] = scene
        btree_data["root"] = self.root_node.id
        btree_data["nodes"] = self.nodes
        btree_data["properties"] = btree_properties
        btree_data["display"] = {
            "camera_x": 484,
            "camera_y": 484,
            "camera_z": 1,
            "x": 0,
            "y": 0,
        }
        btree = Tree(**btree_data)

        ofile = os.path.join(output_dir, "%s.tree.json" % scene)
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        btree.to_json(ofile)
        logger.info("Saved behavior tree to %s", ofile)
        return ofile
