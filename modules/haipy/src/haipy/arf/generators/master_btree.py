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

from slugify import slugify

from haipy.arf.generators.base import ARFGenerator, BTreeGenerator
from haipy.schemas.btree import Tree

logger = logging.getLogger(__name__)


def means_yes(string):
    return string.lower() in ["1", "yes", "1.0", "true"]


class MasterTreeGenerator(ARFGenerator):
    def __init__(self, *args, **kwargs):
        super(MasterTreeGenerator, self).__init__(*args, **kwargs)

        self.btree_generator = BTreeGenerator(
            self.lang, self.header, self.df, self.name
        )
        self.nodes = {}
        self.root_node = None

    def generate(self, dir_params):
        soultalk_topics_dir = dir_params["soultalk_topics_dir"]
        behavior_tree_dir = dir_params["behavior_tree_dir"]
        if "Repeat" in self.df.columns:
            self.df.Repeat = self.df.Repeat.apply(lambda x: "1" if x == "" else x)
        if "RepeatInterval" in self.df.columns:
            self.df.RepeatInterval = self.df.RepeatInterval.apply(
                lambda x: "300" if x == "" else x
            )
        self.generate_behavior(behavior_tree_dir, self.scene_name)

    def add_node(self, node):
        self.nodes[node.id] = node

    def init_main_sequence_node(self, scene):
        """Create the root nodes and returns the main sequence node

                         /-> loop initialization
        root -> sequence --> repeat -> main sequence -> ...
        """
        self.root_node = self.btree_generator.create_sequence_node(scene)
        self.add_node(self.root_node)

        expression = "{{True}}"
        initialization_node = self.btree_generator.create_compute_node(
            scene, expression, title="Compute: Init"
        )
        self.add_node(initialization_node)

        repeat_node = self.btree_generator.create_repeat_node(scene, -1)
        self.add_node(repeat_node)

        main_sequence_node = self.btree_generator.create_sequence_node(scene)
        self.add_node(main_sequence_node)

        repeat_node.child = main_sequence_node.id

        self.root_node.children.append(initialization_node.id)
        self.root_node.children.append(repeat_node.id)

        return main_sequence_node

    def generate_behavior(self, output_dir, root_scene):
        scene = "%s:%s" % (root_scene, self.name)
        main_sequence_node = self.init_main_sequence_node(scene)

        wait_node = self.btree_generator.create_wait_node(scene, 200)
        self.add_node(wait_node)
        main_sequence_node.children.append(wait_node.id)

        priority_sequence_node = self.btree_generator.create_priority_sequence_node(
            scene
        )
        self.add_node(priority_sequence_node)
        main_sequence_node.children.append(priority_sequence_node.id)

        for i, row in self.df.iterrows():
            if row.Scene:
                if means_yes(row.Active):
                    logger.info("Add scene %r", row.Scene)
                    if row.Scene == "free chat":  # free chat is a builtin tree
                        tree_name = "free chat"
                    else:
                        tree_name = slugify(
                            f"{root_scene} {self.script_name} {row.Scene}",
                            lowercase=False,
                        )
                    run_tree_node = self.btree_generator.create_run_tree_node(
                        scene, tree_name, row
                    )
                    self.add_node(run_tree_node)
                    priority_sequence_node.children.append(run_tree_node.id)

        btree_data = {}
        btree_data["version"] = "0.3.0"
        btree_data["scope"] = "tree"
        btree_data["id"] = self.btree_generator.new_id(scene)
        btree_data["title"] = slugify(scene, lowercase=False)
        btree_data["root"] = self.root_node.id
        btree_data["nodes"] = self.nodes
        btree_data["display"] = {
            "camera_x": 484,
            "camera_y": 484,
            "camera_z": 1,
            "x": 0,
            "y": 0,
        }
        btree = Tree(**btree_data)

        ofile = os.path.join(
            output_dir, "%s.tree.json" % slugify(scene, lowercase=False)
        )
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        btree.to_json(ofile)
        logger.info("Saved behavior tree to %s", ofile)
        return ofile
