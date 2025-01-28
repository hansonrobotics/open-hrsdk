#!/usr/bin/env python
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

import datetime
import json
import logging
import os
import pathlib
import re
import tempfile
import uuid
import warnings
from collections import OrderedDict
from typing import Union

import pandas as pd
from slugify import slugify

from haipy.arf.generators import registered_generators
from haipy.arf.sniff import sniff
from haipy.schemas.airtable_schemas import LANGUAGE_CODES, Sheet
from haipy.schemas.btree import Node, Tree
from haipy.text_processing.template_renderer import Renderer
from haipy.utils import (
    LANGUAGE_BCP47_CODES,
    create_soultalk_package_meta,
    dump_yaml,
    mkdir,
    parse_yaml_str,
)

logger = logging.getLogger(__name__)

TAILING_PUNCTUATORS = re.compile(r"""[.?!。？！]+$""", flags=re.UNICODE)
renderer = Renderer(variable_start_string="[<", variable_end_string=">]")

warnings.simplefilter(action="ignore", category=FutureWarning)

LANG_CODE_MAPPING = {
    "en": "en-US",
    "hk": "yue-Hant-HK",
    "zh": "cmn-Hans-CN",
    "ko": "ko-KR",
}

ACTION_TITLES = {
    "chat": 'Chat "<text>"',
    "compute": "Compute <expression>",
    "say": "Say <text>",
}

INTERNAL_HEADER_FIELDS = ["Generator", "Priority", "Start", "End", "Variables"]


def remove_puncturaion_marks(text):
    text = re.sub(TAILING_PUNCTUATORS, "", text)
    return text


def reduce(array):
    if len(array) == 1:
        return array[0]
    else:
        return array


def clean_up_df(df):
    return (
        df.dropna(how="all")
        .fillna("")
        .astype("str")
        .apply(lambda x: x.str.strip() if x.dtype == "object" else x)
    )


def filter_event_stage(df, event_stage):
    df["EventStage"].fillna("CURRENT_EVENT", inplace=True)
    df["EventStage"] = df["EventStage"].str.upper()
    df = df.apply(lambda x: x.str.strip() if x.dtype == "object" else x)
    if event_stage is None:
        event_stage = "CURRENT_EVENT"
    df = df[df["EventStage"] == event_stage.upper()]
    return df


cwd = os.path.dirname(os.path.abspath(__file__))
BEHAVIOR_DIR = os.environ.get("BEHAVIOR_DIR", os.path.join(cwd, "../../behavior_trees"))

DEFAULT_NODE_DEFS = {
    "Sequence": {
        "title": "Sequence",
        "category": "Composite",
        "description": "",
        "properties": {},
    },
    "Priority": {
        "title": "Priority",
        "category": "Composite",
        "description": "",
        "properties": {},
    },
    "Wait": {
        "title": "Wait <milliseconds>ms",
        "category": "Action",
        "description": "",
        "properties": {"milliseconds": 0},
    },
    "Repeater": {
        "title": "Repeat <maxLoop>x",
        "category": "Decorator",
        "description": "",
        "properties": {"maxLoop": -1},
    },
    "RepeatUntilSuccess": {
        "title": "Repeat Until Success",
        "category": "Decorator",
        "description": "",
        "properties": {"maxLoop": -1},
    },
    "RepeatUntilFailure": {
        "title": "Repeat Until Failure",
        "category": "Decorator",
        "description": "",
        "properties": {"maxLoop": -1},
    },
    "MaxTime": {
        "title": "Max <maxTime>ms",
        "category": "Decorator",
        "description": "",
        "properties": {"maxTime": 0},
    },
    "Inverter": {
        "title": "Inverter",
        "category": "Decorator",
        "description": "",
        "properties": {},
    },
}
global_trees = {}
global_node_defs = {}
global_node_defs.update(DEFAULT_NODE_DEFS)


def cast_to_number(i):
    if isinstance(i, int) or isinstance(i, float):
        return i
    try:
        if "." in i:
            return float(i)
        else:
            return int(i)
    except (TypeError, ValueError):
        return i


def load_node_defs():
    for node_def_file in pathlib.Path(os.path.join(BEHAVIOR_DIR, "nodes")).glob(
        "**/*.tree.json"
    ):
        with node_def_file.open() as f:
            data = json.load(f)
            global_node_defs[data["name"]] = data


def load_trees(project_name):
    for tree_file in pathlib.Path(os.path.join(BEHAVIOR_DIR, project_name)).glob(
        "**/*.tree.json"
    ):
        with tree_file.open() as f:
            data = json.load(f)
            global_trees[data["title"]] = data
            global_trees[data["id"]] = data


def create_node(row):
    node = {}
    node["id"] = row["ID"]
    if row["Category"] in ["Action", "Composite", "Decorator"]:
        node["name"] = row["Name"]
    elif row["Category"] in ["Tree"]:
        node["name"] = global_trees[row["Name"]]["id"]
    else:
        raise ValueError("Unknown category %s", row["Category"])
    node["title"] = row["Title"]
    node["description"] = ""
    node["properties"] = {}
    for line in row["Properties"].splitlines():
        key, value = line.split(":", 1)
        node["properties"][key] = cast_to_number(value.strip())
    node["display"] = {
        "x": int(row["X"]),
        "y": int(row["Y"]),
    }
    if row["Category"] == "Decorator":
        node["child"] = row["Children"]
    elif row["Category"] == "Composite":
        node["children"] = [i.strip() for i in row["Children"].split(",")]

    return node


class SpreadsheetConverter(object):
    def __init__(self, meta, record, filename, output_dir):
        self.meta = meta
        self.record = record
        self.lang = LANGUAGE_BCP47_CODES[record.fields.Language]
        self.filename = filename
        self.root_output_dir = output_dir
        self.soultalk_topics_dir = os.path.join(output_dir, f"soultalk/topics")
        self.soultalk_intent_dir = os.path.join(output_dir, f"soultalk/intents")
        self.timelines_dir = os.path.join(output_dir, "timelines", meta["name"])
        self.behavior_tree_dir = os.path.join(output_dir, "behavior_trees")
        self.tts_dir = os.path.join(output_dir, "tts")
        self.dir_params = {
            "soultalk_topics_dir": self.soultalk_topics_dir,
            "soultalk_intent_dir": self.soultalk_intent_dir,
            "timelines_dir": self.timelines_dir,
            "behavior_tree_dir": self.behavior_tree_dir,
            "tts_dir": self.tts_dir,
        }
        self.spreadsheet = pd.read_excel(
            filename, sheet_name=None, engine="openpyxl", dtype="str"
        )
        self.topic_name = os.path.splitext(os.path.basename(filename))[0]

    def generate_from_template(self):
        template_result = []
        for sheet_name, sheet_data in self.spreadsheet.items():
            logger.info("Processing sheet %r", sheet_name)
            sheet_name = sheet_name.strip()
            name = slugify(f"{self.record.fields.Name} {sheet_name}", lowercase=False)

            with tempfile.NamedTemporaryFile(suffix=".csv") as fp:
                csvfile = fp.name
                sheet_data.to_csv(csvfile, index=None)
                try:
                    header, skiprows = sniff(csvfile)
                except Exception as ex:
                    logger.error('Can\'t process %r. Error "%s"', sheet_name, ex)
                    continue
                if "Generator" in header:
                    generator_name = header["Generator"].lower()
                    if generator_name in registered_generators:
                        # enlist sheet record
                        id = f"{self.record.id}_{slugify(sheet_name, separator='_')}"
                        sheet = Sheet(
                            id=id, name=sheet_name, header=header, skiprows=skiprows
                        )
                        event_record = self.meta["event_record"]
                        if event_record:
                            sheet.event_status = event_record.fields.Status
                        if event_record and event_record.fields.Start:
                            setattr(sheet, "start", event_record.fields.Start)
                        if event_record and event_record.fields.End:
                            setattr(sheet, "end", event_record.fields.End)
                        self.record.fields.Sheets[sheet_name] = sheet
                        for k, v in header.items():
                            if k in ["Start", "End"]:
                                setattr(sheet, k.lower(), v)
                                continue
                            elif k == "Variables":
                                sheet.variables = v
                            else:
                                sheet.parameters[k] = v
                        if "Language" in header:
                            lang = LANGUAGE_BCP47_CODES[header["Language"]]
                        else:
                            lang = self.lang
                        cls = registered_generators[generator_name]
                        generator = cls(
                            self.record, sheet_name, lang, self.meta, csvfile
                        )
                        sheet.scene = generator.scene_name
                        result = generator.generate(self.dir_params)
                        sheet.arf_events = generator.arf_events[:]
                        if result:
                            template_result.append(result)
                    else:
                        logger.error("Generator %r is not found", header["Generator"])
                        continue
                else:
                    logger.warning(
                        'No header found in the sheet "%s/%s"', name, sheet_name
                    )
        return template_result

    def to_chat_rules(self, event_stage: Union[str, None] = None):
        """Parses rules and writes to yaml file"""
        topic_file = os.path.join(self.soultalk_topics_dir, "%s.yaml" % self.topic_name)

        if "parameters" in self.spreadsheet:
            parameters = self.spreadsheet["parameters"]
            parameters = parameters.dropna(how="all")  # drop complete empty rows
            parameters = parameters.fillna("")
            parameters = parameters.apply(
                lambda x: x.str.strip() if x.dtype == "object" else x
            )
            context = dict(zip(parameters.Parameter, parameters.Value))
        else:
            context = {}

        if "rules" in self.spreadsheet:
            df = self.spreadsheet["rules"]
            df = clean_up_df(df)
            if "EventStage" in df.columns:
                df = filter_event_stage(df, event_stage)
        else:
            return

        # validate the mandatory columns
        for column in ["Trigger", "Robot"]:
            if column not in df.columns:
                raise ValueError('Column "%s" was missing' % column)

        rules = []
        for rule_count, row in df.iterrows():
            rule = OrderedDict()
            rule["name"] = "rule%s" % rule_count
            if "Probability" in row and row["Probability"]:
                try:
                    rule["probability"] = float(row["Probability"])
                except ValueError as ex:
                    logger.error(ex)

            if row.Trigger:
                text = row.Trigger
                if "[<" in text or ">]" in text:
                    if not context:
                        raise ValueError("No parameters")
                    text = renderer.render(text, context)
                patterns = text.splitlines()
                intents = [p[1:] for p in patterns if p.startswith("~")]
                inputs = [p for p in patterns if not p.startswith("~")]
                if inputs:
                    rule["input"] = reduce([i.strip() for i in inputs if i.strip()])
                if intents:
                    rule["intent"] = reduce([i.strip() for i in intents if i.strip()])

            if "InputContext" in row and row.InputContext:
                rule["input_context"] = reduce(row.InputContext.splitlines())

            if row.Robot:
                text = row.Robot
                if "[<" in text or ">]" in text:
                    if not context:
                        raise ValueError("No parameters")
                    text = renderer.render(text, context)
                if text.startswith(">"):
                    text = text[1:].strip()
                    rule["output"] = text
                else:
                    rule["output"] = reduce(
                        [t.strip() for t in text.splitlines() if t.strip()]
                    )

            if "ASRContext" in row and row.ASRContext:
                rule["action"] = {
                    "name": "set_asr_context",
                    "properties": {"context": row.ASRContext.split(",")},
                }

            if "OutputContext" in row and row.OutputContext:
                rule["output_context"] = reduce(row.OutputContext.splitlines())

            if hasattr(row, "TTL") and row.TTL != "":
                rule["ttl"] = int(row.TTL)

            rules.append(rule)

        if rules:
            mkdir(self.soultalk_topics_dir)
            package_file = os.path.join(self.soultalk_topics_dir, "../package.yaml")
            if not os.path.isfile(package_file):
                dump_yaml(create_soultalk_package_meta(), package_file)
            mkdir(self.soultalk_topics_dir)
            data = OrderedDict()
            data["name"] = self.topic_name
            data["settings"] = {"ttl": -1, "priority": "higher"}
            data["rules"] = rules
            dump_yaml(data, topic_file)
            logger.info("Write rule file to %s", topic_file)
            return topic_file

    def to_intents(self, event_stage: Union[str, None] = None):
        """Parses intent data"""
        intent_file = os.path.join(
            self.soultalk_intent_dir, "%s.yaml" % self.topic_name
        )

        if "intents" in self.spreadsheet:
            df = self.spreadsheet["intents"]
        else:
            return
        df = clean_up_df(df)

        # validate the mandatory columns
        for column in ["Intent", "Samples"]:
            if column not in df.columns:
                raise ValueError('Column "%s" was missing' % column)

        intents = []
        for rule_count, row in df.iterrows():
            intent = OrderedDict()
            intent["intent"] = row.Intent
            samples = row.Samples.splitlines()
            samples = [remove_puncturaion_marks(sample) for sample in samples]
            if len(samples) == 1:
                samples = samples * 2  # replicate the sample for nlu
                logger.warning("Insufficient samples for intent %r", row.Intent)
            intent["samples"] = samples
            if len(samples) > 0:
                intents.append(intent)

        if intents:
            mkdir(self.soultalk_intent_dir)
            data = OrderedDict()
            data["intents"] = intents
            dump_yaml(data, intent_file)
            logger.info("Write intent file to %s", intent_file)
            return intent_file

    def to_timelines(self, event_stage: Union[str, None] = None):
        if "timelines" in self.spreadsheet:
            timeline_output_dir = os.path.join(self.timelines_dir, self.topic_name)
            mkdir(timeline_output_dir)
            df = self.spreadsheet["timelines"]
            df = clean_up_df(df)
            fnames = []
            for i, row in df.iterrows():
                text = row["Text"]
                if "OutputContext" in row:
                    for context in row["OutputContext"].splitlines():
                        text += " |c, %s|" % context
                fname = os.path.join(timeline_output_dir, "%s.yaml" % (i + 1))
                node = OrderedDict()
                node["lang"] = LANG_CODE_MAPPING.get(self.lang, "en-US")
                node["name"] = "speech"
                node["text"] = text
                node["start_time"] = 0
                node["volume"] = 1
                node["pitch"] = 1
                node["duration"] = 1
                node["speed"] = 1
                dump_yaml({"nodes": [node]}, fname)
                fnames.append(fname)
            return fnames

    def to_sheet_meta(self):
        sheet_data = []
        for sheet_name in self.spreadsheet.keys():
            if sheet_name in self.record.fields.Sheets:
                sheet = self.record.fields.Sheets[sheet_name]
                if sheet.header["Generator"] in [
                    "BasicARFGenerator",
                    "SimpleARFGenerator",
                ]:
                    sheet_data.append(sheet.dict())
        if sheet_data:
            mkdir(self.root_output_dir)
            meta_file = os.path.join(self.root_output_dir, "sheets.yaml")
            dump_yaml(sheet_data, meta_file)

    def to_prompt_templates(self, generator_results):
        # prompt templates
        prompt_templates = {}
        for generator_result in generator_results:
            for template_date in generator_result.get("prompt_templates", []):
                prompt_templates.update(
                    {template_date["name"]: template_date["prompt"]}
                )
        if prompt_templates:
            mkdir(self.root_output_dir)
            ofile = os.path.join(self.root_output_dir, "prompt_templates.yaml")
            if os.path.exists(ofile):
                pass
                # merge
            dump_yaml(prompt_templates, ofile)

        # prompt presets
        prompt_presets = {}
        for generator_result in generator_results:
            prompt_presets.update(generator_result.get("prompt_presets", {}))
        if prompt_presets:
            mkdir(self.root_output_dir)
            ofile = os.path.join(self.root_output_dir, "prompt_presets.yaml")
            dump_yaml({"presets": prompt_presets}, ofile)

    def to_behavior_tree(self):
        load_trees("hrif")
        load_node_defs()

        if "behavior" in self.spreadsheet:
            df = self.spreadsheet["behavior"]
            df = clean_up_df(df)
            df.Index = df.Index.str.replace(
                r"\.0$", ""
            )  # remove the tailing .0 in the string
            df.Name = df.Name.str.replace(
                r"\.0$", ""
            )  # remove the tailing .0 in the string
            df.Children = df.Children.str.replace(
                r"\.0$", ""
            )  # remove the tailing .0 in the string
            df.X = df.X.str.replace(r"\.0$", "")  # remove the tailing .0 in the string
            df.Y = df.Y.str.replace(r"\.0$", "")  # remove the tailing .0 in the string
        else:
            return

        nodes = {}

        # 1. build the mapping
        id_mapping = {}
        for _, row in df.iterrows():
            if row.get("ID"):
                id_mapping[row["Index"]] = row["ID"]
            else:
                id_mapping[row["Index"]] = str(
                    uuid.uuid3(uuid.NAMESPACE_DNS, self.topic_name + str(row))
                )
                logger.info(
                    "Created new ID %s => %s", row["Index"], id_mapping[row["Index"]]
                )

        # 2. map index back to ids
        for i, row in df.iterrows():
            if row["Category"] == "Root":
                df.loc[i, "ID"] = id_mapping[row["Index"]]
                df.loc[i, "Name"] = id_mapping[row["Name"]]
            else:
                df.loc[i, "ID"] = id_mapping[row["Index"]]
                if row["Children"]:
                    children = [
                        id_mapping[id.strip()] for id in row["Children"].split(",")
                    ]
                    df.loc[i, "Children"] = ", ".join(children)
        btree = {}
        btree["version"] = "0.3.0"
        btree["scope"] = "tree"
        btree_display = {
            "camera_x": 300,
            "camera_y": 200,
            "camera_z": 1,
            "x": 0,
            "y": 0,
        }
        for _, row in df.iterrows():
            if row["Category"] == "Root":
                btree["id"] = row["ID"]
                btree["title"] = slugify(self.topic_name, lowercase=False)
                btree["description"] = ""
                btree["root"] = row["Name"]
                btree["properties"] = {}
                btree_display["x"] = int(row["X"])
                btree_display["y"] = int(row["Y"])
                for line in row["Properties"].splitlines():
                    key, value = line.split(":", 1)
                    btree["properties"][key] = cast_to_number(value.strip())
            else:
                node = create_node(row)
                nodes[node["id"]] = node

        btree["nodes"] = nodes
        btree["display"] = btree_display
        ofile = os.path.join(
            self.behavior_tree_dir,
            "%s.tree.json" % slugify(self.topic_name, lowercase=False),
        )
        mkdir(self.behavior_tree_dir)
        with open(ofile, "w") as f:
            json.dump(btree, f, indent=2)
        logger.info("Saved behavior tree to %s", ofile)
        return ofile

    def to_linear_behavior_tree(self):
        if "linear behavior" in self.spreadsheet:
            df = self.spreadsheet["linear behavior"]
            df = clean_up_df(df)
        else:
            return
        self._to_linear_behavior_tree(df)

    def _to_linear_behavior_tree(self, df):
        load_node_defs()

        nodes = {}

        root_node_data = {
            "id": str(uuid.uuid3(uuid.NAMESPACE_DNS, self.topic_name + "root")),
            "name": "Sequence",
            "title": "Sequence",
            "display": {"x": 204, "y": 0},
            "children": [],
        }
        root_node = Node(**root_node_data)
        nodes[root_node.id] = root_node

        n_children = df.shape[0]
        height = 84 * (n_children - 1)
        for i, row in df.iterrows():
            props = {}
            for line in row.Properties.splitlines():
                key, value = line.split(":", 1)
                props[key] = cast_to_number(value.strip())
            node_data = {
                "id": str(uuid.uuid3(uuid.NAMESPACE_DNS, self.topic_name + str(row))),
                "name": row.Action,
                "title": ACTION_TITLES.get(row.Action),
                "properties": props,
                "display": {"x": 408, "y": int(84 * i - height / 2)},
            }
            node = Node(**node_data)
            nodes[node.id] = node
            root_node.children.append(node.id)

        btree_data = {}
        btree_data["version"] = "0.3.0"
        btree_data["scope"] = "tree"
        btree_data["id"] = str(uuid.uuid3(uuid.NAMESPACE_DNS, self.topic_name + "tree"))
        btree_data["title"] = slugify(self.topic_name, lowercase=False)
        btree_data["root"] = root_node.id
        btree_data["nodes"] = nodes
        btree_data["display"] = {
            "camera_x": 484,
            "camera_y": 484,
            "camera_z": 1,
            "x": 0,
            "y": 0,
        }
        btree = Tree(**btree_data)

        ofile = os.path.join(
            self.behavior_tree_dir,
            "%s.tree.json" % slugify(self.topic_name, lowercase=False),
        )
        mkdir(self.behavior_tree_dir)
        btree.to_json(ofile)
        logger.info("Saved behavior tree to %s", ofile)
        return ofile

    @staticmethod
    def from_behavior_tree(filename, output_dir="."):
        """Converts behavior tree to spreadsheet"""
        load_trees("hrif")
        load_node_defs()
        with open(filename) as f:
            data = json.load(f)
        rows = []

        def format_properties(node):
            if node["properties"]:
                return "\n".join(
                    ["%s: %s" % (k, v) for k, v in node["properties"].items()]
                )
            else:
                return ""

        # add root node
        root = {}
        root["ID"] = data["id"]
        root["Name"] = data["root"]
        root["Category"] = "Root"
        root["Properties"] = format_properties(data)
        root["Children"] = ""
        root["X"] = data["display"]["x"]
        root["Y"] = data["display"]["y"]
        rows.append(root)

        # DFS sort on the nodes
        nodes = data["nodes"]  # dict: node id -> node
        sorted_nodes = []
        rootid = data["root"]

        def pick_node(nodeid, sorted_nodes):
            node = nodes.pop(nodeid)
            sorted_nodes.append(node)
            if node.get("children"):
                for child in node.get("children"):
                    pick_node(child, sorted_nodes)
            if node.get("child"):
                pick_node(node.get("child"), sorted_nodes)

        if rootid:
            pick_node(rootid, sorted_nodes)
        else:
            logger.warning("No root node")
            sorted_nodes = nodes.values()

        for node in sorted_nodes:
            row = {}
            row["ID"] = node["id"]
            row["Title"] = node["title"]
            node_def = global_node_defs.get(node["name"])
            if node_def:
                row["Name"] = node["name"]
                row["Category"] = node_def["category"].title()
            else:
                row["Category"] = "Tree"
                if node["name"] in global_trees:
                    row["Name"] = global_trees[node["name"]]["title"]
                else:
                    row["Name"] = node["name"]
            row["Properties"] = format_properties(node)
            if node.get("children"):
                row["Children"] = ", ".join(node.get("children"))
            elif node.get("child"):
                row["Children"] = node.get("child")
            else:
                row["Children"] = ""
            row["X"] = node["display"]["x"]
            row["Y"] = node["display"]["y"]
            rows.append(row)

        # map ids to index
        id_mapping = {row["ID"]: str(i) for i, row in enumerate(rows)}
        for row in rows:
            if row["Category"] == "Root":
                row["Index"] = id_mapping[row["ID"]]
                row["Name"] = id_mapping[row["Name"]]
            else:
                row["Index"] = id_mapping[row["ID"]]
                if row["Children"]:
                    children = [
                        id_mapping[id.strip()] for id in row["Children"].split(",")
                    ]
                    row["Children"] = ", ".join(children)

        df = pd.DataFrame(
            rows,
            columns=[
                "Index",
                "Category",
                "Name",
                "Title",
                "Properties",
                "Children",
                "X",
                "Y",
            ],
        )

        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        ofile = os.path.join(
            output_dir, "%s.xlsx" % slugify(data["title"], lowercase=False)
        )
        df.to_excel(
            ofile,
            sheet_name=slugify(data["title"], lowercase=False),
            index=False,
        )
        logger.info("Saved to %s", ofile)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--lang",
        default="en",
        dest="lang",
        choices=LANG_CODE_MAPPING.keys(),
        help="the language code",
    )
    parser.add_argument(
        "-o", default="output", dest="output", help="the output directory"
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--file", help="the spreadsheet file to convert")
    group.add_argument(
        "--bt",
        help=(
            "the behavior tree file. If provided it converts "
            "the behavior tree to spreadsheet"
        ),
    )
    args = parser.parse_args()

    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    if args.bt:
        SpreadsheetConverter.from_behavior_tree(args.bt, args.output)
    else:
        converter = SpreadsheetConverter("robot", args.lang, args.file, args.output)
        converter.to_chat_rules()
        converter.to_intents()
        converter.to_timelines()
        converter.to_behavior_tree()
        converter.to_linear_behavior_tree(args.output)
