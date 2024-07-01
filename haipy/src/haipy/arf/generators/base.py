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
import random
import re
import uuid
from abc import ABCMeta, abstractmethod
from collections import OrderedDict

import pandas as pd
import yaml
from slugify import slugify

from haipy.schemas.airtable_schemas import EventStatus, ScriptRecord, ScriptStatus
from haipy.schemas.btree import Node, Tree
from haipy.text_processing.template_renderer import Renderer
from haipy.utils import (
    LANGUAGE_BCP47_CODES,
    LANGUAGE_ISO639_CODES,
    create_soultalk_package_meta,
    dump_yaml,
    mkdir,
    reduce,
    to_bool,
    to_list,
)

logger = logging.getLogger(__name__)


SOULTALK_HEADER_TEMPLATE = """
name: main
language: en
type: ''
settings:
  priority: normal
omni_rules: []
rules: []
"""

MACRO_PATTERN = re.compile(r"""\w+\s*\(.*\)""", flags=re.UNICODE)  # func(args)


def is_initiate_event_trigger(trigger: str) -> bool:
    return trigger.startswith("event.init.")


def is_arf_event_trigger(trigger: str) -> bool:
    return trigger.startswith("event.arf.")


def is_initiate_gambit_trigger(trigger: str) -> bool:
    return trigger == "event.init.gambit"


class ARFGenerator(metaclass=ABCMeta):
    def __init__(self, record: ScriptRecord, sheet_name, lang, meta, infile):
        """
        record: aritable script record
        sheet_name: the sheet name in a spreadsheet
        """
        self.record = record
        self.sheet = record.fields.Sheets[sheet_name]
        self.header = self.sheet.header
        self.meta = meta
        self.script_name = record.fields.Name
        self.sheet_name = sheet_name
        self.name = slugify(
            f"{self.script_name}:{sheet_name}", lowercase=False, separator="-"
        )
        self.lang = lang
        self.tag = []
        event = (
            self.meta["event_record"].fields.Event
            if "event_record" in self.meta and self.meta["event_record"]
            else None
        )
        self.namespace = slugify(f"{self.script_name}", lowercase=False, separator="-")
        self.scene_name = slugify(
            f"{self.namespace}:{sheet_name}", lowercase=False, separator="-"
        )
        self.arf_events = []

        try:
            self.df = pd.read_csv(
                infile, skiprows=self.sheet.skiprows, skip_blank_lines=True
            ).dropna(how="all")
        except pd.errors.EmptyDataError:
            self.df = pd.DataFrame()
        self.df.columns = self.df.columns.str.strip()

        def _rename(col):
            columns = {"answer": "Robot", "questions": "Question"}
            return columns.get(col.lower(), col)

        self.df.rename(
            columns=_rename,
            inplace=True,
        )

        # when the header is incorrectly read
        # this happens when the header is empty
        # use the first row as the new header
        if not self.df.columns.empty and self.df.columns[0].startswith("Unnamed:"):
            new_header = self.df.iloc[0]
            self.df = self.df[1:]
            self.df.columns = new_header

        required_columns = [
            "Action",
            "ForceCannedResponse",
            "ContextTimer ",
            "GeneralPriming",
            "InputContext",
            "OutputContext",
            "ResponsePriming",
            "Robot",
            "SceneTimer",
            "SituationalPriming",
            "TTL",
            "Trigger",
        ]
        # TODO this list is not complete
        for col in required_columns:
            if col not in self.df.columns:
                self.df[col] = ""

        self.clean_data()
        # self.render_data()

    def render_data(self):
        renderer = Renderer()

        # render Robot and Priming
        if self.sheet.variables:
            context = {}
            context.update(self.sheet.variables)
            for i, row in self.df.iterrows():
                for col in [
                    "Robot",
                    "GeneralPriming",
                    "SituationalPriming",
                    "ResponsePriming",
                ]:
                    if col in row:
                        out = []
                        for text in row[col].splitlines():
                            text = text.strip()
                            if text and ("{%" in text or "{{" in text):
                                try:
                                    text = renderer.render(text, context)
                                    out.append(text)
                                except Exception as ex:
                                    logger.error(ex)
                            else:
                                out.append(text)
                        self.df.at[i, col] = "\n".join(out)

    def clean_data(self):
        self.df = (
            self.df.dropna(how="all")
            .fillna("")
            .astype("str")
            .apply(lambda x: x.str.strip() if x.dtype == "object" else x)
        )

    def init_soultalk_topic(self, ttl=None, type=None):
        topic = yaml.safe_load(SOULTALK_HEADER_TEMPLATE)
        topic["language"] = LANGUAGE_ISO639_CODES[self.lang]
        if ttl:
            topic["settings"]["ttl"] = ttl
        if type:
            topic["settings"]["type"] = type
        if "Priority" in self.header:
            topic["settings"]["priority"] = self.header["Priority"]
        if self.record.fields.Status == ScriptStatus.Done:
            # only the completed script has start time and end time constraints
            if self.sheet.event_status not in [EventStatus.Permanent, EventStatus.Test]:
                if self.sheet.start:
                    topic["settings"]["start"] = self.sheet.start
                if self.sheet.end:
                    topic["settings"]["end"] = self.sheet.end
        topic["name"] = self.name
        if "Tag" in self.header and self.header["Tag"]:
            self.tag = self.header["Tag"]
        omni_rules = []
        topic["omni_rules"] = omni_rules
        activation_rule = OrderedDict()
        deactivation_rule = OrderedDict()
        if "ActivationTrigger" in self.header:
            patterns = to_list(self.header["ActivationTrigger"])
            events = [p[6:] for p in patterns if p.startswith("event.")]
            inputs = [
                p
                for p in patterns
                if not p.startswith("~")
                and not MACRO_PATTERN.match(p)
                and not p.startswith("event.")
            ]
            if inputs:
                activation_rule["input"] = reduce(
                    [i.strip() for i in inputs if i.strip()]
                )
            if events:
                activation_rule["event"] = reduce(
                    [i.strip() for i in events if i.strip()]
                )
            if "ActivationAnswer" in self.header:
                answers = to_list(self.header["ActivationAnswer"])
                activation_rule["output"] = reduce(
                    [i.strip() for i in answers if i.strip()]
                )
            activation_rule["ttl"] = -1
            activation_rule["tag"] = ["activate", "priority"]
            activation_rule["action"] = {
                "name": "start-scene",
                "properties": {"scene": self.scene_name},
            }
            if "ActivationCondition" in self.header:
                activation_rule["condition"] = [
                    "{{%s}}" % cond.strip()
                    for cond in to_list(self.header["ActivationCondition"])
                    if cond.strip()
                ]
        if "DeactivationTrigger" in self.header:
            inputs = to_list(self.header["DeactivationTrigger"])
            deactivation_rule["input"] = reduce(
                [i.strip() for i in inputs if i.strip()]
            )
            if "DeactivationAnswer" in self.header:
                answers = to_list(self.header["DeactivationAnswer"])
                deactivation_rule["output"] = reduce(
                    [i.strip() for i in answers if i.strip()]
                )
            deactivation_rule["ttl"] = -1
            deactivation_rule["tag"] = ["deactivate", "priority"]
            deactivation_rule["action"] = {
                "name": "stop-scene",
                "properties": {"scene": self.scene_name},
            }
        if activation_rule:
            omni_rules.append(activation_rule)
        if deactivation_rule:
            omni_rules.append(deactivation_rule)
        if not omni_rules:
            del topic["omni_rules"]

        if "KnowledgeBase" in self.header:
            topic["knowledge_base"] = [
                doc for doc in to_list(self.header["KnowledgeBase"]) if doc.strip()
            ]

        return topic

    def write_soultalk_topic(self, topic, output_dir):
        outfile = os.path.join(output_dir, f"{self.name}.yaml")
        mkdir(output_dir)
        package_file = os.path.join(output_dir, "../package.yaml")
        if not os.path.isfile(package_file):
            dump_yaml(create_soultalk_package_meta(), package_file)
        dump_yaml(topic, outfile)
        logger.info("Saved soultalk topic file to %s", outfile)

    def construct_soultalk_rules(self):
        """Parses rows and construct soultalk rules"""
        rules = []
        for rule_count, row in self.df.iterrows():
            rule = OrderedDict()
            rule["name"] = f"{self.name}-rule{rule_count}"
            if "Probability" in row and row["Probability"]:
                try:
                    rule["probability"] = float(row["Probability"])
                except ValueError as ex:
                    logger.error(ex)

            if self.__class__.__name__ != "SimpleARFGenerator":
                if ("Trigger" not in row or not row.Trigger) and (
                    "Question" not in row or not row.Question
                ):
                    # ignore the rows that have no triggers and no question
                    continue

            rule["set"] = []

            if "Instruction" in row and row.Instruction:
                instructions = [
                    f"({instruction})"
                    for instruction in row.Instruction.splitlines()
                    if instruction.strip()
                ]
                rule["output"] = reduce(instructions)
                if "Trigger" not in row or not row.Trigger:
                    if "Group" in row and row.Group:
                        rule["event"] = f"arf.gambit.{row.Group}"
                    else:
                        rule["event"] = f"arf.gambit.a{rule_count}"
                    self.arf_events.append(f'event.{rule["event"]}')
                rule["output_context"] = uuid.uuid4().hex
                if "Reply Instruction" in row and row["Reply Instruction"]:
                    rule["set"].append(
                        {
                            "key": "reply_instruction",
                            "value": row["Reply Instruction"],
                            "ttl": 60,
                        }
                    )
                rule["set"].append({"key": "block_chat", "value": True})
                rule["action"] = {
                    "name": "set",
                    "properties": {
                        "block_chat": False,
                        "arf_count_down": (
                            60 * int(float(row.Turns))
                            if "Turns" in row and row.Turns
                            else 60
                        ),
                    },
                }

                # turns = 1
                # if "Turns" in row and row.Turns:
                #    turns = int(float(row.Turns))

                # for turn in range(turns):
                #    rule2 = OrderedDict()
                #    if rules:
                #        rule2["input_context"] = rules[-1]["output_context"]
                #    else:
                #        rule2["input_context"] = rule["output_context"]
                #    rule2["input"] = "*"
                #    rule2["output"] = "*"
                #    rule2["output_context"] = uuid.uuid4().hex
                #    if (
                #        "DefaultTemplate" in self.header
                #        and self.header["DefaultTemplate"]
                #    ):
                #        rule2["prompt_template"] = self.header["DefaultTemplate"]
                #    if turn + 1 == turns:
                #        rule2["action"] = {"name": "NEXTTOPIC"}
                #    rules.append(rule2)

            if "Attribute" in row and row.Attribute:
                rule["attribute"] = reduce(
                    [
                        item.strip()
                        for item in row.Attribute.splitlines()
                        if item.strip()
                    ]
                )
            initiate_event_flag = False
            if "Trigger" in row and row.Trigger:
                patterns = row.Trigger.splitlines()
                initiate_event_flag = any(
                    [is_initiate_event_trigger(p) for p in patterns]
                )
                patterns = [p.strip() for p in patterns if p.strip()]
                intents = [p[1:] for p in patterns if p.startswith("~")]
                events = [p[6:] for p in patterns if p.startswith("event.")]
                negative_inputs = [p[1:] for p in patterns if p.startswith("!")]
                inputs = [
                    p
                    for p in patterns
                    if not p.startswith("~")
                    and not p.startswith("!")
                    and not MACRO_PATTERN.match(p)
                    and not p.startswith("event.")
                ]
                self.arf_events.extend([p for p in patterns if is_arf_event_trigger(p)])
                macros = [
                    p
                    for p in patterns
                    if not p.startswith("~") and MACRO_PATTERN.match(p)
                ]
                if inputs:
                    rule["input"] = reduce([i.strip() for i in inputs if i.strip()])
                if events:
                    rule["event"] = reduce([i.strip() for i in events if i.strip()])
                if intents:
                    rule["intent"] = reduce([i.strip() for i in intents if i.strip()])
                if macros:
                    rule["macro"] = reduce([i.strip() for i in macros if i.strip()])
                if negative_inputs:
                    rule["exclude"] = reduce(
                        [i.strip() for i in negative_inputs if i.strip()]
                    )

            if "Question" in row and row.Question:
                rule["samples"] = reduce(
                    [i.strip() for i in row.Question.splitlines() if i.strip()]
                )

            if "InputContext" in row and row.InputContext:
                rule["input_context"] = reduce(
                    [
                        f"{self.scene_name}-{context}"
                        for context in row.InputContext.splitlines()
                    ]
                )

            if "Robot" in row and row.Robot:
                text = row.Robot
                if "AudioPath" in row and row.AudioPath:
                    rule["action"] = {
                        "name": "play-audio-clip",
                        "properties": {
                            "audio-clip": row.AudioPath,
                            "text": text,
                            "lang": "en-US",
                        },
                    }
                else:
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
                rule["output_context"] = reduce(
                    [
                        f"{self.scene_name}-{context}"
                        for context in row.OutputContext.splitlines()
                    ]
                )

            priming = {}
            if "GeneralPriming" in row and row.GeneralPriming:
                priming["GeneralPriming"] = row.GeneralPriming
            if "SituationalPriming" in row and row.SituationalPriming:
                priming["SituationalPriming"] = row.SituationalPriming
            if "ResponsePriming" in row and row.ResponsePriming:
                priming["ResponsePriming"] = row.ResponsePriming
            if priming:
                priming["ForceCannedResponse"] = row.ForceCannedResponse and to_bool(
                    row.ForceCannedResponse
                )
                rule["action"] = {"name": "priming", "properties": priming}

            if "PromptTemplate" in row and row.PromptTemplate:
                rule["prompt_template"] = row.PromptTemplate

            if hasattr(row, "TTL") and row.TTL != "":
                rule["ttl"] = int(float(row.TTL))

            if hasattr(row, "Tag") and row.Tag:
                rule["tag"] = reduce(
                    [t.strip() for t in re.split("\t|,", row.Tag) if t.strip()]
                )
            elif self.tag:
                rule["tag"] = self.tag[:]

            if "FollowUp" in row and to_bool(row.FollowUp):
                if "tag" in rule:
                    rule["tag"].append("followup")
                else:
                    rule["tag"] = ["followup"]

            if "tag" in rule and "repeat" in rule["tag"] and "ttl" not in rule:
                rule["ttl"] = -1
            if initiate_event_flag:
                rule["ttl"] = -1

            if (
                "input_context" in rule
                and "output_context" in rule
                and rule["input_context"] == rule["output_context"]
            ):
                # prevent infinite loop
                rule["ttl"] = 1

            rules.append(rule)
        return rules

    @abstractmethod
    def generate(self, dir_params):
        """Generates ARF files"""
        pass


class BTreeGenerator(object):
    def __init__(self, lang, header, df, name):
        self.lang = lang
        self.header = header
        self.df = df
        self.name = name
        self.counter = 0
        self.nodes = {}
        self.root_node = None

    def new_id(self, scene):
        self.counter += 1
        return str(uuid.uuid3(uuid.NAMESPACE_DNS, f"{self.name}{scene}{self.counter}"))

    def create_sequence_node(self, scene):
        node_data = {
            "id": self.new_id(scene),
            "name": "Sequence",
            "title": "Sequence",
            "display": {"x": 204, "y": 0},
            "children": [],
        }
        return Node(**node_data)

    def create_repeat_node(self, scene, maxLoop):
        maxLoop = int(float(maxLoop))
        props = {"maxLoop": maxLoop}
        node_data = {
            "id": self.new_id(scene),
            "name": "Repeater",
            "title": "Repeat <maxLoop>x",
            "properties": props,
            "display": {"x": 204, "y": 0},
            "child": "",
        }
        return Node(**node_data)

    def create_repeat_until_failure_node(self, scene, maxLoop):
        maxLoop = int(float(maxLoop))
        props = {"maxLoop": maxLoop}
        node_data = {
            "id": self.new_id(scene),
            "name": "RepeatUntilFailure",
            "title": "Repeat Until Failure <maxLoop>x",
            "properties": props,
            "display": {"x": 204, "y": 0},
            "child": "",
        }
        return Node(**node_data)

    def create_say_node(self, scene, text, lang, audio_clip=""):
        lang = LANGUAGE_BCP47_CODES.get(lang, lang)
        props = {
            "text": text,
            "lang": lang,
            "wait_for_stop": True,
            "audio_clip": audio_clip,
        }
        node_data = {
            "id": self.new_id(scene),
            "name": "Say",
            "title": 'Say "<text>"',
            "properties": props,
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_compute_node(self, scene, expression, title='Evaluate "<expression>"'):
        props = {"expression": expression}
        node_data = {
            "id": self.new_id(scene),
            "name": "Evaluate",
            "title": title,
            "description": expression,
            "properties": props,
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_chat_node(self, scene, text, turns, speech_timeout):
        props = {
            "text": text,
            "turns": turns,
            "speech_timeout": speech_timeout,
        }
        node_data = {
            "id": self.new_id(scene),
            "name": "Chat",
            "title": 'Chat "<text>"',
            "properties": props,
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_auto_gpt_node(
        self, scene, objective, background, topic, turns, guest, character, lang
    ):
        lang = LANGUAGE_BCP47_CODES.get(lang, lang)
        props = {
            "objective": objective,
            "background": background,
            "topic": topic,
            "turns": turns,
            "guest": guest,
            "character": character,
            "lang": lang,
        }
        node_data = {
            "id": self.new_id(scene),
            "name": "AutoGPT",
            "title": 'AutoGPT "<objective>"',
            "properties": props,
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_change_state_node(self, scene, trigger):
        props = {
            "trigger": trigger,
        }
        node_data = {
            "id": self.new_id(scene),
            "name": "ChangeState",
            "title": 'Change State "<trigger>"',
            "properties": props,
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_timer_node(
        self,
        scene,
        timer="",
        message="",
        input_context="",
        output_context="",
        type="context",
        title=None,
    ):
        props = {
            "timer": timer,
            "message": message,
            "input_context": input_context,
            "output_context": output_context,
            "type": type,
        }
        title = title or 'Timer "<type>:<input_context>:<output_context>:<timer>"'
        node_data = {
            "id": self.new_id(scene),
            "name": "Timer",
            "title": title,
            "properties": props,
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_always_success_node(self, scene):
        node_data = {
            "id": self.new_id(scene),
            "name": "AlwaysSuccess",
            "title": "AlwaysSuccess",
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_decision_node(self, scene):
        node_data = {
            "id": self.new_id(scene),
            "name": "Decision",
            "title": "Decision",
            "children": [],
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_parallel_node(self, scene):
        node_data = {
            "id": self.new_id(scene),
            "name": "Parallel",
            "title": "Parallel",
            "children": [],
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_run_tree_node(self, scene, tree):
        props = {"tree": tree}
        node_data = {
            "id": self.new_id(scene),
            "name": "RunTree",
            "title": 'RunTree "<tree>"',
            "properties": props,
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_wait_for_context_node(
        self, scene, context, value="-", timeout="-", message="-", title=None
    ):
        props = {
            "context": context,
            "value": value,
            "timeout": timeout,
            "message": message,
        }
        title = title or 'WaitForContext "<context>"'
        node_data = {
            "id": self.new_id(scene),
            "name": "WaitForContext",
            "title": title,
            "properties": props,
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_priority_sequence_node(self, scene):
        node_data = {
            "id": self.new_id(scene),
            "name": "PrioritySequence",
            "title": "Priority Sequence",
            "children": [],
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_wait_node(self, scene, milliseconds):
        props = {"milliseconds": milliseconds}
        node_data = {
            "id": self.new_id(scene),
            "name": "Wait",
            "title": "Wait <milliseconds>ms",
            "properties": props,
            "display": {"x": 0, "y": 0},
        }
        return Node(**node_data)

    def create_set_node(self, scene, props):
        node_data = {
            "id": self.new_id(scene),
            "name": "Set",
            "title": "Set",
            "properties": props,
            "display": {"x": 204, "y": 0},
            "child": "",
        }
        return Node(**node_data)

    def node_layout(self, x, nodes, spread=1):
        tree_height = spread * 84 * (len(nodes) - 1)
        for n_child, node in enumerate(nodes):
            node.display.x = int(x)
            node.display.y = int(spread * 84 * n_child - tree_height / 2)

    def add_node(self, node):
        self.nodes[node.id] = node

    def generate_behavior(self, output_dir, namespace, sheet_name):
        scene = slugify(f"{namespace}:{sheet_name}", lowercase=False, separator="-")

        self.root_node = self.create_sequence_node(scene)
        self.add_node(self.root_node)

        compute_scene_node = self.create_compute_node(
            scene, '{{% set scene = "{}" %}}'.format(scene)
        )
        self.add_node(compute_scene_node)
        self.root_node.children.append(compute_scene_node.id)

        scene_start_time_node = self.create_compute_node(
            scene, "{% set scene_start_at = now %}"
        )
        self.add_node(scene_start_time_node)
        self.root_node.children.append(scene_start_time_node.id)

        self.parallel_root_node = self.create_parallel_node(scene)
        self.add_node(self.parallel_root_node)
        self.root_node.children.append(self.parallel_root_node.id)

        if "Beginning" in self.header and self.header["Beginning"]:
            beginning_node = self.create_say_node(
                scene, self.header["Beginning"], self.lang
            )
            self.add_node(beginning_node)
            self.root_node.children.append(beginning_node.id)

        # scene context blocker
        parallel_seq_node = self.create_sequence_node(scene)
        self.add_node(parallel_seq_node)
        self.parallel_root_node.children.append(parallel_seq_node.id)

        parallel_gambit_seq_node = self.create_sequence_node(scene)
        self.add_node(parallel_gambit_seq_node)
        self.parallel_root_node.children.append(parallel_gambit_seq_node.id)

        # context/scene timer sequence
        parallel_repeat_node = self.create_repeat_node(scene, -1)
        self.add_node(parallel_repeat_node)
        self.parallel_root_node.children.append(parallel_repeat_node.id)
        parallel_repeat_seq_node = self.create_sequence_node(scene)
        self.add_node(parallel_repeat_seq_node)
        parallel_repeat_node.child = parallel_repeat_seq_node.id

        # idle timer
        if "Idle" in self.header and self.header["Idle"]:
            self.header["IdleFrequency"] = self.header["Idle"]
        if "IdleFrequency" in self.header and self.header["IdleFrequency"]:
            timer = self.header["IdleFrequency"]
            parallel_repeat_node2 = self.create_repeat_node(scene, -1)
            self.add_node(parallel_repeat_node2)
            self.parallel_root_node.children.append(parallel_repeat_node2.id)
            parallel_repeat_idle_seq = self.create_sequence_node(scene)
            self.add_node(parallel_repeat_idle_seq)
            parallel_repeat_node2.child = parallel_repeat_idle_seq.id

            idle_timer_node = self.create_timer_node(
                scene, timer, message="event.idle", type="idle"
            )
            self.add_node(idle_timer_node)
            parallel_repeat_idle_seq.children.append(idle_timer_node.id)

            wait_node = self.create_wait_node(scene, 1000)
            self.add_node(wait_node)
            parallel_repeat_idle_seq.children.append(wait_node.id)

        default_speech_timeout = self.header.get("SpeechTimeout", 5.0)

        def format_output_context(raw_context):
            if isinstance(raw_context, list):
                context = ",".join(
                    [f"context.output.{scene}-{ctx}" for ctx in raw_context]
                )
            else:
                context = f"context.output.{scene}-{raw_context}"
            return context

        if "ExitPoint" in self.header and self.header["ExitPoint"]:
            if isinstance(self.header["ExitPoint"], list):
                title = 'Wait For "%s"' % ",".join(self.header["ExitPoint"])
            else:
                title = 'Wait For "%s"' % self.header["ExitPoint"]
            context = format_output_context(self.header["ExitPoint"])
            exit_context_node = self.create_wait_for_context_node(
                scene, context, title=title
            )
            self.add_node(exit_context_node)
            parallel_seq_node.children.append(exit_context_node.id)

        for i, row in self.df.iterrows():
            if is_initiate_event_trigger(row.Trigger):
                if "Turns" in row and row.Turns:
                    turns = int(float(row.Turns))
                elif is_initiate_gambit_trigger(row.Trigger):
                    turns = 1  # default 1 turn for gambit
                else:
                    turns = 0
                if "Speech Timeout" in row and row["Speech Timeout"]:
                    speech_timeout = float(row["Speech Timeout"])
                else:
                    speech_timeout = default_speech_timeout
                node = self.create_chat_node(scene, row.Trigger, turns, speech_timeout)
                self.add_node(node)
                parallel_gambit_seq_node.children.append(node.id)
            elif row.Trigger.startswith("state.trigger."):
                trigger = row.Trigger[len("state.trigger.") :]
                node = self.create_change_state_node(scene, trigger)
                self.add_node(node)
                parallel_gambit_seq_node.children.append(node.id)
                if row.Robot.strip():
                    node = self.create_say_node(scene, row.Robot.strip(), self.lang)
                    self.add_node(node)
                    parallel_gambit_seq_node.children.append(node.id)
            elif row.Trigger == "action.say":
                node = self.create_say_node(scene, row.Robot.strip(), row.Language)
                self.add_node(node)
                parallel_gambit_seq_node.children.append(node.id)

            if "InputContext" in row and row.InputContext:
                # only single context
                input_context = row.InputContext.splitlines()[0]
            else:
                input_context = "-"
            if "OutputContext" in row and row.OutputContext:
                # only single context
                output_context = row.OutputContext.splitlines()[0]
            else:
                output_context = "-"

            if row.Trigger == "event.gambit":
                wait_node = self.create_wait_node(
                    scene, self.header.get("GambitFrequency", 65) * 1000
                )
                self.add_node(wait_node)
                parallel_gambit_seq_node.children.append(wait_node.id)

                title = 'Gambit on context "%s"' % row.InputContext
                node = self.create_wait_for_context_node(
                    scene,
                    format_output_context(row.InputContext),
                    message="event.gambit",
                    title=title,
                )
                self.add_node(node)
                parallel_gambit_seq_node.children.append(node.id)

            # find the timeout triggers
            timeout_triggers = [
                trigger
                for trigger in row.Trigger.splitlines()
                if trigger.startswith("event.timeout.")
            ]
            if timeout_triggers:
                timeout_trigger = timeout_triggers[0]
            else:
                timeout_trigger = "event.timeout"

            timer_props = {
                "message": timeout_trigger,
                "input_context": f"{scene}-{input_context}",
                "output_context": f"{scene}-{output_context}",
            }

            if "ContextTimer" in row and row.ContextTimer and row.Trigger:
                timer = float(row.ContextTimer.strip())
                timer_props["timer"] = timer
                timer_props["type"] = "context"
                timer_props["title"] = (
                    f'Timer "context:{input_context}:{output_context}:{timer}"'
                )
                timer_node = self.create_timer_node(scene, **timer_props)
                self.add_node(timer_node)
                parallel_repeat_seq_node.children.append(timer_node.id)
            if "SceneTimer" in row and row.SceneTimer and row.Trigger:
                timer = float(row.SceneTimer.strip())
                timer_props["timer"] = timer
                timer_props["type"] = "scene"
                timer_props["title"] = (
                    f'Timer "scene:{input_context}:{output_context}:{timer}"'
                )
                timer_node = self.create_timer_node(scene, **timer_props)
                self.add_node(timer_node)
                parallel_repeat_seq_node.children.append(timer_node.id)

        if not parallel_seq_node.children:
            self.parallel_root_node.children.remove(parallel_seq_node.id)
        else:
            wait_node = self.create_wait_node(scene, 1000)
            self.add_node(wait_node)
            parallel_seq_node.children.append(wait_node.id)
        if not parallel_gambit_seq_node.children:
            self.parallel_root_node.children.remove(parallel_gambit_seq_node.id)

        # add a wait node to the parallel branches
        wait_node = self.create_wait_node(scene, 1000)
        self.add_node(wait_node)
        parallel_repeat_seq_node.children.append(wait_node.id)

        if "Ending" in self.header and self.header["Ending"]:
            ending_node = self.create_say_node(scene, self.header["Ending"], self.lang)
            self.add_node(ending_node)
            self.root_node.children.append(ending_node.id)

        remove_scene_start_time_node = self.create_compute_node(
            scene, '{% set scene_start_at = "" %}'
        )
        self.add_node(remove_scene_start_time_node)
        self.root_node.children.append(remove_scene_start_time_node.id)

        reset_scene_node = self.create_compute_node(scene, '{% set scene = "" %}')
        self.add_node(reset_scene_node)
        self.root_node.children.append(reset_scene_node.id)

        if "SetState" in self.header and self.header["SetState"]:
            state = self.header["SetState"]
            set_node = self.create_set_node(scene, {"state": state})
            self.add_node(set_node)
            self.root_node.children.append(set_node.id)
            compute_scene_node = self.create_compute_node(
                scene, '{{% set state = "{}" %}}'.format(state)
            )
            self.add_node(compute_scene_node)
            self.root_node.children.append(compute_scene_node.id)

        if "NextScene" in self.header and self.header["NextScene"]:
            scene_name = slugify(
                f'{namespace}:{self.header["NextScene"]}',
                lowercase=False,
                separator="-",
            )
            next_scene_node = self.create_run_tree_node(scene, scene_name)
            self.add_node(next_scene_node)
            self.root_node.children.append(next_scene_node.id)
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
        repeat = self.header.get("Repeat", 1)
        return {
            "tree": scene,
            "type": "behavior",
            "sheet": sheet_name,
            "repeat": repeat,
        }
