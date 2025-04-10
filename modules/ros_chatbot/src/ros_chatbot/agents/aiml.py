##
## Copyright (C) 2017-2025 Hanson Robotics
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

import logging
import os
import random
import re
import uuid
from collections import defaultdict
from pprint import pformat

import yaml

from ros_chatbot.pyaiml import Kernel
from ros_chatbot.utils import abs_path, shorten

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.aiml")


class AIMLAgent(SessionizedAgent):
    type = "AIMLAgent"

    def __init__(self, id, lang, character_yaml):
        super(AIMLAgent, self).__init__(id, lang)
        self.aiml_files = []
        self.kernel = Kernel()
        self.kernel.verbose(False)
        self.current_topic = ""
        self.trace_pattern = re.compile(
            r".*/(?P<fname>.*), (?P<tloc>\(.*\)), (?P<pname>.*), (?P<ploc>\(.*\))"
        )
        self.properties = {}
        self.load(character_yaml)
        self.base = os.path.dirname(os.path.expanduser(character_yaml))
        if not self.base.endswith("/"):
            self.base = self.base + "/"

    def load(self, character_yaml):
        logger.info("Loading character")
        with open(character_yaml) as f:
            config = yaml.safe_load(f)
            try:
                errors = []
                root_dir = os.path.dirname(os.path.realpath(character_yaml))
                if "property_file" in config:
                    self.set_property_file(abs_path(root_dir, config["property_file"]))
                if "aiml" in config:
                    aiml_files = [abs_path(root_dir, f) for f in config["aiml"]]
                    errors = self.load_aiml_files(self.kernel, aiml_files)
                self.print_duplicated_patterns()
                if errors:
                    raise Exception(
                        "Loading {} error {}".format(character_yaml, "\n".join(errors))
                    )
            except KeyError as ex:
                logger.exception(ex)

    def get_properties(self):
        return self.properties

    def replace_aiml_abs_path(self, trace):
        if isinstance(trace, list):
            trace = [f.replace(self.base, "") for f in trace]
        return trace

    def load_aiml_files(self, kernel, aiml_files):
        errors = []
        for f in aiml_files:
            if "*" not in f and not os.path.isfile(f):
                logger.warning("%s is not found", f)
            errors.extend(kernel.learn(f))
            logger.debug("Load %s", f)
            if f not in self.aiml_files:
                self.aiml_files.append(f)
        return errors

    def set_property_file(self, propname):
        try:
            with open(propname) as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split("=")
                    key = parts[0].strip()
                    value = parts[1].strip()
                    self.kernel.setBotPredicate(key, value)
                    self.properties[key] = value
                logger.info("Set properties file %s", propname)
        except Exception as ex:
            logger.error("Couldn't open property file %r: %s", propname, ex)

    def new_session(self):
        sid = str(uuid.uuid4())
        return sid

    def set_properties(self, props):
        for key, value in self.properties.items():
            self.kernel.setBotPredicate(key, value)

    def reset_topic(self, sid):
        self.current_topic = ""
        self.kernel.setPredicate("topic", "", sid)
        logger.info("Topic is reset")

    def chat(self, agent_sid, request):
        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question

        answer = self.kernel.respond(request.question, agent_sid, query=False)
        if "response_limit" in self.config:
            answer, _ = shorten(answer, self.config["response_limit"])

        response.answer = answer

        response.attachment["emotion"] = self.kernel.getPredicate("emotion", agent_sid)
        response.attachment["performance"] = self.kernel.getPredicate(
            "performance", agent_sid
        )
        response.attachment["topic"] = self.kernel.getPredicate("topic", agent_sid)

        traces = self.kernel.getTraceDocs()
        if traces:
            logger.debug("Trace: %s", traces)
            patterns = []
            for trace in traces:
                match_obj = self.trace_pattern.match(trace)
                if match_obj:
                    patterns.append(match_obj.group("pname"))
            response.attachment["pattern"] = patterns
            if patterns:
                first = patterns[0]
                if "*" in first or "_" in first:
                    pattern_len = len(first.strip().split())
                    if "*" not in first:
                        response.attachment["ok_match"] = True
                    if pattern_len > 3 and pattern_len > 0.9 * len(
                        request.question.strip().split()
                    ):
                        response.attachment["ok_match"] = True
                else:
                    response.attachment["exact_match"] = True
            traces = self.replace_aiml_abs_path(traces)
            response.trace = "\n".join(traces)
            self.score(response)

        response.end()
        return response

    def reset(self, sid):
        self.kernel._deleteSession(sid)
        return sid

    def get_context(self, sid):
        context = self.kernel.getSessionData(sid) or {}

        # remove internal context (stats with _)
        for k in list(context.keys()):
            if k.startswith("_"):
                del context[k]

        return context

    def set_context(self, sid, context: dict):
        for k, v in context.items():
            if v and isinstance(v, str):
                if k.startswith("_"):
                    continue
                self.kernel.setPredicate(k, v, sid)
                logger.info("Set predicate %s=%s", k, v)
                if k in ["firstname", "fullname"]:
                    self.kernel.setPredicate("name", v, sid)

    def remove_context(self, sid, key):
        if key in list(self.get_context(sid).keys()):
            del self.kernel._sessions[sid][key]
            logger.info("Removed context %s", key)
            return True
        else:
            logger.debug("No such context %s", key)
            return False

    def get_templates(self):
        templates = []
        root = self.kernel._brain._root
        self.kernel._brain.get_templates(root, templates)
        return templates

    def print_duplicated_patterns(self):
        patterns = defaultdict(list)
        for t in self.get_templates():
            key = (t[1]["pattern"].lower(), t[1]["that"].lower(), t[1]["topic"].lower())
            patterns[key].append(t[1])
        for pattern in patterns:
            if len(patterns[pattern]) > 1:
                logger.error(
                    "Duplicated patterns %s\n%s\n",
                    len(patterns[pattern]),
                    pformat(patterns[pattern]),
                )

    def said(self, session, text):
        sid = session.sid
        outputHistory = self.kernel.getPredicate(self.kernel._outputHistory, sid)
        if isinstance(outputHistory, list):
            outputHistory.append(text)
            logger.info("Add '%s' to output history", text)

    def score(self, response):
        response.attachment["score"] = 50
        if response.attachment.get("ok_match"):
            response.attachment["score"] += 10
        elif response.attachment.get("exact_match"):
            response.attachment["score"] += 20
        input_len = len(response.question)
        if input_len > 100:
            response.attachment["score"] -= 20  # penalty on long input

        # suppress long answer
        if len(response.answer.split()) > 80:
            response.attachment["score"] -= 20

        response.attachment["score"] += random.randint(
            -10, 10
        )  # give it some randomness
        if (
            "minimum_score" in self.config
            and response.attachment["score"] < self.config["minimum_score"]
        ):
            logger.info(
                "Score didn't pass lower threshold: %s", response.attachment["score"]
            )
            response.attachment["score"] = -1
        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.answer
        ):
            response.attachment["score"] = -1
            logger.warning("Question response is not allowed")

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
