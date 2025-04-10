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
import random
import re
import socket
import uuid

from ros_chatbot.utils import shorten

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.chatscript")


class ChatScriptAgent(SessionizedAgent):
    type = "ChatScriptAgent"

    def __init__(self, id, lang, host="localhost", port=1024, timeout=2):
        super(ChatScriptAgent, self).__init__(id, lang)
        self.host = host
        if self.host not in ["localhost", "127.0.0.1"]:
            logger.warning("Server host: %r", self.host)
        self.port = port
        self.timeout = timeout
        self.cs_variable_pattern = re.compile(
            """.* variable: .*\$(?P<name>\S+) = (?P<value>.*)"""  # noqa
        )
        self.preferred_topics = []
        self.blocked_topics = []
        self.allow_gambit = False

    def say(self, username, question):
        to_say = "{username}\0{botname}\0{question}\0".format(
            username=username, botname="", question=question
        )
        response = ""
        connection = None
        try:
            connection = socket.create_connection(
                (self.host, self.port), timeout=self.timeout
            )
            connection.sendall(to_say.encode())  # chatscript only accepts bytes
            try:
                while True:
                    chunk = connection.recv(4096)
                    if chunk:
                        response += chunk.decode()  # decode bytes to str
                    else:
                        break
            except socket.timeout as e:
                logger.error("Timeout {}".format(e))
        except Exception as ex:
            logger.error("Connection error {}".format(ex))
        finally:
            if connection is not None:
                connection.close()

        if "No such bot" in response:
            logger.error(response)
            response = ""

        response = response.strip()
        return response

    def new_session(self):
        sid = str(uuid.uuid4())
        self.sid = sid
        return sid

    def reset(self, sid=None):
        self.say(sid or self.sid, ":reset")

    def is_template(self, text):
        if "{%" in text:
            return True
        return False

    def add_topics(self, topics):
        logger.info("Add topics %s", topics)
        for topic in topics:
            if not topic.startswith("~"):
                topic = "~" + topic
            self.say(self.sid, "add topic %s" % topic)

    def gambit_topic(self, topic):
        if not topic.startswith("~"):
            topic = "~" + topic
        return self.say(self.sid, "gambit topic %s" % topic)

    def chat(self, agent_sid, request):
        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question

        answer = self.say(agent_sid, request.question)
        if not answer:
            response.trace = "Not responsive"

        if self.is_template(answer):
            answer = ""

        answer = re.sub(r"""\[callback.*\]""", "", answer)

        trace = self.say(agent_sid, ":why")
        try:
            trace_tokens = trace.split(" ")
            topic = trace_tokens[0].split(".")[0]
            logger.info("%s topic %s", self.id, topic)
            response.attachment["topic"] = topic
        except Exception as ex:
            logger.exception(ex)

        if "response_limit" in self.config:
            answer, res = shorten(answer, self.config["response_limit"])
        response.answer = answer

        if answer:
            response.trace = str(trace)
            is_quibble = "xquibble_" in trace or "keywordlessquibbles" in trace
            is_gambit = (
                "gambit" in trace or "randomtopic" in trace
            ) and "howzit" not in trace
            is_repeating = "repeatinput" in trace
            if is_gambit:
                logger.info("Gambit response")
            if is_quibble:
                logger.info("Quibble response")
            if is_repeating:
                logger.info("Repeating response")
            response.attachment["quibble"] = is_quibble
            response.attachment["gambit"] = is_gambit
            response.attachment["repeat_input"] = is_repeating
            self.score(response)

        response.end()

        return response

    # TODO: only export user variables
    # def get_context(self, sid):
    #    response = self.say(sid, ":variables")
    #    context = {}
    #    for line in response.splitlines():
    #        matchobj = self.cs_variable_pattern.match(line)
    #        if matchobj:
    #            name, value = matchobj.groups()
    #            context[name] = value
    #    return context

    def set_context(self, sid, context: dict):
        for k, v in context.items():
            if v and isinstance(v, str):
                v = " ".join(v.split()).replace(
                    " ", "_"
                )  # variables in CS use _ instead of space
                self.say(sid, ":do ${}={}".format(k, v))
                logger.info("Set {}={}".format(k, v))

    def set_config(self, config, base):
        super(ChatScriptAgent, self).set_config(config, base)

        if "initial_topics" in config:
            logger.error("initial_topics config is not supported")
            # config should be session independent
            # self.add_topics(config["initial_topics"])

        if "preferred_topics" in config:
            self.preferred_topics = config["preferred_topics"]
        if "blocked_topics" in config:
            self.blocked_topics = config["blocked_topics"]
        if "allow_gambit" in config:
            self.allow_gambit = config["allow_gambit"]

    def score(self, response):
        response.attachment["score"] = 50
        if response.attachment.get("topic"):
            topic = response.attachment["topic"].strip("~")
            topic = topic.lower()
            if topic in self.preferred_topics:
                response.attachment["preferred"] = True
                if response.attachment.get("gambit") and not self.allow_gambit:
                    response.attachment["score"] = -1  # disable gambit
                elif response.attachment.get("quibble"):
                    response.attachment["score"] = 60
                else:
                    response.attachment["score"] = 70
            elif topic in self.blocked_topics:
                response.attachment["blocked"] = True
                response.attachment["score"] = -1
                response.attachment["blocked"] = True
            else:
                if response.attachment.get("gambit") and not self.allow_gambit:
                    response.attachment["score"] = -1  # disable gambit
                elif response.attachment.get("quibble"):
                    response.attachment["score"] = 30
            input_len = len(response.question)
            if input_len > 100:
                response.attachment["score"] -= 20

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
            logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
