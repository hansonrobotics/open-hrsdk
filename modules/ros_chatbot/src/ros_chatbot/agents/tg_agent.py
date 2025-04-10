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
import shutil
import threading
import uuid
from subprocess import PIPE, Popen

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.tg_agent")
cwd = os.path.dirname(os.path.abspath(__file__))


class WordSub(dict):
    """All-in-one multiple-string-substitution class."""

    def _wordToRegex(self, word):
        """Convert a word to a regex object which matches the word."""
        if word != "" and word[0].isalpha() and word[-1].isalpha():
            return "\\b%s\\b" % re.escape(word)
        else:
            return r"\b%s\b" % re.escape(word)

    def _update_regex(self):
        """Build re object based on the keys of the current
        dictionary.

        """
        self._regex = re.compile("|".join(map(self._wordToRegex, list(self.keys()))))
        self._regexIsDirty = False

    def __init__(self, defaults={}):
        """Initialize the object, and populate it with the entries in
        the defaults dictionary.

        """
        self._regex = None
        self._regexIsDirty = True
        for k, v in list(defaults.items()):
            self[k] = v

    def __call__(self, match):
        """Handler invoked for each regex match."""
        return self[match.group(0)]

    def __setitem__(self, i, y):
        self._regexIsDirty = True
        super(type(self), self).__setitem__(i, y)

    def sub(self, text):
        """Translate text, returns the modified text."""
        if self._regexIsDirty:
            self._update_regex()
        return self._regex.sub(self, text)


class TGAgent(SessionizedAgent):
    type = "TGAgent"

    def __init__(self, id, lang, api_id, api_hash, session, user_id, timeout=5):
        super(TGAgent, self).__init__(id, lang)
        self.api_id = str(api_id)
        self.api_hash = str(api_hash)
        self.session = "/tmp/tg_%s_%s.session" % (id, uuid.uuid1().hex)
        shutil.copy(session, self.session)  # the session file passed in is read only
        self.user_id = str(user_id)
        self.timeout = timeout
        self.lock = threading.RLock()

        self.person2_subbers = WordSub(
            {
                "I": "Sophia",
                "am": "is",
                "my": "Sophia's",
                "mine": "Sophia's",
                "myself": "Sophia herself",
                "I'm": "Sophia is",
                "I'd": "Sophia would",
                "I'll": "Sophia will",
                "I've": "Sophia has",
            }
        )

    def _get_question_length(self, question):
        if question:
            return len(question.split())
        else:
            return 0

    def _ask(self, question, timeout=None):
        timeout = timeout or self.timeout
        script = os.path.join(cwd, "../../../scripts/tg.py")
        if not os.path.isfile(script):
            script = "/opt/hansonrobotics/ros/lib/ros_chatbot/tg.py"
        cmd = [
            "python",
            script,
            "chat",
            "--api-id",
            self.api_id,
            "--api-hash",
            self.api_hash,
            "--session",
            self.session,
            "--id",
            self.user_id,
            "--question",
            question,
            "--timeout",
            str(timeout) if timeout else "-1",
        ]

        with self.lock:
            logger.info("cmd: %s", cmd)
            with Popen(cmd, stdout=PIPE) as proc:
                answer = proc.stdout.read()
                answer = answer.decode("utf-8")
                logger.info("id: %s, answer %s", self.id, answer)
                return answer

    def set_config(self, config, base):
        super(TGAgent, self).set_config(config, base)

    # def feedback(self, response):
    #    if response.agent_id != self.id:
    #        # update prime
    #        text = response.answer
    #        prime_text = self.person2_subbers.sub(text)
    #        logger.warning("prime %s, text: %s", prime_text, text)
    #        if prime_text != text:
    #            # only update the prime when there is pronoun
    #            logger.info("prime text %s", prime_text)
    #            self._ask(prime_text)

    def new_session(self):
        """The blenderbot doesn't maintain the session. Whenever it needs to
        start a new conversation, it will simply reset the current session"""
        sid = str(uuid.uuid4())
        text = self.config.get("prime.text")
        if text:
            prime_text = " ".join(text)
            logger.info("prime text %s", prime_text)
            # self._ask("/start")
            self._ask(prime_text)
        else:
            logger.info("no prime text")
        return sid

    def chat(self, agent_sid, request, timeout=None):
        if agent_sid is None:
            logger.warning("Agent session was not provided")
            return
        if (
            "min_question_length" in self.config
            and self._get_question_length(request.question)
            < self.config["min_question_length"]
        ):
            logger.info(
                "Ignore short question: %s", self._get_question_length(request.question)
            )
            return

        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question
        if timeout is None:
            timeout = request.context.get("timeout")  # timeout by request

        answer = self._ask(request.question, timeout)
        if not answer:
            response.trace = "Not responsive"
        else:
            response.attachment[
                "match_excluded_expressions"
            ] = self.check_excluded_expressions(answer)
            answer = self.post_processing(answer)
            response.answer = answer
            self.score(response)

        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 60
        if response.attachment["match_excluded_expressions"]:
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        else:
            input_len = len(response.question)
            if input_len > 100:
                response.attachment["score"] = 80
        response.attachment["score"] += random.randint(
            -10, 10
        )  # give it some randomness

        if (
            "minimum_score" in self.config
            and response.attachment["score"] < self.config["minimum_score"]
        ):
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
