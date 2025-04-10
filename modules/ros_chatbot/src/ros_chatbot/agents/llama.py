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

import logging
import re
import time
import uuid
from threading import Lock

import requests

from .model import AgentResponse, SessionizedAgent

logger = logging.getLogger("hr.ros_chatbot.agents.llama")

# Constant to approximate tokens per word.
# to make the settings more understandable we use words instead of tokens
TOKENS_IN_WORD = 1.4

BAR_TEXT = re.compile(r"(\|)([^\|]+)\1")

LANGUAGE_BCP47_CODES = {
    "Arabic": "ar-SA",
    "Cantonese": "yue-Hant-HK",
    "Chinese": "cmn-Hans-CN",
    "Czech": "cs-CZ",
    "English": "en-US",
    "French": "fr-FR",
    "German": "de-DE",
    "Hindi": "hi-IN",
    "Hungarian": "hu-HU",
    "Italian": "it-IT",
    "Japanese": "ja-JP",
    "Korean": "ko-KR",
    "Mandarin": "cmn-Hans-CN",
    "Norwegian": "no-NO",
    "Polish": "pl-PL",
    "Russian": "ru-RU",
    "Spanish": "es-ES",
}

LANGUGE_INV_MAP = {v: k for k, v in LANGUAGE_BCP47_CODES.items()}


class LlamaAgent(SessionizedAgent):
    type = "LlamaAgent"

    def __init__(self, id, lang, host="localhost", port=9300, timeout=3):
        super(LlamaAgent, self).__init__(id, lang)
        self.host = host
        self.port = port
        if self.host not in ["localhost", "127.0.0.1"]:
            logger.warning("Llama server: %s:%s", self.host, self.port)
        self.timeout = timeout
        self.history = []
        self.history_lock = Lock()

        self.support_priming = True

    def new_session(self):
        self.history = []
        return str(uuid.uuid4())

    def reset_session(self):
        self.history = []

    def word_count(self, text):
        if text:
            return len(text.split())
        else:
            return 0

    def format_history(self, max_words, ignore_last_s=3.0) -> str:
        # History is list of tupples time, type (U, Q), message
        # Sort by time first
        # Ignore last_seconds is reqyuired as very recent history might be placeholder utterances, als speech that is duplicate to question.
        with self.history_lock:
            self.history.sort(key=lambda x: x[0])
        cut_of_time = time.time() - ignore_last_s
        # Reverse iterator to go through the history from the latest to the oldest
        words_left = max_words
        entry_words_left = self.config["max_length_of_one_entry"]
        same_type_entries = 0
        history_buf = []
        current_entry_type = ""
        with self.history_lock:
            for log in reversed(self.history):
                if log[0] > cut_of_time:
                    continue
                # Make sure not to reach global limit
                entry_words_left = min(entry_words_left, words_left)
                # Append message to the current entry
                if log[1] == current_entry_type:
                    # Reached max amount of same type entries
                    if same_type_entries < 1:
                        continue
                    entry_words_left -= self.word_count(log[2])
                    words_left -= self.word_count(log[2])
                    if entry_words_left > 0:
                        history_buf[-1] += f". {log[2]}"
                        same_type_entries -= 1
                    continue
                # New entry type
                entry_words_left = self.config["max_length_of_one_entry"]
                current_entry_type = log[1]
                same_type_entries = (
                    self.config["max_tts_msgs"]
                    if current_entry_type == "Q"
                    else self.config["max_stt_msgs"]
                )
                words_left -= self.word_count(log[2])
                if words_left < 0:
                    break
                history_buf.append(f"{log[1]}: {log[2]}")

        joined_history = "\n".join(reversed(history_buf))
        return joined_history

    def _get_prompt(self, question, lang=""):
        """Make sure the length of the prompt is within the maximum length limit which
        is 2049 tokens. however that represents around 1400 words to be safe"""
        language = self.language_prompt(lang)
        general_priming = self.config["general_prime"]
        situational_priming = self.config["situational_prime"]
        response_priming = self.config["response_prime"]
        prompt = (
            f"{general_priming}\n{situational_priming}\n{response_priming}\n{question}"
        )
        words = self.word_count(prompt)
        remaining_words = (
            int(self.config["max_length_of_the_prompt"] * TOKENS_IN_WORD) - words
        )
        if remaining_words < 0:
            logger.warning("Prompt is too long, and cant accomodate any history")
        return f"### Instruction: {general_priming}\n{situational_priming}\n{response_priming}.\n{self.format_history(remaining_words)}\n{language}\n### Question: {question}\n### Response:"

    def character_said(self, message: str, lang: str) -> str:
        """
        Function that keeps the history of what the bot is saying
        """
        message = BAR_TEXT.sub("", message)
        message = message.strip()
        if not message:
            return
        with self.history_lock:
            self.history.append((time.time(), "A", message, lang))
        self.update_history()

    def language_prompt(self, lang="en-US"):
        lang = LANGUGE_INV_MAP.get(lang, lang)
        return f"Answer in {lang}.\n"

    def speech_heard(self, message: str, lang: str):
        if message and message.startswith("event."):
            return
        with self.history_lock:
            # Speech is usually heard before it gets callback so we put it back one second in the past
            self.history.append((time.time() - 1, "Q", message, lang))
        self.update_history()

    def priming(self, request):
        """Update priming statements"""
        logger.info("Priming %r...", request.question[:100])
        self.set_config({"situational_prime": request.question}, base=False)

    def update_history(self):
        with self.history_lock:
            cut_time = time.time() - self.config["keep_history_min"] * 60
            self.history = [h for h in self.history if h[0] > cut_time]
            self.history.sort(key=lambda x: x[0])

    def ask(self, request, prompt):
        timeout = request.context.get("timeout") or self.timeout
        try:
            params = {
                "prompt": prompt,
                "params": {
                    "frequency_penalty": self.config["frequency_penalty"],
                    "presence_penalty": self.config["presence_penalty"],
                    "temperature": self.config["temperature"],
                    "max_tokens": self.config["max_tokens"],
                    "repeat_penalty": self.config["repeat_penalty"],
                    "stop": ["#"],
                },
            }
            response = requests.post(
                "http://{host}:{port}/chat".format(host=self.host, port=self.port),
                json=params,
                timeout=timeout,
            )
        except Exception as ex:
            logger.error(ex)
            return ""
        if response.status_code == requests.codes.ok:
            json = response.json()
            if "response" in json and json["response"] and "answer" in json["response"]:
                return json["response"]["answer"]

    def chat(self, agent_sid, request):
        if agent_sid is None:
            logger.warning("Agent session was not provided")
            return
        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question
        if request.question:
            try:
                prompt = self._get_prompt(request.question, lang=request.lang)
            except Exception as e:
                logger.exception("Failed to get prompt: %s", e)
                return
            logger.info("Prompt: %r, tokens %s", prompt, len(prompt.split()))
            answer = self.ask(request, prompt)
            if answer:
                response.attachment[
                    "match_excluded_expressions"
                ] = self.check_excluded_expressions(answer)
                answer = self.post_processing(answer)
                response.answer = answer
                self.score(response)
        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 80
        if response.attachment["match_excluded_expressions"]:
            response.attachment["score"] = -1
            response.attachment["blocked"] = True

        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.text
        ):
            response.attachment["score"] = -1
            logger.warning("Question response %s is not allowed", response.text)

        if response.attachment.get("blocked"):
            logger.warning("Response is blocked")
            response.attachment["score"] = -1
