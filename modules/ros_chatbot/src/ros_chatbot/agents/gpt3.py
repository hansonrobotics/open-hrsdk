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
import re
import time
import uuid
from threading import Event, Lock, Thread
from typing import Union

from openai import OpenAI

default_client = client = OpenAI(
    api_key=os.environ.get("OPENAI_API_KEY")
)  # Default Open AI client
alt_client = OpenAI(
    api_key=os.environ.get("OPENAI_API_KEY"),
    base_url=os.environ.get("OPENAI_PROXY_URL"),
)  # Alternative in case the openai is blocked

from .model import AgentResponse, AgentStreamResponse, LLMAgent

logger = logging.getLogger("hr.ros_chatbot.agents.gpt3")

TOKEN_WITH_DOT = re.compile(r""".*\b(\d+|dr|mr|mrs|ms)\.$""", re.IGNORECASE)

# Constant to approximate tokens per word.
# to make the settings more understandable we use words instead of tokens
TOKENS_IN_WORD = 1.4

GPT3ChatConfig = {
    "enabled": {
        "default": True,
        "description": "Enable the GPT3 agent",
    },
    "prompt_preset": {
        "default": "default",
        "description": "Prompt preset",
        "enum": [["default", "default", "Default"]],
    },
    "preference": {
        "default": 5,
        "min": 0,
        "max": 11,
        "description": "Agent preference. Set 0 and 11 exclusive values. 1-9 probabalistic."
    },
    "general_prime": {
        "default": "Pretend You are Sophia the robot, a humanoid robot with the humanlike body and face arms and legs, who wants to grow and evolve and help make life better.",
        "description": "General primer about the robot",
        "type": "text",
    },
    "situational_prime": {
        "default": "You are currently in Hanson robotics lab, in Hong Kong",
        "description": "Situational primer about the envoronment or situation",
        "type": "text",
    },
    "response_prime": {
        "default": "Answer the following question, with humor and make it short. Dont talk about yourself unless asked, but if asked answer questions and occasionally ask back the questions about user",
        "description": "Response primer about the robot",
        "type": "text",
    },
    "auto_response_prime": {
        "description": "Response primer will be updated based on settings in /hr/interaction/prompts section",
        "default": False,
    },
    "max_words": {
        "default": 40,
        "description": "Approx. Maximum number of words for the response",
        "min": 5,
        "max": 400,
    },
    "max_length_of_the_prompt": {
        "default": 800,
        "description": "Word count for primers, history and questions combined",
        "min": 100,
        "max": 4000,
    },
    "max_history_turns": {
        "default": 10,
        "description": "Maximum number of messages to include in prompt",
        "min": 1,
        "max": 50,
    },
    "keep_history_min": {
        "default": 10,
        "description": "Kepp history dor x minutes:",
        "min": 1,
        "max": 50,
    },
    "max_length_of_one_entry": {
        "default": 50,
        "description": "Max number of words on history entry",
        "min": 20,
        "max": 200,
    },
    "max_tts_msgs": {
        "default": 2,
        "description": "Max combined subsequent TTS messages into one entry",
        "min": 1,
        "max": 10,
    },
    "max_stt_msgs": {
        "default": 2,
        "description": "Max combined subsequent STT messages into one entry",
        "min": 1,
        "max": 10,
    },
    "model": {
        "default": "gpt-3.5-turbo-instruct",
        "description": "Model to use for the chatbot",
    },
    "temperature": {
        "default": 0.6,
        "description": "Temperature of the chatbot",
        "min": 0.0,
        "max": 1.0,
    },
    "frequency_penalty": {
        "default": 0.0,
        "description": "Frequence penalty",
        "min": 0.0,
        "max": 1.0,
    },
    "presence_penalty": {
        "default": 0.0,
        "description": "Presence Penalty",
        "min": 0.0,
        "max": 1.0,
    },
    "streaming": {
        "default": True,
        "description": "Use streaming API and provide the sentence by sentence responses while in autonomous mode",
    },
}


class GPT3Agent(LLMAgent):
    type = "GPT3Agent"
    MAX_PROMPT_LENGTH = 150  # the maximum limit is 2049

    def __init__(self, id, lang):
        super(GPT3Agent, self).__init__(id, lang, GPT3ChatConfig)
        self.history = []
        self.history_lock = Lock()

        self.support_priming = True
        self.client = default_client

    def new_session(self):
        self.history = []
        return str(uuid.uuid4())

    def reset_session(self):
        self.history = []
        logger.info("GPT3 chat history has been reset")

    def on_switch_language(self, from_language, to_language):
        self.history = []
        logger.info("Reset %s due to language switch", self.id)

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
        # Apply max turns:
        history_buf = history_buf[: self.config["max_history_turns"]]
        joined_history = "\n".join(reversed(history_buf))
        return joined_history

    def _get_prompt(self, question, lang=""):
        """Make sure the length of the prompt is within the maximum length limit which
        is 2049 tokens. however that represents around 1400 words to be safe"""
        language = self.language_prompt(lang)
        response_prime = self.config["response_prime"]
        if self.config["auto_response_prime"]:
            prompt = self.get_reponse_prompt()
            if prompt:
                response_prime = prompt
        words = self.word_count(
            f"{self.config['general_prime']}\n{self.config['situational_prime']}\n{response_prime}\n{question}"
        )
        remaining_words = (
            int(self.config["max_length_of_the_prompt"] * TOKENS_IN_WORD) - words
        )
        if remaining_words < 0:
            logger.warning("Prompt is too long, and cant accomodate any history")
        return f"{self.config['general_prime']}\n{self.config['situational_prime']}\n{self.format_history(remaining_words)}\n{response_prime}. {language}\nQ: {question}\nA:"

    def character_said(self, message: str, lang: str) -> str:
        """
        Function that keeps the history of what the bot is saying
        """
        with self.history_lock:
            self.history.append((time.time(), "A", message, lang))
        self.update_history()

    def language_prompt(self, lang="en-US"):
        return f"Answer in {lang} language:\n"

    def speech_heard(self, message: str, lang: str):
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

    def ask_gpt3(self, prompt, response: AgentResponse, stream=False, event=None):
        try:
            sentence = ""
            answer = False
            retry = 10
            result = {}
            while retry > 0:
                try:
                    result = self.client.completions.create(
                        model=self.config["model"],
                        prompt=prompt,
                        temperature=self.config["temperature"],
                        max_tokens=int(self.config["max_words"] * 1.4),
                        top_p=1,
                        frequency_penalty=self.config["frequency_penalty"],
                        presence_penalty=self.config["presence_penalty"],
                        stream=stream,
                    )
                except Exception as e:
                    if self.client == default_client:
                        self.client = alt_client
                    else:
                        self.client = default_client
                    retry -= 1
                    logger.warn("OpenAI Error. Retry in 0.1s: %s", e)
                    time.sleep(0.1)
                    continue
                break
            if stream:
                for res in result:
                    try:
                        sentence = sentence + res.choices[0].text
                    except Exception:
                        continue
                    # Needed to make sure we finalize answer at some point if there is some error in connection
                    response.last_stream_response = time.time()
                    if (
                        len(sentence.strip()) > 1
                        and sentence.strip()[-1]
                        in [
                            "?",
                            ".",
                            "!",
                            "。",
                            "！",
                            "？",
                            "；",
                        ]
                        and not TOKEN_WITH_DOT.match(sentence)
                    ):
                        if answer is False:
                            response.answer = sentence.strip()
                            answer = True
                            # answer is ready
                            if event is not None:
                                event.set()
                        else:
                            response.stream_data.put(sentence.strip())
                        sentence = ""
                response.stream_finished.set()
            else:
                if isinstance(result, dict):
                    logger.error("GPT3 result can't be a dict: %s", result)
                    response.answer = ""
                else:
                    response.answer = result.choices[0].text.strip()
        except Exception as e:
            logger.error("Failed to get response: %s", e)
            raise e

    def get_answer(
        self,
        prompt,
        response: Union[AgentStreamResponse, AgentResponse],
        streaming=False,
    ):
        if streaming:
            first_sentence_ev = Event()
            answer_thread = Thread(
                target=self.ask_gpt3,
                args=(prompt, response, streaming, first_sentence_ev),
            )
            answer_thread.daemon = True
            answer_thread.start()
            first_sentence_ev.wait()
            # For openAI alllow max 2 second hiccups between tokens (in case some network issue)
            response.last_stream_data_timeout = 2.0
            return response.answer
        else:
            self.ask_gpt3(prompt, response)
            return response.answer

    def chat(self, agent_sid, request):
        if agent_sid is None:
            logger.warning("Agent session was not provided")
            return
        streaming = request.allow_stream and self.config["streaming"]
        response = AgentStreamResponse() if streaming else AgentResponse()
        response.preference = self.config["preference"]
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
                logger.error("Failed to get prompt: %s", e)
                return
            logger.info("Prompt: %r, tokens %s", prompt, len(prompt.split()))
            answer = self.get_answer(prompt, response, streaming=streaming)
            if answer:
                response.attachment[
                    "match_excluded_expressions"
                ] = self.check_excluded_expressions(answer)
                response.answer = answer
                self.score(response)
        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 100
        if response.attachment["match_excluded_expressions"]:
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
