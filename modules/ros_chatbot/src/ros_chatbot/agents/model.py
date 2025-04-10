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

import datetime
import json
import logging
import re
import time
import uuid
from abc import ABCMeta, abstractmethod
from collections import ChainMap
from queue import Queue
from threading import Event
from typing import List

import rospy
from benedict import benedict
from haipy.chat_history import ChatHistory
from haipy.nlp.translate import TranslateClient
from haipy.text_processing.template_renderer import Renderer
from haipy.utils import LANGUAGE_CODES_NAMES
from langchain_core.output_parsers import StrOutputParser

from ros_chatbot.utils import (
    DEFAULT_PROMPT_TEMPLATE,
    get_current_time_str,
    get_named_entities,
    remove_puncuation_marks,
    to_list,
)


class IntentClassifier(object, metaclass=ABCMeta):
    @abstractmethod
    def detect_intent(self, text, lang):
        """Detects intent for the text and language"""
        pass


class SentimentClassifier(object, metaclass=ABCMeta):
    @abstractmethod
    def detect_sentiment(self, text, lang):
        """Detects sentiment for the text and language"""
        pass


class AgentRequest(object):
    def __init__(self):
        self.sid = ""  # session id
        self.app_id = "chat-ros"
        self.request_id = ""
        self.time = get_current_time_str()
        self.lang = ""
        self.question = ""
        self.audio = ""  # path of the audio if the request is from Speech-to-Text
        self.tag = ""  # tag for the conversation
        self.source = ""
        self.context = {}
        self.scene = ""
        self.user_id = ""  # graph user id
        self.session_context = None
        # Allow stream
        self.allow_stream = False
        self.hybrid_mode = False

    def __repr__(self):
        return '<AgentRequest(question="%r" lang="%r" request_id="%r")>' % (
            self.question,
            self.lang,
            self.request_id,
        )

    def to_dict(self):
        return {
            "sid": self.sid,
            "app_id": self.app_id,
            "request_id": self.request_id,
            "time": self.time,
            "lang": self.lang,
            "question": self.question,
            "audio": self.audio,
            "tag": self.tag,
            "source": self.source,
            "context": self.context,
            "scene": self.scene,
            "user_id": self.user_id,
        }


class AgentResponse(object):
    def __init__(self):
        self.sid = ""
        self.agent_id = ""
        self.request_id = ""
        self.response_id = ""
        self.agent_sid = ""
        self.start_dt = get_current_time_str()
        self.end_dt = get_current_time_str()
        self.lang = ""
        self.question = ""
        self.answer = ""
        self.trace = ""
        self.preference = -1
        self.attachment = {}

    def end(self):
        self.end_dt = get_current_time_str()

    def valid(self):
        answer = remove_puncuation_marks(self.answer)
        return bool(answer) or self.attachment.get("state") == 1

    def __repr__(self):
        if self.valid:
            return (
                '<AgentResponse(agent="%r" answer="%r" lang="%r" preference="%i" )>'
                % (
                    self.agent_id,
                    self.answer,
                    self.lang,
                    self.preference,
                )
            )
        else:
            return '<AgentResponse(agent="%r" answer=invalid lang="%r")' % (
                self.agent_id,
                self.lang,
            )

    def to_dict(self):
        return {
            "sid": self.sid,
            "agent_id": self.agent_id,
            "request_id": self.request_id,
            "response_id": self.response_id,
            "agent_sid": self.agent_sid,
            "start_dt": self.start_dt,
            "end_dt": self.end_dt,
            "lang": self.lang,
            "question": self.question,
            "answer": self.answer,
            "trace": self.trace,
            "attachment": self.attachment,
        }


class AgentStreamResponse(AgentResponse):
    """
    A class to handle streaming responses from an agent.
    """

    def __init__(self):
        super(AgentStreamResponse, self).__init__()
        # Streaming response data
        self.last_stream_response = 0
        # Agents need to set timeout by themselves as it depends in a way they get data (token by token, vs sentence by sentence).
        self.last_stream_data_timeout = 1
        self.stream_error = None
        self.stream_data = Queue()
        self.stream_finished = Event()
        self.stream_response_timeout = 10

    def stream_timeout(self):
        return self.last_stream_response + self.last_stream_data_timeout < time.time()


class AgentRequestExt(AgentRequest):
    original_lang: str = ""
    original_question: str = ""


class Agent(metaclass=ABCMeta):
    def __init__(self, id, lang):
        if not id:
            raise ValueError("Agent id was missing")
        self.id = id
        self.languages = to_list(lang)
        self.weight = 0.5
        self.level = 250  # level in range [200, 300) for builtin agent
        self.promotion = 0
        self.allow_repeat = False
        self.support_priming = False
        self._config = ChainMap().new_child()
        self.config = {}
        self.translate_client = TranslateClient()
        self.runtime_config_description = None
        self.prompt_responses = False
        self.logger = logging.getLogger(
            f"hr.ros_chatbot.agents.{self.__class__.__qualname__}.{self.id}"
        )

    def set_config(self, config, base=False, update_server=False):
        """base: whether it is the base config"""
        if base:
            self._config.maps[1].update(config)
        else:
            self._config.maps[0].update(config)
        if "weight" in self._config:
            self.weight = self._config["weight"]
        if "level" in self._config:
            self.level = self._config["level"]
        self.config = benedict(self._config)  # update config after every change

    def reset_config(self, config_keys=[]):
        """Resets the config to the baseline"""
        if config_keys:
            for key in config_keys:
                if key in self._config.maps[0]:
                    del self._config.maps[0][key]
        else:
            self._config.maps[0].clear()
        self.config = benedict(self._config)

    def current_level(self):
        # Adjust level based on promotion. Promoted agents has lower level.
        return self.level - self.promotion

    @abstractmethod
    def chat(self, agent_sid, request):
        pass

    def character_said(self, message: str, lang: str):
        # Text beeing said by character. By default can be ignored. Might be usefull for agents that makes their own priming statements
        pass

    def speech_heard(self, message: str, lang: str):
        # Text beeing heard by character. By default can be ignored. Might be usefull for agents that makes their own priming statements
        pass

    @property
    def enabled(self):
        return self.config.get("enabled", True)

    @enabled.setter
    def enabled(self, enabled):
        self._config.maps[1]["enabled"] = enabled
        self.config = benedict(self._config)

    def __repr__(self):
        return "<%r(id=%r)>" % (self.__class__, self.id)

    def handle_translate(self, request, response):
        """Translates responses to target language in the original request"""
        if response.answer:
            if isinstance(request, AgentRequestExt):
                result = self.translate_client.translate(
                    response.answer, request.lang, request.original_lang
                )
                if result and result["translated"]:
                    response.attachment["media_question"] = request.question
                    response.attachment["media_answer"] = response.answer
                    response.attachment["media_lang"] = request.lang
                    response.answer = result["text"]
                    response.lang = request.original_lang

    def check_named_entity(self, text):
        entities = get_named_entities(text)
        white_entity_list = self.config.get("white_entity_list", [])
        white_entity_list = [w.lower() for w in white_entity_list]
        if entities:
            for entity in entities:
                if entity["label"] in ["PERSON", "GPE", "ORG", "DATE"]:
                    if entity["text"].lower() in white_entity_list:
                        continue
                    self.logger.warning(
                        "Risky named entities detected %r (%s)",
                        entity["text"],
                        entity["label"],
                    )
                    return True
        return False

    def check_excluded_expressions(self, text):
        if "excluded_regular_expressions" in self.config:
            for exp in self.config["excluded_regular_expressions"]:
                pattern = re.compile(r"%s" % exp, re.IGNORECASE)
                if pattern.search(text):
                    return True
        return False

    def check_excluded_question(self, text):
        if "excluded_question_patterns" in self.config:
            for exp in self.config["excluded_question_patterns"]:
                pattern = re.compile(r"%s" % exp, re.IGNORECASE)
                if pattern.search(text):
                    return True
        return False

    def post_processing(self, answer):
        if answer and "substitutes" in self.config:
            for substitute in self.config["substitutes"]:
                pattern = re.compile(r"%s" % substitute["expression"], re.IGNORECASE)
                repl = substitute["replace"] or ""
                _answer = pattern.sub(repl, answer)
                if answer != _answer:
                    self.logger.warning("Replace answer %r to %r", answer, _answer)
                    answer = _answer
        if "*" in answer:
            answer = answer.replace("*", "|")
        return answer


class SessionizedAgent(Agent):
    # TODO: life cycle of session or persist session

    @abstractmethod
    def new_session(self):
        """
        Starts a new session.

        Returns the new session id
        """
        pass


class ConfigurableAgent(SessionizedAgent):
    def __init__(self, id: str, lang: str, runtime_config_description: dict):
        super().__init__(id, lang)
        self.runtime_config_callback = None
        self.runtime_config_description = runtime_config_description
        self.chat_history = ChatHistory("default.default.history")
        self.renderer = Renderer()
        self.exclude_context_variables = []
        self.output_parser = StrOutputParser()
        # Default configuration for agent is set as base
        super(ConfigurableAgent, self).set_config(
            {k: v["default"] for k, v in self.runtime_config_description.items()}, True
        )

    def new_session(self):
        return str(uuid.uuid4())

    def update_server_config(self, config={}):
        try:
            if callable(self.runtime_config_callback):
                # Filter only dynamic configuration updates
                config = {
                    k: v
                    for k, v in config.items()
                    if k in self.runtime_config_description
                }
                self.runtime_config_callback(config)
        except Exception:
            self.logger.error("Error updating server config")


class LLMAgent(ConfigurableAgent):
    def __init__(self, id: str, lang: str, runtime_config_description: dict):
        super(LLMAgent, self).__init__(id, lang, runtime_config_description)
        self.chat_history = ChatHistory("default.default.history")
        self.renderer = Renderer()

        self.output_parser = StrOutputParser()

    def reset_session(self):
        self.logger.info("ChatGPT chat history has been reset")

    def get_reponse_prompt(self):
        try:
            return rospy.get_param("/hr/interaction/prompts/response_prompt", "")
        except rospy.ServiceException as e:
            self.logger.error(e)
            return False

    def _parse_prompt_template(self, prompt_template: str) -> List[str]:
        """Extract variables from the prompt template"""
        from jinja2 import Environment, meta

        env = Environment()
        ast = env.parse(prompt_template)
        variables = meta.find_undeclared_variables(ast)
        prompt_variables = sorted(list(variables))
        return prompt_variables

    def get_prompt_str(self, request: AgentRequest, format=None):
        prompt_template = self._get_prompt_template(request)
        prompt_variables = self._parse_prompt_template(prompt_template)
        if not prompt_variables:
            prompt_variables = [
                "input",
                "location",
                "interlocutor",
                "objective",
                "situational_prime",
                "dynamic_situational_prime",
                "general_prime",
                "response_prime",
                "webui_language",
                "past_conversation_summaries",
            ]
        context = self._build_context(request, format, prompt_variables)
        for variable in self.exclude_context_variables:
            if variable in context:
                del context[variable]

        return self._render_prompt(prompt_template, context)

    def _build_context(
        self, request: AgentRequest, format: str, prompt_variables: List[str]
    ):
        self.chat_history.set_sid(request.sid)
        current_time = datetime.datetime.now()
        context = {
            "input": request.question,
            "history": self.chat_history.format_history_text(request.question, format),
            "language": LANGUAGE_CODES_NAMES.get(request.lang, request.lang),
            "current_date": current_time.strftime("%Y-%m-%d"),
            "next_week": (current_time + datetime.timedelta(days=7)).strftime(
                "%Y-%m-%d"
            ),
            "current_time": current_time.strftime("%H:%M"),
            "general_prime": self.config.get("general_prime", ""),
            "situational_prime": self.config.get("situational_prime", ""),
            "dynamic_situational_prime": self.config.get("dynamic_situational_prime", ""),
            "agent_id": self.id,
            "agent_type": self.type,
        }

        response_primes = []

        if request.session_context:
            for key in prompt_variables:
                if request.session_context.get(key) and key not in [
                    "history",
                    "input",
                ]:  # not override some context variables
                    context[key] = request.session_context.get(key)
            context["global_workspace_enabled"] = request.session_context.get(
                "global_workspace_enabled", False
            )
            context["instant_situational_prompt"] = request.session_context.get(
                "instant_situational_prompt", ""
            )

            # 1. Append response prime from session context
            if request.session_context.get("response_prime", ""):
                response_primes.append(
                    request.session_context.get("response_prime", "")
                )

        # 2. Append response prime if auto_response_prime is enabled
        if self.config.get("auto_response_prime", False):
            response_prompt = self.get_reponse_prompt()
            if response_prompt:
                response_primes.append(response_prompt.strip())

        # 3. If emotion driven response primer is enabled, append the emotion driven response style
        if request.session_context.get("emotion_driven_response_primer", False):
            emotion_driven_response_style = request.session_context.get(
                "emotion_driven_response_style"
            )
            if emotion_driven_response_style:
                response_primes.append(
                    f"Your response style that is influenced by your emotion: {emotion_driven_response_style}"
                )

        context["response_prime"] = "\n".join(
            "-   " + prime for prime in response_primes
        )
        self.logger.info("response_prime: %s", context["response_prime"])

        self.logger.info("context: %s", context)
        return context

    def _render_prompt(self, template, context):
        prompt_str = self.renderer.render(
            template=template,
            context=context,
            compact=False,
        )
        prompt_str = re.sub("\n{2,}", "\n\n", prompt_str)
        prompt_str = prompt_str.replace("{", "{{").replace("}", "}}")
        self.logger.info("Prompt: \n%s", prompt_str)
        return prompt_str

    def _get_prompt_template(self, request: AgentRequest):
        """The order of precedence is:
        1. prompt_template in the agent configuration
        2. prompt_template in the scene context
        3. default prompt template based on model ID
        """
        # 1. Check for prompt template in the agent configuration
        # Deprecated
        prompt_template = self.config.get("prompt_template")
        if prompt_template and prompt_template.strip():
            self.logger.info("Using prompt template from agent configuration")
            return prompt_template

        # 2. Check for prompt template in the scene context
        if request.session_context:
            # 2.1. Check for prompt template in the prompt templates in the session context
            prompt_templates = request.session_context.get("prompt_templates", [])
            global_workspace_enabled = request.session_context.get(
                "global_workspace_enabled", False
            )
            for template in prompt_templates:
                template = json.loads(template)
                conditions = template["conditions"]
                if ("global_workspace" in conditions) != global_workspace_enabled:
                    continue
                if self.id in conditions or "any_agent" in conditions:
                    self.logger.warning(
                        "Using prompt template %r from session context: id %r, conditions %r",
                        template["name"],
                        self.id,
                        conditions,
                    )
                    return template["template"]

            # 2.2. Check for prompt template in the scene context
            prompt_template = request.session_context.get("prompt_template")
            if prompt_template and prompt_template.strip():
                self.logger.info("Using prompt template from scene context")
                return prompt_template

        # 3. Fallback to default prompt template
        self.logger.info(
            "No prompt template found, using default template based on model ID: %s",
            self.model_id,
        )

        model_prefix = self.model_id.split(".", 1)[0]
        default_template_name = {
            "anthropic": "claude",
            "aws-anthropic": "claude",
            "aws-meta": "llama",
        }.get(model_prefix, "gpt")

        if request.session_context:
            prompt_template = request.session_context.get(
                f"default_prompt_template_{default_template_name}"
            )
            if prompt_template:
                return prompt_template

        prompt_template = DEFAULT_PROMPT_TEMPLATE[default_template_name]
        request.session_context[
            f"default_prompt_template_{default_template_name}"
        ] = prompt_template
        return prompt_template
