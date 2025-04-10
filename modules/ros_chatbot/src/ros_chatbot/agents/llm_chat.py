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

import os
import re
import time
import uuid
from threading import Event, Thread
from typing import Union

import openai
from haipy.memory_manager.memory_agent import MemoryAgent
from langchain.prompts import ChatPromptTemplate

from ros_chatbot.utils import get_llm

from .model import AgentRequest, AgentResponse, AgentStreamResponse, LLMAgent

TOKEN_WITH_DOT = re.compile(r""".*\b(\d+|dr|mr|mrs|ms)\.$""", re.IGNORECASE)
ENDING_PUNCTUATIONS = ["?", ".", "!", "。", "！", "？", "；"]


class LLMChatAgent(LLMAgent):
    type = "LLMChatAgent"

    def __init__(self, id, lang, model_id, model_kwargs, runtime_config_description):
        super().__init__(id, lang, runtime_config_description)
        self.default_llm = get_llm(model_id, model_kwargs)
        self.alt_llm = None
        # OpenAI models require a proxy for certain regions including HK
        if model_id.startswith("openai."):
            base_url = os.environ.get("OPENAI_PROXY_URL")
            if base_url:
                model_kwargs["base_url"] = base_url
            self.alt_llm = get_llm(model_id, model_kwargs)
            self.logger.info(f"Alt LLM: {self.alt_llm}")
        self.llm = self.default_llm
        if self.llm is None:
            raise RuntimeError("The LLM model is not found %r", model_id)
        self.model_id = model_id

    def ask_llm(
        self,
        request: AgentRequest,
        prompt_str: str,
        response: Union[AgentStreamResponse, AgentResponse],
        streaming=False,
    ):
        """
        Send a prompt to the language model and handle the response.

        Args:
            request (AgentRequest): The request object.
            prompt_str (str): The prompt string to send to the language model.
            response (Union[AgentStreamResponse, AgentResponse]): The response object to store the model's answer.
            streaming (bool): Whether to use streaming mode for the response.

        Returns:
            str: The final answer or first sentence from the language model.
        """
        try:
            prompt = ChatPromptTemplate.from_template(
                prompt_str, template_format="jinja2"
            )
            chain = prompt | self.llm | self.output_parser

            if streaming:
                first_sentence_ev = Event()

                def handle_streaming():
                    try:
                        response.stream_error = None
                        sentence = ""
                        for ret in chain.stream({}):
                            if ret:
                                sentence += ret
                                if (
                                    len(sentence.strip()) > 1
                                    and sentence.strip()[-1] in ENDING_PUNCTUATIONS
                                    and not TOKEN_WITH_DOT.match(sentence)
                                ):
                                    if not first_sentence_ev.is_set():
                                        response.answer = sentence.strip()
                                        first_sentence_ev.set()
                                    else:
                                        response.stream_data.put(sentence.strip())
                                    sentence = ""
                            response.last_stream_response = time.time()
                        first_sentence_ev.set()
                        response.stream_finished.set()
                    except Exception as e:
                        response.stream_error = e
                        first_sentence_ev.set()
                        pass

                Thread(target=handle_streaming, daemon=True).start()
                first_sentence_ev.wait()  # Wait for the first sentence to be set
                if response.stream_error:
                    self.logger.warning("Stream error: %s", response.stream_error)
                    raise response.stream_error
                # For openAI alllow max 2 second hiccups between tokens (in case some network issue)
                response.last_stream_data_timeout = 2.0
                return response.answer
            else:
                ret = chain.invoke({})
                if ret:
                    return ret.strip()
                else:
                    self.logger.warning("No result")
                    return None
        except openai.PermissionDeniedError:
            if self.alt_llm is not None:
                if self.llm == self.default_llm:
                    self.llm = self.alt_llm
                    self.logger.warning("Switching to alternative LLM")
                    return self.ask_llm(
                        request, prompt_str, response, streaming=streaming
                    )
                else:
                    self.llm = self.default_llm

    def chat(self, agent_sid, request):
        if agent_sid is None:
            self.logger.warning("Agent session was not provided")
            return
        streaming = request.allow_stream and self.config["streaming"]
        response = AgentStreamResponse() if streaming else AgentResponse()
        response.preference = self.config.get("preference", -1)
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question

        if request.question:
            try:
                format = "llama3" if "llama3" in self.model_id else None
                prompt = self.get_prompt_str(request, format=format)
            except Exception as e:
                self.logger.exception("Failed to get prompt: %s", e)
                return
            self.logger.info("Prompt %s", prompt)
            answer = self.ask_llm(request, prompt, response, streaming=streaming)
            if answer:
                response.attachment[
                    "match_excluded_expressions"
                ] = self.check_excluded_expressions(answer)
                answer = self.post_processing(answer)
                response.answer = answer
                self.score(response)
        self.handle_translate(request, response)
        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 100
        if response.attachment["match_excluded_expressions"]:
            response.attachment["score"] = -1
            response.attachment["blocked"] = True


class ToolCallingLLMChatAgent(LLMChatAgent):
    type = "ToolCallingLLMChatAgent"

    def __init__(self, id, lang, model_id, model_kwargs, runtime_config_description):
        super().__init__(id, lang, model_id, model_kwargs, runtime_config_description)
        self.allow_repeat = True
        self.memory_agent = MemoryAgent(self.llm)
        self.exclude_context_variables = ["input"]

    def ask_llm(
        self,
        request: AgentRequest,
        prompt_str: str,
        response: Union[AgentStreamResponse, AgentResponse],
        streaming=False,
    ):
        """
        Send a prompt to the language model and handle the response.

        Args:
            request (AgentRequest): The request object.
            prompt_str (str): The prompt string to send to the language model.
            response (Union[AgentStreamResponse, AgentResponse]): The response object to store the model's answer.
            streaming (bool): Whether to use streaming mode for the response.

        Returns:
            str: The final answer or first sentence from the language model.
        """

        try:
            self.memory_agent.llm = self.llm
            self.memory_agent.prompt = prompt_str
            self.memory_agent.session_context = request.session_context
            if streaming:
                self.memory_agent.enable_placeholder_utterances = True
                first_sentence_ev = Event()

                def handle_streaming():
                    for result in self.memory_agent.stream(
                        request.question,
                        language=request.lang,
                    ):
                        if result and result.strip():
                            if not first_sentence_ev.is_set():
                                first_sentence_ev.set()
                                response.answer = result.strip()
                            else:
                                response.stream_data.put(result.strip())
                        response.last_stream_response = time.time()
                    first_sentence_ev.set()
                    response.stream_finished.set()

                Thread(target=handle_streaming, daemon=True).start()
                first_sentence_ev.wait()  # Wait for the first sentence to be set
                return response.answer
            else:
                self.memory_agent.enable_placeholder_utterances = False
                return self.memory_agent.query(
                    request.question,
                    language=request.lang,
                )
        except openai.PermissionDeniedError:
            if self.alt_llm is not None:
                if self.llm == self.default_llm:
                    self.llm = self.alt_llm
                    self.memory_agent.llm = self.llm
                    self.logger.warning("Switching to alternative LLM")
                    return self.ask_llm(
                        request, prompt_str, response, streaming=streaming
                    )
                else:
                    self.llm = self.default_llm


class OpenAIChatAgent(LLMChatAgent):
    type = "OpenAIChatAgent"


class LlamaChatAgent(LLMChatAgent):
    type = "LlamaChatAgent"


class ClaudeChatAgent(LLMChatAgent):
    type = "ClaudeChatAgent"
