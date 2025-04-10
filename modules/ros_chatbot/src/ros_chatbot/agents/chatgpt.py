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

from openai import OpenAI

api_key = os.environ.get("OPENAI_API_KEY")
openai_proxy_url = os.environ.get("OPENAI_PROXY_URL")

from .model import AgentResponse, AgentStreamResponse, LLMAgent

TOKEN_WITH_DOT = re.compile(r""".*\b(\d+|dr|mr|mrs|ms)\.$""", re.IGNORECASE)


# TODO: The 'openai.organization' option isn't read in the client API. You will need to pass it when you instantiate the client, e.g. 'OpenAI(organization=os.environ.get("OPENAI_ORG_ID"))'
# openai.organization = os.environ.get("OPENAI_ORG_ID")

ChatGPTConfig = {
    "template_name": {
        "default": "default",
        "description": "The name of the template to use",
        "enum": [
            ["default", "default", "Default prompt template"],
        ],
    },
    "prompt_preset": {
        "default": "default",
        "description": "Prompt preset",
        "enum": [["default", "default", "Default"]],
    },
    "general_prime": {
        "default": "Pretend You are Sophia the robot, a humanoid robot with the humanlike body and face arms and legs, who wants to grow and evolve and help make life better.",
        "description": "General primer about the robot",
        "type": "text",  # Text means multiline input vs string for single line
    },
    "situational_prime": {
        "default": "You are currently in Hanson robotics lab, in Hong Kong",
        "description": "Situational primer about the envoronment or situation",
        "type": "text",
    },
    "topical_prime": {
        "default": "",
        "description": "Information about current subject. Could be changed for different stage of conversation",
        "type": "text",
    },
    "response_prime": {
        "default": "Respond to user input below with humor and make it short. Dont talk about yourself unless asked.",
        "description": "Response primer used then user asks something",
        "type": "text",
    },
    "auto_response_prime": {
        "description": "Response primer will be updated based on settings in /hr/interaction/prompts section",
        "default": False,
    },
    "prompt_prime": {
        "default": "Response to the command below should be short and to the point. Add little bit of humor where appropriate.",
        "description": "Prompt primer used the robot is instructed to do something without user input",
        "type": "text",
    },
    "max_words": {
        "default": 40,
        "description": "Approx. Maximum number of words for the response",
        "min": 5,
        "max": 200,
    },
    "max_length_of_the_prompt": {
        "default": 3800,
        "description": "Word count for primers, history and user input. Limit if the performance is too slow.",
        "min": 100,
        "max": 3800,
    },
    "max_history_turns": {
        "default": 20,
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
        "description": "Presence penalty",
        "min": 0.0,
        "max": 1.0,
    },
    "priming_strategy": {
        "default": "CHAT",
        "desciption": "Different priming startegies for experimentation",
        "enum": [
            [
                "CHAT",
                "CHAT",
                "Most priming is done as system message. History is split and situational append to last message",
            ],
            ["USER", "USER", "All Character and history is primed as user prompt"],
        ],
    },
    "next_turn_instruction": {
        "default": "",
        "description": "Instruction to add to the next dialog turn. Will reset after robot says something. Will be ignored if its a prompt and not chat instruction",
    },
    "streaming": {
        "default": False,
        "description": "Use streaming API and provide the sentence by sentence responses while in autonomous mode",
    },
}


class ChatGPTAgent(LLMAgent):
    type = "ChatGPTAgent"

    def __init__(self, id, lang):
        super(ChatGPTAgent, self).__init__(id, lang, ChatGPTConfig)
        self.openai_client = OpenAI(api_key=api_key)
        self.proxy_client = OpenAI(api_key=api_key, base_url=openai_proxy_url)
        self.client = self.openai_client

        self.status = {}
        self.prompt_responses = True
        # This will be adjusted based on the actual data.
        self.tokens_in_word = 1.4

    def new_session(self):
        self.set_config({"status": ""}, True)
        return str(uuid.uuid4())

    def reset_session(self):
        self.set_config({"status": ""}, True)
        self.logger.info("ChatGPT chat history has been reset")

    def on_switch_language(self, from_language, to_language):
        self.logger.info("Reset %s due to language switch", self.id)

    def ask_chatgpt(
        self,
        prompt,
        response: AgentResponse,
        streaming=False,
        answerReady: Event = None,
    ):
        def switch_client():
            if self.client == self.openai_client:
                self.client = self.proxy_client
            else:
                self.client = self.openai_client

        def handle_streaming(result):
            ENDING_PUNCTUATIONS = ["?", ".", "!", "。", "！", "？", "；"]
            sentence = ""
            answer = False
            for res in result:
                try:
                    if res.choices[0].delta.content is not None:
                        sentence += res.choices[0].delta.content
                except Exception as e:
                    self.logger.error(
                        "concatinating stream data error: %s, data %s",
                        e,
                        res,
                    )
                    continue
                response.last_stream_response = time.time()
                if (
                    len(sentence.strip()) > 1
                    and sentence.strip()[-1] in ENDING_PUNCTUATIONS
                    and not TOKEN_WITH_DOT.match(sentence)
                ):
                    if not answer:
                        response.answer = sentence.strip()
                        answer = True
                        if answerReady is not None:
                            answerReady.set()
                    else:
                        response.stream_data.put(sentence.strip())
                    sentence = ""
            if answerReady is not None:
                answerReady.set()
            response.stream_finished.set()

        def handle_non_streaming(result):
            response.answer = result.choices[0].message.content.strip()

        try:
            retry = 10
            result = {}
            while retry > 0:
                try:
                    result = self.client.chat.completions.create(
                        model=self.config["model"],
                        messages=[{"role": "user", "content": prompt}],
                        temperature=self.config["temperature"],
                        max_tokens=int(self.config["max_words"] * self.tokens_in_word),
                        top_p=1,
                        frequency_penalty=self.config["frequency_penalty"],
                        presence_penalty=self.config["presence_penalty"],
                        stream=streaming,
                    )
                except Exception as e:
                    self.logger.warn("OpenAI Error. Retry in 0.1s: %s", e)
                    switch_client()
                    retry -= 1
                    time.sleep(0.1)
                    continue
                break

            if not result:
                self.logger.error("No result")
                return
            if streaming:
                handle_streaming(result)
            else:
                handle_non_streaming(result)
        except Exception as e:
            self.logger.error("Failed to get response: %s", e)
            raise e

    def get_answer(self, prompt, response: AgentResponse, streaming=False):
        if streaming:
            first_sentence_ev = Event()
            answer_thread = Thread(
                target=self.ask_chatgpt,
                args=(prompt, response, streaming, first_sentence_ev),
            )
            answer_thread.daemon = True
            answer_thread.start()
            first_sentence_ev.wait()
            # For openAI alllow max 2 second hiccups between tokens (in case some network issue)
            response.last_stream_data_timeout = 2.0
            return response.answer
        else:
            self.ask_chatgpt(prompt, response)
            return response.answer

    def chat(self, agent_sid, request):
        self.status = {"errors": []}
        if agent_sid is None:
            self.logger.warning("Agent session was not provided")
            return

        streaming = request.allow_stream and self.config["streaming"]
        response = AgentStreamResponse() if streaming else AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question

        if request.question:
            try:
                prompt = self.get_prompt_str(request)
            except Exception as e:
                self.logger.exception("Failed to get prompt: %s", e)
                return
            answer = self.get_answer(
                prompt,
                response=response,
                streaming=streaming,
            )
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


class GPT4Agent(ChatGPTAgent):
    type = "GPT4Agent"
