#!/usr/bin/env python

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

import collections
import datetime
import logging
import os
import threading
import uuid
from queue import Queue
from typing import Dict, List

import emoji
import haipy.memory_manager as mm

from ros_chatbot.agents import registered_agents
from ros_chatbot.agents.model import AgentRequest
from ros_chatbot.chat_agent_schedular import ChatAgentSchedular
from ros_chatbot.db import update_published_response, write_request, write_responses
from ros_chatbot.response_ranker import ResponseRanker
from ros_chatbot.response_resolver import (
    PreferentialResponseResolver,
    ProbabilisticResponseResolver,
    SimpleResponseResolver,
)
from ros_chatbot.safety_classifier import TextSafetyClassifier
from ros_chatbot.session_manager import SessionManager
from ros_chatbot.utils import abs_path, load_agent_config

from .agents.model import Agent, AgentResponse, AgentStreamResponse

logger = logging.getLogger(__name__)


class AttrDict(dict):
    def __init__(self, *args, **kwargs):
        super(AttrDict, self).__init__(*args, **kwargs)
        self.__dict__ = self


class ChatServer(object):
    def __init__(self):
        self.responses = {}
        self.requests = {}  # id -> request
        self.request_docs = {}  # id -> request documents
        self.response_docs = {}  # id -> response documents
        # preload information
        db_thread = threading.Thread(target=self.db_tasks_thread)
        self.db_tasks = Queue()
        db_thread.daemon = True
        db_thread.start()
        self.document_manager = mm.DocumentManager(
            mongo_uri=os.environ.get("CLOUD_MONGO_DATABASE_URL")
        )

        config = load_agent_config()
        self.agent_specs = config["agents"]
        self.agent_configs = {
            agent["args"]["id"]: agent.get("config", {}) for agent in config["agents"]
        }
        self.resolver_type = config.get("resolver_type", "simple")
        self.translators = config.get("translators", [])

        # install agents
        self.agents: Dict[str, Agent] = {}
        for agent_spec in self.agent_specs:
            if agent_spec["type"] not in registered_agents:
                raise ValueError("Unknown agent type: %s" % agent_spec["type"])
            if agent_spec["type"] == "TranslatorAgent":  # ignore
                continue
            agent = self._create_agent(agent_spec)
            if agent:
                logger.info("Created agent %s", agent.id)
                self.init_config_agent(agent)
                self.agents[agent.id] = agent
            else:
                logger.error("Agent %s was not created", agent_spec["type"])

        self.ranker = ResponseRanker(self.agents)
        # All other resolvers are absolete, as they designed for older agents.
        if self.resolver_type == "simple":
            self.resolver = PreferentialResponseResolver(self.ranker)
            logger.info("Simple resolver")
        elif self.resolver_type == "probablistic":
            self.resolver = ProbabilisticResponseResolver(self.ranker)
            logger.info("Probablistic resolver")
        elif self.resolver_type == "preferential":
            self.resolver = PreferentialResponseResolver(self.ranker)
            logger.info("Preferential resolver")
            print("Preferential resolver")
        else:
            raise ValueError("Unknown resolver type %r", self.resolver_type)

        self.chat_agent_schedular = ChatAgentSchedular(self.agents, self.translators)
        self.session_manager = SessionManager(self.agents)
        self.text_safety_classifier = TextSafetyClassifier()

    def db_tasks_thread(self):
        while True:
            task = self.db_tasks.get()
            # each task is a list that first element is callable, with other eleme
            if task:
                try:
                    task[0](*task[1:])
                except Exception as ex:
                    logger.error(ex)

    def _create_agent(self, agent_spec):
        args = agent_spec["args"]
        if agent_spec["type"] not in registered_agents:
            raise ValueError("Unknown controller type: %s" % agent_spec["type"])

        cls = registered_agents[agent_spec["type"]]

        if agent_spec["type"] == "AIMLAgent":
            # load agent specs
            HR_CHATBOT_WORLD_DIR = os.environ.get("HR_CHATBOT_WORLD_DIR", "")
            agent_spec_file = os.path.join(HR_CHATBOT_WORLD_DIR, "agents.yaml")
            root_dir = os.path.dirname(os.path.realpath(agent_spec_file))
            args["character_yaml"] = abs_path(root_dir, args["character_yaml"])

        if "media_agent" in args:
            media_agent_spec = args["media_agent"]
            if media_agent_spec["type"] in [
                "TranslatorAgent",
                "GPT2Agent",
                "AI21Agent",
            ]:
                raise ValueError("Media agent can't be nested")
            args["media_agent"] = self._create_agent(media_agent_spec)
            if agent_spec["type"] == "TranslatorAgent":
                args["media_language"] = media_agent_spec["args"]["lang"]

        try:
            agent = cls(**args)
            return agent
        except Exception as ex:
            logger.exception(
                "Initializing agent %s with args %s. Error %s", cls, args, ex
            )

    def init_config_agent(self, agent):
        """Configure the agent for the first time"""
        agent_config = self.agent_configs.get(agent.id)
        if agent_config:
            agent.set_config(agent_config, base=True)

    def config_agents(self, configs):
        """Set agent configs"""
        if configs:
            for agent_id, config in list(configs.items()):
                agent = self.agents.get(agent_id)
                if agent:
                    agent.set_config(config, base=False)
                else:
                    logger.error("Agent %r was not found", agent_id)

    def config_agent_types(self, configs):
        """Set agent configs by its type"""
        if configs:
            for agent_type, config in list(configs.items()):
                for agent in self.agents.values():
                    if agent.type == agent_type:
                        agent.set_config(config, base=False)

    def reset_all_agents(self):
        for agent in self.agents.values():
            agent.reset_config()

    @property
    def installed_agent_types(self):
        return {agent.type for agent in self.agents.values()}

    @property
    def agent_id_type_mapping(self):
        return {agent.id: agent.type for agent in self.agents.values()}

    def write_request_to_mongos(self, request):
        try:
            request_doc = mm.ChatRequest(
                user_id="default",
                conversation_id=request.sid,
                text=request.question,
                lang=request.lang,
                audio=request.audio,
                context=request.context or {},
            )
            self.document_manager.add_document(request_doc)
            self.request_docs[request.request_id] = request_doc
        except Exception as ex:
            logger.error(ex)

    def new_request(
        self, sid, text, lang, audio="", source="", context=None, session_context=None
    ) -> AgentRequest:
        # TODO: Check session
        if not text:
            raise ValueError("text is empty")
        request = AgentRequest()
        request.sid = sid
        request.request_id = str(uuid.uuid4())
        request.question = text
        request.lang = lang
        request.audio = audio
        request.source = source
        request.context = context or {}
        request.session_context = session_context

        try:
            self.db_tasks.put([write_request, AttrDict(request.to_dict())])
            # write_request(request)
        except Exception as ex:
            print(ex)
            logger.error("Can't wrrite the request to DB %s", ex)

        self.requests[request.request_id] = request
        self.write_request_to_mongos(request)

        return request

    def on_switch_language(self, from_language, to_language):
        for agent in self.agents.values():
            if hasattr(agent, "on_switch_language"):
                agent.on_switch_language(from_language, to_language)

    def write_responses_to_mongodb(self, responses):
        try:
            for response in responses:
                response_doc = mm.ChatResponse(
                    text=response.answer,
                    lang=response.lang,
                    agent_id=response.agent_id,
                    conversation_id=response.sid,
                    attachment=response.attachment,
                    request=self.request_docs.get(response.request_id),
                    trace=response.trace,
                )
                self.response_docs[response.response_id] = response_doc
        except Exception as ex:
            logger.error(ex)

    def record_responses(self, responses: List[AgentResponse]):
        self.write_responses_to_mongodb(responses)
        self.db_tasks.put([write_responses, responses])
        for response in responses:
            self.responses[response.response_id] = response

    def early_stop(self, fast_score, responses):
        """Should the batch chat tasks stop early?"""
        if fast_score > 0:
            for response in responses:
                agent = self.agents.get(response.agent_id)
                score = self.ranker.score(agent, response)
                if (
                    score >= fast_score
                    and not response.attachment.get("non-verbal")
                    and not isinstance(response, AgentStreamResponse)
                ):
                    logger.warning(
                        "Stopped early with agent %s score %s", response.agent_id, score
                    )
                    return True
        return False

    def find_external_used_responses(self, responses):
        """Finds the response with True state in its attachment
        True state means the response has been handled by external component
        so the response with this state has to be executed for consistance.
        """
        return [
            response
            for response in responses
            if "state" in response.attachment and response.attachment.get("state") == 1
        ]

    def check_safety(self, response):
        if self.text_safety_classifier.classify(response.answer, response.lang):
            return True
        else:
            logger.warning("Response %s is unsafe", response)
            return False

    def demojize(self, response):
        response.answer = emoji.demojize(response.answer)
        return response

    def chat_with_ranking(self, request):
        agent_sessions = self.session_manager.agent_sessions()
        if not agent_sessions:
            logger.error("No agent sessions")
            return

        for responses in self.chat_agent_schedular.chat(request, agent_sessions):
            if responses:
                responses = [self.demojize(response) for response in responses]
                responses = [
                    response for response in responses if self.check_safety(response)
                ]
            if responses:
                responses = self.ranker.rank(responses)
                try:
                    self.record_responses(responses)
                except Exception as ex:
                    logger.error("Failed to write responses to DB %s", ex)
                if self.find_external_used_responses(responses):
                    logger.info("Found external handled response")
                    break
                yield responses

    def chat_with_resolving(self, request, fast_score=0, interrupt_event=None):
        """
        fast_score: the lower bound of response score the agents

        Returns the resolved response and all the other responses
        """
        agent_sessions = self.session_manager.agent_sessions()
        if not agent_sessions:
            logger.error("No agent sessions")
            return
        self.chat_agent_schedular.interrupt = interrupt_event
        # Wait for all responses from agent_schedular, agent schedular would report all responses then they become available, and no agent is still thinking with higher level.
        all_responses = []
        # self.chat_agent_schedular
        for responses in self.chat_agent_schedular.chat(request, agent_sessions):
            if responses:
                responses = [self.demojize(response) for response in responses]
                responses = [
                    response for response in responses if self.check_safety(response)
                ]
            if responses:
                all_responses += responses
                if self.find_external_used_responses(responses):
                    break
                if self.early_stop(fast_score, responses):
                    break

        if all_responses:
            logger.info("Got %s responses in total", len(all_responses))
            try:
                self.record_responses(all_responses)
            except Exception as ex:
                logger.error("Failed to write responses to DB %s", ex)
            external_responses = self.find_external_used_responses(all_responses)
            if external_responses:
                logger.info("Found external handled response")
                response = self.resolver.resolve(external_responses)
            else:
                response = self.resolver.resolve(all_responses)
            if response:
                response.attachment["published"] = True
                self.publish(response.response_id, resolver_type=self.resolver.type)
                # the first response is the published response
                return [response] + [
                    r for r in all_responses if not r.attachment.get("published")
                ]
        else:
            logger.info("No responses. Request %r", request.question)

    def add_record(self, text):
        self.ranker.add_record(text)

    def reset(self, sid):
        return self.session_manager.reset(sid)

    def get_context(self, sid):
        agent_sessions = self.session_manager.agent_sessions()
        if not agent_sessions:
            logger.error("No agent sessions")
            return

        context = collections.defaultdict(dict)
        for agent_id, session_id in agent_sessions.items():
            agent = self.agents.get(agent_id)
            if (
                agent
                and agent.config.get("share_context")
                and hasattr(agent, "get_context")
                and isinstance(agent.get_context, collections.Callable)
            ):
                agent_context = agent.get_context(session_id) or {}
                for k, v in list(agent_context.items()):
                    context[k] = v
                logger.info("Got agent %s context %s", agent.id, context)
        return context

    def set_context(
        self, session_id, context: dict, output: bool = False, finished: bool = True
    ):
        agent_sessions = self.session_manager.agent_sessions()
        if not agent_sessions:
            logger.error("No agent sessions")
            return

        if output:
            for agent_id, _ in agent_sessions.items():
                agent = self.agents.get(agent_id)
                if (
                    agent
                    and agent.config.get("share_context")
                    and hasattr(agent, "set_output_context")
                    and isinstance(agent.set_output_context, collections.Callable)
                ):
                    if finished:
                        logger.info("Finish output context: %s %s", agent_id, context)
                    else:
                        logger.info("Set output context: %s %s", agent_id, context)
                    agent.set_output_context(session_id, context, finished)
        else:
            for agent_id, _ in agent_sessions.items():
                agent = self.agents.get(agent_id)
                if (
                    agent
                    and agent.config.get("share_context")
                    and hasattr(agent, "set_context")
                    and isinstance(agent.set_context, collections.Callable)
                ):
                    agent.set_context(session_id, context)
                    logger.info("Set context: %s %s", agent_id, context)

    def set_timeout(self, timeout, min_wait_for=0.0):
        self.chat_agent_schedular.set_timeout(timeout, min_wait_for)

        # set agent timeout
        for agent in self.agents.values():
            if hasattr(agent, "timeout"):
                agent.timeout = timeout

    def publish(self, response_id, **kwargs):
        """Publishes the response"""
        try:
            response_doc = self.response_docs.get(response_id)
            if response_doc:
                response_doc.published_at = datetime.datetime.utcnow()
                self.document_manager.add_document(response_doc)
            self.db_tasks.put([update_published_response, response_id, kwargs])
            logger.info("Wrote published response to DB successfully")
        except Exception as ex:
            logger.error("Failed to write published response to DB %s", ex)

    def update_response_document(self, response_id, text):
        response_doc = self.response_docs.get(response_id)
        if response_doc:
            logger.info("Updating response document %r with %r", response_doc, text)
            response_doc.text = text
            self.document_manager.add_document(response_doc)

    def run_reflection(self, sid, text, lang):
        results = []
        logger.info("Reflecting on %r", text)
        for agent in self.agents.values():
            if hasattr(agent, "run_reflection"):
                _results = agent.run_reflection(sid, text, lang)
                if _results:
                    results += _results
        return results

    def feedback(self, response, hybrid):
        context = {}
        for agent in self.agents.values():
            if hasattr(agent, "feedback"):
                feedback_result = agent.feedback(
                    response.request_id, response.agent_id == agent.id, hybrid
                )
                if feedback_result and isinstance(feedback_result, dict):
                    context.update(feedback_result)
        return context

    def get_main_agent(self, lang):
        for agent in self.agents.values():
            if agent.config.get("role") == "main" and lang in agent.languages:
                return agent
