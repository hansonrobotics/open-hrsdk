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

import concurrent
import logging
import random
import threading
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError, as_completed
from itertools import groupby
from queue import Empty, Queue

from haipy.nlp.translate import TranslateClient

from ros_chatbot.agents.model import AgentRequestExt
from ros_chatbot.response_resolver import PreferentialResponseResolver

logger = logging.getLogger(__name__)


class ChatAgentSchedular(object):
    def __init__(self, agents, translators, timeout=2):
        self.agents = agents
        self.translators = translators
        # Keep track of running agents, and make sure we dont wait unecessarily
        self.running_agents_lock = threading.Lock()
        # Interrupt event
        self.interrupt = None
        self.timeout = timeout
        self.min_wait_for = 0.0
        self.translate_client = TranslateClient()

    def set_timeout(self, timeout, min_wait_for=0.0):
        self.timeout = timeout
        self.min_wait_for = min_wait_for

    def expand_translate_requests(self, request):
        """expand with translate requests and agents"""
        requests = {}
        for translator in self.translators:
            if (
                request.lang in translator["language_codes"]
                and translator["media_language"] != request.lang
            ):
                result = self.translate_client.translate(
                    request.question, request.lang, translator["media_language"]
                )
                if result and result["translated"]:
                    media_request = AgentRequestExt()
                    media_request.sid = request.sid
                    media_request.request_id = request.request_id
                    media_request.time = request.time
                    media_request.lang = translator["media_language"]
                    media_request.question = result["text"]
                    media_request.audio = request.audio
                    media_request.tag = request.tag
                    media_request.context = request.context
                    media_request.scene = request.scene
                    media_request.user_id = request.user_id
                    media_request.session_context = request.session_context
                    media_request.original_lang = request.lang
                    media_request.original_question = request.question

                    for agent in self.agents.values():
                        if agent.enabled and agent.id in translator["media_agents"]:
                            requests[agent.id] = media_request  # specify agent request
        return requests

    def chat(self, request, agent_sessions):
        agents = [agent for agent in self.agents.values() if agent.id in agent_sessions]
        logger.info("All agents %r", [agent.id for agent in agents])

        if request.question.startswith(":"):
            return
        translate_requests = self.expand_translate_requests(request)
        translate_agents = [agent for agent in agents if agent.id in translate_requests]
        if request.question.lower().startswith("event."):
            allowed_agents = [
                "SoulTalkAgent",
            ]
            agents = [agent for agent in agents if agent.type in allowed_agents]
            translate_agents = [
                agent for agent in translate_agents if agent.type in allowed_agents
            ]
            if not agents:
                logger.warning("No soultalk agent")
        is_prompt = request.question[0] == "{" and request.question[-1] == "}"
        if is_prompt:
            agents = [agent for agent in agents if agent.prompt_responses is True]
            translate_agents = [
                agent for agent in translate_agents if agent.prompt_responses is True
            ]
            if not agents:
                logger.warning("No prompt agents enabled")
        agents = [agent for agent in agents if request.lang in agent.languages]
        if translate_agents:
            logger.info("Translate agents %s", translate_agents)
            agents.extend(translate_agents)
        if not agents:
            logger.warning("No agents for chat request")
            return
        else:
            logger.info("Using agents %s", [agent.id for agent in agents])

        if "agent" in request.context and request.context["agent"]:
            try:
                requested_agents = [
                    name.strip()
                    for name in request.context.pop("agent").split(",")
                    if name.strip()
                ]
                agents = [agent for agent in agents if agent.id in requested_agents]
                if len(agents) == 0:
                    logger.error("The agent with id %r is not found", requested_agents)
                    return
                logger.info("Use agents %s", [agent.id for agent in agents])
            except Exception as ex:
                logger.error(ex)
                return
        else:
            agents = [agent for agent in agents if agent.enabled]

        def keyfunc(agent):
            return 150 if agent.level < 150 else 250

        agent_batches = {}
        levels = []
        agents = sorted(agents, key=keyfunc)
        for k, g in groupby(agents, key=keyfunc):
            agent_batches[k] = list(g)
            levels.append(k)

        levels = sorted(levels)
        # before_builtin_agents = [agent for agent in agents if 100 <= agent.level < 200]
        # among_builtin_agents = [agent for agent in agents if 200 <= agent.level < 300]
        # after_builtin_agents = [agent for agent in agents if 300 <= agent.level < 400]

        for n_batch, level in enumerate(levels, 1):
            agent_batch = agent_batches[level]
            logger.info(
                "Batch %s/%s: agents: (%s) %s",
                n_batch,
                len(agent_batches),
                len(agent_batch),
                [agent.id for agent in agent_batch],
            )
            for responses in self._chat(
                request, agent_batch, agent_sessions, translate_requests
            ):
                responses = [response for response in responses if response.valid()]
                if responses:
                    logger.info("Yielded valid responses %s", len(responses))
                yield responses

    # Keeps track of agents that are still running
    def _agent_chat(self, agent, session, request, running_agents):
        # Keep reference to the same dict. Each requests will create new dict
        result = None
        try:
            result = agent.chat(session, request)
        except Exception as ex:
            logger.exception(ex)
            raise ex
        finally:
            with self.running_agents_lock:
                running_agents.pop(agent.id, None)
                current_lvl = agent.current_level()
                if len(running_agents) == 0:
                    min_level_waiting = 1000
                else:
                    min_level_waiting = min(
                        a.current_level() for a in running_agents.values()
                    )
        return (result, current_lvl, min_level_waiting)

    def _chat(self, request, agents, agent_sessions, translate_requests):
        if not agents:
            return []

        timeout = request.context.get("timeout")  # timeout by request
        results = Queue()
        running_agents = {agent.id: agent for agent in agents}
        tasks = [
            (
                self._agent_chat,
                agent,
                agent_sessions.get(agent.id),
                translate_requests.get(agent.id, request),
                running_agents,
            )
            for agent in agents
        ]

        job = threading.Thread(target=self._run_tasks, args=(tasks, results, timeout))
        job.deamon = True
        job.start()
        waiting_responses = []
        # Keeps track until first responses can be published
        start = time.time()
        # Flag that its time to publish the responses regardles of queue wait
        finished = False  # all tasks are finished
        publish_responses = request.hybrid_mode  # Set true to hybrid
        preference_sum = PreferentialResponseResolver.preferences(agents)
        while True:
            try:
                response = results.get(block=False)
                if response is None:
                    finished = True
                else:
                    # Got response but its None
                    if response[0] is None:
                        continue
                    # Preference to be random but why wait, we can randomize before ranking
                    if preference_sum > 0 and not publish_responses:
                        chance = random.random()
                        if chance * preference_sum < response[0].preference:
                            for w in waiting_responses:
                                w[0].preference = 0
                            publish_responses = True
                        else:
                            preference_sum -= max(0, response[0].preference)
                    # Return preference 11 results immediatly
                    if response[0].preference > 10:
                        publish_responses = True
            except Empty:
                response = None  # Response is None
                if self.interrupt and self.interrupt.is_set():
                    logger.info("Interrupted no more results will be returned")
                    self.interrupt.clear()
                    return
            # Put results to waiting queue
            if response is not None:  # Not the end of all threads or timeout
                # Gets agent result, agent level, and current minimum level waiting
                res, lvl, min_lvl = response
                if res is not None:
                    waiting_responses.append((res, lvl, min_lvl))
            # Check if its time to publish the responses
            its_time = (
                (time.time() - start > self.min_wait_for)
                or finished
                or publish_responses
            )
            something_there = len(waiting_responses) > 0
            if something_there and its_time:
                min_lvl = max([res[2] for res in waiting_responses])
                # Yield the responses that has no agents of lower level
                published_responses = [
                    res[0] for res in waiting_responses if res[1] < min_lvl + 1
                ]
                # publish
                if published_responses:
                    yield published_responses
                waiting_responses = [
                    res for res in waiting_responses if res[1] > min_lvl
                ]
            # Break on the end
            if finished:
                break
            time.sleep(0.02)

        # if there are fallback responses publish for ranking
        yield [res[0] for res in waiting_responses]
        return

    def _run_tasks(self, tasks, results, timeout=None):
        not_graceful = True
        timeout = timeout or self.timeout

        with ThreadPoolExecutor(max_workers=20) as executor:
            fs = {executor.submit(*task) for task in tasks}
            try:
                for future in as_completed(fs, timeout=timeout):
                    try:
                        response = future.result()
                        if response is not None:
                            results.put(response)
                    except Exception as ex:
                        logger.exception(ex)
            except TimeoutError as ex:
                logger.error("Timeout: %s", ex)
                if not_graceful:
                    # https://gist.github.com/clchiou/f2608cbe54403edb0b13
                    executor._threads.clear()
                    concurrent.futures.thread._threads_queues.clear()
            finally:
                results.put(None)  # poison item indicates the process is done
